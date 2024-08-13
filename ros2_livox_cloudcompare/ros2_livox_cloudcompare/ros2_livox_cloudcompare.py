import rclpy
from rclpy.node import Node
import psutil

from std_msgs.msg import String
from pointcloud_registration_msgs.srv import Registration
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped

import cloudComPy as cc               # import the CloudComPy module
import numpy as np
import transforms3d as tf3d

class CloudCompareLivox(Node):

    def __init__(self):
        super().__init__('CloudCompareRegistration')

        self.declare_parameter("PC_FILE", "/Datasets/G40_Poincloud_Tiny.las")
        self.declare_parameter("ICP_overlap_ratio", 0.1)

        
        pc_file = self.get_parameter("PC_FILE").value
        self.cloud = cc.loadPointCloud(pc_file)

        self.srv = self.create_service(Registration, 'register', self.register_cb)

    # Registration.srv:
    # sensor_msgs/PointCloud2 pointcloud (Assumed in sensor frame)
    # geometry_msgs/Transform initial_transform (Between sensor frame and world frame)
    # ---
    # geometry_msgs/TransformStamped header
    def register_cb(self, request, response):
        sensor_name = request.name
        self.get_logger().info(f'Incoming request to register {sensor_name}')

        # Transform into a cloudcompare pointcloud
        pc_msg = request.pointcloud
        init_transf = request.initial_transform
        cc_pc = self.pointcloud2_to_cc(sensor_name, pc_msg, init_transf)

        # Crop PointCloud down if specified
        crop_corners = request.crop_corner
        if len(crop_corners) > 0:
            cc_pc = self.crop_pointcloud(cc_pc, crop_corners)

        # Perform Cloud2Cloud Distance and clip on threshold
        threshold = request.clip_distance_threshold
        cc_pc = self.clip_distance(cc_pc, threshold)

        # Perform ICP (Using CC ICP)
        # https://www.simulation.openfields.fr/documentation/CloudComPy/html/userUseCases.html#cloud-registration
        res=cc.ICP(data=cc_pc, model=self.cloud,
                    minRMSDecrease=1.e-5, maxIterationCount=20,         # defaults
                    removeFarthestPoints=False,                         # defaults
                    method=cc.CONVERGENCE_TYPE.MAX_ITER_CONVERGENCE,    # defaults
                    adjustScale=False,                                  # defaults
                    finalOverlapRatio = self.get_parameter("ICP_overlap_ratio").value,
                    randomSamplingLimit = 100000) 

        # Change transform back into ROS2
        response.transform = self.ccGLMatrix_to_transform(res.transMat)
        response.transform.header = init_transf.header
        response.transform.child_frame_id = init_transf.child_frame_id

        return response
    
    def pointcloud2_to_cc(self, pc_name, pointcloud2_msg, init_transform):
        # Use read_points to convert the PointCloud2 message to a generator of points
        points = pc2.read_points(pointcloud2_msg, skip_nans=True)
        
        # Convert the generator to a list and then to a NumPy array
        xyz_array = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)

        # Create a point cloud object in cloudcompare-py
        cc_pc = cc.ccPointCloud(pc_name)
        cc_pc.coordsFromNPArray_copy(xyz_array[:, :3])

        # Apply Initial Transformation
        tf = init_transform.transform
        quaternion = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
        rotmat = tf3d.quaternions.quat2mat(quaternion)
        depl = (tf.translation.x, tf.translation.y, tf.translation.z)

        # transform = cc.ccGLMatrix.FromQuaternionAndTranslation(quaternion, depl) # <- doesnt seem to work? not found.
        transform = cc.ccGLMatrix(tuple(rotmat[0,:]), 
                                  tuple(rotmat[1,:]), 
                                  tuple(rotmat[2,:]), 
                                  depl)
        cc_pc.applyRigidTransformation(transform)

        return cc_pc

    # cc_pc is the clound compare pointcloud format
    # threshold
    def clip_distance(self, cc_pc, threshold):
        # https://www.simulation.openfields.fr/documentation/CloudComPy/html/userUseCases.html#compute-distances-between-two-clouds-c2c
        nbCpu = psutil.cpu_count()
        bestOctreeLevel = cc.DistanceComputationTools.determineBestOctreeLevel(self.cloud, None, cc_pc)
        params = cc.Cloud2CloudDistancesComputationParams()
        params.maxThreadCount = nbCpu
        params.octreeLevel = bestOctreeLevel
        res = cc.DistanceComputationTools.computeCloud2CloudDistances(cc_pc, self.cloud, params)
        # This should generate a scalar field with the distances

        # Now filter by scalar fields
        nsf = cc_pc.getNumberOfScalarFields()
        self.get_logger().info(f"Num Scalar Feilds: {nsf}")
        sfc = cc_pc.getScalarField(nsf - 1)
        cc_pc.setCurrentOutScalarField(nsf - 1)
        fcloud = cc.filterBySFValue(threshold, sfc.getMax(), cc_pc) # Original threshold is 0.01 for the example
        
        return fcloud

    # cc_pc is the clound compare pointcloud format
    # crop_cube is geometry_msgs/msg/Point[]
    # https://www.simulation.openfields.fr/documentation/CloudComPy/html/userUseCases.html#cut-a-cloud-or-a-mesh-with-a-polyline
    def crop_pointcloud(self, cc_pc, crop_cube):
        mnpt = crop_cube[0]
        mxpt = crop_cube[1]
        polyline_pts = np.array([
            [mnpt.x, mnpt.y, mnpt.z],  # Bottom-front-left
            [mxpt.x, mnpt.y, mnpt.z],  # Bottom-front-right
            [mxpt.x, mxpt.y, mnpt.z],  # Bottom-back-right
            [mnpt.x, mxpt.y, mnpt.z],  # Bottom-back-left
            [mnpt.x, mnpt.y, mxpt.z],  # Top-front-left
            [mxpt.x, mnpt.y, mxpt.z],  # Top-front-right
            [mxpt.x, mxpt.y, mxpt.z],  # Top-back-right
            [mnpt.x, mxpt.y, mxpt.z],  # Top-back-left
        ])
        temppc = cc.ccPointCloud()
        temppc.coordsFromNPArray_copy(polyline_pts)
        cut_polyline = cc.ccPolyline(temppc)
        cut_polyline.setClosed(True)

        for i in range(3):
            res_pc = cc_pc.crop2D(cut_polyline, i, True)
            if res_pc is not None:
                cc_pc = res_pc
            else:
                self.get_logger().warn(f"Cropping Pointcloud, Ortho {i} returned Empty")
        return cc_pc

    def ccGLMatrix_to_transform(self, glMat):
        
        mat = np.array(glMat.data()).reshape(4,4)
        quat = tf3d.quaternions.mat2quat(mat[:3, :3])
        trans = mat[3,:3] 

        tf = TransformStamped()
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]

        tf.transform.translation.x = trans[0]
        tf.transform.translation.y = trans[1]
        tf.transform.translation.z = trans[2]

        return tf


    def timer_callback(self):
        msg = String()
        msg.data = f'{self.cloud.getName()}: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s", pc_name: "%s"' % (msg.data, self.cloud.getName()))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CloudCompareLivox()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()