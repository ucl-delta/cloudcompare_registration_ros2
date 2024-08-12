MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

CONTAINERNAME?=cloudcompy310_build

all: build

build:
	docker build -t $(CONTAINERNAME) $(MAKEFILE_DIR)

run: build
	bash $(MAKEFILE_DIR)/run_container.sh -n ${CONTAINERNAME}

run_bash: build
	bash $(MAKEFILE_DIR)/run_container.sh -n ${CONTAINERNAME} -e -m

test: build
	bash $(MAKEFILE_DIR)/run_container.sh -n ${CONTAINERNAME} -a /run_tests.sh

.PHONY: all build run run_bash test