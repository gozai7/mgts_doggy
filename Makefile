.DEFAULT_GOAL := help
CONFIG ?= configs/competition/dataset.yaml
PROFILE ?=
BACKEND ?=
ROS_APT_MIRROR ?= http://packages.ros.org/ros2/ubuntu
PHASE ?= mapping
COMPOSE ?= docker compose
DATA_DIR ?= $(CURDIR)/data
RESULTS_DIR ?= $(CURDIR)/results
export DATA_DIR RESULTS_DIR
RUN = $(COMPOSE) run --rm competition
SELECT = $(if $(PROFILE),--profile $(PROFILE)) $(if $(BACKEND),--backend $(BACKEND))
.PHONY: help build build-gpu build-localizers bundle-gpu doctor normalize prepare mapping localization baseline odometry occupancy evaluate run-all benchmark-localization smoke-test test-unit legacy-build legacy-help
help:
	@echo 'make build; edit configs/competition/dataset.yaml; make doctor; make baseline; make mapping; make localization; make occupancy; make evaluate'
	@echo 'CONFIG=... chooses a configuration inside the project. make test-unit runs without ROS. Legacy: make -f Makefile.legacy help'
	@echo 'Profiles: make localization PROFILE=baseline|beluga|ndt_kiss|beluga_kiss; make odometry processes B; make benchmark-localization'
build:
	$(COMPOSE) build competition
build-gpu:
	docker compose -f compose.yaml -f compose.gpu.yaml build competition
bundle-gpu:
	bash tools/bundle_gpu.sh
doctor normalize prepare mapping localization baseline odometry occupancy evaluate run-all benchmark-localization:
	$(RUN) python3 -m competition.runner $@ --config /mgts/$(CONFIG) --phase $(PHASE) $(SELECT)
build-localizers:
	docker build -f Dockerfile.localizers --target localizers --build-arg BUILD_JOBS=$(or $(BUILD_JOBS),2) --build-arg ROS_APT_MIRROR=$(ROS_APT_MIRROR) -t mgts-localizers:cpu .
test-unit:
	python3 -m pytest -q tests/competition
smoke-test:
	$(RUN) bash tools/ros_smoke.sh
legacy-build:
	$(MAKE) -f Makefile.legacy build
legacy-help:
	$(MAKE) -f Makefile.legacy help
