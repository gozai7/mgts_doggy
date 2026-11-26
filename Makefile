.DEFAULT_GOAL := help
CONFIG ?= configs/competition/dataset.yaml
PHASE ?= mapping
COMPOSE ?= docker compose
DATA_DIR ?= $(CURDIR)/data
RESULTS_DIR ?= $(CURDIR)/results
export DATA_DIR RESULTS_DIR
RUN = $(COMPOSE) run --rm competition
.PHONY: help build build-gpu bundle-gpu doctor normalize prepare mapping localization baseline occupancy evaluate run-all smoke-test test-unit legacy-build legacy-help
help:
	@echo 'Official Go2 bags: make go2-final (download bags, build GPU image if missing, map A and localize B)'
	@echo 'make build; edit configs/competition/dataset.yaml; make doctor; make baseline; make mapping; make localization; make occupancy; make evaluate'
	@echo 'CONFIG=... chooses a configuration inside the project. make test-unit runs without ROS. Legacy: make -f Makefile.legacy help'
build:
	$(COMPOSE) build competition
build-gpu:
	docker compose -f compose.yaml -f compose.gpu.yaml build competition
bundle-gpu:
	bash tools/bundle_gpu.sh
doctor normalize prepare mapping localization baseline occupancy evaluate run-all:
	$(RUN) python3 -m competition.runner $@ --config /mgts/$(CONFIG) --phase $(PHASE)
test-unit:
	python3 -m pytest -q tests/competition
smoke-test:
	$(RUN) bash tools/ros_smoke.sh
legacy-build:
	$(MAKE) -f Makefile.legacy build
legacy-help:
	$(MAKE) -f Makefile.legacy help

SCENE ?= equipment
SCENE_ROOT ?= $(CURDIR)/results/lidar-test-kit
.PHONY: scenes scene-test
scenes:
	python3 -m sim.lidar_suite "$(SCENE_ROOT)" $(foreach name,$(SCENES),--scene "$(name)")
scene-test:
	DATA_DIR="$(abspath $(SCENE_ROOT))/$(SCENE)/data" $(COMPOSE) -f compose.yaml -f compose.gpu.yaml run --rm --pull never competition python3 -m competition.runner run-all --config /data/dataset.yaml
	python3 -m sim.score_scene "$(SCENE_ROOT)/$(SCENE)/reference" "$(RESULTS_DIR)/lidar-scenes/$(SCENE)"

.PHONY: go2-rviz go2-map go2-route-odometry go2-locate
go2-rviz:
	bash tools/go2_rviz.sh $(PHASE)
go2-map: GO2_ACTION=mapping
go2-route-odometry: GO2_ACTION=route-odometry
go2-locate: GO2_ACTION=localization
go2-map go2-route-odometry go2-locate:
	DATA_DIR="$(CURDIR)/data/official" $(COMPOSE) -f compose.yaml -f compose.gpu.yaml run --rm --pull never -e MGTS_REPLAY_1X=1 -v "$(CURDIR)/competition:/mgts/competition:ro" competition python3 -m competition.runner $(GO2_ACTION) --config /mgts/configs/competition/official-go2.yaml

RUN_NAME ?= go2-live-$(shell date +%Y%m%d-%H%M%S)
.PHONY: go2-final
go2-final:
	bash tools/go2_inputs.sh
	@docker image inspect mgts-competition-glim:gpu >/dev/null 2>&1 || $(MAKE) build-gpu
	@run="$(RUN_NAME)"; $(MAKE) go2-live RUN_NAME="$$run" ARGS=--window-se3 && \
	  echo "Results: $(RESULTS_DIR)/$$run/ | RViz: make go2-result RUN_NAME=$$run"

.PHONY: go2-live
go2-live: go2-map go2-route-odometry
	DATA_DIR="$(CURDIR)/data/official" $(COMPOSE) -f compose.yaml -f compose.gpu.yaml run --rm --pull never -e OPENBLAS_NUM_THREADS=1 -v "$(CURDIR)/competition:/mgts/competition:ro" -v "$(CURDIR)/tools:/mgts/tools:ro" competition python3 -m tools.go2_track "/results/$(RUN_NAME)" --glim $(ARGS)

.PHONY: go2-result
go2-result:
	bash tools/go2_rviz.sh $(if $(wildcard $(RESULTS_DIR)/$(RUN_NAME)/mapping_source.json),glim,result) "$(RUN_NAME)"
