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
	@echo 'Go2 final: make go2-final A=/path/to/map_bag B=/path/to/route_bag'
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
A ?= data/mapping
B ?= data/localization
go2-final: export GO2_A = $(A)
go2-final: export GO2_B = $(B)
go2-final: export GO2_RUN = $(RUN_NAME)
.PHONY: go2-final go2-demo-data
go2-demo-data:
	bash tools/go2_inputs.sh
go2-final:
	@for bag in "$$GO2_A" "$$GO2_B"; do \
	  test -f "$$bag/metadata.yaml" || { test -f "$$bag" && test "$${bag%.mcap}" != "$$bag"; } || \
	    { echo "Expected a ROS 2 bag directory or .mcap file: $$bag" >&2; exit 2; }; done
	@docker image inspect mgts-competition-glim:gpu >/dev/null 2>&1 || $(MAKE) build-gpu
	$(COMPOSE) -f compose.yaml -f compose.gpu.yaml run --rm --pull never \
	  -e MGTS_REPLAY_1X=1 -e OPENBLAS_NUM_THREADS=1 \
	  -v "$$(realpath "$$GO2_A"):/inputs/A.mcap:ro" -v "$$(realpath "$$GO2_B"):/inputs/B.mcap:ro" \
	  -v "$(CURDIR)/competition:/mgts/competition:ro" -v "$(CURDIR)/tools:/mgts/tools:ro" \
	  competition python3 -m tools.go2_final "$$GO2_RUN"

.PHONY: go2-live
go2-live: go2-map go2-route-odometry
	DATA_DIR="$(CURDIR)/data/official" $(COMPOSE) -f compose.yaml -f compose.gpu.yaml run --rm --pull never -e OPENBLAS_NUM_THREADS=1 -v "$(CURDIR)/competition:/mgts/competition:ro" -v "$(CURDIR)/tools:/mgts/tools:ro" competition python3 -m tools.go2_track "/results/$(RUN_NAME)" --glim $(ARGS)

.PHONY: go2-result
go2-result:
	bash tools/go2_rviz.sh $(if $(wildcard $(RESULTS_DIR)/$(RUN_NAME)/mapping_source.json),glim,result) "$(RUN_NAME)"
