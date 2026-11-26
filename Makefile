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
