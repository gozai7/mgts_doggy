.DEFAULT_GOAL := help
.PHONY: help build build-solver robot teleop stop maps map localize eval test

# Use the prepared engine when available, respecting explicit Docker settings.
ifeq ($(strip $(DOCKER_CONTEXT)$(DOCKER_HOST)),)
ifneq ($(shell docker context inspect mgts --format '{{.Name}}' 2>/dev/null),)
export DOCKER_CONTEXT := mgts
endif
endif

A ?= A
B ?= B
MAP ?= map-$(A)
ROUTE ?= routes/$(MAP)/$(B)
MODE ?= mapping
export A B MAP ROUTE MODE ARGS

help: ## показать команды
	@awk 'BEGIN {FS = ": ## "} /^[a-z-]+: ## / {printf "  make %-10s %s\n", $$1, $$2}' Makefile

build: ## собрать образы Gazebo и решателя
	docker compose build sim solve

build-solver: ## собрать только SLAM, AMCL и чекер
	docker compose build solve

robot: SCENE = office
robot: ## записать обход A; MODE=inspection записывает маршрут B
	@set -f; session="$$A"; if [ "$$MODE" = inspection ]; then session="$$B"; fi; \
	exec docker compose run --rm --name mgts-capture sim "$$MODE" "$$session" --scene "$$SCENE" $$ARGS

teleop: ## управлять Go2 из второго терминала
	docker exec -it mgts-capture /mgts/entrypoint ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.25 -p turn:=0.5

stop: ## завершить запись с сохранением bag
	docker stop --signal=SIGINT --timeout=60 mgts-capture

maps: ## показать имена доступных карт
	@for yaml in runs/*/map.yaml; do [ -f "$$yaml" ] || continue; dir=$${yaml%/map.yaml}; printf '%s\n' "$${dir#runs/}"; done

map: ## построить карту MAP по записи A, например make map A=survey MAP=office
	@set -f; exec docker compose run --rm solve map "/data/$$A" "/runs/$$MAP" $$ARGS

localize: ## построить маршрут B на выбранной карте, например make localize B=inspection MAP=office
	@set -f; exec docker compose run --rm solve localize "/data/$$B" "/runs/$$ROUTE" --map "/runs/$$MAP/map.yaml" $$ARGS

eval: ## оценить сохранённый маршрут B на карте MAP
	@set -f; route="$$ROUTE"; \
	if [ "$(origin ROUTE)" = file ] && [ ! -d "runs/$$route" ] && [ -f "runs/route-$$B/run.json" ]; then route="route-$$B"; fi; \
	exec docker compose run --rm eval "/runs/$$MAP" "/runs/$$route" --truth-root /truth $$ARGS

test: ## без MAP — smoke-тест; с MAP и B — локализация и оценка на выбранной карте
ifneq ($(origin MAP),file)
	$(MAKE) localize
	$(MAKE) eval ARGS=
else
	docker run --rm --network none -v "$(CURDIR)/tests:/tests:ro" -v "$(CURDIR)/sim:/mgts/sim:ro" mgts-solver bash /tests/smoke.sh
endif

SCENE ?= corridor
RATE ?= 3
export SCENE
.PHONY: bags bench

bags: ## создать тестовые A/B и эталон для SCENE=corridor|colonnade|eight|atrium|atrium-opaque
	docker compose run --rm --entrypoint /mgts/entrypoint sim python3 /mgts/sim/scene_bags.py "$$SCENE"

# Named benchmark outputs follow the same MAP and ROUTE convention as make test.
ifeq ($(origin MAP),file)
bench: MAP = $(SCENE)
endif
ifeq ($(origin ROUTE),file)
bench: ROUTE = routes/$(MAP)/$(SCENE)-B
endif

bench: ## построить карту по готовой паре SCENE и сравнить AMCL с офлайн-уточнением
	@if [ ! -f "runs/$$MAP/run.json" ]; then \
		$(MAKE) map A="$(SCENE)-A" MAP="$(MAP)" ARGS="--profile /mgts/configs/planar.yaml --rate $(RATE)"; \
	fi
	$(MAKE) test B="$(SCENE)-B" MAP="$(MAP)" ROUTE="$(ROUTE)" ARGS="--profile /mgts/configs/planar.yaml --rate $(RATE) --refine $(ARGS)"
