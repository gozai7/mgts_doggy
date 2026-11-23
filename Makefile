.DEFAULT_GOAL := help
.PHONY: help build robot teleop stop map localize eval test

# Use the prepared engine when available, respecting explicit Docker settings.
ifeq ($(strip $(DOCKER_CONTEXT)$(DOCKER_HOST)),)
ifneq ($(shell docker context inspect mgts --format '{{.Name}}' 2>/dev/null),)
export DOCKER_CONTEXT := mgts
endif
endif

A ?= A
B ?= B
MAP ?= map-$(A)
ROUTE ?= route-$(B)
MODE ?= mapping
export A B MAP ROUTE MODE ARGS

help: ## показать команды
	@awk 'BEGIN {FS = ": ## "} /^[a-z]+: ## / {printf "  make %-10s %s\n", $$1, $$2}' Makefile

build: ## собрать образы Gazebo и решателя
	docker compose build sim solve

robot: ## записать обход A; MODE=inspection записывает маршрут B
	@set -f; session="$$A"; if [ "$$MODE" = inspection ]; then session="$$B"; fi; \
	exec docker compose run --rm --name mgts-capture sim "$$MODE" "$$session" $$ARGS

teleop: ## управлять Go2 из второго терминала
	docker exec -it mgts-capture /mgts/entrypoint ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.25 -p turn:=0.5

stop: ## завершить запись с сохранением bag
	docker stop --signal=SIGINT --timeout=60 mgts-capture

map: ## построить карту по A и сохранить в runs/MAP
	@set -f; exec docker compose run --rm solve map "/data/$$A" "/runs/$$MAP" $$ARGS

localize: ## локализовать B на карте MAP и сохранить в runs/ROUTE
	@set -f; exec docker compose run --rm solve localize "/data/$$B" "/runs/$$ROUTE" --map "/runs/$$MAP/map.yaml" $$ARGS

eval: ## проверить маршрут; эталон подключается при наличии обеих записей истины
	@set -f; set -- $$ARGS; if [ -d "truth/$$A" ] && [ -d "truth/$$B" ]; then \
	    set -- --truth-a "/truth/$$A" --truth-b "/truth/$$B" "$$@"; fi; \
	exec docker compose run --rm eval "/runs/$$MAP" "/runs/$$ROUTE" "$$@"

test: ## запустить сквозной тест на двух тестовых bag
	docker run --rm --network none -v "$(CURDIR)/tests:/tests:ro" mgts-solver bash /tests/smoke.sh
