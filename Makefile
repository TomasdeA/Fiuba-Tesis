SHELL := /bin/bash

.PHONY: build shell run clean fmt

build:
	docker compose build

shell:
	docker compose run --rm dev bash

run:
	docker compose up -d && docker exec -it tesis_nav_dev bash

clean:
	docker compose down -v

fmt:
	find src -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" | xargs -r clang-format -i
