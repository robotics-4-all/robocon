IMAGE_ID ?= roboconnect

ONESHELL:
.PHONY: docker-build
.DEFAULT_GOAL := help

define PRINT_HELP_PYSCRIPT
import re, sys

for line in sys.stdin:
	match = re.match(r'^([a-zA-Z_-]+):.*?## (.*)$$', line)
	if match:
		target, help = match.groups()
		print("%-20s %s" % (target, help))
endef
export PRINT_HELP_PYSCRIPT

help:
	@python -c "$$PRINT_HELP_PYSCRIPT" < $(MAKEFILE_LIST)

docker-build: ## Build Docker Image
	docker build -t ${IMAGE_ID} .

docker-run: ## Run Docker Container
	docker run -it --rm -p 8080:8080 ${IMAGE_ID}