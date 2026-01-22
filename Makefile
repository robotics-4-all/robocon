IMAGE_ID ?= robocon

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

test: docker-test ## Run tests in Docker container (alias for docker-test)

docker-test: ## Build Docker image and run tests inside container
	docker build -t ${IMAGE_ID}-test --target test -f Dockerfile.test .
	docker run --rm ${IMAGE_ID}-test