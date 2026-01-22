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

docker-up:
	docker compose up --build -d

docker-down:
	docker compose down

test: docker-test ## Run tests in Docker container (alias for docker-test)

docker-test: ## Build Docker image and run tests inside container
	docker build -t ${IMAGE_ID}-test --target test -f Dockerfile.test .
	docker run --rm ${IMAGE_ID}-test

# Version Management Commands
version-show: ## Show current version
	@grep "^current_version" .bumpversion.cfg | sed 's/current_version = //'

version-patch: ## Bump patch version (0.0.X)
	bump2version patch

version-minor: ## Bump minor version (0.X.0)
	bump2version minor

version-major: ## Bump major version (X.0.0)
	bump2version major

version-prerelease: ## Bump to prerelease version (e.g., 0.2.3-alpha.1)
	bump2version prerelease

version-build: ## Bump build version (e.g., 0.2.3+001)
	bump2version build

version-tag: ## Create and push version tag
	@VERSION=$$(grep "current_version" .bumpversion.cfg | sed 's/current_version = //'); \
	echo "Current version: $$VERSION"; \
	git tag -a "v$$VERSION" -m "Version $$VERSION"; \
	git push origin "v$$VERSION"