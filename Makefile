#!/usr/bin/make -f

# Fedora packaging for Open-RMF
# Follows the pattern of ~/git_repos/rosfed

DISTRO ?= jazzy
ARCH ?= $(shell uname -m)
FEDORA_VER ?= $(shell rpm -E %fedora 2>/dev/null || echo "39")
BUILD_DIR = fedrmf
SPECS_DIR = $(BUILD_DIR)/SPECS
SOURCES_DIR = $(BUILD_DIR)/SOURCES
RPMS_DIR = $(BUILD_DIR)/RPMS
SRPMS_DIR = $(BUILD_DIR)/SRPMS
BUILD_ROOT = $(BUILD_DIR)/BUILD

# New generated specs directory
SPECS_GENERATED_DIR = specs_generated
TEMPLATES_DIR = templates
CFG_DIR = cfg
PATCHES_DIR = patches

# Build options (rosfed-style parameters)
REBUILD ?= 
ONLY_NEW ?= 
ENABLE_NET ?= 
VERBOSE ?= 

# COPR and chroot options
COPR_REPO ?= 
CHROOT ?= fedora-$(FEDORA_VER)-$(ARCH)
MOCK_CONFIG ?= 

# RMF repositories
RMF_REPOS_URL = https://raw.githubusercontent.com/open-rmf/rmf/$(DISTRO)/rmf.repos

# Build script with parameters
PYTHON_ARGS = $(DISTRO)
ifneq ($(REBUILD),)
PYTHON_ARGS += -r
endif
ifneq ($(ONLY_NEW),)
PYTHON_ARGS += --only-new
endif
ifneq ($(ENABLE_NET),)
PYTHON_ARGS += --enable-net
endif
ifneq ($(VERBOSE),)
PYTHON_ARGS += --verbose
endif
ifneq ($(COPR_REPO),)
PYTHON_ARGS += --copr-repo $(COPR_REPO)
endif
ifneq ($(CHROOT),)
PYTHON_ARGS += --chroot $(CHROOT)
endif
ifneq ($(MOCK_CONFIG),)
PYTHON_ARGS += --mock-config $(MOCK_CONFIG)
endif

.PHONY: all clean setup fetch-repos generate-specs build-rpms install-deps help list-packages rebuild only-new enable-net build-order copr-build mock-build generate-specs-from-yaml dep-tree download-sources update-cfg clean-changelogs build-srpms-ordered

all: setup generate-specs

help:
	@echo "Open-RMF Fedora Builder (rosfed-style)"
	@echo "====================================="
	@echo ""
	@echo "Targets:"
	@echo "  setup        - Create build directories"
	@echo "  fetch-repos  - Download rmf.repos file"
	@echo "  update-cfg   - Update cfg/<distro>/ from distro/<distro>.repo file"
	@echo "  generate-specs - Generate specs from cfg/<distro>/*.yaml using templates"
	@echo "  clean-changelogs - Clean all changelogs and reset versions to blank"
	@echo "  build-rpms   - Build all RPM packages"
	@echo "  install-deps - Install build dependencies"
	@echo "  list-packages - List available packages"
	@echo "  list-existing - List existing packages"
	@echo "  build-order  - Create dependency-based build order"
	@echo "  dep-tree     - Show dependency tree and build order"
	@echo "  download-sources - Download source tarballs for all packages"
	@echo "  download-sources-ordered - Download sources in build order"
	@echo "  build-srpms  - Build SRPMs from downloaded sources"
	@echo "  build-srpms-ordered - Build SRPMs in dependency order (downloads sources first)"
	@echo "  copr-build   - Build packages in COPR (generates SRPMs in build order and submits)"
	@echo "  mock-build   - Build packages using mock"
	@echo "  clean        - Clean build artifacts"
	@echo ""
	@echo "rosfed-style Parameters:"
	@echo "  REBUILD=1    - Rebuild all packages (-r)"
	@echo "  ONLY_NEW=1   - Only process new packages (--only-new)"
	@echo "  ENABLE_NET=1 - Enable network during build (--enable-net)"
	@echo "  VERBOSE=1    - Enable verbose output (--verbose)"
	@echo ""
	@echo "COPR and Chroot Parameters:"
	@echo "  COPR_REPO=user/repo - Target COPR repository"
	@echo "  CHROOT=fedora-ver-arch - Target chroot (default: $(CHROOT))"
	@echo "  MOCK_CONFIG=path - Custom mock configuration file"
	@echo ""
	@echo "Variables:"
	@echo "  DISTRO       - ROS distribution (default: jazzy)"
	@echo "  ARCH         - Target architecture (default: $(ARCH))"
	@echo "  FEDORA_VER   - Fedora version (default: $(FEDORA_VER))"
	@echo ""
	@echo "Examples:"
	@echo "  make generate-specs-from-yaml DISTRO=jazzy"
	@echo "  make build-rpms-from-yaml DISTRO=jazzy"
	@echo "  make copr-build-from-yaml COPR_REPO=user/repo DISTRO=jazzy"
	@echo "  make dep-tree PACKAGE=rmf_traffic"
	@echo "  make download-sources-ordered"

setup:
	@echo "Setting up build environment for ROS $(DISTRO)..."
	mkdir -p $(SPECS_DIR) $(SOURCES_DIR) $(RPMS_DIR) $(SRPMS_DIR) $(BUILD_ROOT) $(SPECS_GENERATED_DIR) $(PATCHES_DIR)
	@echo "Build directories created."

fetch-repos: setup
	@echo "Fetching rmf.repos for $(DISTRO)..."
	wget -O $(BUILD_DIR)/rmf.repos $(RMF_REPOS_URL)
	@echo "rmf.repos downloaded."

# Clean changelogs and reset versions
clean-changelogs:
	@echo "Cleaning all changelogs and resetting versions to blank..."
	python3 generate_specs.py --distro $(DISTRO) --clean-changelogs
	@echo "Changelogs cleaned. Run 'make update-cfg' to repopulate from distro file."

# Original spec generation method
update-cfg:
	@echo "Updating cfg/$(DISTRO)/ from distro/$(DISTRO).repo..."
	@if [ ! -f distro/$(DISTRO).repo ]; then \
		echo "Error: distro/$(DISTRO).repo not found. Please create it manually."; \
		echo "This file should contain the package definitions in repos format."; \
		exit 1; \
	fi
	python3 generate_specs.py --distro $(DISTRO) --update-cfg --repos-file distro/$(DISTRO).repo --commit-msg "Updated from distro/$(DISTRO).repo"
	@echo "Configuration files updated."

generate-specs: update-cfg
	@echo "Generating spec files from cfg/$(DISTRO)/ configurations..."
	python3 generate_specs.py --distro $(DISTRO) --cfg-dir cfg --template-dir $(TEMPLATES_DIR) --output-dir $(SPECS_GENERATED_DIR) --patches-dir $(PATCHES_DIR)
	@echo "Spec files generated in $(SPECS_GENERATED_DIR)/"

# Generate spec for single package
generate-spec-package:
	@if [ -z "$(PACKAGE)" ]; then \
		echo "Error: PACKAGE must be specified (e.g., PACKAGE=rmf_utils)"; \
		exit 1; \
	fi
	@echo "Generating spec for package $(PACKAGE)..."
	python3 generate_specs.py --distro $(DISTRO) --cfg-dir cfg --template-dir $(TEMPLATES_DIR) --output-dir $(SPECS_GENERATED_DIR) --patches-dir $(PATCHES_DIR) --package $(PACKAGE)

list-packages:
	@echo "Listing packages from cfg configuration..."
	python3 generate_specs.py --distro $(DISTRO) --list-packages

# Dependency tree and build order management
dep-tree: 
	@echo "Building dependency tree..."
	python3 build_dependency_tree.py --cfg-dir $(CFG_DIR) --build-dir $(BUILD_DIR) build-order --save

show-dep-tree:
	@if [ -z "$(PACKAGE)" ]; then \
		echo "Error: PACKAGE must be specified (e.g., PACKAGE=rmf_traffic)"; \
		exit 1; \
	fi
	@echo "Showing dependency tree for $(PACKAGE)..."
	python3 build_dependency_tree.py --cfg-dir $(CFG_DIR) tree $(PACKAGE)

show-package-info:
	@if [ -z "$(PACKAGE)" ]; then \
		echo "Error: PACKAGE must be specified (e.g., PACKAGE=rmf_utils)"; \
		exit 1; \
	fi
	@echo "Showing information for $(PACKAGE)..."
	python3 build_dependency_tree.py --cfg-dir $(CFG_DIR) info $(PACKAGE)

list-packages-with-deps:
	@echo "Listing packages with dependencies..."
	python3 build_dependency_tree.py --cfg-dir $(CFG_DIR) list --with-deps

# Source download management
download-sources: setup
	@echo "Downloading source tarballs for all packages..."
	python3 build_dependency_tree.py --cfg-dir $(CFG_DIR) --sources-dir $(SOURCES_DIR) download --all

download-sources-ordered: setup dep-tree
	@echo "Downloading source tarballs in build order..."
	python3 build_dependency_tree.py --cfg-dir $(CFG_DIR) --sources-dir $(SOURCES_DIR) download --build-order

download-source-package: setup
	@if [ -z "$(PACKAGE)" ]; then \
		echo "Error: PACKAGE must be specified (e.g., PACKAGE=rmf_utils)"; \
		exit 1; \
	fi
	@echo "Downloading source for $(PACKAGE)..."
	python3 build_dependency_tree.py --cfg-dir $(CFG_DIR) --sources-dir $(SOURCES_DIR) download --package $(PACKAGE)

# SRPM building
build-srpms: generate-specs-from-yaml download-sources-ordered
	@echo "Building SRPMs from generated specs and downloaded sources..."
	@for spec in $(SPECS_GENERATED_DIR)/*.spec; do \
		if [ -f "$$spec" ]; then \
			package_name=$$(basename "$$spec" .spec); \
			echo "Building SRPM for $$package_name..."; \
			rpmbuild --define "_topdir $(PWD)/$(BUILD_DIR)" \
				--define "_sourcedir $(PWD)/$(SOURCES_DIR)" \
				--define "_srcrpmdir $(PWD)/$(SRPMS_DIR)" \
				-bs "$$spec" || echo "Failed to build SRPM for $$package_name"; \
		fi \
	done
	@echo "SRPM build completed. Check $(SRPMS_DIR)/ for packages."

build-srpm-package: setup
	@if [ -z "$(PACKAGE)" ]; then \
		echo "Error: PACKAGE must be specified (e.g., PACKAGE=rmf_utils)"; \
		exit 1; \
	fi
	@echo "Building SRPM for $(PACKAGE)..."
	@$(MAKE) generate-spec-package PACKAGE=$(PACKAGE)
	@$(MAKE) download-source-package PACKAGE=$(PACKAGE)
	@spec_file="$(SPECS_GENERATED_DIR)/ros-$(DISTRO)-$$(echo $(PACKAGE) | sed 's/_/-/g').spec"; \
	if [ -f "$$spec_file" ]; then \
		rpmbuild --define "_topdir $(PWD)/$(BUILD_DIR)" \
			--define "_sourcedir $(PWD)/$(SOURCES_DIR)" \
			--define "_srcrpmdir $(PWD)/$(SRPMS_DIR)" \
			-bs "$$spec_file"; \
	else \
		echo "Spec file not found: $$spec_file"; \
		exit 1; \
	fi

build-order:
	@echo "Creating dependency-based build order..."
	python3 build_dependency_tree.py --cfg-dir cfg/$(DISTRO) build-order --save
	@echo "Build order created and saved to cfg/build_order.json"

load-build-order:
	@echo "Loading existing build order..."
	python3 build_rmf_fedora.py --load-build-order $(DISTRO)

show-build-order: load-build-order

install-deps:
	@echo "Installing build dependencies..."
	sudo dnf install -y \
		rpm-build \
		rpmdevtools \
		cmake \
		gcc-c++ \
		git \
		wget \
		python3 \
		python3-yaml \
		python3-networkx \
		python3-jinja2 \
		eigen3-devel \
		yaml-cpp-devel \
		json-devel \
		opencv-devel \
		qt5-qtbase-devel \
		boost-devel \
		openssl-devel \
		sqlite-devel \
		libcurl-devel \
		pybind11-devel
	@echo "Build dependencies installed."

install-copr-deps: install-deps
	@echo "Installing COPR build dependencies..."
	sudo dnf install -y \
		copr-cli \
		mock \
		fedpkg
	@echo "COPR dependencies installed."

# Build RPMs from generated specs
build-rpms-from-yaml: generate-specs-from-yaml
	@echo "Building RPM packages from generated specs..."
	@for spec in $(SPECS_GENERATED_DIR)/*.spec; do \
		if [ -f "$$spec" ]; then \
			echo "Building $$spec..."; \
			rpmbuild --define "_topdir $(PWD)/$(BUILD_DIR)" \
				--define "_builddir $(PWD)/$(BUILD_ROOT)" \
				--define "_rpmdir $(PWD)/$(RPMS_DIR)" \
				--define "_srcrpmdir $(PWD)/$(SRPMS_DIR)" \
				--define "_sourcedir $(PWD)/$(SOURCES_DIR)" \
				-ba "$$spec" || echo "Failed to build $$spec"; \
		fi \
	done
	@echo "RPM build completed. Check $(RPMS_DIR)/ for packages."

# COPR build from YAML specs
copr-build-from-yaml: build-srpms
	@echo "Building packages in COPR from YAML configs..."
	@if [ -z "$(COPR_REPO)" ]; then \
		echo "Error: COPR_REPO must be specified (e.g., COPR_REPO=user/repo)"; \
		exit 1; \
	fi
	@echo "Target COPR repo: $(COPR_REPO)"
	@echo "Target chroot: $(CHROOT)"
	@for srpm in $(SRPMS_DIR)/*.src.rpm; do \
		if [ -f "$$srpm" ]; then \
			echo "Submitting $$srpm to COPR..."; \
			copr-cli build $(COPR_REPO) "$$srpm" --chroot $(CHROOT) || echo "Failed to submit $$srpm"; \
		fi \
	done

copr-build-package: setup
	@if [ -z "$(PACKAGE)" ]; then \
		echo "Error: PACKAGE must be specified (e.g., PACKAGE=rmf_utils)"; \
		exit 1; \
	fi
	@if [ -z "$(COPR_REPO)" ]; then \
		echo "Error: COPR_REPO must be specified (e.g., COPR_REPO=user/repo)"; \
		exit 1; \
	fi
	@echo "Building $(PACKAGE) in COPR..."
	@$(MAKE) build-srpm-package PACKAGE=$(PACKAGE)
	@srpm_file=$$(find $(PWD)/$(SRPMS_DIR) -name "ros-$(DISTRO)-$$(echo $(PACKAGE) | sed 's/_/-/g')-*.src.rpm" | head -1); \
	if [ -f "$$srpm_file" ]; then \
		echo "Submitting $$srpm_file to COPR..."; \
		copr-cli build $(COPR_REPO) "$$srpm_file" --chroot $(CHROOT); \
	else \
		echo "SRPM not found for $(PACKAGE)"; \
		exit 1; \
	fi

copr-build: generate-specs build-order
	@echo "Building packages in COPR with proper build order..."
	@if [ -z "$(COPR_REPO)" ]; then \
		echo "Error: COPR_REPO must be specified (e.g., COPR_REPO=user/repo)"; \
		exit 1; \
	fi
	@echo "Target COPR repo: $(COPR_REPO)"
	@echo "Target chroot: $(CHROOT)"
	@echo "Step 1: Downloading sources in build order..."
	python3 build_dependency_tree.py --cfg-dir cfg/$(DISTRO) --sources-dir $(SOURCES_DIR) download --build-order
	@echo "Step 2: Building SRPMs in build order..."
	@build_order=$$(python3 -c "import json; data=json.load(open('cfg/build_order.json')); print(' '.join(data['build_order']))"); \
	for package in $$build_order; do \
		spec_file="$(SPECS_GENERATED_DIR)/ros-$(DISTRO)-$$(echo $$package | tr '_' '-').spec"; \
		if [ -f "$$spec_file" ]; then \
			echo "Building SRPM for $$package..."; \
			rpmbuild --define "_topdir $(PWD)/$(BUILD_DIR)" \
				--define "_sourcedir $(PWD)/$(SOURCES_DIR)" \
				--define "_srcrpmdir $(PWD)/$(SRPMS_DIR)" \
				-bs "$$spec_file" || echo "Failed to build SRPM for $$package"; \
		else \
			echo "Warning: Spec file not found for $$package: $$spec_file"; \
		fi \
	done
	@echo "Step 3: Submitting SRPMs to COPR in build order..."
	@build_order=$$(python3 -c "import json; data=json.load(open('cfg/build_order.json')); print(' '.join(data['build_order']))"); \
	for package in $$build_order; do \
		srpm_file="$(SRPMS_DIR)/ros-$(DISTRO)-$$(echo $$package | tr '_' '-')-*.src.rpm"; \
		if ls $$srpm_file 1> /dev/null 2>&1; then \
			echo "Submitting $$package to COPR..."; \
			copr-cli build $(COPR_REPO) $$srpm_file --chroot $(CHROOT) || echo "Failed to submit $$package to COPR"; \
			sleep 10; \
		else \
			echo "Warning: SRPM not found for $$package"; \
		fi \
	done
	@echo "COPR build submission completed!"

mock-build: generate-specs
	@echo "Building packages with mock..."
	@echo "Target chroot: $(CHROOT)"
	@for spec in $(SPECS_DIR)/*.spec; do \
		if [ -f "$$spec" ]; then \
			package_name=$$(basename "$$spec" .spec); \
			echo "Building $$package_name with mock..."; \
			python3 -c "from build_rmf_fedora import RMFFedoraBuilder; \
				b = RMFFedoraBuilder(chroot='$(CHROOT)', verbose=True); \
				b.mock_build_package('$${package_name#ros-$(DISTRO)-}'.replace('-', '_'))"; \
		fi \
	done

build-rpms: generate-specs
	@echo "Building RPM packages..."
	@for spec in $(SPECS_DIR)/*.spec; do \
		if [ -f "$$spec" ]; then \
			echo "Building $$spec..."; \
			rpmbuild --define "_topdir $(PWD)/$(BUILD_DIR)" \
				--define "_builddir $(PWD)/$(BUILD_ROOT)" \
				--define "_rpmdir $(PWD)/$(RPMS_DIR)" \
				--define "_srcrpmdir $(PWD)/$(SRPMS_DIR)" \
				--define "_sourcedir $(PWD)/$(SOURCES_DIR)" \
				-ba "$$spec" || echo "Failed to build $$spec"; \
		fi \
	done
	@echo "RPM build completed. Check $(RPMS_DIR)/ for packages."

list-packages:
	@echo "Generated packages:"
	@find $(SPECS_DIR) -name "*.spec" -exec basename {} .spec \; 2>/dev/null || echo "No spec files found. Run 'make generate-specs' first."

list-existing:
	@echo "Listing existing packages..."
	python3 build_rmf_fedora.py --list-existing $(DISTRO)

check-deps:
	@echo "Checking ROS $(DISTRO) dependencies..."
	@command -v ros2 >/dev/null 2>&1 || (echo "ROS 2 not found. Please install ros-$(DISTRO)-desktop first." && exit 1)
	@source /opt/ros/$(DISTRO)/setup.bash && echo "ROS $(DISTRO) environment ready."

check-copr:
	@echo "Checking COPR configuration..."
	@command -v copr-cli >/dev/null 2>&1 || (echo "copr-cli not found. Run 'make install-copr-deps'" && exit 1)
	@if [ -n "$(COPR_REPO)" ]; then \
		echo "Target COPR repo: $(COPR_REPO)"; \
		copr-cli list || echo "Run 'copr-cli login' to authenticate"; \
	else \
		echo "No COPR_REPO specified"; \
	fi
	@echo "Target chroot: $(CHROOT)"

check-mock:
	@echo "Checking mock configuration..."
	@command -v mock >/dev/null 2>&1 || (echo "mock not found. Run 'make install-copr-deps'" && exit 1)
	@echo "Available mock configs:"
	@ls /etc/mock/*.cfg | head -5 | sed 's/.*\///;s/.cfg//'
	@echo "Target chroot: $(CHROOT)"

clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(BUILD_DIR) $(SPECS_GENERATED_DIR)
	@echo "Build artifacts cleaned."

# Convenience targets for rosfed-style usage
rebuild: 
	@$(MAKE) generate-specs REBUILD=1

only-new:
	@$(MAKE) generate-specs ONLY_NEW=1

enable-net:
	@$(MAKE) generate-specs ENABLE_NET=1

rebuild-only-new:
	@$(MAKE) generate-specs REBUILD=1 ONLY_NEW=1

rebuild-enable-net:
	@$(MAKE) generate-specs REBUILD=1 ENABLE_NET=1

only-new-enable-net:
	@$(MAKE) generate-specs ONLY_NEW=1 ENABLE_NET=1

rebuild-only-new-enable-net:
	@$(MAKE) generate-specs REBUILD=1 ONLY_NEW=1 ENABLE_NET=1

verbose:
	@$(MAKE) generate-specs VERBOSE=1

# COPR convenience targets
copr-setup: install-copr-deps check-copr

copr-rebuild:
	@$(MAKE) copr-build REBUILD=1

copr-only-new:
	@$(MAKE) copr-build ONLY_NEW=1

# Mock convenience targets  
mock-setup: install-copr-deps check-mock

mock-rebuild:
	@$(MAKE) mock-build REBUILD=1

mock-only-new:
	@$(MAKE) mock-build ONLY_NEW=1

# YAML-based convenience targets
yaml-specs:
	@$(MAKE) generate-specs-from-yaml

yaml-build:
	@$(MAKE) build-rpms-from-yaml

yaml-copr:
	@$(MAKE) copr-build-from-yaml

yaml-srpms:
	@$(MAKE) build-srpms

# Development targets
dev-setup: install-deps
	@echo "Setting up development environment..."
	sudo dnf install -y \
		colcon-common-extensions \
		python3-rosdep \
		python3-vcstool
	@echo "Development environment ready."

# Install ROS 2 Jazzy if not present
install-ros2:
	@echo "Installing ROS 2 $(DISTRO)..."
	@if ! command -v ros2 >/dev/null 2>&1; then \
		echo "ROS 2 not found. Installing..."; \
		sudo dnf install -y \
			https://github.com/ros/rosdistro/releases/latest/download/ros2-jazzy-*.rpm || \
		echo "Please install ROS 2 $(DISTRO) manually from https://docs.ros.org/en/jazzy/Installation/Fedora-Install-Binary-Packages.html"; \
	else \
		echo "ROS 2 is already installed."; \
	fi

# Quick test build of a single package
test-build: generate-specs
	@echo "Running test build..."
	@if [ -f "$(SPECS_DIR)/ros-$(DISTRO)-rmf-utils.spec" ]; then \
		rpmbuild --define "_topdir $(PWD)/$(BUILD_DIR)" \
			--define "_builddir $(PWD)/$(BUILD_ROOT)" \
			--define "_rpmdir $(PWD)/$(RPMS_DIR)" \
			--define "_srcrpmdir $(PWD)/$(SRPMS_DIR)" \
			--define "_sourcedir $(PWD)/$(SOURCES_DIR)" \
			-ba "$(SPECS_DIR)/ros-$(DISTRO)-rmf-utils.spec"; \
	else \
		echo "No test package found. Generate specs first."; \
	fi

test-build-yaml: generate-specs-from-yaml
	@echo "Running test build from YAML..."
	@if [ -f "$(SPECS_GENERATED_DIR)/ros-$(DISTRO)-rmf-utils.spec" ]; then \
		rpmbuild --define "_topdir $(PWD)/$(BUILD_DIR)" \
			--define "_builddir $(PWD)/$(BUILD_ROOT)" \
			--define "_rpmdir $(PWD)/$(RPMS_DIR)" \
			--define "_srcrpmdir $(PWD)/$(SRPMS_DIR)" \
			--define "_sourcedir $(PWD)/$(SOURCES_DIR)" \
			-ba "$(SPECS_GENERATED_DIR)/ros-$(DISTRO)-rmf-utils.spec"; \
	else \
		echo "No test package found. Generate specs first."; \
	fi

test-copr-build: generate-specs
	@echo "Running test COPR build..."
	@if [ -z "$(COPR_REPO)" ]; then \
		echo "Error: COPR_REPO must be specified"; \
		exit 1; \
	fi
	@if [ -f "$(SPECS_DIR)/ros-$(DISTRO)-rmf-utils.spec" ]; then \
		python3 -c "from build_rmf_fedora import RMFFedoraBuilder; \
			b = RMFFedoraBuilder(copr_repo='$(COPR_REPO)', chroot='$(CHROOT)', verbose=True); \
			b.copr_build_package('rmf_utils')"; \
	else \
		echo "No test package found. Generate specs first."; \
	fi

test-mock-build: generate-specs
	@echo "Running test mock build..."
	@if [ -f "$(SPECS_DIR)/ros-$(DISTRO)-rmf-utils.spec" ]; then \
		python3 -c "from build_rmf_fedora import RMFFedoraBuilder; \
			b = RMFFedoraBuilder(chroot='$(CHROOT)', verbose=True); \
			b.mock_build_package('rmf_utils')"; \
	else \
		echo "No test package found. Generate specs first."; \
	fi

test-srpm:
	@echo "Testing SRPM build for rmf_utils..."
	@$(MAKE) build-srpm-package PACKAGE=rmf_utils

test-copr-single:
	@echo "Testing single package COPR build for rmf_utils..."
	@if [ -z "$(COPR_REPO)" ]; then \
		echo "Error: COPR_REPO must be specified"; \
		exit 1; \
	fi
	@$(MAKE) copr-build-package PACKAGE=rmf_utils COPR_REPO=$(COPR_REPO)

# Show current configuration
show-config:
	@echo "Current Configuration:"
	@echo "====================="
	@echo "DISTRO:     $(DISTRO)"
	@echo "ARCH:       $(ARCH)"
	@echo "FEDORA_VER: $(FEDORA_VER)"
	@echo "CHROOT:     $(CHROOT)"
	@echo "COPR_REPO:  $(if $(COPR_REPO),$(COPR_REPO),not set)"
	@echo "REBUILD:    $(if $(REBUILD),enabled,disabled)"
	@echo "ONLY_NEW:   $(if $(ONLY_NEW),enabled,disabled)"
	@echo "ENABLE_NET: $(if $(ENABLE_NET),enabled,disabled)"
	@echo "VERBOSE:    $(if $(VERBOSE),enabled,disabled)"
	@echo ""
	@echo "Directories:"
	@echo "CFG_DIR:               $(CFG_DIR)"
	@echo "TEMPLATES_DIR:         $(TEMPLATES_DIR)"
	@echo "PATCHES_DIR:           $(PATCHES_DIR)"
	@echo "SPECS_GENERATED_DIR:   $(SPECS_GENERATED_DIR)"
	@echo ""
	@echo "Python command: python3 build_rmf_fedora.py $(PYTHON_ARGS)" 

# Build SRPMs in dependency order
build-srpms-ordered: generate-specs build-order
	@echo "Building SRPMs in dependency order..."
	@echo "Step 1: Downloading sources in build order..."
	python3 build_dependency_tree.py --cfg-dir cfg/$(DISTRO) --sources-dir $(SOURCES_DIR) download --build-order
	@echo "Step 2: Building SRPMs in build order..."
	@build_order=$$(python3 -c "import json; data=json.load(open('cfg/build_order.json')); print(' '.join(data['build_order']))"); \
	for package in $$build_order; do \
		spec_file="$(SPECS_GENERATED_DIR)/ros-$(DISTRO)-$$(echo $$package | tr '_' '-').spec"; \
		if [ -f "$$spec_file" ]; then \
			echo "Building SRPM for $$package..."; \
			rpmbuild --define "_topdir $(shell pwd)/$(BUILD_DIR)" \
				--define "_sourcedir $(shell pwd)/$(SOURCES_DIR)" \
				--define "_srcrpmdir $(shell pwd)/$(SRPMS_DIR)" \
				-bs "$$spec_file" || echo "Failed to build SRPM for $$package"; \
		else \
			echo "Warning: Spec file not found for $$package: $$spec_file"; \
		fi \
	done
	@echo "SRPM build completed in dependency order. Check $(SRPMS_DIR)/ for packages."

copr-build: build-srpms-ordered
	@echo "Submitting SRPMs to COPR in build order..."
	@if [ -z "$(COPR_REPO)" ]; then \
		echo "Error: COPR_REPO must be specified (e.g., COPR_REPO=user/repo)"; \
		exit 1; \
	fi
	@echo "Target COPR repo: $(COPR_REPO)"
	@echo "Target chroot: $(CHROOT)"
	@build_order=$$(python3 -c "import json; data=json.load(open('cfg/build_order.json')); print(' '.join(data['build_order']))"); \
	for package in $$build_order; do \
		srpm_file="$(SRPMS_DIR)/ros-$(DISTRO)-$$(echo $$package | tr '_' '-')-*.src.rpm"; \
		if ls $$srpm_file 1> /dev/null 2>&1; then \
			echo "Submitting $$package to COPR..."; \
			build_output=$$(copr-cli build $(COPR_REPO) $$srpm_file --chroot $(CHROOT) 2>&1); \
			echo "$$build_output"; \
			build_id=$$(echo "$$build_output" | grep -oP 'Created builds: \K\d+' | head -n1); \
			if [ -n "$$build_id" ]; then \
				echo "Build ID: $$build_id for package: $$package"; \
				copr-cli watch-build $$build_id || { \
					echo "Build $$build_id failed for $$package"; \
					log_url="https://download.copr.fedorainfracloud.org/results/$(COPR_REPO)/$(CHROOT)/0$$build_id-ros-$(DISTRO)-$$(echo $$package | tr '_' '-')/builder-live.log.gz"; \
					echo "Build log URL: $$log_url"; \
					echo "Analyzing build failure..."; \
					python3 -c "from build_failure_analyzer import analyze_build_failure; analyze_build_failure('$$log_url', '$$package', '$(DISTRO)')" || echo "Log analysis failed"; \
					echo "ERROR: Build failed for $$package - stopping due to dependency order"; \
					exit 1; \
				}; \
			else \
				echo "Failed to extract build ID for $$package"; \
				exit 1; \
			fi; \
			sleep 10; \
		else \
			echo "Warning: SRPM not found for $$package"; \
			exit 1; \
		fi \
	done
	@echo "COPR build submission completed successfully!" 