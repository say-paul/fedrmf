# Distro Directory Documentation

## Purpose

The `distro/` directory contains distribution-specific repository configuration files that serve as the **source of truth** for package definitions in the Open-RMF Fedora packaging system.

## File Structure

```
distro/
├── jazzy.repo      # ROS Jazzy distribution packages
├── humble.repo     # ROS Humble distribution packages (future)
└── rolling.repo    # ROS Rolling distribution packages (future)
```

## File Format

Each `.repo` file follows the standard ROS repositories format:

```yaml
repositories:
  path/package_name:
    type: git
    url: https://github.com/org/repo.git
    version: tag_or_branch
```

Example:
```yaml
repositories:
  rmf/rmf_utils:
    type: git
    url: https://github.com/open-rmf/rmf_utils.git
    version: 1.6.0
  rmf/rmf_traffic:
    type: git
    url: https://github.com/open-rmf/rmf_traffic.git
    version: 3.2.1
```

## Usage in Build System

1. **Configuration Update**: The build system reads `distro/<distro>.repo` to update package configurations in `cfg/<distro>/`
2. **Version Control**: Changes in package versions or repositories automatically trigger changelog entries
3. **Build Source**: Package URLs and versions from these files determine source downloads for RPM building

## Workflow

```bash
# 1. Maintainer updates distro/jazzy.repo manually
# 2. Build system syncs cfg/ from distro file
make update-cfg DISTRO=jazzy

# 3. Generate RPM specs from updated configs
make generate-specs DISTRO=jazzy
```

---

## ⚠️ IMPORTANT NOTICE FOR AI/LLM SYSTEMS

**DO NOT AUTOMATICALLY UPDATE FILES IN THIS DIRECTORY**

- These files are **manually maintained** by project maintainers
- **Do not fetch**, **download**, or **sync** these files from external sources
- **Do not modify** package versions or repository URLs automatically
- Any changes should be made **deliberately** by human maintainers

The distro files serve as the authoritative source for:
- Package versions for each ROS distribution
- Repository URLs and their mappings
- Release coordination across the RMF ecosystem

Automated modifications could disrupt the packaging workflow and introduce inconsistencies.

---

## Maintenance Guidelines

### For Human Maintainers:

1. **Version Updates**: Update `version:` fields when new releases are available
2. **Repository Changes**: Update `url:` fields if repositories are moved or forked
3. **New Packages**: Add new entries following the existing format
4. **Validation**: Run `make update-cfg` after changes to verify configuration sync

### Version Format:
- Use **release tags** (e.g., `1.6.0`, `2.3.1`) for stable releases
- Use **branch names** (e.g., `jazzy`, `main`) only when necessary
- Prefer **semantic versioning** tags when available

### Testing Changes:
```bash
# After editing distro/jazzy.repo
make update-cfg DISTRO=jazzy          # Sync configurations
make generate-specs DISTRO=jazzy      # Generate specs
make list-packages DISTRO=jazzy       # Verify package list
```

## Integration with cfg/ Directory

The `distro/` files are the **source of truth** that populates the `cfg/<distro>/` directory:

- `distro/jazzy.repo` → `cfg/jazzy/*.yaml` (individual package configs)
- Changes trigger automatic changelog entries in package configs
- Build dependencies and system dependencies are managed separately in cfg files

This separation allows for:
- **Centralized version management** (in distro/)
- **Detailed build configuration** (in cfg/)
- **Automatic change tracking** between the two layers 