#!/usr/bin/env python3

import argparse
import json
import shutil
import subprocess
import sys
from typing import Iterable, List, Set, Any


def run_copr_cli(args: List[str]) -> subprocess.CompletedProcess:
    """Run copr-cli with the given arguments and return the completed process.

    Raises a RuntimeError if the command fails.
    """
    cmd = ["copr-cli", *args]
    try:
        completed = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "copr-cli not found. Install it with: sudo dnf install copr-cli"
        ) from exc

    if completed.returncode != 0:
        raise RuntimeError(
            f"Command failed: {' '.join(cmd)}\nSTDOUT:\n{completed.stdout}\nSTDERR:\n{completed.stderr}"
        )
    return completed


def _extract_pkg_name(build: Any) -> str:
    """Best-effort extraction of package name from a copr build JSON object."""
    # Known places for the package name across CLI/API variants
    candidates: List[Any] = [
        build.get("package_name"),
        build.get("pkg"),
        build.get("package"),  # sometimes a dict or a string
        build.get("name"),  # beware: may not be strictly the package name
    ]
    for cand in candidates:
        if isinstance(cand, str) and cand:
            return cand
        if isinstance(cand, dict):
            # common nested structure: { "name": "pkg" }
            nested_name = cand.get("name")
            if isinstance(nested_name, str) and nested_name:
                return nested_name
    return ""


def _list_build_ids_json(project: str, package_name: str) -> List[int]:
    completed = run_copr_cli(["list-builds", project, "--output-format", "json"])  # json output
    builds = json.loads(completed.stdout)
    ids: List[int] = []
    for build in builds:
        pkg = _extract_pkg_name(build)
        if pkg == package_name:
            build_id = build.get("id")
            try:
                ids.append(int(build_id))
            except (TypeError, ValueError):
                continue
    return sorted(set(ids))


def _list_build_ids_text(project: str, package_name: str) -> List[int]:
    # Fallback: parse text-row output: "<id>\t<package>\t<state>"
    completed = run_copr_cli(["list-builds", project])
    ids: List[int] = []
    for line in completed.stdout.splitlines():
        parts = line.strip().split()  # copr prints tabs/spaces; split on whitespace
        if len(parts) < 2:
            continue
        maybe_id, maybe_pkg = parts[0], parts[1]
        if maybe_pkg != package_name:
            continue
        try:
            ids.append(int(maybe_id))
        except ValueError:
            continue
    return sorted(set(ids))


def list_build_ids(project: str, package_name: str) -> List[int]:
    """Return all build IDs for the given package in the given project.

    Tries JSON first, then falls back to parsing text output.
    """
    # Try JSON path first
    try:
        ids = _list_build_ids_json(project, package_name)
    except json.JSONDecodeError:
        ids = []
    except RuntimeError as exc:
        # If copr-cli failed for some reason, re-raise
        raise

    if ids:
        return ids

    # Fallback to text parsing (older CLI defaults)
    return _list_build_ids_text(project, package_name)


def _extract_state(build: Any) -> str:
    """Best-effort extraction of build state from a copr build JSON object."""
    for key in ("state", "status", "build_state", "state_str"):
        val = build.get(key)
        if isinstance(val, str) and val:
            return val
    return ""


def _list_builds_json(project: str, package_name: str) -> List[dict]:
    completed = run_copr_cli(["list-builds", project, "--output-format", "json"])  # json output
    builds_raw = json.loads(completed.stdout)
    result: List[dict] = []
    for build in builds_raw:
        pkg = _extract_pkg_name(build)
        if pkg != package_name:
            continue
        try:
            bid = int(build.get("id"))
        except (TypeError, ValueError):
            continue
        state = _extract_state(build)
        result.append({"id": bid, "state": state})
    # sort ascending by id
    result.sort(key=lambda b: b["id"])
    return result


def _list_builds_text(project: str, package_name: str) -> List[dict]:
    completed = run_copr_cli(["list-builds", project])
    result: List[dict] = []
    for line in completed.stdout.splitlines():
        parts = line.strip().split()
        # Expect at least: <id> <package> <state>
        if len(parts) < 3:
            continue
        maybe_id, maybe_pkg, maybe_state = parts[0], parts[1], parts[2]
        if maybe_pkg != package_name:
            continue
        try:
            bid = int(maybe_id)
        except ValueError:
            continue
        result.append({"id": bid, "state": maybe_state})
    result.sort(key=lambda b: b["id"])
    return result


def list_builds(project: str, package_name: str) -> List[dict]:
    """Return list of builds as dicts {id:int, state:str} for a package.

    Tries JSON first, then falls back to parsing text output.
    """
    try:
        builds = _list_builds_json(project, package_name)
    except json.JSONDecodeError:
        builds = []
    except RuntimeError:
        raise
    if builds:
        return builds
    return _list_builds_text(project, package_name)


def chunked(items: List[int], size: int) -> Iterable[List[int]]:
    """Yield chunks of the given size from items."""
    for i in range(0, len(items), size):
        yield items[i : i + size]


def delete_builds(build_ids: List[int], dry_run: bool, chunk_size: int) -> None:
    """Delete builds via 'copr-cli delete-build', possibly in chunks."""
    if not build_ids:
        print("Nothing to delete.")
        return

    if dry_run:
        print("[DRY-RUN] Would delete the following build IDs:")
        print(" ".join(str(bid) for bid in build_ids))
        return

    if shutil.which("copr-cli") is None:
        raise RuntimeError("copr-cli not found in PATH. Install with: sudo dnf install copr-cli")

    for group in chunked(build_ids, chunk_size):
        cmd = ["delete-build", *[str(bid) for bid in group]]
        print(f"Deleting builds: {' '.join(str(bid) for bid in group)}")
        run_copr_cli(cmd)


def parse_args(argv: List[str]) -> argparse.Namespacehttps://download.copr.fedorainfracloud.org/results/saypaul/open-rmf/fedora-42-x86_64/09445127-ros-jazzy-rmf-api-msgs/builder-live.log.gz:
    parser = argparse.ArgumentParser(
        description=(
            "Delete all COPR builds for a package in a project, except the build IDs provided."
        )
    )
    parser.add_argument(
        "project",
        help="COPR project in the form 'owner/project' (e.g., saypaul/open-rmf)",
    )
    parser.add_argument(
        "package_name",
        help="Package name as shown by 'copr-cli list-builds <project>' (e.g., ros-jazzy-rmf-simulation)",
    )
    parser.add_argument(
        "keep_ids",
        nargs="*",
        help="One or more build IDs to keep (all others will be deleted)",
    )
    parser.add_argument(
        "--keep-latest-succeeded",
        action="store_true",
        help=(
            "Keep only the most recent (highest ID) build if its state is 'succeeded'; "
            "delete the rest. If the latest build did not succeed, no deletion is performed."
        ),
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=100,
        help="Number of build IDs to delete per copr-cli invocation (default: 100)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be deleted without performing any deletion",
    )
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)

    try:
        builds = list_builds(project=args.project, package_name=args.package_name)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if not builds:
        print(
            f"No builds found for package '{args.package_name}' in project '{args.project}'."
        )
        return 0

    all_ids = [b["id"] for b in builds]

    keep: Set[int] = set()
    if args.keep_latest_succeeded:
        latest = max(builds, key=lambda b: b["id"])  # highest ID assumed most recent
        if str(latest.get("state", "")).lower() == "succeeded":
            keep = {latest["id"]}
            print(f"Latest build {latest['id']} is 'succeeded'. Will keep only this build.")
        else:
            print(
                f"Latest build {latest['id']} state is '{latest.get('state','')}'. "
                "Not deleting anything."
            )
            return 0
    else:
        if not args.keep_ids:
            print(
                "Either provide one or more KEEP IDs or use --keep-latest-succeeded.",
                file=sys.stderr,
            )
            return 2
        try:
            keep = {int(x) for x in args.keep_ids}
        except ValueError:
            print("Keep IDs must be integers.", file=sys.stderr)
            return 2

    to_delete = [bid for bid in all_ids if bid not in keep]

    if not to_delete:
        print(
            f"All builds for '{args.package_name}' are in the keep list. Nothing to delete."
        )
        return 0

    print(
        f"Found {len(all_ids)} builds for '{args.package_name}'. Keeping {len(keep)}: "
        + ", ".join(str(x) for x in sorted(keep))
    )
    print(f"Deleting {len(to_delete)} builds...")

    try:
        delete_builds(build_ids=to_delete, dry_run=args.dry_run, chunk_size=args.chunk_size)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 3

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:])) 