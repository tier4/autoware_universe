"""
Workflow sync script for autoware_universe.

Syncs workflows from autowarefoundation/autoware_universe according to workflow_sync_setting.yaml
"""

from __future__ import annotations

import logging
import os
import sys
from io import StringIO
from pathlib import Path

import requests

# Use ruamel.yaml for formatting preservation
from ruamel.yaml import YAML
from ruamel.yaml import YAMLError as yaml_YAMLError
from ruamel.yaml.scalarstring import PlainScalarString

# Set up logging
logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

# Configuration
GITHUB_TOKEN = os.environ.get("GITHUB_TOKEN")
UPSTREAM_REPO = "autowarefoundation/autoware_universe"
WORKFLOWS_DIR = Path(".github/workflows")
SETTINGS_FILE = "workflow_sync_setting.yaml"
RAW_URL_TEMPLATE = "https://raw.githubusercontent.com/{repo}/main/.github/workflows/{filename}"

# Initialize YAML parser with maximum formatting preservation
yaml = YAML()
yaml.preserve_quotes = True  # Preserve original quote styles
yaml.map_indent = 2
yaml.sequence_indent = 4
yaml.sequence_dash_offset = 0
yaml.default_flow_style = False
yaml.allow_unicode = True
yaml.encoding = "utf-8"
yaml.width = 4096  # Prevent line wrapping
yaml.indent(mapping=2, sequence=4, offset=2)
# Configure to avoid unnecessary quotes for simple strings
yaml.default_style = None


def load_settings(path: str) -> dict:
    """Load workflow sync settings from YAML file."""
    try:
        # Use a separate YAML instance for settings to avoid conflicts
        settings_yaml = YAML(typ="safe")
        with Path(path).open() as f:
            return settings_yaml.load(f)
    except FileNotFoundError:
        logger.exception("Error: Settings file '%s' not found.", path)
        sys.exit(1)
    except Exception:
        logger.exception("Error: Invalid YAML in '%s'", path)
        sys.exit(1)


def github_raw_url(filename: str) -> str:
    """Generate GitHub raw URL for a workflow file."""
    return RAW_URL_TEMPLATE.format(repo=UPSTREAM_REPO, filename=filename)


def download_workflow(filename: str) -> str | None:
    """Download a workflow file from GitHub."""
    # HTTP status code constants
    http_not_found = 404
    http_ok = 200

    url = github_raw_url(filename)
    headers = {"Authorization": f"token {GITHUB_TOKEN}"} if GITHUB_TOKEN else {}

    logger.info("  Downloading from: %s", url)

    try:
        response = requests.get(url, headers=headers, timeout=30)

        if response.status_code == http_not_found:
            logger.warning("  Warning: Workflow '%s' not found in upstream repository", filename)
            result = None
        elif response.status_code != http_ok:
            logger.error(
                "  Error: Failed to download '%s': %s %s",
                filename,
                response.status_code,
                response.text,
            )
            result = None
        else:
            result = response.text

        return result  # noqa: TRY300

    except requests.RequestException:
        logger.exception("  Error: Network error downloading '%s'", filename)
        return None


def set_nested_value(data: dict, keys: list[str], value: str) -> None:
    """Set a nested value in a YAML structure while preserving types."""
    current = data
    for key in keys[:-1]:
        if key not in current:
            current[key] = {}
        current = current[key]

    final_key = keys[-1]

    # For simple alphanumeric strings with hyphens/dots (like ubuntu-22.04-m),
    # use PlainScalarString to avoid quotes
    if (
        isinstance(value, str)
        and value  # not empty
        and not value.startswith("${{")  # not a GitHub expression
        and '"' not in value  # no quotes in the value
        and "'" not in value  # no quotes in the value
        and not value.isspace()
    ):  # not just whitespace
        current[final_key] = PlainScalarString(value)
    else:
        # For complex values, parse as YAML to preserve original behavior
        try:
            temp_yaml = YAML(typ="safe")
            parsed_value = temp_yaml.load(StringIO(value))
            current[final_key] = parsed_value
        except yaml_YAMLError:
            current[final_key] = value


def apply_modifications(content: str, updates: list) -> dict | None:
    """Apply modifications to workflow YAML content while preserving formatting."""
    try:
        # Load the YAML while preserving formatting
        yaml_obj = yaml.load(StringIO(content))
        if yaml_obj is None:
            logger.error("  Error: Empty or invalid YAML content")
            return None
    except yaml_YAMLError:
        logger.exception("  Error: Invalid YAML content")
        return None

    for update in updates:
        # Handle both string format and dict format from YAML
        if isinstance(update, dict):
            # If it's a dict, convert it to key: value format
            for key_path, value in update.items():
                logger.info("    Updating %s = %s", key_path, value)
                keys = key_path.split(".")
                set_nested_value(yaml_obj, keys, str(value))
        elif isinstance(update, str):
            # Handle string format "key.path: value"
            if ":" not in update:
                logger.warning("  Warning: Skipping invalid update format: %s", update)
                continue

            key_path, value = update.split(":", 1)
            key_path = key_path.strip()
            value = value.strip()

            logger.info("    Updating %s = %s", key_path, value)
            keys = key_path.split(".")
            set_nested_value(yaml_obj, keys, value)
        else:
            logger.warning("  Warning: Skipping invalid update format: %s", update)

    return yaml_obj


def write_workflow_file(filepath: Path, content: str | dict, *, is_yaml_obj: bool = False) -> None:
    """Write workflow content to file."""
    try:
        with filepath.open("w", encoding="utf-8") as f:
            if is_yaml_obj:
                # Use ruamel.yaml to preserve formatting
                yaml.dump(content, f)
            else:
                f.write(content)
        logger.info("  ✓ Written to %s", filepath)
    except OSError:
        logger.exception("  Error: Failed to write '%s'", filepath)


def check_unique_tier4_workflows(unique_tier4_workflows: list) -> None:
    """Check existence of unique TIER IV workflows."""
    if unique_tier4_workflows:
        logger.info("\n🔍 Checking unique TIER IV workflows...")
        for workflow in unique_tier4_workflows:
            workflow_path = WORKFLOWS_DIR / workflow
            if workflow_path.exists():
                logger.info("  ✓ Found: %s", workflow)
            else:
                logger.warning("  ⚠️  Missing: %s", workflow)


def process_keep_workflows(keep_workflows: list) -> None:
    """Process 'keep' workflows by downloading and writing them."""
    logger.info("\n🔗 Syncing %s 'keep' workflows...", len(keep_workflows))
    for workflow in keep_workflows:
        logger.info("\n📥 Syncing (keep): %s", workflow)
        content = download_workflow(workflow)
        if content:
            write_workflow_file(WORKFLOWS_DIR / workflow, content)


def process_modify_workflows(modify_workflows: dict) -> None:
    """Process 'modify' workflows by downloading, modifying, and writing them."""
    logger.info("\n🔧 Syncing %s 'modify' workflows...", len(modify_workflows))
    for workflow, modifications in modify_workflows.items():
        logger.info("\n📥 Syncing (modify): %s", workflow)
        content = download_workflow(workflow)
        if content:
            updates = modifications.get("updates", [])
            logger.info("  Applying %s modifications...", len(updates))

            modified_yaml = apply_modifications(content, updates)
            if modified_yaml:
                write_workflow_file(WORKFLOWS_DIR / workflow, modified_yaml, is_yaml_obj=True)


def check_extra_workflows(
    keep_workflows: list,
    modify_workflows: dict,
    unique_tier4_workflows: list,
) -> None:
    """Check for extra workflows not mentioned in settings."""
    logger.info("\n🔍 Checking for extra workflows...")
    if WORKFLOWS_DIR.exists():
        local_workflows = {p.name for p in WORKFLOWS_DIR.glob("*.yaml") if p.is_file()}
        local_workflows.update({p.name for p in WORKFLOWS_DIR.glob("*.yml") if p.is_file()})

        expected_workflows = (
            set(keep_workflows) | set(modify_workflows.keys()) | set(unique_tier4_workflows)
        )
        extra_workflows = local_workflows - expected_workflows

        if extra_workflows:
            logger.warning(
                "\n⚠️  Found %s extra workflows not mentioned in settings:",
                len(extra_workflows),
            )
            for workflow in sorted(extra_workflows):
                logger.warning("   - %s", workflow)
            logger.warning(
                "\n💡 These workflows are present locally but not in 'keep' or 'modify' sections.",
            )
            logger.warning(
                "   Consider adding them to 'ignore' section if they should remain local-only.",
            )
        else:
            logger.info(
                "✅ No extra workflows found - all local workflows are managed by settings.",
            )
    else:
        logger.info("i  No local workflows directory found yet.")


def main() -> None:
    logger.info("🔄 Starting workflow sync...\n")

    # Check for GitHub token
    if not GITHUB_TOKEN:
        logger.error("❌ Error: GITHUB_TOKEN environment variable is required.")
        logger.error("   Set it with: export GITHUB_TOKEN=your_token_here")
        sys.exit(1)

    # Load settings
    logger.info("📖 Loading settings from %s", SETTINGS_FILE)
    settings = load_settings(SETTINGS_FILE)

    workflows_config = settings.get("workflows", {})
    keep_workflows = workflows_config.get("keep", [])
    ignore_workflows = set(workflows_config.get("ignore", []))
    modify_workflows = workflows_config.get("modify", {})
    unique_tier4_workflows = workflows_config.get("unique_tier4_workflows", [])

    logger.info("   Keep: %s workflows", len(keep_workflows))
    logger.info("   Modify: %s workflows", len(modify_workflows))
    logger.info("   Ignore: %s workflows", len(ignore_workflows))
    logger.info("   Unique TIER IV: %s workflows", len(unique_tier4_workflows))

    # Ensure workflows directory exists
    WORKFLOWS_DIR.mkdir(parents=True, exist_ok=True)
    logger.info("\n📁 Workflows directory: %s", WORKFLOWS_DIR)

    # Process workflows
    check_unique_tier4_workflows(unique_tier4_workflows)
    process_keep_workflows(keep_workflows)
    process_modify_workflows(modify_workflows)
    check_extra_workflows(keep_workflows, modify_workflows, unique_tier4_workflows)

    logger.info("\n✅ Workflow sync completed!")


if __name__ == "__main__":
    main()
