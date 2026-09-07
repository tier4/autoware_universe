# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Invariants of this package's launch files.

A launch file fails at deployment time or, worse, half-works: the node comes up,
publishes on a topic nobody reads, and the failure looks like a model that
produces nothing. These checks are cheap and static -- no ROS graph, no engine,
no container -- so they run in CI on every change.
"""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"
LAUNCH_FILES = sorted(LAUNCH_DIR.glob("*.xml"))


def _ids(paths):
    return [p.name for p in paths]


@pytest.fixture(scope="module")
def launch_files():
    assert LAUNCH_FILES, f"no launch files found under {LAUNCH_DIR}"
    return LAUNCH_FILES


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=_ids(LAUNCH_FILES))
def test_is_well_formed(path):
    """
    The file parses.

    Worth its own test because the XML rules are easy to trip over in prose: a
    comment may not contain a double hyphen, so an em-dash written as ``--``
    inside an explanation makes the whole launch file unloadable, and the error
    surfaces only when someone runs it.
    """
    ET.parse(path)


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=_ids(LAUNCH_FILES))
def test_composable_node_inherits_the_pushed_namespace(path):
    """
    No composable node pins an absolute namespace.

    ``launch_ros`` combines a composable node's namespace with the enclosing
    ``push-ros-namespace`` through ``prefix_namespace()``, which returns the
    node's own namespace unchanged when it is absolute. So ``namespace="/"``
    discards the enclosing namespace and puts the node at the root, while the
    standalone branch of the same launch file stays where it was put. The two
    spellings of one deployment then publish on different topics, and whatever
    was wired to the namespaced path -- a planning evaluator, a validator --
    reads an empty topic with no error anywhere.

    Omitting the attribute inherits, and inherits ``/`` when nothing was pushed,
    so the plain container case is unaffected.
    """
    for node in ET.parse(path).getroot().iter("composable_node"):
        namespace = node.get("namespace")
        assert namespace is None or not namespace.startswith("/"), (
            f"{path.name}: composable node '{node.get('name')}' pins the absolute "
            f"namespace '{namespace}', which discards any push-ros-namespace "
            f"around it. Omit the attribute to inherit."
        )


def _interface(node):
    """The parameters and remappings a node element declares, as a comparable set."""
    params = {
        ("param", p.get("from") or p.get("name"), p.get("value"))
        for p in node.findall("param")
    }
    remaps = {("remap", r.get("from"), r.get("to")) for r in node.findall("remap")}
    return params | remaps


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=_ids(LAUNCH_FILES))
def test_standalone_and_composed_branches_agree(path):
    """
    Where a launch file offers both a standalone node and a composed one, the two
    declare the same parameters and remappings.

    They are two spellings of one deployment, kept apart only because the XML
    frontend has no way to share a body between ``<node>`` and
    ``<composable_node>``. Nothing but this check notices when an argument is
    added to one and forgotten in the other, and the branch that was forgotten
    is by definition the one nobody runs day to day.
    """
    root = ET.parse(path).getroot()
    standalone = root.findall("node")
    composed = [n for load in root.iter("load_composable_node") for n in load]
    composed = [n for n in composed if n.tag == "composable_node"]
    if not standalone or not composed:
        pytest.skip(f"{path.name} does not offer both branches")

    for node in standalone:
        twins = [c for c in composed if c.get("name") == node.get("name")]
        if not twins:
            continue
        for twin in twins:
            missing = _interface(node) - _interface(twin)
            extra = _interface(twin) - _interface(node)
            assert not missing and not extra, (
                f"{path.name}: the standalone and composed spellings of "
                f"'{node.get('name')}' have drifted.\n"
                f"  only standalone: {sorted(missing)}\n"
                f"  only composed:   {sorted(extra)}"
            )


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=_ids(LAUNCH_FILES))
def test_every_substituted_argument_is_declared(path):
    """
    Every ``$(var x)`` in the file has a matching ``<arg name="x">``.

    These are leaf launch files: an including file passes values in, but the
    argument still has to be declared here. An undeclared one raises only when
    the branch containing it is taken, which for the composed branch means the
    day someone first composes.
    """
    source = path.read_text()
    declared = {a.get("name") for a in ET.parse(path).getroot().iter("arg")}
    used = set(re.findall(r"\$\(var ([A-Za-z0-9_]+)\)", source))
    undeclared = sorted(used - declared)
    assert not undeclared, f"{path.name}: substituted but never declared: {undeclared}"
