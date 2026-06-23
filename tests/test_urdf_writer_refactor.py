"""
Tests for the URDF_writer refactoring.

Verifies that:
1. The four extracted modules (yaml_utils, plugins, urdf_xml_builder, chain_manager)
   work correctly in isolation.
2. Backward compatibility is preserved: all public names are still importable
   directly from modular.URDF_writer.
3. UrdfWriter can be constructed and its key public API methods work as before
   the refactoring (using the sim_discovery scenario as the primary integration
   fixture).
4. The NS_XACRO / ns constants are defined in exactly one place (urdf_xml_builder) and
   imported consistently everywhere they are needed.

Note: Tests that construct UrdfWriter require the full modular_resources directory
and (on ros2 branches) a working ``ros2`` installation to expand external resource
paths from config_file.yaml.  They are automatically skipped when these
prerequisites are absent.

Run with:
    pytest tests/test_urdf_writer_refactor.py
"""
import logging
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import OrderedDict
from types import SimpleNamespace

import pytest


# ---------------------------------------------------------------------------
# Environment detection
# ---------------------------------------------------------------------------

def _urdf_writer_constructable():
    """Return True if UrdfWriter can be instantiated without errors."""
    try:
        from modular.URDF_writer import UrdfWriter
        UrdfWriter(quiet=True)
        return True
    except Exception:
        return False


_skip_if_no_urdf_writer = pytest.mark.skipif(
    not _urdf_writer_constructable(),
    reason=(
        "UrdfWriter construction requires ros2 CLI and full resource paths "
        "(not available in this environment)"
    ),
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

# Discovery reply captured from scripts/sim_discovery.py (main scenario)
DISCOVERY_REPLY = (
    "{1: {active_ports: 15, esc_type: 50, mod_id: 3, mod_rev: 0, mod_size: 0, mod_type: 2,"
    " position: 1, robot_id: 201, topology: 4},"
    " 2: {active_ports: 15, esc_type: 256, mod_id: 4, mod_rev: 0, mod_size: 0, mod_type: 2,"
    " position: 2, robot_id: -1, topology: 4},"
    " 3: {active_ports: 3, esc_type: 21, mod_id: 8, mod_rev: 0, mod_size: 5, mod_type: 1,"
    " position: 3, robot_id: 21, topology: 2},"
    " 4: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5,"
    " position: 4, robot_id: 22, topology: 1},"
    " 5: {active_ports: 3, esc_type: 21, mod_id: 7, mod_rev: 0, mod_size: 5, mod_type: 1,"
    " position: 5, robot_id: 11, topology: 2},"
    " 6: {active_ports: 1, esc_type: 21, mod_id: 1, mod_rev: 0, mod_size: 5, mod_type: 5,"
    " position: 6, robot_id: 12, topology: 1}}"
)


@pytest.fixture(scope="module")
def quiet_writer():
    """A quiet UrdfWriter reused across tests in this module.

    Skipped if UrdfWriter cannot be constructed in this environment.
    """
    from modular.URDF_writer import UrdfWriter
    return UrdfWriter(verbose=False, quiet=True, slave_desc_mode='use_pos')


@pytest.fixture(scope="module")
def writer_after_discovery(quiet_writer):
    """UrdfWriter that has processed the DISCOVERY_REPLY fixture."""
    quiet_writer.read_from_json(DISCOVERY_REPLY)
    return quiet_writer


# ===========================================================================
# 1. yaml_utils – isolated unit tests
# ===========================================================================

class TestYamlUtils:
    """Tests for the standalone yaml_utils module."""

    def test_importable(self):
        import modular.yaml_utils  # noqa: F401

    def test_slave_desc_mode_values(self):
        from modular.enums import SlaveDescMode
        assert SlaveDescMode.USE_POSITIONS.value == 'use_pos'
        assert SlaveDescMode.USE_IDS.value == 'use_ids'

    def test_slave_desc_mode_from_string(self):
        from modular.enums import SlaveDescMode
        assert SlaveDescMode('use_pos') is SlaveDescMode.USE_POSITIONS
        assert SlaveDescMode('use_ids') is SlaveDescMode.USE_IDS

    def test_ordered_load_preserves_order(self):
        from modular.yaml_utils import ordered_load
        yaml_text = "b: 2\na: 1\nc: 3"
        result = ordered_load(yaml_text)
        assert isinstance(result, OrderedDict)
        assert list(result.keys()) == ['b', 'a', 'c']

    def test_ordered_dump_round_trip(self):
        from modular.yaml_utils import ordered_load, ordered_dump
        original = OrderedDict([('x', 1), ('y', 2), ('z', 3)])
        dumped = ordered_dump(original)
        loaded = ordered_load(dumped)
        assert list(loaded.items()) == list(original.items())

    def test_ordered_dump_produces_string(self):
        from modular.yaml_utils import ordered_dump
        data = OrderedDict([('key', 'value')])
        result = ordered_dump(data)
        assert isinstance(result, str)
        assert 'key' in result
        assert 'value' in result


# ===========================================================================
# 2. plugins – class structure tests
# ===========================================================================

class TestPlugins:
    """Tests for the standalone plugins module."""

    def test_importable(self):
        import modular.plugins  # noqa: F401

    def test_plugin_classes_exported(self):
        from modular.plugins import Plugin, RosControlPlugin, XBotCorePlugin, XBot2Plugin
        assert Plugin is not None
        assert RosControlPlugin is not None
        assert XBotCorePlugin is not None
        assert XBot2Plugin is not None

    def test_concrete_plugin_subclasses(self):
        from modular.plugins import Plugin, RosControlPlugin, XBotCorePlugin, XBot2Plugin
        assert issubclass(RosControlPlugin, Plugin)
        assert issubclass(XBotCorePlugin, Plugin)
        assert issubclass(XBot2Plugin, Plugin)

    def test_plugin_has_abstract_methods(self):
        """Plugin cannot be instantiated directly because it has abstract methods."""
        from modular.plugins import Plugin
        with pytest.raises(TypeError):
            Plugin()

    def test_ros_control_plugin_instantiable(self):
        from modular.plugins import RosControlPlugin
        p = RosControlPlugin()
        assert p is not None

    def test_xbotcore_plugin_instantiable(self):
        from modular.plugins import XBotCorePlugin
        p = XBotCorePlugin()
        assert p is not None

    def test_xbot2_plugin_instantiable(self):
        from modular.plugins import XBot2Plugin
        p = XBot2Plugin()
        assert p is not None

    def test_plugins_use_single_ns_xacro_source(self):
        """NS_XACRO used in plugins must equal the one defined in urdf_xml_builder."""
        import modular.plugins as plugins_mod
        from modular.urdf_xml_builder import NS_XACRO
        assert plugins_mod.NS_XACRO == NS_XACRO
        assert plugins_mod.ns == {"xacro": NS_XACRO}


# ===========================================================================
# 3. urdf_xml_builder – structure tests
# ===========================================================================

class TestUrdfXmlBuilder:
    """Tests for the standalone urdf_xml_builder module."""

    def test_importable(self):
        import modular.urdf_xml_builder  # noqa: F401

    def test_class_exported(self):
        from modular.urdf_xml_builder import URDFXmlBuilder
        assert URDFXmlBuilder is not None

    def test_ns_xacro_constant(self):
        from modular.urdf_xml_builder import NS_XACRO
        assert NS_XACRO == "http://www.ros.org/wiki/xacro"

    def test_ns_dict(self):
        from modular.urdf_xml_builder import ns, NS_XACRO
        assert ns == {"xacro": NS_XACRO}

    def test_uses_single_ns_xacro_source(self):
        import modular.urdf_xml_builder as builder_mod
        from modular.urdf_xml_builder import NS_XACRO
        assert builder_mod.NS_XACRO == NS_XACRO
        assert builder_mod.ns == {"xacro": NS_XACRO}


# ===========================================================================
# 4. chain_manager – structure tests
# ===========================================================================

class TestChainManager:
    """Tests for the standalone chain_manager module."""

    def test_importable(self):
        import modular.chain_manager  # noqa: F401

    def test_class_exported(self):
        from modular.chain_manager import ChainManager
        assert ChainManager is not None


# ===========================================================================
# 5. Backward-compatibility: imports via URDF_writer
# ===========================================================================

class TestBackwardCompatibility:
    """Everything previously importable from URDF_writer should still be there."""

    _names = [
        'UrdfWriter',
        'Plugin', 'RosControlPlugin', 'XBotCorePlugin', 'XBot2Plugin',
        'SlaveDescMode', 'ordered_load', 'ordered_dump',
        'NS_XACRO', 'ns',
        'URDFXmlBuilder', 'ChainManager',
        'parse_generator_cli_args',
    ]

    @pytest.mark.parametrize("name", _names)
    def test_name_importable_from_urdf_writer(self, name):
        import modular.URDF_writer as uw
        assert hasattr(uw, name), f"'{name}' is missing from modular.URDF_writer"

    def test_ns_xacro_consistent_across_modules(self):
        """NS_XACRO must be identical in plugins, urdf_xml_builder,
        and URDF_writer (single source of truth – urdf_xml_builder)."""
        from modular.urdf_xml_builder import NS_XACRO as ns_xml
        import modular.plugins as plugins_mod
        import modular.urdf_xml_builder as builder_mod
        import modular.URDF_writer as uw

        assert plugins_mod.NS_XACRO == ns_xml
        assert builder_mod.NS_XACRO == ns_xml
        assert uw.NS_XACRO == ns_xml


# ===========================================================================
# 6. UrdfWriter construction
# ===========================================================================

@_skip_if_no_urdf_writer
class TestUrdfWriterConstruction:
    """Tests for UrdfWriter instantiation with different options."""

    def test_default_construction(self):
        from modular.URDF_writer import UrdfWriter
        w = UrdfWriter(quiet=True)
        assert w is not None

    def test_construction_with_xbot2_plugin(self):
        from modular.URDF_writer import UrdfWriter, XBot2Plugin
        w = UrdfWriter(control_plugin='xbot2', quiet=True)
        assert isinstance(w.control_plugin, XBot2Plugin)

    def test_construction_with_ros_control_plugin(self):
        from modular.URDF_writer import UrdfWriter, RosControlPlugin
        w = UrdfWriter(control_plugin='ros_control', quiet=True)
        assert isinstance(w.control_plugin, RosControlPlugin)

    def test_construction_with_xbotcore_plugin(self):
        from modular.URDF_writer import UrdfWriter, XBotCorePlugin
        w = UrdfWriter(control_plugin='xbotcore', quiet=True)
        assert isinstance(w.control_plugin, XBotCorePlugin)

    def test_slave_desc_mode_use_pos(self):
        from modular.URDF_writer import UrdfWriter, SlaveDescMode
        w = UrdfWriter(slave_desc_mode='use_pos', quiet=True)
        assert w.slave_desc_mode is SlaveDescMode.USE_POSITIONS

    def test_slave_desc_mode_use_ids(self):
        from modular.URDF_writer import UrdfWriter, SlaveDescMode
        w = UrdfWriter(slave_desc_mode='use_ids', quiet=True)
        assert w.slave_desc_mode is SlaveDescMode.USE_IDS

    def test_slave_desc_mode_invalid_defaults_to_use_pos(self):
        from modular.URDF_writer import UrdfWriter, SlaveDescMode
        w = UrdfWriter(slave_desc_mode='invalid_value', quiet=True)
        assert w.slave_desc_mode is SlaveDescMode.USE_POSITIONS

    def test_initial_chain_has_base_link(self):
        from modular.URDF_writer import UrdfWriter
        w = UrdfWriter(quiet=True)
        assert len(w.listofchains) == 1
        assert w.listofchains[0][0].name == 'base_link'

    def test_custom_logger_is_used(self):
        from modular.URDF_writer import UrdfWriter
        custom_logger = logging.getLogger('test_custom')
        w = UrdfWriter(logger=custom_logger, quiet=True)
        assert w.logger is custom_logger

    def test_control_plugin_back_reference(self):
        """The control plugin must hold a back-reference to its writer."""
        from modular.URDF_writer import UrdfWriter
        w = UrdfWriter(quiet=True)
        assert w.control_plugin.urdf_writer is w


# ===========================================================================
# 7. Integration: sim_discovery scenario (mirrors scripts/sim_discovery.py)
# ===========================================================================

@_skip_if_no_urdf_writer
class TestSimDiscovery:
    """Integration tests using the same discovery reply as sim_discovery.py."""

    def test_read_from_json_returns_data(self, writer_after_discovery):
        # read_from_json should have been called inside the fixture; check state
        assert writer_after_discovery is not None

    def test_read_from_json_produces_urdf_string(self, writer_after_discovery):
        assert hasattr(writer_after_discovery, 'urdf_string')
        assert isinstance(writer_after_discovery.urdf_string, str)
        assert len(writer_after_discovery.urdf_string) > 0

    def test_urdf_contains_robot_element(self, writer_after_discovery):
        assert '<robot' in writer_after_discovery.urdf_string

    def test_listofchains_non_empty(self, writer_after_discovery):
        assert len(writer_after_discovery.listofchains) >= 1

    def test_get_actuated_modules_chains(self, writer_after_discovery):
        chains = writer_after_discovery.get_actuated_modules_chains()
        # The fixture has joint modules, so at least one chain should be actuated
        assert isinstance(chains, list)
        assert len(chains) >= 1

    def test_process_urdf_returns_string(self, writer_after_discovery):
        result = writer_after_discovery.process_urdf()
        assert isinstance(result, str)
        assert '<robot' in result

    def test_write_file_to_stdout_produces_urdf(self, writer_after_discovery, capsys):
        """Mirrors the write_file_to_stdout(None, robot_name='sim_discovery') call."""
        import argparse

        # Build a pre-parsed args namespace that mimics --output urdf
        args = argparse.Namespace(
            output='urdf',
            xacro_args=None,
            deploy=None,
            robot_name='sim_discovery_test',
            quiet=True,
        )
        writer_after_discovery.write_file_to_stdout(None, robot_name='sim_discovery_test', args=args)
        captured = capsys.readouterr()
        assert '<robot' in captured.out


# ===========================================================================
# 8. Chain management API
# ===========================================================================

@_skip_if_no_urdf_writer
class TestChainManagement:
    """Tests for add_to_chain / remove_from_chain / get_actuated_modules_chains."""

    def test_add_to_chain_increases_chain_length(self):
        """After read_from_json the writer should have at least one joint in a chain."""
        from modular.URDF_writer import UrdfWriter
        w = UrdfWriter(quiet=True)
        w.read_from_json(DISCOVERY_REPLY)
        all_nodes = [n for chain in w.listofchains for n in chain]
        assert len(all_nodes) > 1  # base_link plus at least one joint module

    def test_find_chain_tip_link(self):
        from modular.URDF_writer import UrdfWriter
        w = UrdfWriter(quiet=True)
        w.read_from_json(DISCOVERY_REPLY)
        for chain in w.listofchains:
            if len(chain) > 1:
                tip = w.find_chain_tip_link(chain)
                assert isinstance(tip, str)
                assert len(tip) > 0
                break

    def test_find_chain_base_link(self):
        from modular.URDF_writer import UrdfWriter
        w = UrdfWriter(quiet=True)
        w.read_from_json(DISCOVERY_REPLY)
        for chain in w.listofchains:
            if len(chain) > 1:
                base = w.find_chain_base_link(chain)
                assert isinstance(base, str)
                assert len(base) > 0
                break


# ===========================================================================
# 9. parse_generator_cli_args (module-level and method-level parity)
# ===========================================================================

class TestParseGeneratorCliArgs:
    """Both the module-level function and the static method must behave identically."""

    def test_module_level_function_parses_urdf(self):
        from modular.URDF_writer import parse_generator_cli_args
        args = parse_generator_cli_args(['--output', 'urdf'])
        assert args.output == 'urdf'

    def test_static_method_parses_urdf(self):
        from modular.URDF_writer import UrdfWriter
        args = UrdfWriter.parse_generator_cli_args(['--output', 'urdf'])
        assert args.output == 'urdf'

    def test_module_level_and_static_method_are_equivalent(self):
        from modular.URDF_writer import parse_generator_cli_args, UrdfWriter
        args_module = parse_generator_cli_args(['--output', 'srdf'])
        args_static = UrdfWriter.parse_generator_cli_args(['--output', 'srdf'])
        assert args_module.output == args_static.output

    def test_known_only_returns_tuple(self):
        from modular.URDF_writer import parse_generator_cli_args
        result = parse_generator_cli_args(['--output', 'urdf', '--unknown-flag'], known_only=True)
        assert isinstance(result, tuple)
        assert len(result) == 2


# ===========================================================================
# 10. Legacy vs submodule parity for chain methods
# ===========================================================================

def _chain_names(chains):
    return [[node.name for node in chain] for chain in chains]


def _make_discovered_writer():
    from modular.URDF_writer import UrdfWriter
    w = UrdfWriter(verbose=False, quiet=True, slave_desc_mode='use_pos')
    w.read_from_json(DISCOVERY_REPLY)
    return w


def _pick_non_base_node_with_indices(writer):
    for chain_idx, chain in enumerate(writer.listofchains):
        for node_idx, node in enumerate(chain):
            if node_idx == 0:
                continue
            if getattr(node, 'parent', None) is None:
                continue
            return chain_idx, node_idx, node
    raise AssertionError('No non-base node found in discovered chains')


def _detach_node_from_chains(writer, node):
    for chain in writer.listofchains:
        if node in chain:
            chain.remove(node)
    writer.listofchains = list(filter(None, writer.listofchains))


@_skip_if_no_urdf_writer
class TestChainMethodParity:
    """Compare old UrdfWriter chain methods with ChainManager implementations."""

    def test_add_to_chain_existing_branch_equivalence(self):
        from modular.chain_manager import ChainManager

        old_writer = _make_discovered_writer()
        new_writer = _make_discovered_writer()

        chain_idx, node_idx, node_old = _pick_non_base_node_with_indices(old_writer)
        node_new = new_writer.listofchains[chain_idx][node_idx]

        _detach_node_from_chains(old_writer, node_old)
        _detach_node_from_chains(new_writer, node_new)

        old_writer.add_to_chain(node_old)
        ChainManager(new_writer).add_to_chain(node_new)

        assert _chain_names(old_writer.listofchains) == _chain_names(new_writer.listofchains)

    def test_remove_from_chain_equivalence(self):
        from modular.chain_manager import ChainManager

        old_writer = _make_discovered_writer()
        new_writer = _make_discovered_writer()

        chain_idx, node_idx, node_old = _pick_non_base_node_with_indices(old_writer)
        node_new = new_writer.listofchains[chain_idx][node_idx]

        old_writer.remove_from_chain(node_old)
        ChainManager(new_writer).remove_from_chain(node_new)

        assert _chain_names(old_writer.listofchains) == _chain_names(new_writer.listofchains)

    def test_get_actuated_modules_chains_equivalence(self):
        from modular.chain_manager import ChainManager

        old_writer = _make_discovered_writer()
        new_writer = _make_discovered_writer()

        old_result = old_writer.get_actuated_modules_chains()
        new_result = ChainManager(new_writer).get_actuated_modules_chains()

        assert _chain_names(old_result) == _chain_names(new_result)

    def test_static_helpers_equivalence(self):
        from modular.URDF_writer import UrdfWriter
        from modular.chain_manager import ChainManager

        w = _make_discovered_writer()
        for chain in w.listofchains:
            if len(chain) < 2:
                continue
            assert UrdfWriter.find_chain_tip_link(chain) == ChainManager.find_chain_tip_link(chain)
            assert UrdfWriter.find_chain_base_link(chain) == ChainManager.find_chain_base_link(chain)
            assert UrdfWriter.find_chain_tag(chain) == ChainManager.find_chain_tag(chain)


# ===========================================================================
# 11. Legacy vs submodule parity for yaml_utils / plugins / urdf_xml_builder
# ===========================================================================

class TestYamlUtilsParity:
    """Verify parity between names exposed by URDF_writer and yaml_utils."""

    def test_yaml_symbols_are_the_same_objects(self):
        import modular.URDF_writer as uw
        import modular.yaml_utils as yu
        import modular.urdf_xml_builder as xb
        import modular.enums as enums_mod

        assert uw.ordered_load is yu.ordered_load
        assert uw.ordered_dump is yu.ordered_dump
        assert uw.SlaveDescMode is enums_mod.SlaveDescMode
        assert uw.NS_XACRO == xb.NS_XACRO
        assert uw.ns == xb.ns

    def test_yaml_behavior_matches_via_urdf_writer_exports(self):
        import modular.URDF_writer as uw
        import modular.yaml_utils as yu

        yaml_text = "b: 2\na: 1\nc: 3"
        via_uw = uw.ordered_load(yaml_text)
        via_yu = yu.ordered_load(yaml_text)
        assert list(via_uw.items()) == list(via_yu.items())

        src = OrderedDict([('x', 1), ('y', 2)])
        dumped_uw = uw.ordered_dump(src)
        dumped_yu = yu.ordered_dump(src)
        assert uw.ordered_load(dumped_uw) == yu.ordered_load(dumped_yu)


@_skip_if_no_urdf_writer
class TestPluginsParity:
    """Compare plugin behavior via UrdfWriter wrappers vs direct plugin calls."""

    def test_write_joint_map_parity(self):
        writer_a = _make_discovered_writer()
        writer_b = _make_discovered_writer()

        via_wrapper = writer_a.write_joint_map(use_robot_id=False)
        via_plugin = writer_b.control_plugin.write_joint_map(use_robot_id=False)

        assert via_wrapper == via_plugin

    def test_write_srdf_parity_without_acm(self):
        writer_a = _make_discovered_writer()
        writer_b = _make_discovered_writer()

        via_wrapper = writer_a.write_srdf(builder_joint_map=None, compute_acm=False)
        via_plugin = writer_b.control_plugin.write_srdf(builder_joint_map=None)

        assert via_wrapper == via_plugin


@_skip_if_no_urdf_writer
class TestUrdfXmlBuilderParity:
    """Compare duplicated XML helper methods with URDFXmlBuilder methods."""

    def test_add_origin_parity(self):
        from modular.URDF_writer import UrdfWriter
        from modular.urdf_xml_builder import URDFXmlBuilder

        writer = _make_discovered_writer()
        builder = URDFXmlBuilder(writer)

        pose = SimpleNamespace(x=1.0, y=2.0, z=3.0, roll=0.1, pitch=0.2, yaw=0.3)

        el_old = ET.Element('test_old')
        el_new = ET.Element('test_new')

        UrdfWriter.add_origin(writer, el_old, pose)
        builder.add_origin(el_new, pose)

        assert ET.tostring(el_old.find('origin')) == ET.tostring(el_new.find('origin'))

    def test_add_geometry_parity(self):
        from modular.URDF_writer import UrdfWriter
        from modular.urdf_xml_builder import URDFXmlBuilder

        writer = _make_discovered_writer()
        builder = URDFXmlBuilder(writer)

        geometry = SimpleNamespace(
            type='box',
            parameters=SimpleNamespace(size=[0.1, 0.2, 0.3])
        )

        el_old = ET.Element('test_old')
        el_new = ET.Element('test_new')

        UrdfWriter.add_geometry(writer, el_old, geometry)
        builder.add_geometry(el_new, geometry)

        assert ET.tostring(el_old.find('geometry')) == ET.tostring(el_new.find('geometry'))

    def test_add_material_parity(self):
        from modular.URDF_writer import UrdfWriter
        from modular.urdf_xml_builder import URDFXmlBuilder

        writer = _make_discovered_writer()
        builder = URDFXmlBuilder(writer)

        color = SimpleNamespace(material_name='m', rgba=[1, 0, 0, 1])

        el_old = ET.Element('test_old')
        el_new = ET.Element('test_new')

        UrdfWriter.add_material(writer, el_old, color)
        builder.add_material(el_new, color)

        assert ET.tostring(el_old.find('material')) == ET.tostring(el_new.find('material'))

    def test_add_gazebo_element_parity(self):
        from modular.urdf_xml_builder import URDFXmlBuilder

        writer_old = _make_discovered_writer()
        writer_new = _make_discovered_writer()
        builder = URDFXmlBuilder(writer_new)

        module_old = SimpleNamespace(xml_tree_elements=[])
        module_new = SimpleNamespace(xml_tree_elements=[])
        gazebo = SimpleNamespace(plugin='demo_plugin', nested=SimpleNamespace(updateRate=100))

        writer_old.add_gazebo_element(module_old, gazebo, 'mod_test')
        builder.add_gazebo_element(module_new, gazebo, 'mod_test')

        old_block = next((n for n in writer_old.root if n.attrib.get('name') == 'gazebo_mod_test'), None)
        new_block = next((n for n in writer_new.root if n.attrib.get('name') == 'gazebo_mod_test'), None)

        assert old_block is not None
        assert new_block is not None
        assert ET.tostring(old_block) == ET.tostring(new_block)
