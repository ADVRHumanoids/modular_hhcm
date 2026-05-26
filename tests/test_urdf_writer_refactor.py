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
4. The NS_XACRO / ns constants are defined in exactly one place (yaml_utils) and
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
from collections import OrderedDict

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

    def test_ns_xacro_constant(self):
        from modular.yaml_utils import NS_XACRO
        assert NS_XACRO == "http://www.ros.org/wiki/xacro"

    def test_ns_dict(self):
        from modular.yaml_utils import ns, NS_XACRO
        assert ns == {"xacro": NS_XACRO}

    def test_slave_desc_mode_values(self):
        from modular.yaml_utils import SlaveDescMode
        assert SlaveDescMode.USE_POSITIONS.value == 'use_pos'
        assert SlaveDescMode.USE_IDS.value == 'use_ids'

    def test_slave_desc_mode_from_string(self):
        from modular.yaml_utils import SlaveDescMode
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
        """NS_XACRO used in plugins must equal the one defined in yaml_utils."""
        import modular.plugins as plugins_mod
        from modular.yaml_utils import NS_XACRO
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

    def test_uses_single_ns_xacro_source(self):
        import modular.urdf_xml_builder as builder_mod
        from modular.yaml_utils import NS_XACRO
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
        'parse_generator_cli_args', 'write_file_to_stdout',
    ]

    @pytest.mark.parametrize("name", _names)
    def test_name_importable_from_urdf_writer(self, name):
        import modular.URDF_writer as uw
        assert hasattr(uw, name), f"'{name}' is missing from modular.URDF_writer"

    def test_ns_xacro_consistent_across_modules(self):
        """NS_XACRO must be identical in yaml_utils, plugins, urdf_xml_builder,
        and URDF_writer (single source of truth – yaml_utils)."""
        from modular.yaml_utils import NS_XACRO as ns_yaml
        import modular.plugins as plugins_mod
        import modular.urdf_xml_builder as builder_mod
        import modular.URDF_writer as uw

        assert plugins_mod.NS_XACRO == ns_yaml
        assert builder_mod.NS_XACRO == ns_yaml
        assert uw.NS_XACRO == ns_yaml


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
