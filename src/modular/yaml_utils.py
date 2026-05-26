"""
YAML loading/dumping utilities, the SlaveDescMode enumeration, and shared
xacro namespace constants.

Extracted from URDF_writer.py to keep it focused on robot-model building.
"""
import yaml
from collections import OrderedDict
from enum import Enum


class MyDumper(yaml.Dumper):

    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, False)


# noinspection PyPep8Naming
def ordered_load(stream, Loader=yaml.SafeLoader, object_pairs_hook=OrderedDict):
    class OrderedLoader(Loader):
        pass

    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))

    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)

    return yaml.load(stream, OrderedLoader)


# noinspection PyPep8Naming
def ordered_dump(data, stream=None, Dumper=MyDumper, **kwds):
    class OrderedDumper(Dumper):
        pass

    def _dict_representer(dumper, data):
        return dumper.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
            data.items())

    OrderedDumper.add_representer(OrderedDict, _dict_representer)

    # to avoid printing tags. TODO: Find a better way to do this. This changes the global yaml emitter
    def noop(self, *args, **kw):
        pass

    yaml.emitter.Emitter.process_tag = noop

    return yaml.dump(data, stream, OrderedDumper, **kwds)


class SlaveDescMode(str, Enum):
    """Slave description mode"""
    USE_POSITIONS = 'use_pos'
    USE_IDS = 'use_ids'


# ---------------------------------------------------------------------------
# Shared xacro namespace constants
# ---------------------------------------------------------------------------
NS_XACRO = "http://www.ros.org/wiki/xacro"
ns = {"xacro": NS_XACRO}
