import sys
import glob
import yaml
from collections import OrderedDict

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

def ordered_dump(data, stream=None, Dumper=yaml.SafeDumper, **kwds):
    class OrderedDumper(Dumper):
        pass

    def _dict_representer(dumper, data):
        return dumper.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
            data.items())

    OrderedDumper.add_representer(OrderedDict, _dict_representer)

    return yaml.dump(data, stream, OrderedDumper, **kwds)

def keys_exists(element, keys):
    '''
    Check if *keys (nested) exists in `element` (dict).
    '''
    if not isinstance(element, dict):
        raise AttributeError('keys_exists() expects dict as first argument.')
    if len(keys) == 0:
        raise AttributeError('keys_exists() expects at least two arguments, one given.')

    _element = element
    for key in keys:
        try:
            _element = _element[key]
        except KeyError:
            return False
    return True

def listRecursive (d, key, path = None):
    found_bool = False
    if not path: path = []
    for k in path: d = d[k]
    for k, v in d.items ():
        if isinstance (v, OrderedDict):
            for path, value, found in listRecursive (v, key, path + [k] ):
                yield path, value, found
        if k == key:
            found_bool = True
            yield path + [k], v, found_bool

def get_from_dict(element, keys):
    for k in keys:
        try:
            element = element[k]
        except KeyError:
            element = element.setdefault(k, {})
    return element

# Main function called when the script is not imported as a module but run directly
def main():

    if len(sys.argv) < 2:
        raise AttributeError('updateYAML expects as arguments the path+key+value of the nested dictionary to be updated')

    for file in glob.glob("*.yaml"):
        my_dict = {}
        with open(file, 'r+') as stream:
            try:
                #global my_dict
                my_dict = ordered_load(stream)
                #print(my_dict, 'pippo'   )
            except yaml.YAMLError as exc:
                print(exc)
                continue

        if keys_exists(my_dict, sys.argv[1:-1]) : #(sys.argv[1]), (sys.argv[2])):
                print((file, 'Nothing to update'))
        else:
            new_dict = get_from_dict(my_dict, sys.argv[1:-2])
            new_dict[sys.argv[-2]] = sys.argv[-1]
            #print(new_dict)
            print((file, 'missing field...updating YAML'))
            with open(file, 'w') as stream:
                ordered_dump(my_dict, stream=stream, Dumper=yaml.SafeDumper,  default_flow_style=False, line_break='\n\n', indent=4)

if __name__ == '__main__':
    main()


# dict_path = []
                # if len(sys.argv) > 2 :
                #     dict_path = sys.argv[2:]
                # #print(dict_path)
                # try:
                #     for path, value, found in listRecursive (my_dict, sys.argv[1], dict_path):
                #         print (path, value, found)
                #         print(found)
                #         if not found :
                #             print(my_dict)
                #             sub_dict = my_dict
                #             for k in dict_path : sub_dict = sub_dict[k]
                #             sub_dict[sys.argv[1]] = ''
                #             print(sub_dict)
                #         print(my_dict)
                # except KeyError:
                #     pass
