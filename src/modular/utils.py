import pkg_resources
import yaml
import os
import subprocess
import io
import json
import re
from numbers import Number
from collections.abc import Sequence

class ResourceFinder:
    """Class to find resources in the package"""
    def __init__(self, config_file='config_file.yaml'):
        self.cfg = self.get_yaml(config_file)

        self.resources_paths = self.get_all_resources_paths()

    def get_all_resources_paths(self):
        resources_paths = []
        # Add internal resources path. By default the path to the resources is at cfg['resources_path']
        resources_paths.append(['resources_path'])
        # Add external resources paths. The paths are mapped at cfg['external_resources']
        for path in self.nested_access(['external_resources']):
            resources_paths.append(['external_resources', path])
        return resources_paths

    def nested_access(self, keylist):
        """Access a nested dictionary"""
        val = dict(self.cfg)
        for key in keylist:
            val = val[key]
        return val

    def get_expanded_path(self, relative_path):
        # Expand the home directory
        expanded_path = os.path.expanduser(self.nested_access(relative_path))
        # Expand environment variables
        expanded_path = os.path.expandvars(expanded_path)
        def path_substitution(match):
            # Get the command to execute: $(cmd)
            cmd = re.search(r"\$\(([^\)]+)\)", match.group()).group(1)
            # Return the output of the command            
            cmd_output = subprocess.check_output(cmd.split(), stderr=subprocess.DEVNULL).decode('utf-8').rstrip()
            return cmd_output
        try:
            # The dollar sign is used to execute a command and get the output. What is inside the parenthesis is substituted with the output of the command: $(cmd) -> output of cmd
            expanded_path = re.sub(r"\$\(([^\)]+)\)", path_substitution, expanded_path)
        except (subprocess.CalledProcessError, TypeError):
            msg = 'Executing ' + expanded_path + ' resulted in an error. Path substitution cannot be completed. Are the required environment variables set?'
            raise RuntimeError(msg)
            
        return expanded_path

    def find_resource_path(self, resource_name, relative_path=None):
        """Return a relative filesystem path for specified resource"""
        if relative_path:
            if 'external_resources' in relative_path:
                resource_path = self.find_external_resource_path(resource_name, relative_path)
            else:
                resource_path = '/'.join((self.nested_access(relative_path), resource_name))
        else:
            resource_path = resource_name
        return resource_path

    def find_external_resource_path(self, resource_name, relative_path=None):
        """Return a filesystem path for specified external resource"""
        resource_path = '/'.join((self.get_expanded_path(relative_path), resource_name))
        return resource_path
    
    def find_resource_absolute_path(self, resource_name, relative_path=None):
        """Return an absolute filesystem path for specified resource"""
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        if relative_path:
            if 'external_resources' in relative_path:
                resource_path = self.find_external_resource_path(resource_name, relative_path)
            else:
                resource_path = '/'.join((curr_dir, self.nested_access(relative_path), resource_name))
        else:
            resource_path = '/'.join((curr_dir, resource_name))
        return resource_path

    @staticmethod
    def is_resource_external(relative_path=None):
        """Does the relative path means this is an external resource?"""
        if relative_path:
            if 'external_resources' in relative_path:
                return True
        return False

    def get_string(self, resource_name, relative_path=None):
        """Return specified resource as a string"""
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.is_resource_external(relative_path):
            resource_string = ''
            with open(resource_path, 'r', encoding="utf-8") as file:
                resource_string = file.read()
        else:
            resource_string = pkg_resources.resource_string(resource_package, resource_path)
        return resource_string

    def get_stream(self, resource_name, relative_path=None):
        """Return a readable file-like object for specified resource"""
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.is_resource_external(relative_path):
            resource_stream = open(resource_path, 'r', encoding="utf-8")
        else:
            resource_stream = pkg_resources.resource_stream(resource_package, resource_path)
        return resource_stream

    def get_filename(self, resource_name, relative_path=None):
        """Return a true filesystem path for specified resource"""
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.is_resource_external(relative_path):
            resource_filename = resource_path
        else:
            resource_filename = pkg_resources.resource_filename(resource_package, resource_path)
        return resource_filename
    
    def get_listdir(self, resource_name, relative_path=None):
        """List the contents of the named resource directory"""
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.resource_exists(resource_name, relative_path):
            if self.is_resource_external(relative_path):
                resource_listdir = os.listdir(resource_path)
            else:
                resource_listdir = pkg_resources.resource_listdir(resource_package, resource_path)
        else:
            resource_listdir = []
        return resource_listdir
    
    def resource_exists(self, resource_name, relative_path=None):
        """Does the named resource exist?"""
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.is_resource_external(relative_path):
            resource_exists = os.path.exists(resource_path) # TODO: check if correct
        else:
            resource_exists = pkg_resources.resource_exists(resource_package, resource_path)
        return resource_exists
    
    def resource_isdir(self, resource_name, relative_path=None):
        """Is the named resource a directory?"""
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.is_resource_external(relative_path):
            resource_isdir = os.path.isdir(resource_path)
        else:
            resource_isdir = pkg_resources.resource_isdir(resource_package, resource_path)
        return resource_isdir

    def get_yaml(self, resource_name, relative_path=None):
        """Load the yaml file specified resource and return it as a dict"""
        with self.get_stream(resource_name, relative_path) as stream:
            try:
                yaml_dict = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                yaml_dict = {}
                print(exc)
        return yaml_dict
    
    def get_json(self, resource_name, relative_path=None):
        """Load the json file specified resource and return it as a dict"""
        with self.get_stream(resource_name, relative_path) as stream:
            try:
                json_dict = json.load(stream)
            except json.JSONDecodeError as exc:
                json_dict = {}
                print(exc)
        return json_dict

class ModularResourcesManager:
    def __init__(self, resource_finder):
        self.resource_finder = resource_finder

        self.default_offset_values = {"x":0.0, "y":0.0, "z": 0.0, "roll":0.0, "pitch":0.0, "yaw":0.0}
        # dictionary with the keys being the module name and the values being a dictionary specifing
        # the default values for the keys x, y, z, roll, pitch, yaw
        self.default_offsets_dict = {}
        # dictionary with the keys being the module name and the values being a dictionary specifing
        # the allowed values for the keys x, y, z, roll, pitch, yaw (or a subset of them)
        self.allowed_offsets_dict = {}

        self.available_modules_dict = {}
        self.available_modules_headers = []
        self.init_available_modules()

        self.available_families = []
        self.init_available_families()

        self.available_addons_dict = {}
        self.available_addons_headers = []
        self.init_available_addons()

    def expand_listdir(self, starting_path, res_path):
        """Expand listdir to include subdirectories recursively"""
        listdir = self.resource_finder.get_listdir(starting_path, res_path)
        list_to_remove = []
        list_to_add = []
        for el in listdir:
            if self.resource_finder.resource_isdir(starting_path + '/' + el, res_path):
                new_listdir = self.expand_listdir(starting_path + '/'+ el, res_path)
                new_listdir = [el + '/' + new_el for new_el in new_listdir]
                list_to_remove.append(el)
                list_to_add += new_listdir
        for el in list_to_remove:
            listdir.remove(el)
        listdir += list_to_add
        return listdir

    def set_default_offset_values(self, name, offset_dict):
        """
        Set the default offset values for the modules and addons
        The offset_dict is a dictionary with the keys x, y, z, roll, pitch, yaw (or a subset of them) and the values can be numbers or sequences.

        - If the value a number, the user can set any value for the offset (default is the dictionary entry value).
        - If the value is an empty list, the user can set any value for the offset (default is 0.0).
        - If the value is a sequence of allowed elements. The first element of the sequence is the default value.
          This elements can either be numbers or a dictionaries with the keys 'label' and 'value' to show a label in the GUI and its numerical value respectively.

        example (yaml format):

        offsets:
            x: 0.1
            y: []
            yaw:[0, {label: "-π/2", value: -1.5707963267948966}]
        """
        if not isinstance(offset_dict, dict):
            return

        self.default_offsets_dict[name] = self.default_offset_values.copy()
        self.allowed_offsets_dict[name] = {}

        for key in offset_dict:
            # If the value is is a number, set the default value
            if isinstance(offset_dict[key], Number):
                self.default_offset_values[name][key]=float(offset_dict[key])

            # if the value is a sequence, use the first element to set the default value
            elif isinstance(offset_dict[key], Sequence) and len(offset_dict[key]) > 0:
                # if the first element is a number, set it as the default value
                if isinstance(offset_dict[key][0], Number):
                    self.default_offset_values[name][key]=float(offset_dict[key][0])

                # if the first element is a dictionary, set the default value to the value of the key 'value'
                elif isinstance(offset_dict[key], dict) and 'value' in offset_dict[key][0]:
                    self.default_offset_values[name][key]=float(offset_dict[key][0]["value"])

                # store the allowed values in a separate dictionary
                self.allowed_offsets_dict[name][key]=[]
                for subkey in offset_dict[key]:
                    if isinstance(offset_dict[key][subkey], Number):
                        self.allowed_offsets_dict[name][key].append(float(offset_dict[key][subkey]))
                    if isinstance(subkey, dict) and 'value' in offset_dict[key][subkey]:
                        self.allowed_offsets_dict[name][key].append(float(offset_dict[key][subkey]['value']))

    def get_default_offset_values(self, name):
        """
        Get the default offset values for the modules and addons
        return a dictionary with the keys x, y, z, roll, pitch, yaw and float values.
        """
        if name in self.default_offsets_dict:
            return self.default_offsets_dict[name]
        return self.default_offset_values

    def get_allowed_offset_values(self, name):
        """
        Get the allowed offset values for the modules and addons
        return a dictionary with the keys x, y, z, roll, pitch, yaw (or a subset of them) and float values.
        """
        if name in self.allowed_offsets_dict:
            return self.allowed_offsets_dict[name]
        return {}


    def init_available_modules(self):
        for res_path in self.resource_finder.resources_paths:
            resource_names_list = ['yaml/' + el for el in self.expand_listdir('yaml', res_path)]
            resource_names_list += ['json/' + el for el in self.expand_listdir('json', res_path)]
            for res_name in resource_names_list:
                if res_name.endswith('.json'):
                    res_dict = self.resource_finder.get_json(res_name, res_path)
                    self.available_modules_dict[res_dict['header']['name']] = res_dict
                    self.available_modules_headers.append(res_dict['header'])
                    if 'offset' in res_dict['header']:
                        self.set_default_offset_values(res_dict['header']['name'], res_dict['header']['offset'])
                elif res_name.endswith('.yaml'):
                    res_dict = self.resource_finder.get_yaml(res_name, res_path)
                    self.available_modules_dict[res_dict['header']['name']] = res_dict
                    self.available_modules_headers.append(res_dict['header'])
                    if 'offset' in res_dict['header']:
                        self.set_default_offset_values(res_dict['header']['name'], res_dict['header']['offset'])

    def init_available_families(self):
        for res_path in self.resource_finder.resources_paths:
            if self.resource_finder.resource_exists('families.yaml', res_path):
                self.available_families += (self.resource_finder.get_yaml('families.yaml', res_path)['families'])

    def init_available_addons(self):
        for res_path in self.resource_finder.resources_paths:
            resource_names_list = ['module_addons/yaml/' + el for el in self.expand_listdir('module_addons/yaml', res_path)]
            resource_names_list += ['module_addons/json/' + el for el in self.expand_listdir('module_addons/json', res_path)]
            for res_name in resource_names_list:
                if res_name.endswith('.json'):
                    res_dict = self.resource_finder.get_json(res_name, res_path)
                    self.available_addons_dict[res_dict['header']['name']] = res_dict
                    self.available_addons_headers.append(res_dict['header'])
                elif res_name.endswith('.yaml'):
                    res_dict = self.resource_finder.get_yaml(res_name, res_path)
                    self.available_addons_dict[res_dict['header']['name']] = res_dict
                    self.available_addons_headers.append(res_dict['header'])

    def get_available_modules(self):
        return self.available_modules_headers
    
    def get_available_modules_dict(self):
        return self.available_modules_dict
    
    def get_available_module_types(self):
        module_types = [el['type'] for el in self.available_modules_headers]
        return list(dict.fromkeys(module_types))
    
    def get_available_families(self):
        return self.available_families

    def get_available_family_ids(self):
        family_ids = [el['id'] for el in self.available_families]
        return list(dict.fromkeys(family_ids))
    
    def get_available_family_groups(self):
        family_groups = [el['group'] for el in self.available_families]
        return list(dict.fromkeys(family_groups))

    def get_available_addons(self):
        return self.available_addons_headers
    
    def get_available_addons_dict(self):
        return self.available_addons_dict
    
    def get_available_addon_types(self):
        addon_types = [el['type'] for el in self.available_addons_headers]
        return list(dict.fromkeys(addon_types))
    