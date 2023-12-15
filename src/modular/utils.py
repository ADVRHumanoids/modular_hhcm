import pkg_resources
import yaml
import os
import subprocess
import io
import json

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
        """Return an expanded filesystem path for specified resource"""
        expanded_dir = os.path.expanduser(self.nested_access(relative_path))
        expanded_dir = os.path.expandvars(expanded_dir)
        if '$' in expanded_dir:
            expanded_dir = expanded_dir[1:]
            
            if (expanded_dir.startswith('{',) and expanded_dir.endswith('}')) or (expanded_dir.startswith('(',) and expanded_dir.endswith(')')):
                expanded_dir = expanded_dir[1:-1]

            try:
                expanded_dir = subprocess.check_output(expanded_dir.split(), stderr=subprocess.DEVNULL).decode('utf-8').rstrip()
            except subprocess.CalledProcessError:
                expanded_dir = ''

        return expanded_dir

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
    
    def init_available_modules(self):
        for res_path in self.resource_finder.resources_paths:
            resource_names_list = ['yaml/' + el for el in self.expand_listdir('yaml', res_path)]
            resource_names_list += ['json/' + el for el in self.expand_listdir('json', res_path)]
            for res_name in resource_names_list:
                if res_name.endswith('.json'):
                    res_dict = self.resource_finder.get_json(res_name, res_path)
                    self.available_modules_dict[res_dict['header']['name']] = res_dict
                    self.available_modules_headers.append(res_dict['header'])
                elif res_name.endswith('.yaml'):
                    res_dict = self.resource_finder.get_yaml(res_name, res_path)
                    self.available_modules_dict[res_dict['header']['name']] = res_dict
                    self.available_modules_headers.append(res_dict['header'])

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
    