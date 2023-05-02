import pkg_resources
import yaml
import os
import subprocess
import rospkg
import io

class ResourceFinder:
    def __init__(self, config_file='config_file.yaml'):
        self.cfg = self.get_yaml(config_file)
        # init RosPack
        rospack = rospkg.RosPack()

    def nested_access(self, keylist):
        val = dict(self.cfg)
        for key in keylist:
            val = val[key]
        return val

    def get_expanded_path(self, relative_path):
        expanded_dir = os.path.expanduser(self.nested_access(relative_path))
        expanded_dir = os.path.expandvars(expanded_dir)
        if '$' in expanded_dir:
            expanded_dir = expanded_dir[1:]
            if (expanded_dir.startswith('{',) and expanded_dir.endswith('}')) or (expanded_dir.startswith('(',) and expanded_dir.endswith(')')):
                expanded_dir = expanded_dir[1:-1]
            expanded_dir = subprocess.check_output(expanded_dir.split()).decode('utf-8').rstrip()

        return expanded_dir

    def find_resource_path(self, resource_name, relative_path=None):
        if relative_path:
            if 'external_resources' in relative_path:
                resource_path = self.find_external_resource_path(resource_name, relative_path)
            else:
                resource_path = '/'.join((self.nested_access(relative_path), resource_name))
        else:
            resource_path = resource_name
        return resource_path

    def find_external_resource_path(self, resource_name, relative_path=None):
        resource_path = '/'.join((self.get_expanded_path(relative_path), resource_name))
        return resource_path
    
    def find_resource_absolute_path(self, resource_name, relative_path=None):
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
        if relative_path:
            if 'external_resources' in relative_path:
                return True
        return False

    def get_string(self, resource_name, relative_path=None):
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
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.is_resource_external(relative_path):
            resource_stream = open(resource_path, 'r', encoding="utf-8")
        else:
            resource_stream = pkg_resources.resource_stream(resource_package, resource_path)
        return resource_stream

    def get_filename(self, resource_name, relative_path=None):
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        if self.is_resource_external(relative_path):
            resource_filename = resource_path
        else:
            resource_filename = pkg_resources.resource_filename(resource_package, resource_path)
        return resource_filename

    def get_yaml(self, resource_name, relative_path=None):
        with self.get_stream(resource_name, relative_path) as stream:
            try:
                yaml_dict = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                yaml_dict = {}
                print(exc)
        return yaml_dict
