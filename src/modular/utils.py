import pkg_resources
import yaml

class ResourceFinder:
    def __init__(self, config_file='config_file.yaml'):
        self.cfg = self.get_yaml(config_file)

    def find_resource_path(self, resource_name, relative_path=None):
        if relative_path:
            resource_path = '/'.join((self.cfg[relative_path], resource_name))
        else:
            resource_path = resource_name
        return resource_path

    def get_string(self, resource_name, relative_path=None):
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        resource_string = pkg_resources.resource_string(resource_package, resource_path)
        return resource_string

    def get_stream(self, resource_name, relative_path=None):
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        resource_stream = pkg_resources.resource_stream(resource_package, resource_path)
        return resource_stream

    def get_filename(self, resource_name, relative_path=None):
        resource_package = __name__
        resource_path = self.find_resource_path(resource_name, relative_path)
        resource_filename = pkg_resources.resource_filename(resource_package, resource_path)
        return resource_filename

    def get_yaml(self, resource_name, relative_path=None):
        with self.get_stream(resource_name, relative_path) as stream:
            try:
                yaml_dict = yaml.load(stream)
            except yaml.YAMLError as exc:
                yaml_dict = {}
                print(exc)
        return yaml_dict
