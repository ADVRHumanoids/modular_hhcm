import yaml
import sys

class DictAsMember(dict):
    def __getattr__(self, name):
        value = self[name]
        if isinstance(value, dict):
            value = DictAsMember(value)
        return value

def read_yaml(filename):
    with open(filename, 'r') as stream:
        try:
            data=yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

        #subdict=dict(proximal='proximal', distal='distal', joint='joint'))
    # my_dict['kinematics'] = {'sub_property': 1}

    return DictAsMember(data)

def main():
    my_dict = read_yaml(sys.argv[1])
    print(my_dict.kinematics)

if __name__ == '__main__':
  main()