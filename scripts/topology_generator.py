import zmq
import random
import json
from anytree import AnyNode, RenderTree
#from anytree.exporter import DictExporter
from dictexporter import DictExporter
import pprint

pp = pprint.PrettyPrinter(indent=4)

context = zmq.Context()

# socket = context.socket(zmq.PUB)
socket = context.socket(zmq.REP)

# bind to the subscriber. * is needed here instead of localhost otherwise error. TODO: understand why
socket.bind("tcp://*:5555")

data = {'modules': [{'type': 'master_cube', 'id': 1, 'connections': [0, 2, 3, 0]}, {'type': 'module_joint_B', 'id': 2, 'connections': [1, 0, 0, 0]}, {'type': 'module_joint_B', 'id': 3, 'connections': [1, 0, 0, 4]}, {'type': 'module_link_300mm_B', 'id': 4, 'connections': [3, 0, 0, 0]}]}

pp.pprint(data)

def dict_exporter(_root):
        myDict = {'modules': []}
        myDict = iter_child(_root, myDict)
        pp.pprint(myDict)
        return myDict

def iter_child(_module, _myDict):
        for _child in _module.children:
                print(_child)
                _childrens = [-1, -1, -1]
                if _child.type == 'master_cube':
                        i = 0
                        for c in _child.children :
                                _childrens[i] = c.id
                                i+=1
                elif len(_child.children)>0 :
                        c = _child.children[0]#.pop()
                        print(c)
                        _childrens[2] = c.id
                _connections = [_child.parent.id] + _childrens
                _subDict = {'type': _child.type, 'id': _child.id, 'connections': _connections}
                _myDict['modules'].append(_subDict)
                _myDict = iter_child(_child, _myDict)
        return _myDict

def generate_random_tree():
        n=1
        modules = []
        types = ['master_cube', 'module_joint_B', 'module_link_300mm_B']
        root = AnyNode(id=0)
        new_module = AnyNode(id=1, type='master_cube', parent=root)
        modules.append(new_module)
        while n < 10:
                n+=1
                moduleType = random.choice(types)
                moduleParent = select_parent(moduleType, modules)
                newModule = AnyNode(id=n, type=moduleType, parent=moduleParent)
                modules.append(newModule)
        print(RenderTree(root))
        return root

def select_parent(_moduleType, _modules):
        _moduleParent = random.choice(_modules)
        if _moduleParent.type == 'master_cube' :
                if len(_moduleParent.children) >= 3 :  
                        _moduleParent = select_parent(_moduleParent, _modules)
        elif _moduleParent.type == 'module_link_300mm_B' :
                if len(_moduleParent.children) >= 1 :
                        _moduleParent = select_parent(_moduleParent, _modules)
        elif _moduleParent.type == 'module_joint_B' :
                if (_moduleType == 'module_joint_B') or (len(_moduleParent.children) >= 1) :
                        _moduleParent = select_parent(_moduleParent, _modules)
        return _moduleParent        


def main():
    while True:
        #socket.send_json(data)
        
  
        # When using replier directly, instead of publisher
        message = socket.recv()

        root = generate_random_tree()
        # exporter = DictExporter()
        # oldDict = exporter.export(root)
        # print(oldDict)
        data = dict_exporter(root)
        # myDict = exporter.export(root)
        # for k,v in oldDict.items() :
        #         print(k,v) 
        #         if k == 'children':
        #                 connections = []
        #                 for child in v:
        #                         print(child['id'])
        #                         connections.append(child['id'])
        #                 del myDict[k]
        #                 myDict["connections"] = connections


        # process task
        print("Received request: %s" % message)
        
        # Reply with the data
        socket.send_json(data)

if __name__ == '__main__':
  main()