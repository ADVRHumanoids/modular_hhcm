import zmq
from random import randrange
import json

context = zmq.Context()
socket = context.socket(zmq.PUB)
# bind to the subscriber. * is needed here instead of localhost otherwise error. TODO: understand why
socket.bind("tcp://*:5556")

data = {'modules': [{'type': 'master_cube', 'id': 1, 'connections': [0, 2, 3, 0]}, {'type': 'module_joint_B', 'id': 2, 'connections': [1, 0, 0, 0]}, {'type': 'module_joint_B', 'id': 3, 'connections': [1, 0, 0, 4]}, {'type': 'module_link_300mm_B', 'id': 4, 'connections': [3, 0, 0, 0]}]}

def main():
    while True:

        socket.send_json(data)

if __name__ == '__main__':
  main()