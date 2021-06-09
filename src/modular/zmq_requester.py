import zmq
import argparse
import yaml
import json
import modular.protobuf.ec_boards_base_input_pb2 as repl_cmd
import pkg_resources
from protobuf_to_dict import protobuf_to_dict, dict_to_protobuf

#from modular.protobuf import ec_boards_base_input_pb2 as repl_cmd
#
## Could be any dot-separated package/module name or a "Requirement"
#resource_package = __name__
##resource_path = '/'.join(('templates', 'temp_file'))  # Do not use os.path.join()
#repl_yaml = pkg_resources.resource_string(resource_package, 'repl.yaml')
## or for a file-like stream:
##template = pkg_resources.resource_stream(resource_package, resource_path)

repl_yaml = "./repl.yaml"

def repl_option():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file_yaml", dest="repl_yaml", action="store", default=repl_yaml)
    parser.add_argument("-c", dest="cmd_exec_cnt", action="store", type=int, default=1)
    args = parser.parse_args()
    dict_opt = vars(args)
    return dict_opt

class MultiPartMessage(object):
    header = None

    @classmethod
    def recv(cls, socket):
        """Reads key-value message from socket, returns new instance."""
        return cls.from_msg(socket.recv_multipart())

    @property
    def msg(self):
        return [self.header]

    def send(self, socket, identity=None):
        """Send message to socket"""
        msg = self.msg
        if identity:
            msg = [identity] + msg
        socket.send_multipart(msg)

class EscCmdMessage(MultiPartMessage):
    header = b"ESC_CMD"

    def __init__(self, cmd):
        self.cmd = cmd

    @property
    def msg(self):
        # returns list of all message frames as a byte-string:
        return [self.header, self.cmd]

class EcatMasterCmdMessage(MultiPartMessage):
    header = b"MASTER_CMD"

    def __init__(self, cmd):
        self.cmd = cmd

    @property
    def msg(self):
        # returns list of all message frames as a byte-string:
        return [self.header, self.cmd]

class zmqIO(object):

    def __init__(self, uri):

        #  Prepare our context and sockets
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REQ)
        self.socket.connect("tcp://"+uri)

    def send_to(self, cmd):
        "dict -> protobuf -> serialize to string -> send through socket"
        cmd_pb = dict_to_protobuf(repl_cmd.Repl_cmd, cmd)
        # print(cmd_pb)
        if cmd['type'] == "ECAT_MASTER_CMD":
            cmd_msg = EcatMasterCmdMessage(cmd_pb.SerializeToString())
        else:
            cmd_msg = EscCmdMessage(cmd_pb.SerializeToString())
        cmd_msg.send(self.socket)

    def recv_from(self):
        rep_data = self.socket.recv()
        rep = repl_cmd.Cmd_reply()
        # fill protobuf mesg
        rep.ParseFromString(rep_data)
        print(rep)
        d = protobuf_to_dict(rep)
        yaml_msg = yaml.safe_load(d['msg'])
        json_msg = json.dumps(yaml_msg)
        print(json_msg)
        return json_msg #rep
