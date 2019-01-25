import zmq
import threading

class Server:
    def __init__(self):

        self.url_listener = "inproc://listener"

        # Prepare our context and sockets
        self.context = zmq.Context()#.instance()

        # Socket to talk to listener
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(self.url_listener)

        self.thread = threading.Thread(target=self.listener_routine)#, args=(self.url_listener,))
        self.thread.start()

    def listener_routine(self):
        """Listener routine"""

        # Socket to talk to web-server thread
        self.webserver_socket = self.context.socket(zmq.REP)
        self.webserver_socket.bind(self.url_listener)

        # # Socket to talk to topology generator node
        # topology_socket = context.socket(zmq.SUB)
        # topology_socket.connect("tcp://*:5556")
        #
        # # Subscribe to zipcode, default is NYC, 10001
        # zip_filter = "10001"
        #
        # # Python 2 - ascii bytes to unicode str
        # if isinstance(zip_filter, bytes):
        #     zip_filter = zip_filter.decode('ascii')
        #
        # topology_socket.setsockopt_string(zmq.SUBSCRIBE, zip_filter)
        #
        # string = topology_socket.recv_string()

        while True:
            message = self.webserver_socket.recv()
            print("Received request: %s" % message)
            self.webserver_socket.send(b"Topology_REP")
