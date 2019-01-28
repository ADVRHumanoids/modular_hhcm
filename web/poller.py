import zmq
import threading

class ZmqPoller:
    def __init__(self):

        self.url_poller = "inproc://poller"

        # initialize topology var. This will be reset by the poller thread each time the subscriber receive a message.
        # It will be sent as a reply for each request by the main thread.
        self.topology = "None"

        # Prepare our context and sockets
        self.context = zmq.Context()#.instance()

        # Socket to talk to poller
        self.requester = self.context.socket(zmq.REQ)
        self.requester.connect(self.url_poller)

        # This will launch a thread where the poller will run
        self.thread = threading.Thread(target=self.poller_routine)#, args=(self.url_poller,))
        self.thread.start()

    def poller_routine(self):
        """Poller routine"""

        # Socket to talk to web-server thread
        self.replier = self.context.socket(zmq.REP)
        self.replier.bind(self.url_poller)

        # Socket to talk to topology generator node
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect("tcp://localhost:5556")
        # Subscribe to zipcode, default is NYC, 10001
        zip_filter = "10001"

        # Python 2 - ascii bytes to unicode str
        if isinstance(zip_filter, bytes):
            zip_filter = zip_filter.decode('ascii')
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, zip_filter)

        # Initialize poll set
        self.poller = zmq.Poller()
        self.poller.register(self.replier, zmq.POLLIN)
        self.poller.register(self.subscriber, zmq.POLLIN)

        # Process messages from both sockets
        while True:

            try:
                socks = dict(self.poller.poll())
            except KeyboardInterrupt:
                break

            if self.replier in socks:
                message = self.replier.recv()
                # process task
                print("Received request: %s" % message)
                self.replier.send(self.topology)

            if self.subscriber in socks:
                message = self.subscriber.recv()
                # process weather update
                print("Received from publisher: %s" % message)
                self.topology = message

            # message = self.webserver_socket.recv()
            # print("Received request: %s" % message)
            # self.webserver_socket.send(b"Topology_REP")
