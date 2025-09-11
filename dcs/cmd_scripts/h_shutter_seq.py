# conduct a heimdallr shutter sequence, and signal when done

import zmq
import dcs.ZMQutils


class HShutterSeq:
    def __init__(self, dark_time, beam_time):

        self.mds = self._open_mds_connection()

        self.dark_time = dark_time
        self.beam_time = beam_time

        self.mcs_client = dcs.ZMQutils.ZmqReq("tcp://192.168.100.2:7019")

    def _open_mds_connection(self):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 10000)
        server_address = "tcp://192.168.100.2:5555"
        socket.connect(server_address)
        return socket

    def send_and_recv_ack(self, msg):
        # recieve ack
        print(f"sending {msg}")
        resp = self.mcs_client.send_payload(msg)
        if resp is None or resp.get("ok") == False:
            print(resp)
            print("Failed to send offsets to MCS")
        else:
            print("msg acked")

    def run(self):
        pass


def main():
    import argparse
    import time

    parser = argparse.ArgumentParser(description="Conduct a heimdallr shutter sequence")
    parser.add_argument(
        "--dark_time",
        type=float,
        default=5.0,
        help="Time in seconds for dark shutter (default: 5.0)",
    )
    parser.add_argument(
        "--beam_time",
        type=float,
        default=5.0,
        help="Time in seconds for beam shutter (default: 5.0)",
    )

    args = parser.parse_args()

    shutter_seq = HShutterSeq(args.dark_time, args.beam_time)

    shutter_seq.run()
