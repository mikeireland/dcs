# conduct a heimdallr shutter sequence, and signal when done

import zmq
import dcs.ZMQutils
import time


class HShutterSeq:
    def __init__(self, dark_time, beam_time, use_splay):

        self.mds = self._open_mds_connection()

        self.dark_time = dark_time
        self.beam_time = beam_time
        self.use_splay = use_splay

        self.settling_time = 1.5

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
            print("Failed to send complete to MCS")
        else:
            print("msg acked")

    def _send_and_get_response(self, message):
        # print("sending", message)
        self.mds.send_string(message)
        response = self.mds.recv_string()
        # print("response", response)
        return response.strip()

    def run(self):
        if self.use_splay:
            raise NotImplementedError("Splay shutter sequence not implemented yet")

        # first take all shutters
        self._send_and_get_response("h_shut close 2")
        self._send_and_get_response("h_shut close 3")
        self._send_and_get_response("h_shut close 4")
        self._send_and_get_response("h_shut open 1")

        time.sleep(self.beam_time + self.settling_time)

        self._send_and_get_response("h_shut close 1")
        self._send_and_get_response("h_shut open 2")

        time.sleep(self.beam_time + self.settling_time)

        self._send_and_get_response("h_shut close 2")
        self._send_and_get_response("h_shut open 3")

        time.sleep(self.beam_time + self.settling_time)

        self._send_and_get_response("h_shut close 3")
        self._send_and_get_response("h_shut open 4")

        time.sleep(self.beam_time + self.settling_time)

        # optimized - send complete when still needing darks
        msg = {
            "origin": "s_h-shutter",
            "data": [
                {"hdlr_complete": 1},
            ],
        }

        self.send_and_recv_ack(msg)

        self._send_and_get_response("h_shut close 4")

        time.sleep(self.dark_time)

        self._send_and_get_response("h_shut open 1,2,3,4")

    def test_mcs(self):
        msg = {
            "origin": "s_h-shutter",
            "data": [
                {"hdlr_complete": 1},
            ],
        }

        self.send_and_recv_ack(msg)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Conduct a heimdallr shutter sequence")
    parser.add_argument(
        "--dark-time",
        type=float,
        default=5.0,
        help="Time in seconds for dark shutter (default: 5.0)",
    )
    parser.add_argument(
        "--beam-time",
        type=float,
        default=1.0,
        help="Time in seconds for beam shutter (default: 1.0)",
    )
    parser.add_argument(
        "--use-splay",
        action="store_true",
        help="Use splay shutter sequence ",
    )

    parser.add_argument(
        "--test-mcs",
        action="store_true",
        help="test mcs connection, sends hdlr complete only",
    )

    args = parser.parse_args()

    shutter_seq = HShutterSeq(args.dark_time, args.beam_time, args.use_splay)

    if args.test_mcs:
        shutter_seq.test_mcs()
    else:
        shutter_seq.run()
