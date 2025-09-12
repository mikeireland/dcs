import json
import zmq
from typing import Any, Dict, Optional
import logging


class ZmqReq:
    """
    An adapter for a ZMQ REQ socket (client).
    """

    def __init__(self, endpoint: str, timeout_ms: int = 1500):
        self.ctx = zmq.Context.instance()
        self.s = self.ctx.socket(zmq.REQ)
        self.s.RCVTIMEO = timeout_ms
        self.s.SNDTIMEO = timeout_ms
        self.s.connect(endpoint)

    def send_payload(
        self, payload: Dict[str, Any], is_str=False, decode_ascii=True
    ) -> Optional[Dict[str, Any]]:
        """
        is_str: if True, payload is a string and will be sent as-is
        decode_ascii: if True, decode the response as ascii and strip the last character (used False for Cpp interfaces)
        """
        if not is_str:
            self.s.send_string(json.dumps(payload, sort_keys=True))
        else:
            self.s.send_string(payload)

        try:
            if decode_ascii:
                res = self.s.recv().decode("ascii")[:-1]
            else:
                res = self.s.recv_string()

            return json.loads(res)
        except zmq.error.Again as e:
            logging.error(f"ZMQ error occurred: {e}")
            return None
        except json.decoder.JSONDecodeError:
            logging.error(f"JSON decode error occurred")
            return None
