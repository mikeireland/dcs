from dcs.ZMQutils import ZmqReq
import time

# Ensure python-statemachine is installed: pip install python-statemachine
from statemachine import StateMachine, State

import numpy as np
import heapq

N_BASELINES = 6


class HeimdallrStateMachine(StateMachine):
    searching = State("Searching", initial=True)
    sidelobe = State("Sidelobe")
    offload_gd = State("Offload GD")
    servo_on = State("Servo On")

    # linear state machine, states are in order:
    # searching, sidelobe, offload_gd, servo
    # can only move forward one step at a time
    # but can move back any number of steps:
    # forward steps:
    to_sidelobe_from_searching = searching.to(sidelobe)
    to_offload_gd_from_sidelobe = sidelobe.to(offload_gd)
    to_servo_from_offload_gd = offload_gd.to(servo_on)

    # backward steps:
    to_searching_from_sidelobe = sidelobe.to(searching)
    to_searching_from_offload_gd = offload_gd.to(searching)
    to_searching_from_servo = servo_on.to(searching)

    to_sidelobe_from_offload_gd = offload_gd.to(sidelobe)
    to_sidelobe_from_servo = servo_on.to(sidelobe)

    to_offload_gd_from_servo = servo_on.to(offload_gd)

    def __init__(
        self,
        server_endpoint,
        n_lookback=100,
        update_rate=0.1,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.server = ZmqReq(server_endpoint)
        self.n_lookback = n_lookback
        # Get a sample status to determine keys and shapes
        status = self.server.send_payload({"command": "status"})
        if status is None:
            raise RuntimeError(
                "Could not get status from server during initialization."
            )
        self.status_keys = list(status.keys())
        self.status_buffers = {}
        for k, v in status.items():
            arr = np.array(v)
            if arr.ndim == 0 or (arr.ndim == 1 and arr.shape == ()):  # scalar
                self.status_buffers[k] = np.full((n_lookback,), np.nan, dtype=arr.dtype)
            elif arr.ndim == 1:  # time is the first dimension
                self.status_buffers[k] = np.full(
                    (n_lookback, arr.shape[0]), np.nan, dtype=arr.dtype
                )
            elif arr.ndim > 1:
                self.status_buffers[k] = np.full(
                    (n_lookback,) + arr.shape, np.nan, dtype=arr.dtype
                )
            else:
                raise ValueError(
                    f"Unsupported status value shape for key {k}: {arr.shape}"
                )
        self._status_idx = 0  # rolling index for lookback

        self.threshold_lower = 8
        self.threshold_upper = 25

        self.servo_start_gain = 0.05
        self.servo_final_gain = 0.4

        self.update_rate = update_rate  # seconds

        self.n_best_to_keep = 5
        self.best_gd_SNR = [
            [(0, 0) for __ in range(self.n_best_to_keep)] for _ in range(N_BASELINES)
        ]

    def set_threshold(self, value):
        self.server.send_payload(f"set_gd_threshold {value}")
        print(f"Set threshold to {value}")

    def on_enter_searching(self, event):
        from_state = event.transition.source
        self.set_threshold(self.threshold_lower)
        self.server.send_payload('offload "gd"')
        if from_state == self.offload_gd:
            pass
        elif from_state == self.sidelobe:
            pass
        elif from_state == self.servo_on:
            self.server.send_payload('servo "off"')

        # reset button for history...

    def on_enter_sidelobe(self):
        # Operations to perform when entering 'sidelobe'
        self.set_threshold(self.threshold_upper)

        # kicks and see what happens to gd
        print("Kicks should go here...")

    def on_enter_offload_gd(self, event):
        from_state = event.transition.source
        if from_state == self.searching:
            self.set_threshold(self.threshold_lower)
        elif from_state == self.sidelobe:
            self.set_threshold(self.threshold_lower)
        elif from_state == self.servo_on:
            self.server.send_payload('servo "off"')
            self.set_threshold(self.threshold_lower)

    def on_enter_servo_on(self):
        # Operations to perform when entering 'servo_on'
        self.server.send_payload(f"set_gain {self.servo_start_gain}")
        self.server.send_payload('servo "on"')

    def reset_best_gd_SNR(self):
        self.best_gd_SNR = [
            [(0, 0) for __ in range(self.n_best_to_keep)] for _ in range(N_BASELINES)
        ]
        print("best_gd_SNR has been reset.")

    @property
    def M(self):
        return np.array(
            [
                [-1, 1, 0, 0],
                [-1, 0, 1, 0],
                [-1, 0, 0, 1],
                [0, -1, 1, 0],
                [0, -1, 0, 1],
                [0, 0, -1, 1],
            ]
        )

    def poll_transitions(self):
        """
        Poll transition conditions and trigger transitions as needed.
        Fill in the condition checks for each transition below.
        """
        # Poll server and store status
        status = self.server.send_payload({"command": "status"})
        if status is not None:
            for k in self.status_keys:
                arr = np.array(status[k])
                buf = self.status_buffers[k]
                # Handle scalar, 1d, or nd arrays
                if arr.ndim == 0 or (arr.ndim == 1 and arr.shape == ()):  # scalar
                    buf[self._status_idx] = arr
                else:
                    buf[self._status_idx] = arr
            self._status_idx = (self._status_idx + 1) % self.n_lookback

            # load most recent gd_snr and gd_offload
            gd_snr = self.status_buffers.get("gd_snr")
            gd_offload = self.status_buffers.get("gd_offload")
            opds = self.M @ gd_offload[-1]

            for baseline_idx in range(N_BASELINES):
                cur_gdSNR = gd_snr[-1, baseline_idx]
                heapq.heappushpop(
                    self.best_gd_SNR[baseline_idx],
                    (cur_gdSNR, opds[baseline_idx]),
                )

        print("Current State:", self.current_state.name)

        # State transitions
        if self.current_state == self.searching:
            if self.should_go_to_offload_gd(from_state="searching"):
                self.to_offload_gd_from_searching()
            elif self.should_go_to_sidelobe():
                self.to_sidelobe()
        elif self.current_state == self.sidelobe:
            if self.should_go_to_offload_gd(from_state="sidelobe"):
                self.to_offload_gd_from_sidelobe()
            elif self.should_go_to_searching():
                self.to_searching()
        elif self.current_state == self.offload_gd:
            if self.should_go_to_servo_on():
                self.to_servo_on()
            elif self.should_go_to_searching():
                self.to_searching()
            elif self.should_go_to_sidelobe():
                self.to_sidelobe()
        elif self.current_state == self.servo_on:
            if self.should_go_to_offload_gd(from_state="servo_on"):
                self.to_offload_gd_from_servo_on()
            elif self.should_go_to_searching():
                self.to_searching()

    # Placeholder condition methods
    def should_go_to_sidelobe(self):
        # check if the gd_snr is on average between threshold_lower and threshold_upper
        buf = self.status_buffers.get("gd_snr")
        if buf is not None:
            avg_gd_snr = np.mean(buf)
            return self.threshold_lower < avg_gd_snr < self.threshold_upper

    def should_go_to_offload_gd(self, from_state):
        if from_state == "searching":
            # check if median gd_snr for all baselines exceeds lower threshold
            buf = self.status_buffers.get("gd_snr")
            if buf is not None:
                median_gd_snr = np.median(buf, axis=0)
                return np.all(median_gd_snr > self.threshold_lower)

            # alternatively could check if history had 4 or 5 baselines and do the calc...
        elif from_state == "sidelobe":
            # check if median gd_snr for all baselines exceeds upper threshold
            buf = self.status_buffers.get("gd_snr")
            if buf is not None:
                median_gd_snr = np.median(buf, axis=0)
                return np.all(median_gd_snr > self.threshold_upper)
        elif from_state == "servo_on":
            # if all baselines gd_snr drop below lower threshold
            buf = self.status_buffers.get("gd_snr")
            if buf is not None:
                median_gd_snr = np.median(buf, axis=0)
                return np.all(median_gd_snr < self.threshold_lower)

    def should_go_to_servo_on(self):
        # if gd_snr has been above upper threshold over 95% of samples in the lookback
        buf = self.status_buffers.get("gd_snr")
        if buf is not None:
            above_threshold = buf > self.threshold_upper
            fraction_above = np.mean(above_threshold, axis=0)
            return np.all(fraction_above >= 0.95)

    def should_go_to_searching(self):
        # if the gd_snr has dropped below upper threshold consistently for at least 3 baselines out of 6
        buf = self.status_buffers.get("gd_snr")
        if buf is not None:
            below_threshold = buf < self.threshold_upper
            fraction_below = np.mean(below_threshold, axis=0)
            return np.sum(fraction_below >= 0.95) <= 3


def run_state_machine():
    sm = HeimdallrStateMachine("tcp://*:6660", n_lookback=100, update_rate=0.1)
    try:
        while True:
            sm.poll_transitions()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("State machine stopped.")


# Uncomment to run directly
# if __name__ == "__main__":
#     run_state_machine()
