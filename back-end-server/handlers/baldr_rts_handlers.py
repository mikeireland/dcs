# baldr_rts_handler.py
from rts_base import AbstractRTSTask, RTSState, RTSErr

# --- minimal skeletons  ---
class OpenAO(AbstractRTSTask):
    def ok_to_run(self) -> bool:
        self.state, self.err = int(RTSState.READY), int(RTSErr.OK); return True
    def run(self, args=None) -> None:
        self.state = int(RTSState.RUNNING); p = self.params_as_dict()
        # TODO: perform action
        self.set_done()
    def update_DB(self) -> None:
        if callable(self.ctx.mcs_notify):
            self.ctx.mcs_notify({"name":"openao","state":self.state,"err":self.err,"meta":self.metadata})
    def abort(self) -> bool:
        self.state, self.err = int(RTSState.ABORTED), int(RTSErr.ABORTED); self.metadata["error"]="aborted"; return True

class CloseAO(AbstractRTSTask):
    def ok_to_run(self) -> bool:
        self.state, self.err = int(RTSState.READY), int(RTSErr.OK); return True
    def run(self, args=None) -> None:
        self.state = int(RTSState.RUNNING)
        # TODO: perform action
        self.set_done()
    def update_DB(self) -> None:
        if callable(self.ctx.mcs_notify):
            self.ctx.mcs_notify({"name":"closeao","state":self.state,"err":self.err,"meta":self.metadata})
    def abort(self) -> bool:
        self.state, self.err = int(RTSState.ABORTED), int(RTSErr.ABORTED); self.metadata["error"]="aborted"; return True

# --- one place to register all Baldr RTS handlers ---
def register(server) -> None:
    server.register_rts("openao", OpenAO)     # also reachable as "bld_openao"
    server.register_rts("closeao", CloseAO)