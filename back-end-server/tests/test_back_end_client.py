# test_back_end_client.py
import argparse
import datetime as dt
import json
import sys
import time
import zmq
import threading

DEF_HOST = "localhost"
DEF_PORT = 7010

def now_iso():
    return dt.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S")

def build_message(name, parameters=None):
    return {
        "command": {
            "name": name,
            "time": now_iso(),
            "parameters": parameters or []
        }
    }

def send_once(host, port, message):
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.connect(f"tcp://{host}:{port}")
    sock.send_string(json.dumps(message))
    reply = sock.recv_json()
    sock.close()
    return reply

def pretty(reply):
    print(json.dumps(reply, indent=2, sort_keys=True))

def cmd_ping(args):
    msg = build_message("ping")
    pretty(send_once(args.host, args.port, msg))

def cmd_rts(args):
    # accepts either "name" or "bld_name". both work with your server.
    rts_name = args.name if args.raw else f"bld_{args.name}"
    params = []
    for kv in (args.param or []):
        # --param key=value
        if "=" not in kv:
            print(f"ignoring param '{kv}' (need key=value)", file=sys.stderr)
            continue
        k, v = kv.split("=", 1)
        # best-effort type parsing (int/float), else string
        try:
            if v.lower() in ("true","false"):
                val = v.lower() == "true"
            else:
                val = int(v)
        except ValueError:
            try:
                val = float(v)
            except ValueError:
                val = v
        params.append({"name": k, "value": val})
    msg = build_message(rts_name, params)
    pretty(send_once(args.host, args.port, msg))

def cmd_report_jobs(args):
    params = [{"name": "job_id", "value": args.job_id}] if args.job_id else []
    msg = build_message("report_jobs", params)
    pretty(send_once(args.host, args.port, msg))

def cmd_abort(args):
    if not args.job_id:
        print("abort requires --job-id", file=sys.stderr)
        sys.exit(2)
    params = [{"name": "job_id", "value": args.job_id}]
    msg = build_message("abort", params)
    pretty(send_once(args.host, args.port, msg))

def _send_rts_thread(host, port, name, idx):
    # each thread uses its own REQ socket (REQ is not thread-safe)
    rts = f"bld_{name}"
    msg = build_message(rts, [{"name":"idx","value":idx}])
    try:
        reply = send_once(host, port, msg)
        print(f"[{idx}] {reply['reply']['content']}")
    except Exception as e:
        print(f"[{idx}] ERROR: {e}", file=sys.stderr)

def cmd_burst(args):
    """Fire N RTS commands quickly to exercise the server’s bounded pool."""
    threads = []
    for i in range(args.n):
        t = threading.Thread(target=_send_rts_thread, args=(args.host, args.port, args.name, i), daemon=True)
        t.start()
        threads.append(t)
        if args.stagger > 0:
            time.sleep(args.stagger)
    for t in threads:
        t.join()
    # show snapshot
    cmd_report_jobs(args)

def main():
    ap = argparse.ArgumentParser(description="Basic client for BackEndServer (ZMQ REQ).")
    ap.add_argument("--host", default=DEF_HOST)
    ap.add_argument("--port", type=int, default=DEF_PORT)

    sub = ap.add_subparsers(dest="cmd", required=False)

    sp = sub.add_parser("ping", help="send ping")
    sp.set_defaults(func=cmd_ping)

    sr = sub.add_parser("rts", help="send an RTS command (defaults to bld_<name>)")
    sr.add_argument("name", help="RTS name, e.g. openao")
    sr.add_argument("--raw", action="store_true", help="do not prefix with 'bld_'")
    sr.add_argument("--param", action="append", help="key=value (repeatable)")
    sr.set_defaults(func=cmd_rts)

    sj = sub.add_parser("report_jobs", help="list queued/running/done jobs")
    sj.add_argument("--job-id", help="filter a single job id")
    sj.set_defaults(func=cmd_report_jobs)

    sa = sub.add_parser("abort", help="abort a running job by id")
    sa.add_argument("--job-id", required=True)
    sa.set_defaults(func=cmd_abort)

    sb = sub.add_parser("burst", help="fire N RTS commands quickly")
    sb.add_argument("name", help="RTS name, e.g. openao")
    sb.add_argument("-n", type=int, default=10, help="number of RTS to send")
    sb.add_argument("--stagger", type=float, default=0.0, help="seconds between sends")
    sb.set_defaults(func=cmd_burst)

    args = ap.parse_args()

    if not args.cmd:
        # default mini-sequence: ping - bld_openao - report_jobs
        print("-> ping")
        cmd_ping(args)
        print("-> bld_close_baldr_lo")
        args2 = argparse.Namespace(**vars(args), name="openao", raw=False, param=None)
        cmd_rts(args2)
        print("-> report_jobs")
        cmd_report_jobs(args)
        return

    args.func(args)

if __name__ == "__main__":
    main()


"""
python test_back_end_client.py

or for particular tests:
python test_back_end_client.py ping
python test_back_end_client.py rts openao --param beam=1
python test_back_end_client.py report_jobs
python test_back_end_client.py abort --job-id <id>
python test_back_end_client.py burst openao -n 20 --stagger 0.05

"""