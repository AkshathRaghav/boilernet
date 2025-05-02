#!/usr/bin/env python3
import sys
import os
import time
import base64
import requests

INPUT_DIR   = "./input"
RESULTS_DIR   = "./results"
API_BASE = "http://localhost:8000"
STATE_FILE_TMPL = ".client_{job_id}.task"

def save_task_id(job_id, task_id):
    fn = STATE_FILE_TMPL.format(job_id=job_id)
    with open(fn, "w") as f:
        f.write(str(task_id))

def load_task_id(job_id):
    fn = STATE_FILE_TMPL.format(job_id=job_id)
    if not os.path.exists(fn):
        print(f"No stored task_id for job '{job_id}'. Run store first.")
        sys.exit(1)
    return int(open(fn).read().strip())

def do_store(filepath, job_id):
    with open(filepath, "rb") as f:
        files = {"image": (os.path.basename(filepath), f, "application/octet-stream")}
        data = {"operation": "store", "job_identifier": job_id}
        resp = requests.post(f"{API_BASE}/job_store", files=files, data=data)
    resp.raise_for_status()
    js = resp.json()
    tid = js["task_id"]
    print(f"[STORE] enqueued → task_id = {tid}")
    save_task_id(job_id, tid)

def do_enqueue(job_id, command):
    tid = load_task_id(job_id)
    params = {"task_id": tid, "command": command}
    resp = requests.post(f"{API_BASE}/enqueue_op/{job_id}", params=params)
    resp.raise_for_status()
    print(f"[{command.upper()}] {resp.json()['message']}")

def do_poll(job_id, command, out_path=None):
    tid = load_task_id(job_id)
    while True:
        params = {"task_id": tid, "command": command}
        resp = requests.get(f"{API_BASE}/poll_op/{job_id}", params=params)
        resp.raise_for_status()
        js = resp.json()
        status = js["status"]
        print(f"[POLL {command.upper()}] status = {status}")
        if status == "COMPLETE":
            if out_path and js.get("image"):
                data = base64.b64decode(js["image"])
                with open(out_path, "wb") as f:
                    f.write(data)
                print(f"[FETCHED] wrote {len(data)} bytes to {out_path}")
            break
        time.sleep(5)

def main():
    if len(sys.argv) != 3:
        print("Usage: client.py <file> <mode>")
        print("  modes: store, fetch, compute, purge")
        sys.exit(1)

    filepath = os.path.join(INPUT_DIR, sys.argv[1])
    mode     = sys.argv[2].lower()
    job_id   = "user01"


    if mode == "store":
        assert os.path.exists(filepath), f"No such file: {filepath}"
        do_store(filepath, job_id)
    else: 
        do_enqueue(job_id, mode)

    do_poll(job_id, mode, filepath.replace(INPUT_DIR, RESULTS_DIR))


if __name__ == "__main__":
    main()
