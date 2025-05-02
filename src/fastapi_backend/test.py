#!/usr/bin/env python3
"""
sim_client.py

Create tasks using /job_store, enqueue ops, then poll until all complete.
"""

import time
import threading
import requests

API = "http://localhost:8000"

def create_and_run_task(job_id, filename, filepath):
    # 1) Create/store
    files = {'image': open(filepath, 'rb')}
    data  = {'operation': 'store', 'job_identifier': job_id}
    r = requests.post(f"{API}/job_store", files=files, data=data)
    r.raise_for_status()
    task_id = r.json()['task_id']
    print(f"[{job_id}] Stored '{filename}' as task {task_id}")

    # 2) Enqueue compute, compute_poll, fetch, purge in sequence
    # for cmd in ['compute', 'compute_poll', 'fetch', 'purge']:
    for cmd in ['fetch', 'purge']:
        q = {'task_id': task_id, 'command': cmd}
        r = requests.post(f"{API}/enqueue_op/{job_id}", params=q)
        r.raise_for_status()
        print(f"[{job_id}] Enqueued {cmd} for task {task_id}")

    # # 3) Poll loop
    # completed = set()
    # while len(completed) < 4:
    #     for cmd in ['store','compute','compute_poll','fetch','purge']:
    #         try:
    #             r = requests.get(f"{API}/poll_op/{job_id}", params={'task_id':task_id,'command':cmd})
    #             status = r.json()['status']
    #         except requests.HTTPError:
    #             continue
    #         if status == "COMPLETE" and cmd not in completed:
    #             print(f"[{job_id}] {cmd} COMPLETE, result:", r.json().get('image'))
    #             completed.add(cmd)
    #     time.sleep(0.5)

if __name__ == "__main__":
    # simulate two users in parallel
    users = [
        ('userA','file03.png','./input/file03.png'),
        ('userB','file02.png','./input/file02.png'),
    ]
    threads = []
    for uid, fname, path in users:
        t = threading.Thread(target=create_and_run_task, args=(uid,fname,path))
        t.start()
        threads.append(t)
        time.sleep(0.2)

    for t in threads:
        t.join()
    print("All tasks done.")
