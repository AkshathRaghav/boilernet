import streamlit as st
import requests
import time
from PIL import Image
import io
import base64
import uuid
import os
from pathlib import Path


API_BASE = "http://127.0.0.1:8000"
st.set_page_config(layout="wide")

def do_store(uploaded_file, job_id):
    files = {"image": (uploaded_file.name, uploaded_file.getvalue(), uploaded_file.type)}
    data = {"operation": "store", "job_identifier": job_id}
    resp = requests.post(f"{API_BASE}/job_store", files=files, data=data)
    resp.raise_for_status()
    return resp.json()["task_id"]


def do_enqueue(job_id, task_id, command):
    resp = requests.post(
        f"{API_BASE}/enqueue_op/{job_id}",
        params={"task_id": task_id, "command": command},
    )
    resp.raise_for_status()
    return resp.json()


def do_poll(job_id, task_id, command, timeout=30):
    """Poll until status == COMPLETE or until timeout (seconds) expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = requests.get(
            f"{API_BASE}/poll_op/{job_id}",
            params={"task_id": task_id, "command": command},
        )
        resp.raise_for_status()
        js = resp.json()
        print(js)
        status = js["status"]
        if status == "COMPLETE":
            print("Completed op!")
            return js
        time.sleep(0.5)
    raise RuntimeError(f"Polling {command} for task {task_id} timed out")


image_path = Path(__file__).parent / "BoilerNet.png"
image_bytes = image_path.read_bytes()
b64 = base64.b64encode(image_bytes).decode("utf-8")

custom_html = f"""
<div class="banner">
  <img src="data:image/png;base64,{b64}" alt="Banner">
</div>
<style>
  .banner {{ width:100%; height:150px; overflow:hidden; }}
  .banner img {{ width:100%; height:100%; object-fit:contain; }}
</style>
"""

st.components.v1.html(custom_html, height=150)

st.session_state.setdefault("jobs", [])
st.session_state.setdefault("job_count", 0)
st.session_state.setdefault("session_id", str(uuid.uuid4()))

st.divider()
col1, col2 = st.columns([3,5], gap="large")

with col1:
    st.header("Add New Job")
    uploaded = st.file_uploader(
        "Upload up to 5 files (<3 MB each)",
        type=["png","jpg","jpeg"],
        accept_multiple_files=True
    )
    if st.button("Send Job to API"):
        if not uploaded:
            st.warning("Select at least one file.")
        else:
            valid = []
            for f in uploaded:
                if len(f.getvalue()) > 3*1024*1024:
                    st.error(f"{f.name} > 3 MB; skipped.")
                else:
                    valid.append(f)
            if not valid:
                st.error("No valid files remain.")
            else:
                st.session_state["job_count"] += 1
                jid = f"{st.session_state['session_id']}_{st.session_state['job_count']}"
                job = {
                    "job_id": st.session_state["job_count"],
                    "job_identifier": jid,
                    "tasks": [],
                    "can_fetch_results": []
                }
                for f in valid:
                    try:
                        tid = do_store(f, jid)
                        st.success(f"Task {tid} created for {f.name}")
                        job["tasks"].append({"task_id": tid, "file_name": f.name})
                    except Exception as e:
                        st.error(f"Failed to store {f.name}: {e}")
                st.session_state["jobs"].append(job)

with col2:
    st.header("Job Queue")
    jobs = st.session_state["jobs"]
    if not jobs:
        st.info("No jobs yet.")
    else:
        sel = st.selectbox("Select a Job", [f"Job {j['job_id']}" for j in jobs])
        jid_num = int(sel.split()[1])
        job = next(j for j in jobs if j["job_id"] == jid_num)

        st.subheader(f"Job {jid_num} Tasks")
        names = [t["file_name"] for t in job["tasks"]]
        chosen = st.multiselect("Select tasks", names)
        tids = [t["task_id"] for t in job["tasks"] if t["file_name"] in chosen]

        # Build operations list dynamically
        ops = ["store", "compute", "fetch", "purge"]
        if any(tid in job.get("can_fetch_results", []) for tid in tids):
            ops.append("fetch_results")

        op = st.radio("Operation", ops)
        cola, colb = st.columns(2)

        with cola:
            if st.button("Enqueue Operation"):
                for tid, fname in zip(tids, chosen):
                    cmd = "fetch" if op == "fetch_results" else op
                    try:
                        do_enqueue(job["job_identifier"], tid, cmd)
                        st.success(f"{op.upper()} enqueued for task {tid}")
                    except Exception as e:
                        st.error(f"{op.upper()} enqueue failed for task {tid}: {e}")

        with colb:
            if st.button("Poll Status"):
                for tid, fname in zip(tids, chosen):
                    cmd = "fetch" if op == "fetch_results" else op
                    try:
                        result = do_poll(job["job_identifier"], tid, cmd)
                        status = result["status"]
                        st.write(f"Task {tid} → {status}")
                        if status == "COMPLETE":
                            
                            if op == "compute":
                                if tid not in job["can_fetch_results"]:
                                    job["can_fetch_results"].append(tid)
                                st.success(f"Task {tid} COMPUTE complete → FETCH_RESULTS available")
                            elif op == "store": 
                                st.success(f"Task {tid} stored!")
                            elif op == "purge":
                                st.success(f"Task {tid} purged → removing from UI")
                                job["tasks"] = [t for t in job["tasks"] if t["task_id"] != tid]
                            elif op == "fetch_results":
                                raw = base64.b64decode(result.get("image",""))
                                text = raw.decode('utf-8', errors='ignore')
                                st.text_area(f"Results for {fname}", text, height=300)
                            elif result.get("image"):
                                img = Image.open(io.BytesIO(base64.b64decode(result["image"])))
                                st.image(img, caption=f"{op.upper()} result for task {tid}")
                            else:
                                st.success(f"Task {tid} completed ({op}).")
                    except Exception as e:
                        st.error(f"Polling {op.upper()} failed for task {tid}: {e}")
