import streamlit as st
import requests
import time
from PIL import Image
import io
import base64
import uuid
import os

API_BASE = "http://127.0.0.1:8000"
st.set_page_config(layout="wide")

# Banner HTML (unchanged)
custom_html = """
<div class="banner">
  <img src="https://raw.githubusercontent.com/AkshathRaghav/AkshathRaghav.github.io/refs/heads/main/BoilerNet.png" alt="Banner">
</div>
<style>
  .banner { width:100%; height:150px; overflow:hidden; }
  .banner img { width:100%; height:100%; object-fit:contain; }
</style>
"""
st.components.v1.html(custom_html)

# Session state init
if "jobs" not in st.session_state:
    st.session_state["jobs"] = []
if "job_count" not in st.session_state:
    st.session_state["job_count"] = 0
if "session_id" not in st.session_state:
    st.session_state["session_id"] = str(uuid.uuid4())

st.divider()
col1, col2 = st.columns([3,5], gap="large")

# ──────────────────────────────
# Column 1: Submit new job
# ──────────────────────────────
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
                # create job
                st.session_state["job_count"] += 1
                jid = f"{st.session_state['session_id']}_{st.session_state['job_count']}"
                job = {"job_id": st.session_state["job_count"],
                       "job_identifier": jid,
                       "tasks": []}
                # enqueue store for each file
                for idx, f in enumerate(valid, start=1):
                    files = {"image": (f.name, f.getvalue(), f.type)}
                    data  = {"operation":"store","job_identifier":jid,"task_index":idx}
                    try:
                        resp = requests.post(f"{API_BASE}/job_store",
                                             data=data, files=files)
                        resp.raise_for_status()
                        tid = resp.json().get("task_id")
                        st.success(f"Task {tid} created for {f.name}")
                        job["tasks"].append({"task_id":tid,"file_name":f.name})
                    except Exception as e:
                        st.error(f"Failed {f.name}: {e}")
                st.session_state["jobs"].append(job)

# ──────────────────────────────
# Column 2: Job queue & ops
# ──────────────────────────────
with col2:
    st.header("Job Queue")
    jobs = st.session_state["jobs"]
    if not jobs:
        st.info("No jobs yet.")
    else:
        # Job selector
        sel = st.selectbox("Select a Job", [f"Job {j['job_id']}" for j in jobs])
        jid = int(sel.split()[1])
        job = next(j for j in jobs if j["job_id"]==jid)

        st.subheader(f"Job {jid} Tasks")
        # Task list + select
        names = [t["file_name"] for t in job["tasks"]]
        chosen = st.multiselect("Select tasks", names)
        tids   = [t["task_id"] for t in job["tasks"] if t["file_name"] in chosen]

        op = st.radio("Operation", ["compute","fetch","purge"])
        cola, colb = st.columns(2)
        with cola:
            if st.button("Enqueue Operation"):
                for tid in tids:
                    try:
                        resp = requests.post(
                            f"{API_BASE}/enqueue_op/{job['job_identifier']}",
                            params={"task_id":tid,"command":op}
                        )
                        resp.raise_for_status()
                        st.success(f"{op.upper()} enqueued for task {tid}")
                    except Exception as e:
                        st.error(f"{op} enqueue failed: {e}")

        with colb:
            if st.button("Poll Status"):
                for tid in tids:
                    try:
                        resp = requests.get(
                            f"{API_BASE}/poll_op/{job['job_identifier']}",
                            params={"task_id":tid,"command":op}
                        )
                        resp.raise_for_status()
                        result = resp.json()
                        status = result["status"]
                        st.write(f"Task {tid} → {status}")
                        if status == "COMPLETE":
                            if op == "purge":
                                st.success(f"Task {tid} purged → removing from UI")
                                # Remove from session_state
                                job["tasks"] = [t for t in job["tasks"] if t["task_id"] != tid]
                            elif result.get("image"):
                                img = Image.open(io.BytesIO(base64.b64decode(result["image"])))
                                st.image(img, caption=f"{op.upper()} result for task {tid}")
                            else:
                                st.success(f"Task {tid} completed ({op}).")
                    except Exception as e:
                        st.error(f"Poll failed for {tid}: {e}")

        # Delete entire job
        if st.button("Delete Entire Job"):
            try:
                resp = requests.delete(f"{API_BASE}/job/{job['job_identifier']}")
                resp.raise_for_status()
                st.success(f"Job {jid} deleted.")
                st.session_state["jobs"] = [j for j in jobs if j["job_id"]!=jid]
            except Exception as e:
                st.error(f"Job delete failed: {e}")
