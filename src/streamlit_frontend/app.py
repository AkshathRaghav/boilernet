import streamlit as st
import requests
import time
from PIL import Image
import io
import base64
import uuid

API_BASE = "http://127.0.0.1:8000"
st.set_page_config(layout="wide")

custom_html = """
<div class="banner">
    <img src="https://raw.githubusercontent.com/AkshathRaghav/AkshathRaghav.github.io/refs/heads/main/BoilerNet.png" alt="Banner Image">
</div>
<style>
    .banner {
        width: 100%;
        height: 150px;  /* fixed banner height */
        overflow: hidden; /* optional, to avoid any scrollbars */
    }
    .banner img {
        width: 100%;
        height: 100%;
        object-fit: contain; /* scales the image so that the entire image is visible */
    }
</style>

"""

st.components.v1.html(custom_html)
# Initialize session state keys if not already set
if "jobs" not in st.session_state:
    st.session_state["jobs"] = []  # List of jobs, each job is a dict with tasks inside
if "job_count" not in st.session_state:
    st.session_state["job_count"] = 0  # For incrementing job IDs
if "session_id" not in st.session_state:
    st.session_state["session_id"] = str(uuid.uuid4())  # Unique and static per session

st.divider()


# Main layout: Two columns for inputs and job queue
col1, col2 = st.columns([3, 5], gap="large")

# (Col1 remains the same: new job submission)
with col1:
    st.header("Add New Job")
    uploaded_files = st.file_uploader(
        "Upload Files (each must be < 4MB)",
        type=["png", "jpg", "jpeg"],
        accept_multiple_files=True,
    )

    if st.button("Send Job to API"):
        if not uploaded_files:
            st.warning("Please upload at least one file before sending.")
        elif len(uploaded_files) > 5:
            st.error("You cannot upload more than 5 files per job.")
        else:
            valid_files = []
            for file in uploaded_files:
                file_bytes = file.getvalue()
                if len(file_bytes) > 3 * 1024 * 1024:
                    st.error(
                        f"File {file.name} exceeds 3MB size limit. Skipping this file."
                    )
                else:
                    valid_files.append(file)
            if not valid_files:
                st.error("No valid files to process after size check.")
            else:
                st.session_state["job_count"] += 1
                job_number = st.session_state["job_count"]
                job_identifier = f"{st.session_state['session_id']}_{job_number}"
                job = {
                    "job_id": job_number,
                    "job_identifier": job_identifier,
                    "operation": "Compute",
                    "tasks": [],
                }
                for idx, file in enumerate(valid_files, start=1):
                    file_bytes = file.getvalue()
                    if len(file_bytes) > 4 * 1024 * 1024:
                        st.error(
                            f"File {file.name} exceeds 4MB size limit. Skipping this file."
                        )
                        continue
                    payload = {
                        "operation": "store",
                        "info": "",
                        "job_identifier": job_identifier,
                        "task_index": idx,
                    }
                    files = {"image": file_bytes}
                    try:
                        response = requests.post(
                            f"{API_BASE}/job_compute", data=payload, files=files
                        )
                        if response.status_code == 200:
                            task_id = response.json().get("task_id")
                            st.success(
                                f"Task {task_id} for job {job_identifier} created successfully."
                            )
                            job["tasks"].append(
                                {"task_id": task_id, "file_name": file.name}
                            )
                        else:
                            st.error(
                                f"Failed to send file {file.name} to API. Response: {response.text}"
                            )
                    except requests.exceptions.RequestException:
                        st.error("Error connecting to the backend.")
                st.session_state["jobs"].append(job)

with col2:
    st.header("Job Queue")
    if st.session_state["jobs"]:
        job_options = [f"Job {job['job_id']}" for job in st.session_state["jobs"]]
        selected_job_str = st.selectbox("Select a Job", job_options)
    else:
        st.info("No jobs in the queue yet.")
        selected_job_str = None

    if selected_job_str:
        selected_job_number = int(selected_job_str.split(" ")[1])
        job_obj = next(
            (
                job
                for job in st.session_state["jobs"]
                if job["job_id"] == selected_job_number
            ),
            None,
        )
        if job_obj:
            st.subheader(f"Job {selected_job_number} - Task List")
            # Display only file names.
            task_name_to_id = {
                task["file_name"]: task["task_id"] for task in job_obj["tasks"]
            }
            selected_file_names = st.multiselect(
                "Select tasks", options=list(task_name_to_id.keys())
            )
            # Get the list of task_ids corresponding to the selected file names.
            selected_task_ids = [task_name_to_id[name] for name in selected_file_names]
            # Operation selection is now per task.
            operation_choice = st.radio(
                "Select operation for this task", ["Fetch", "Delete", "Compute"]
            )

            if st.button("Execute Operation on Task(s)"):
                for task_id in selected_task_ids:
                    with st.spinner(f"Processing task {task_id}..."):
                        if operation_choice == "Fetch":
                            response = requests.get(
                                f"{API_BASE}/job_fetch/{job_obj['job_identifier']}?task_id={task_id}"
                            )
                        elif operation_choice == "Delete":
                            response = requests.get(
                                f"{API_BASE}/job_delete/{job_obj['job_identifier']}?task_id={task_id}"
                            )
                        elif operation_choice == "Compute":
                            response = requests.get(
                                f"{API_BASE}/job_compute/{job_obj['job_identifier']}?task_id={task_id}"
                            )
                        if response.status_code == 200:
                            result = response.json()
                            if result["status"] == "COMPLETE":
                                with st.expander(
                                    f"Task [{result['task_id']}] status: {result['status']}",
                                    expanded=False,
                                ):
                                    if operation_choice in ["Fetch", "Compute"]:
                                        if "image" in result and result["image"]:
                                            img = Image.open(
                                                io.BytesIO(
                                                    base64.b64decode(result["image"])
                                                )
                                            )
                                            st.image(img)
                                        else:
                                            print(
                                                f"Task {result['task_id']} complete! -> IMAGE NOT PRESENT??"
                                            )
                            elif operation_choice == "Delete":
                                st.success(f"Task {result['task_id']} DELETED!")
                                job_obj["tasks"] = [
                                    t
                                    for t in job_obj["tasks"]
                                    if t["task_id"] != result["task_id"]
                                ]
                            else:
                                st.warning(
                                    f"Task [{result['task_id']}] status: {result['status']}"
                                )
                        else:
                            st.error(f"Failed on task {task_id}: {response.text}")
                        time.sleep(1)
            # New button: Delete Entire Job
            if st.button("Delete Entire Job"):
                response = requests.delete(
                    f"{API_BASE}/job/{job_obj['job_identifier']}"
                )
                if response.status_code == 200:
                    result = response.json()
                    if result.get("status") == "DELETED":
                        st.success(f"Job {job_obj['job_id']} DELETED!")
                        st.session_state["jobs"] = [
                            job
                            for job in st.session_state["jobs"]
                            if job["job_id"] != job_obj["job_id"]
                        ]
                    else:
                        st.warning(
                            f"Job {job_obj['job_id']} PARTIALLY DELETED. Remaining processing tasks: {result.get('remaining_processing')}"
                        )
                else:
                    st.error(f"Failed to delete job: {response.text}")
