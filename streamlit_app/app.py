import streamlit as st
import requests

API_BASE = "http://127.0.0.1:8000"  
st.set_page_config(layout="wide")

def check_task_status(task_str):
    if not task_str: return 0  
    task_id = int(task_str.split(":")[0].split(" ")[1])
    print(st.session_state["tasks"])
    for task in st.session_state["tasks"]:
        if task["task_id"] == task_id:
            if task["status"] == "Completed": return -1
            else: return 1
    return 0
            

if "tasks" not in st.session_state:
    st.session_state["tasks"] = []  

col1, col2 = st.columns([3, 5], gap="large")  

with col1:
    st.header("Inputs")
    uploaded_file = st.file_uploader("Upload File", type=["png", "jpg", "jpeg"])
    additional_info = st.text_input("Additional Information", placeholder="Enter any notes or info")
    operation = st.radio("Select Operation", ["STORE", "COMPUTE"])

    if st.button("Send to API"):
        if not uploaded_file:
            st.warning("Please upload a file before sending.")
        else:
            try:
                payload = {"operation": operation, "info": additional_info}
                files = {"image": uploaded_file.getvalue()}
                response = requests.post(f"{API_BASE}/store_compute", data=payload, files=files)
                
                if response.status_code == 200:
                    task_id = response.json().get("task_id")
                    st.success(f"Task {task_id} created successfully with operation: {operation}")
                    st.session_state["tasks"].append({"task_id": task_id, "status": "In Queue"})
                else:
                    st.error("Failed to send data to API. Please try again.")
            except requests.exceptions.RequestException:
                st.error("Error connecting to the backend.")

with col2:
    st.header("Task Queue")

    if st.session_state["tasks"]:
        task_options = [f"Task {task['task_id']}" for task in st.session_state["tasks"]]
        selected_task = st.selectbox("Select a Task", task_options)
    else:
        st.info("No tasks in the queue yet.")
        selected_task = None

    if selected_task:
        task_id = int(selected_task.split(":")[0].split(" ")[1])
        selected_operation = st.radio("Select Operation", ["FETCH", "FETCH and PURGE"])

        if st.button("Execute Operation"):
            try:
                if selected_operation == "FETCH":
                    response = requests.get(f"{API_BASE}/fetch/{task_id}")
                elif selected_operation == "FETCH and PURGE":
                    response = requests.get(f"{API_BASE}/fetch_purge/{task_id}")

                if response.status_code == 200:
                    st.image(response.content)
                    if selected_operation == "FETCH and PURGE":
                        st.session_state["tasks"] = [task for task in st.session_state["tasks"] if task["task_id"] != task_id]
                else:
                    st.error(f"Failed to use op on {task_id}. Response - {response.text}")
            except requests.exceptions.RequestException:
                st.error("Error connecting to the backend.")

    stat = check_task_status(selected_task)
    print(stat, selected_task)
    if (stat == 1): 
        try:
            task_id = int(selected_task.split(":")[0].split(" ")[1])
            response = requests.get(f"{API_BASE}/status/{task_id}")

            if response.status_code == 200:
                task_status = response.json().get("status")
                st.write(f"Task {task_id} Status: {task_status}")
                for task in st.session_state["tasks"]:
                    if task["task_id"] == task_id:
                        task["status"] = task_status
            else:
                st.error(f"Failed to fetch status for Task {task_id}")
        except requests.exceptions.RequestException:
            st.error("Error connecting to the backend")
    elif (stat == -1): 
        st.write(f"Task {task_id} Status: Completed.")

