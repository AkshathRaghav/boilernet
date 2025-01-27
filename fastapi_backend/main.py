from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.responses import Response
from pydantic import BaseModel
from PIL import Image
import threading
import io
import time
from queue import Queue
from typing import Dict, Optional

class Task(BaseModel):
    task_id: int
    data: bytes  
    status: str = "In Queue"  
    result: Optional[bytes] = None  

def ethernet_worker():
    while True:
        if not task_queue.empty():
            task = task_queue.get()
            task.status = "BoilerNet in Control"
            task_store[task.task_id] = task

            time.sleep(5)  

            img = Image.open(io.BytesIO(task.data)) 
            processed_img = img.convert("L") 
            img_bytes = io.BytesIO()
            processed_img.save(img_bytes, format="PNG")
            task.result = img_bytes.getvalue() 

            task.status = "Completed"
            task_store[task.task_id] = task

        time.sleep(1)

task_queue = Queue()
task_store: Dict[int, Task] = {}  
task_id_counter = 0 

threading.Thread(target=ethernet_worker, daemon=True).start()

app = FastAPI()

@app.post("/store_compute")
async def store_compute_job(image: UploadFile = File(...), operation: str = "compute"):
    global task_id_counter
    task_id_counter += 1

    img_bytes = await image.read()
    task = Task(task_id=task_id_counter, data=img_bytes)
    task_store[task.task_id] = task

    if operation == "store":
        task.status = "Completed"
        task_store[task.task_id] = task
        return {"message": "Store operation completed!", "status": task.status}

    elif operation == "compute":
        task_queue.put(task)
        return {"message": "Task added for computation", "task_id": task.task_id, "status": "In Queue"}

@app.get("/status/{task_id}")
def get_task_status(task_id: int):
    task = task_store.get(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    return {"task_id": task.task_id, "status": task.status}

@app.get("/fetch/{task_id}")
def fetch_image(task_id: int):
    task = task_store.get(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    else: 
        return Response(content=task.result, media_type="image/png")

@app.get("/fetch_purge/{task_id}")
def fetch_and_purge_image(task_id: int):
    task = task_store.get(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    else: 
        task_store.pop(task_id)
        return Response(content=task.result, media_type="image/png")