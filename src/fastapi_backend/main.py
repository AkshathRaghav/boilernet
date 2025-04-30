import time
import hashlib
import random
import asyncio
import itertools
from heapq import heappush, heappop
from typing import Dict, Optional, List
import base64

from fastapi import FastAPI, File, UploadFile, HTTPException, Form, Query, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from contextlib import asynccontextmanager

#######################################################
class Task(BaseModel):
    task_id: int
    job_identifier: str
    data: bytes
    filename: str

class Event(BaseModel):
    event_id: int
    task_id: int
    job_identifier: str
    command: str              # "store","fetch","compute","purge"
    status: str = "In Queue"
    result: Optional[bytes] = None

#######################################################

job_registry: Dict[str, Dict[str, List]] = {}
task_id_counter = 0
event_id_counter = 0

request_stats: Dict[str, Dict[str, float]] = {}


PRIORITIES = {"store": 1, "fetch": 2, "compute": 3, "purge": 4}

def stable_hash_way(filename: str) -> int:
    h = hashlib.md5(filename.encode("utf-8")).digest()
    return h[0] % 2


#######################################################

class Scheduler:
    def __init__(self, ram_limit=1024, ram_pct=0.3):
        self._counter     = itertools.count()
        self._queue       = [] 
        self.in_flight_compute: Optional[Event] = None
        self.compute_way:       Optional[int]   = None
        self.completed: Dict[int, Event] = {}
        # RAM simulation
        self.ram_limit   = ram_limit
        self.ram_thresh  = ram_limit * ram_pct
        self.current_ram = 0
        self.ram_cost    = {"store": 10, "fetch": 20, "compute": 50, "purge": 5}

    def enqueue(self, ev: Event) -> bool:
        cost = self.ram_cost[ev.command]
        if self.current_ram + cost > self.ram_thresh:
            return False
        self.current_ram += cost
        pri = PRIORITIES[ev.command]
        cnt = next(self._counter)
        heappush(self._queue, (pri, cnt, ev))
        return True

    def get_filename_way(self, ev: Event) -> int:
        job = job_registry.get(ev.job_identifier)
        if not job:
            return 0  # Default to 0 if missing
        task = next((t for t in job["tasks"] if t.task_id == ev.task_id), None)
        if not task:
            return 0  # Default to 0 if missing
        return stable_hash_way(task.filename)
        
    async def start(self):
        asyncio.create_task(self._run_loop())

    async def _run_loop(self):
        while True:
            if self.in_flight_compute is None:
                await self._dispatch_next()
            else:
                await self._side_then_poll()
            await asyncio.sleep(0)

    async def _dispatch_next(self):
        if not self._queue:
            await asyncio.sleep(0.1)
            return
        _, _, ev = heappop(self._queue)
        way = self.get_filename_way(ev)
        if ev.command == "compute":
            ret = await self._submit_compute(ev, way)
            if ret == 1:
                ev.status = "Processing"
                self.in_flight_compute = ev
                self.compute_way       = way
            else:
                await self._handle_ret(ev, ret)
        else:
            ret = await self._submit(ev, way)
            await self._handle_ret(ev, ret)

    async def _side_then_poll(self):
        # 1) one non-compute side event
        for idx, (_, _, ev) in enumerate(self._queue):
            if ev.command != "compute":
                _, _, ev = self._queue.pop(idx)
                way = self.get_filename_way(ev)
                ret = await self._submit(ev, way)
                await self._handle_ret(ev, ret)
                break
        # 2) poll compute
        ev = self.in_flight_compute
        ret = await self._poll_compute(ev, self.compute_way)
        if ret == 1:
            ev.status = "Completed"
            ev.result = await self._fetch_compute_result(ev, self.compute_way)
            self.completed[ev.event_id] = ev
            self.current_ram -= self.ram_cost[ev.command]
            self.in_flight_compute = None

    async def _handle_ret(self, ev: Event, ret: int):
        if ret == 1:
            ev.status = "Completed"
            ev.result = await self._fetch_result(ev)
            self.completed[ev.event_id] = ev
            self.current_ram -= self.ram_cost[ev.command]
        elif ret in (0, -1):
            ev.status = "Failed"
            self.completed[ev.event_id] = ev
            self.current_ram -= self.ram_cost[ev.command]
        else:  # -2 busy
            await asyncio.sleep(0.1)
            self.enqueue(ev)

    # ─── Simulated Hardware Calls ────────────────────────────────────────────────

    async def _submit(self, ev: Event, way: int) -> int:
        await asyncio.sleep({"store":0.2,"fetch":0.1,"purge":0.05}[ev.command] + random.uniform(0,0.2))
        return random.choices([1,-2,0,-1], weights=[0.7,0.1,0.1,0.1])[0]

    async def _submit_compute(self, ev: Event, way: int) -> int:
        await asyncio.sleep(0.5 + random.uniform(0,0.2))
        return random.choices([1,-2,0], weights=[0.7,0.2,0.1])[0]

    async def _poll_compute(self, ev: Event, way: int) -> int:
        await asyncio.sleep(0.1)
        return random.choices([1,-2], weights=[0.5,0.5])[0]

    async def _fetch_result(self, ev: Event) -> bytes:
        return b"ok"

    async def _fetch_compute_result(self, ev: Event, way: int) -> bytes:
        return b"computed"

#######################################################

scheduler = Scheduler()

@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Starting scheduler...")
    await scheduler.start()
    yield
    print("Scheduler shutdown...")

app = FastAPI(lifespan=lifespan)

#######################################################


@app.middleware("http")
async def timing_middleware(request: Request, call_next):
    start = time.perf_counter()
    response = await call_next(request)
    elapsed = time.perf_counter() - start

    path = request.url.path
    stats = request_stats.setdefault(path, {"count": 0, "total_time": 0.0})
    stats["count"] += 1
    stats["total_time"] += elapsed
    stats["avg_time"] = stats["total_time"] / stats["count"]

    print(f"[TIMING] {path} → calls={stats['count']} avg={stats['avg_time']*1000:.2f}ms")
    return response

#######################################################

def get_existing_event(job_id: str, task_id: int, cmd: str) -> Optional[Event]:
    job = job_registry.get(job_id)
    if not job:
        return None
    return next((e for e in job["events"] if e.task_id == task_id and e.command == cmd), None)

def _enqueue_event(job_id: str, task_id: int, cmd: str):
    """Only enqueue an event. No polling/checking."""
    global event_id_counter
    if job_id not in job_registry:
        raise HTTPException(404, "Job not found")
    task = next((t for t in job_registry[job_id]["tasks"] if t.task_id == task_id), None)
    if not task:
        raise HTTPException(404, "Task not found")
    existing = get_existing_event(job_id, task_id, cmd)
    if existing:
        raise HTTPException(409, "Event already exists for this operation")
    event_id_counter += 1
    ev = Event(event_id=event_id_counter, task_id=task_id, job_identifier=job_id, command=cmd)
    job_registry[job_id]["events"].append(ev)
    if not scheduler.enqueue(ev):
        raise HTTPException(503, "RAM limit exceeded, try again later")
    return {"message": f"{cmd.upper()} operation enqueued", "task_id": task_id}

def _poll_event(job_id: str, task_id: int, cmd: str):
    """Only check status of an event."""
    ev = get_existing_event(job_id, task_id, cmd)
    if not ev:
        raise HTTPException(404, "Event not found")

    completed = scheduler.completed.get(ev.event_id)
    if completed:
        img = completed.result and base64.b64encode(completed.result).decode()
        return {
            "status": "COMPLETE",
            "task_id": completed.task_id,
            "image": img,
        }
    return {
        "status": ev.status,
        "task_id": ev.task_id,
    }

#######################################################

@app.post("/job_store")
async def create_job(
    image: UploadFile = File(...),
    operation: str = Form(...),
    job_identifier: str = Form(...)
):
    global task_id_counter
    task_id_counter += 1
    data = await image.read()
    t = Task(task_id=task_id_counter, job_identifier=job_identifier, data=data, filename=image.filename)
    job_registry.setdefault(job_identifier, {"tasks": [], "events": []})["tasks"].append(t)

    return _enqueue_event(job_identifier, t.task_id, "store")

@app.post("/enqueue_op/{job_id}")
async def enqueue_op(job_id: str, task_id: int = Query(...), command: str = Query(...)):
    if command not in {"store", "compute", "fetch", "purge"}:
        raise HTTPException(400, "Invalid command type")
    return _enqueue_event(job_id, task_id, command)

@app.get("/poll_op/{job_id}")
async def poll_op(job_id: str, task_id: int = Query(...), command: str = Query(...)):
    if command not in {"store", "compute", "fetch", "purge"}:
        raise HTTPException(400, "Invalid command type")
    return _poll_event(job_id, task_id, command)
