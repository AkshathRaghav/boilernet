import logging

# ─── Logging Configuration ─────────────────────────────────────────────────────
# FastAPI logger (console)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s", 
    force=True
)
logger = logging.getLogger("fastapi")
# Routine logger (file)
routine_logger = logging.getLogger("routine")
routine_logger.setLevel(logging.INFO)
fh = logging.FileHandler('routine.log')
fh.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")

fh.setFormatter(formatter)
routine_logger.addHandler(fh)

import time
import hashlib
import random
import asyncio
import itertools
import socket
import tempfile
import os
from heapq import heappush, heappop
from typing import Dict, Optional, List
import base64
import logging
import struct
import io
from PIL import Image
from fastapi import FastAPI, File, UploadFile, HTTPException, Form, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager
from io import BytesIO
import threading

# ─── Cluster Settings ─────────────────────────────────────────────────────────
IP = "192.168.0.50"
PORT = 8080
TMP_DIR = "/home/araviki/workbench/boilernet/src/fastapi_backend/tmp"
RESULTS_DIR = "/home/araviki/workbench/boilernet/src/fastapi_backend/results"

# ─── Socket Helpers ────────────────────────────────────────────────────────────
def open_socket() -> socket.socket:
    try:
        routine_logger.info(f"Opening socket to {IP}:{PORT}")
        sock = socket.create_connection((IP, PORT), timeout=5)
        routine_logger.info("Socket opened")
        return sock
    except Exception as e:
        routine_logger.error(f"ERROR: Failed to open socket: {e}")
        raise


def close_socket(sock: socket.socket):
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    sock.close()
    routine_logger.info("Socket closed")

# ─── Constants & Packet Types ─────────────────────────────────────────────────
DATA_PACKET_SIZE   = 2030
FILE_NOT_FOUND_ERR = 0xFF

START_WRITE   = 0xA0
DATA_WRITE    = 0xA1
END_WRITE     = 0xA3

START_READ    = 0xB0
DATA_READ     = 0xB1
END_READ      = 0xB3

START_COMPUTE = 0xC0
POLL_COMPUTE  = 0xC1
END_COMPUTE   = 0xC3

DELETE        = 0xE0
DELETE_RET    = 0xE1

# ─── Metrics Storage ────────────────────────────────────────────────────────────
response_times     = []
packet_response_ms = {}

def record_packet_time(name, start, end):
    dt = (end - start) * 1000
    response_times.append(dt)
    packet_response_ms.setdefault(name, []).append(dt)

# ─── Byte Utilities ────────────────────────────────────────────────────────────
def checksum32(data: bytes) -> int:
    return sum(data) & 0xFFFFFFFF

def recvall(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise RuntimeError("ERROR: Socket closed unexpectedly")
        buf.extend(chunk)
    return bytes(buf)

# ─── Acknowledgment FSM ────────────────────────────────────────────────────────
def wait_for_ack(sock: socket.socket, exp_type: int, timeout: float = 5.0) -> int:
    sock.settimeout(timeout)
    start = time.perf_counter()
    try:
        b = sock.recv(1)
        elapsed = time.perf_counter()
        record_packet_time(f"ACK_{exp_type:02X}", start, elapsed)
        if b and b[0] == exp_type:
            return 0
        elif b and b[0] == FILE_NOT_FOUND_ERR:
            return -1
        else:
            return -2
    except socket.timeout:
        record_packet_time(f"ACK_TIMEOUT_{exp_type:02X}", start, time.perf_counter())
        logger.error(f"ERROR: Timeout waiting for ACK {exp_type:02X}")
        return -2

# ─── Packet Builders ───────────────────────────────────────────────────────────
def pad_filename(fn: str, length: int = 80) -> bytes:
    b = fn.encode('utf-8')
    return b[:length].ljust(length, b'\0')

def create_start_write(fn: str) -> bytes:
    return struct.pack("!B80s", START_WRITE, pad_filename(fn))

def create_data_write(chunk: bytes) -> bytes:
    return struct.pack("!BH", DATA_WRITE, len(chunk)) + chunk

def create_end_write(chksum: int) -> bytes:
    return struct.pack("!BI", END_WRITE, chksum)

def create_start_read(fn: str) -> bytes:
    return struct.pack("!B80s", START_READ, pad_filename(fn))

def create_start_compute(fn: str) -> bytes:
    return struct.pack("!B80s", START_COMPUTE, pad_filename(fn))

def create_poll_compute(fn: str) -> bytes:
    return struct.pack("!B80s", POLL_COMPUTE, pad_filename(fn))

def create_delete_packet(fn: str) -> bytes:
    return struct.pack("!B80s", DELETE, pad_filename(fn))

# ─── FSM Implementations ───────────────────────────────────────────────────────
def fsm_write(sock: socket.socket, path: str, user_id: str):
    print("bruh!!")
    try:
        routine_logger.info(f"[fsm_write] START: {path}")
        img = Image.open(path).convert('RGB').resize((224,224))
        buf = io.BytesIO()
        img.save(buf, format='PNG', optimize=True)
        data = buf.getvalue()

        total = len(data)
        n_pkts = (total + DATA_PACKET_SIZE - 1) // DATA_PACKET_SIZE
        fn = f"{user_id}_{os.path.basename(path)}"

        sock.sendall(create_start_write(fn))
        if wait_for_ack(sock, START_WRITE) < 0:
            raise RuntimeError("ERROR: Failed START_WRITE ACK")

        for i in range(n_pkts):
            chunk = data[i*DATA_PACKET_SIZE:(i+1)*DATA_PACKET_SIZE]
            pkt_start = time.perf_counter()
            sock.sendall(create_data_write(chunk))
            record_packet_time("DATA_WRITE", pkt_start, time.perf_counter())
            routine_logger.info(f"[fsm_write] DATA packet {i+1}/{n_pkts} sent")

        chksum = checksum32(data)
        sock.sendall(create_end_write(chksum))
        if wait_for_ack(sock, END_WRITE) < 0:
            raise RuntimeError("ERROR: Failed END_WRITE ACK")
        routine_logger.info("[fsm_write] COMPLETE")
    except Exception as e:
        routine_logger.error(f"ERROR in fsm_write: {e}")
        raise


def fsm_read(sock: socket.socket, out_path: str, user_id: str):
    try:
        fn = f"{user_id}_{os.path.basename(out_path)}"
        routine_logger.info(f"[fsm_read] START: {fn}")
        sock.sendall(create_start_read(fn))
        if wait_for_ack(sock, START_READ) < 0:
            raise RuntimeError("ERROR: Failed START_READ ACK")

        with open(out_path, 'wb') as f:
            while True:
                header = recvall(sock,1)[0]
                if header == DATA_READ:
                    length = struct.unpack("!H", recvall(sock,2))[0]
                    chunk = recvall(sock, length)
                    f.write(chunk)
                    routine_logger.info(f"[fsm_read] DATA_READ {length} bytes")
                elif header == END_READ:
                    checksum = struct.unpack("!I", recvall(sock,4))[0]
                    routine_logger.info(f"[fsm_read] Received checksum {checksum:08X}")
                    break
                else:
                    raise RuntimeError(f"ERROR: Unexpected READ packet {header:02X}")
        routine_logger.info("[fsm_read] COMPLETE")
    except Exception as e:
        routine_logger.error(f"ERROR in fsm_read: {e}")
        raise


def fsm_compute(sock: socket.socket, path: str, user_id: str):
    try:
        fn = f"{user_id}_{os.path.basename(path)}"
        routine_logger.info(f"[fsm_compute] START: {fn}")
        sock.sendall(create_start_compute(fn))
        resp = recvall(sock,1)
        if resp[0] != START_COMPUTE:
            raise RuntimeError("ERROR: Failed to start compute")
        routine_logger.info("[fsm_compute] SENT")
    except Exception as e:
        routine_logger.error(f"ERROR in fsm_compute: {e}")
        raise


def fsm_compute_poll(sock: socket.socket, path: str, user_id: str):
    try:
        fn = f"{user_id}_{os.path.splitext(os.path.basename(path))[0]}_results.txt"
        routine_logger.info(f"[fsm_compute_poll] START: {fn}")
        sock.sendall(create_poll_compute(fn))
        resp = recvall(sock,1)
        if resp[0] == END_COMPUTE:
            routine_logger.info("[fsm_compute_poll] COMPLETE")
        elif resp[0] == WAIT_COMPUTE:
            routine_logger.info("[fsm_compute_poll] IN PROGRESS")
        else:
            raise RuntimeError(f"ERROR: Unexpected POLL reply {resp[0]:02X}")
    except Exception as e:
        routine_logger.error(f"ERROR in fsm_compute_poll: {e}")
        raise


def fsm_delete(sock: socket.socket, path: str, user_id: str):
    try:
        fn = f"{user_id}_{os.path.basename(path)}"
        routine_logger.info(f"[fsm_delete] START: {fn}")
        sock.sendall(create_delete_packet(fn))
        resp = recvall(sock,2)
        if resp[0] != DELETE_RET or resp[1] != 0xAA:
            raise RuntimeError(f"ERROR: Delete failed, flags={resp}")
        routine_logger.info("[fsm_delete] COMPLETE")
    except Exception as e:
        routine_logger.error(f"ERROR in fsm_delete: {e}")
        raise

#######################################################

class Task(BaseModel):
    task_id: int
    job_identifier: str
    filename: str

class Event(BaseModel):
    event_id: int
    task_id: int
    job_identifier: str
    command: str  # "store","fetch","compute","compute_poll","purge"
    status: str = "In Queue"
    result: Optional[bytes] = None

#######################################################
job_registry: Dict[str, Dict[str, List]] = {}
task_id_counter = 0
event_id_counter = 0

request_stats: Dict[str, Dict[str, float]] = {}

# Priority order enforces per-task sequence: store→fetch→compute→compute_poll→purge
PRIORITIES = {"store": 1, "fetch": 2, "compute": 3, "compute_poll": 4, "purge": 5}


class Scheduler:
    def __init__(self, ram_limit=1024, ram_pct=0.3):
        # same init fields
        self._counter     = itertools.count()
        self._queue       = []  # heap of (priority, counter, Event)
        self.in_flight_compute: Optional[Event] = None
        self.completed: Dict[int, Event] = {}
        # RAM simulation
        self.ram_limit   = ram_limit
        self.ram_thresh  = ram_limit * ram_pct
        self.current_ram = 0
        self.ram_cost    = {
            "store": 10,
            "fetch": 20,
            "compute": 50,
            "compute_poll": 0,
            "purge": 5
        }
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def enqueue(self, ev: Event) -> bool:
        with self._lock:
            cost = self.ram_cost.get(ev.command, 0)
            if self.current_ram + cost > self.ram_thresh:
                return False
            self.current_ram += cost
            pri = PRIORITIES.get(ev.command, 10)
            cnt = next(self._counter)
            heappush(self._queue, (pri, cnt, ev))
        return True

    def get_queue_snapshot(self):
        with self._lock:
            return [(ev.event_id, ev.command, ev.status)
                    for _, _, ev in list(self._queue)]

    def start(self):
        """Launch the background thread for the sync run loop."""
        t = threading.Thread(target=self._run_loop, daemon=True)
        t.start()

    def stop(self):
        """Signal the run loop to exit."""
        self._stop.set()

    def _run_loop(self):
        while not self._stop.is_set():
            try:
                if self.in_flight_compute is None:
                    self._dispatch_next()
                else:
                    self._side_then_poll()
            except Exception as e:
                # Catch *any* unexpected exception, log it, *do not kill* the loop
                logger.error(f"ERROR: unexpected scheduler exception: {e}", exc_info=True)
            time.sleep(0.01)  # yield

    def _dispatch_next(self):
        with self._lock:
            if not self._queue:
                return
            _, _, ev = heappop(self._queue)
        try:
            if ev.command == "compute":
                ret = self._submit_compute(ev)
                if ret == 1:
                    ev.status = "Processing"
                    self.in_flight_compute = ev
                else:
                    self._handle_ret(ev, ret)
            else:
                ret = self._submit_operation(ev)
                self._handle_ret(ev, ret)
        except Exception as e:
            # On *any* error, mark this event as failed and continue
            logger.error(f"ERROR: event dispatch for {ev.command} (task {ev.task_id}) failed: {e}", exc_info=True)
            ev.status = "Failed"
            with self._lock:
                self.completed[ev.event_id] = ev
                self.current_ram -= self.ram_cost.get(ev.command, 0)

    def _side_then_poll(self):
        """
        While a compute is in flight, allow exactly one other operation
        (store/fetch/purge) to proceed, then poll the compute.
        All errors are caught per-event to prevent the loop from dying.
        """
        # 1) Pick one non-compute event, if any
        side_ev = None
        with self._lock:
            for idx, (_, _, ev) in enumerate(self._queue):
                if ev.command != "compute":
                    side_ev = ev
                    # remove it from the queue
                    self._queue.pop(idx)
                    break

        if side_ev:
            try:
                ret = self._submit_operation(side_ev)
                self._handle_ret(side_ev, ret)
            except Exception as e:
                # Permanent failure for this side event
                logger.error(f"ERROR: side event {side_ev.command} (task {side_ev.task_id}) failed: {e}", exc_info=True)
                side_ev.status = "Failed"
                with self._lock:
                    self.completed[side_ev.event_id] = side_ev
                    self.current_ram -= self.ram_cost.get(side_ev.command, 0)

        # 2) Now poll the in-flight compute
        ev = self.in_flight_compute
        if not ev:
            return

        try:
            ret = self._poll_compute(ev)
            if ret == 1:
                # Compute complete
                ev.status = "Completed"
                ev.result = self._fetch_compute_result(ev)
                with self._lock:
                    self.completed[ev.event_id] = ev
                    self.current_ram -= self.ram_cost.get(ev.command, 0)
                self.in_flight_compute = None
        except Exception as e:
            # Treat poll errors as permanent failure of that compute
            logger.error(f"ERROR: compute_poll for task {ev.task_id} failed: {e}", exc_info=True)
            ev.status = "Failed"
            with self._lock:
                self.completed[ev.event_id] = ev
                self.current_ram -= self.ram_cost.get(ev.command, 0)
            self.in_flight_compute = None

    def _handle_ret(self, ev: Event, ret: int):
        if ret == 1:
            ev.status = "Completed"
            ev.result = self._fetch_result(ev)
            self.completed[ev.event_id] = ev
            with self._lock:
                self.current_ram -= self.ram_cost.get(ev.command, 0)
        elif ret in (0, -1):
            ev.status = "Failed"
            self.completed[ev.event_id] = ev
            with self._lock:
                self.current_ram -= self.ram_cost.get(ev.command, 0)
        else:  # -2 busy
            time.sleep(0.1)
            self.enqueue(ev)

    # ─── Synchronous Routines Calls ────────────────────────────────────────────────
    def _submit_operation(self, ev: Event) -> int:
        sock = open_socket()
        try:
            routine_logger.info(f"OP {ev.command} START: job={ev.job_identifier} task={ev.task_id}")
            job = job_registry[ev.job_identifier]
            task = next(t for t in job["tasks"] if t.task_id == ev.task_id)
            path = os.path.join(TMP_DIR, task.filename)

            if ev.command == "store":
                fsm_write(sock, path, USER_ID)
            elif ev.command == "fetch":
                fsm_read(sock, path, USER_ID)
                with open(os.path.join(RESULTS_DIR, task.filename), 'rb') as f:
                    ev.result = f.read()
            elif ev.command == "purge":
                fsm_delete(sock, path, USER_ID)

            routine_logger.info(f"OP {ev.command} FINISH: job={ev.job_identifier} task={ev.task_id}")
            return 1
        except Exception as e:
            msg = str(e).lower()
            return -2 if 'busy' in msg else 0
        finally:
            close_socket(sock)

    def _submit_compute(self, ev: Event) -> int:
        try:
            routine_logger.info(f"OP compute START: job={ev.job_identifier} task={ev.task_id}")
            job = job_registry[ev.job_identifier]
            task = next(t for t in job["tasks"] if t.task_id == ev.task_id)
            tmp = os.path.join(TMP_DIR, task.filename)

            sock = open_socket()
            try:
                fsm_compute(sock, tmp, USER_ID)
            finally:
                close_socket(sock)

            routine_logger.info(f"OP compute FINISH: job={ev.job_identifier} task={ev.task_id}")
            return 1
        except Exception as e:
            msg = str(e).lower()
            return -2 if 'busy' in msg else 0

    def _poll_compute(self, ev: Event) -> int:
        try:
            routine_logger.info(f"OP compute_poll START: job={ev.job_identifier} task={ev.task_id}")
            job = job_registry[ev.job_identifier]
            task = next(t for t in job["tasks"] if t.task_id == ev.task_id)
            tmp = os.path.join(TMP_DIR, task.filename)

            sock = open_socket()
            try:
                fsm_compute_poll(sock, tmp, USER_ID)
            finally:
                close_socket(sock)

            routine_logger.info(f"OP compute_poll FINISH: job={ev.job_identifier} task={ev.task_id}")
            return 1
        except Exception as e:
            msg = str(e).lower()
            return -2 if 'busy' in msg else 0

#######################################################
scheduler = Scheduler()

# Create the FastAPI app BEFORE adding middleware
app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["GET","POST"],
    allow_headers=["*"],
)

@asynccontextmanager
async def lifespan(app: FastAPI):
    scheduler.start()
    yield

# Wire up lifecycle
app.router.lifespan_context = lifespan

@app.middleware("http")
async def timing_middleware(request: Request, call_next):
    # On every request, log the scheduler's current queue
    queue_snapshot = scheduler.get_queue_snapshot()
    logger.info(f"Scheduler queue before handling {request.url.path}: {queue_snapshot}")
    start = time.perf_counter()
    response = await call_next(request)
    elapsed = time.perf_counter() - start
    path = request.url.path
    stats = request_stats.setdefault(path, {"count":0, "total_time":0.0})
    stats["count"] += 1
    stats["total_time"] += elapsed
    stats["avg_time"] = stats["total_time"]/stats["count"]
    logger.info(f"[TIMING] {path} → count={stats['count']} avg={stats['avg_time']*1000:.2f}ms")
    return response

#######################################################

def get_existing_event(job_id: str, task_id: int, cmd: str) -> Optional[Event]:
    """Retrieve an event by job, task, and command, or None if missing."""
    job = job_registry.get(job_id)
    if not job:
        return None
    return next(
        (e for e in job["events"]
         if e.task_id == task_id and e.command == cmd),
        None
    )

def _enqueue_event(job_id: str, task_id: int, cmd: str):
    """Enqueue an operation event (store/fetch/compute/compute_poll/purge)."""
    global event_id_counter
    # Validate job & task
    if job_id not in job_registry:
        raise HTTPException(404, "Job not found")
    task = next((t for t in job_registry[job_id]["tasks"]
                 if t.task_id == task_id), None)
    if not task:
        raise HTTPException(404, "Task not found")
    # Prevent duplicates
    if get_existing_event(job_id, task_id, cmd):
        raise HTTPException(409, "Event already exists for this operation")
    # Create & enqueue
    event_id_counter += 1
    ev = Event(
        event_id=event_id_counter,
        task_id=task_id,
        job_identifier=job_id,
        command=cmd,
    )
    job_registry[job_id]["events"].append(ev)
    if not scheduler.enqueue(ev):
        raise HTTPException(503, "RAM limit exceeded, try again later")
    return {"message": f"{cmd.upper()} operation enqueued", "task_id": task_id}

def _poll_event(job_id: str, task_id: int, cmd: str):
    """Return status (and result once complete) for a given event."""
    ev = get_existing_event(job_id, task_id, cmd)
    if not ev:
        raise HTTPException(404, "Event not found")

    completed = scheduler.completed.get(ev.event_id)
    if completed:
        img_b64 = None
        if completed.result:
            img_b64 = base64.b64encode(completed.result).decode()
        return {
            "status": "COMPLETE",
            "task_id": completed.task_id,
            "image": img_b64,
        }

    return {
        "status": ev.status,
        "task_id": ev.task_id,
    }

#######################################################

@app.post("/job_store")
async def create_job(
    image: UploadFile = File(...),
    job_identifier: str = Form(...)
):
    """
    Create a new task under job_identifier, store its image data,
    and enqueue the STORE event.
    """
    global task_id_counter
    task_id_counter += 1

    data = await image.read()

    with open(os.path.join(TMP_DIR, image.filename), 'wb') as f:
        f.write(data)

    t = Task(
        task_id=task_id_counter,
        job_identifier=job_identifier,
        filename=image.filename
    )
    job_registry.setdefault(job_identifier, {"tasks": [], "events": []})["tasks"].append(t)

    return _enqueue_event(job_identifier, t.task_id, "store")


@app.post("/enqueue_op/{job_id}")
async def enqueue_op(
    job_id: str,
    task_id: int = Query(...),
    command: str = Query(...)
):
    """
    Enqueue any of: store, fetch, compute, compute_poll, purge.
    """
    valid_cmds = {"store", "fetch", "compute", "compute_poll", "purge"}
    if command not in valid_cmds:
        raise HTTPException(400, f"Invalid command type: {command!r}")
    return _enqueue_event(job_id, task_id, command)


@app.get("/poll_op/{job_id}")
async def poll_op(
    job_id: str,
    task_id: int = Query(...),
    command: str = Query(...)
):
    """
    Poll the status (and eventual result) of a previously enqueued event.
    """
    return _poll_event(job_id, task_id, command)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        log_level="debug",        # or "info"
        access_log=True,
    )
