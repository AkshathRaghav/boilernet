# app.py

import os
import socket
import struct
import time
import io
import threading
import queue
import shutil
from enum import Enum
import socket, os
from fastapi import FastAPI, File, UploadFile, Form, HTTPException
from fastapi.responses import JSONResponse

from PIL import Image

# ─── Configuration ────────────────────────────────────────────────────────────

INPUT_DIR   = "./input"
TEMP_DIR   = "./tmp"
RESULTS_DIR = "./results"
IP          = "192.168.0.50"
PORT        = 8080
DATA_PACKET_SIZE   = 2030
FILE_NOT_FOUND_ERR = 0xFF
HW_IP   = "192.168.0.50"
HW_PORT = 8080

# Ensure directories exist
os.makedirs(INPUT_DIR, exist_ok=True)
os.makedirs(RESULTS_DIR, exist_ok=True)

# ─── Packet Types ─────────────────────────────────────────────────────────────

START_WRITE    = 0xA0
DATA_WRITE     = 0xA1
MID_WRITE      = 0xA2
END_WRITE      = 0xA3

START_READ     = 0xB0
DATA_READ      = 0xB1
END_READ       = 0xB3

START_COMPUTE  = 0xC0
POLL_COMPUTE   = 0xC1
WAIT_COMPUTE   = 0xC2
END_COMPUTE    = 0xC3

DELETE         = 0xE0
DELETE_RET     = 0xE1

WAY_BUSY       = 0xA5
FUC            = 0xA4

ACK_TIMEOUT_MS = 5000

# ─── Metrics ─────────────────────────────────────────────────────────────────

response_times     = []
packet_response_ms = {}

def record_packet_time(name, start, end):
    dt = (end - start) * 1000
    response_times.append(dt)
    packet_response_ms.setdefault(name, []).append(dt)

# ─── Helpers ─────────────────────────────────────────────────────────────────

def checksum32(data: bytes) -> int:
    return sum(data) & 0xFFFFFFFF

def recvall(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise RuntimeError("Socket closed unexpectedly")
        buf.extend(chunk)
    return bytes(buf)

def wait_for_ack(sock: socket.socket, exp_type: int, timeout: float = 5.0) -> int:
    sock.settimeout(timeout)
    start = time.perf_counter()
    try:
        b = sock.recv(1)
        elapsed = time.perf_counter()
        record_packet_time(f"ACK_{exp_type:02X}", start, elapsed)
        if b and b[0] == exp_type:
            return 0
        elif b and b[0] == WAY_BUSY:
            return -1
        else:
            return -2
    except socket.timeout:
        record_packet_time(f"ACK_TIMEOUT_{exp_type:02X}", start, time.perf_counter())
        return -2

def pad_filename(fn: str, length: int = 80) -> bytes:
    b = fn.encode("utf-8")
    return b[:length].ljust(length, b"\0")

# ─── Packet Creators ──────────────────────────────────────────────────────────

def create_start_write(fn: str) -> bytes:
    return struct.pack("!B80s", START_WRITE, pad_filename(fn))

def create_data_write(chunk: bytes) -> bytes:
    return struct.pack("!BH", DATA_WRITE, len(chunk)) + chunk

def create_mid_write() -> bytes:
    return struct.pack("!B", MID_WRITE)

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

# ─── FSMs ─────────────────────────────────────────────────────────────────────

def fsm_write(sock, path, user_id):
    ext = os.path.splitext(path)[1].lower()
    img = Image.open(path).convert("RGB").resize((224,224))
    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True, compress_level=9)
    data = buf.getvalue()

    total = len(data)
    n_full = total // DATA_PACKET_SIZE
    rem    = total % DATA_PACKET_SIZE
    n_pkts = n_full + bool(rem)
    fn     = f"{user_id}_{os.path.basename(path)}"

    start_all = time.perf_counter()
    sock.sendall(create_start_write(fn))
    ret = wait_for_ack(sock, START_WRITE)
    if ret == -2:
        raise RuntimeError("No START_WRITE ACK")
    elif ret == -1:
        raise RuntimeError("Way busy on start write")

    for i in range(n_pkts):
        chunk     = data[i*DATA_PACKET_SIZE:(i+1)*DATA_PACKET_SIZE]
        pkt_start = time.perf_counter()
        sock.sendall(create_data_write(chunk))
        pkt_end   = time.perf_counter()
        record_packet_time("DATA_WRITE", pkt_start, pkt_end)

    chksum = checksum32(data)
    sock.sendall(create_end_write(chksum))
    ret = wait_for_ack(sock, END_WRITE)
    if ret == -2:
        raise RuntimeError("No END_WRITE ACK")
    elif ret == -1:
        raise RuntimeError("Way busy on end write")

def fsm_read(sock, out_path, user_id):
    fn = f"{user_id}_{os.path.basename(out_path)}"
    sock.sendall(create_start_read(fn))
    ret = wait_for_ack(sock, START_READ)
    if ret == -2:
        raise RuntimeError("No START_READ ACK")
    elif ret == -1:
        raise RuntimeError("Way busy on start read")

    with open(out_path, "wb") as f:
        while True:
            header = recvall(sock, 1)[0]
            if header == DATA_READ:
                length = struct.unpack("!H", recvall(sock,2))[0]
                t0 = time.perf_counter()
                chunk = recvall(sock, length)
                t1 = time.perf_counter()
                record_packet_time("DATA_READ", t0, t1)
                f.write(chunk)
            elif header == END_READ:
                checksum_bytes = recvall(sock, 4)
                remote = struct.unpack("!I", checksum_bytes)[0]
                if ((remote >> 24) & 0xFF) == FILE_NOT_FOUND_ERR:
                    raise FileNotFoundError(fn)
                break
            else:
                raise RuntimeError(f"Unexpected READ pkt: {header:02X}")

def fsm_compute(sock, path, user_id):
    fn = f"{user_id}_{os.path.basename(path)}"
    sock.sendall(create_start_compute(fn))
    resp = sock.recv(2)
    if not resp or resp[0] != START_COMPUTE:
        raise RuntimeError("Failed to start compute")

def fsm_compute_poll(sock, path, user_id):
    fn = f"{user_id}_{os.path.basename(path).split('.')[0]}_results.txt"
    sock.sendall(create_poll_compute(fn))
    resp = sock.recv(2)
    if resp[0] == END_COMPUTE:
        flag = resp[1]
        if (flag & 0xFF) == FILE_NOT_FOUND_ERR:
            raise FileNotFoundError(fn)
    elif resp[0] == WAIT_COMPUTE:
        # still in progress
        pass
    else:
        raise RuntimeError(f"Unexpected POLL pkt: {resp[0]:02X}")

def fsm_delete(sock, path, user_id):
    fn = f"{user_id}_{os.path.basename(path)}"
    sock.sendall(create_delete_packet(fn))
    sock.settimeout(5)
    code = recvall(sock, 2)
    _, flag = code
    if flag != 0xAA:
        raise RuntimeError("DELETE failed on server")

# ─── Event Queue & Consumer ──────────────────────────────────────────────────

class Event:
    def __init__(self, mode: str, path: str, user_id: str):
        self.mode    = mode
        self.path    = path
        self.user_id = user_id

event_queue = queue.Queue()

def consumer():
    while True:
        evt: Event = event_queue.get()
        try:
            with socket.create_connection((IP, PORT), timeout=5) as sock:
                if evt.mode == "write":
                    fsm_write(sock, evt.path, evt.user_id)
                elif evt.mode == "read":
                    fsm_read(sock, evt.path, evt.user_id)
                elif evt.mode == "compute":
                    fsm_compute(sock, evt.path, evt.user_id)
                elif evt.mode == "compute_poll":
                    fsm_compute_poll(sock, evt.path, evt.user_id)
                elif evt.mode == "delete":
                    fsm_delete(sock, evt.path, evt.user_id)
        except Exception as e:
            print(f"[ERROR] {evt.mode} {evt.path}: {e}")
        finally:
            event_queue.task_done()

# start the consumer thread
threading.Thread(target=consumer, daemon=True).start()

# ─── FastAPI App & Endpoints ─────────────────────────────────────────────────

app = FastAPI()

@app.post("/store")
async def store(user_id: str = Form(...), file: UploadFile = File(...)):
    """Upload a file and queue a STORE (write) operation."""
    local_path = os.path.join(TEMP_DIR, file.filename)
    with open(local_path, "wb") as f:
        shutil.copyfileobj(file.file, f)
    event_queue.put(Event("write", local_path, user_id))
    return JSONResponse({"status": "queued", "operation": "write", "file": file.filename})

@app.post("/read")
async def read(filename: str = Form(...), user_id: str = Form(...)):
    """Queue a READ operation. The file will be written to RESULTS_DIR."""
    out_path = os.path.join(RESULTS_DIR, filename)
    event_queue.put(Event("read", out_path, user_id))
    return JSONResponse({"status": "queued", "operation": "read", "output_file": out_path})

@app.post("/compute")
async def compute(filename: str = Form(...), user_id: str = Form(...)):
    """Queue a COMPUTE operation."""
    local_path = os.path.join(TEMP_DIR, filename)
    event_queue.put(Event("compute", local_path, user_id))
    return JSONResponse({"status": "queued", "operation": "compute", "file": filename})

@app.post("/compute_poll")
async def compute_poll(filename: str = Form(...), user_id: str = Form(...)):
    """Queue a COMPUTE_POLL operation."""
    local_path = os.path.join(TEMP_DIR, filename)
    event_queue.put(Event("compute_poll", local_path, user_id))
    return JSONResponse({"status": "queued", "operation": "compute_poll", "file": filename})

@app.delete("/delete")
async def delete(filename: str = Form(...), user_id: str = Form(...)):
    """Queue a DELETE operation."""
    local_path = os.path.join(TEMP_DIR, filename)
    event_queue.put(Event("delete", local_path, user_id))
    return JSONResponse({"status": "queued", "operation": "delete", "file": filename})
