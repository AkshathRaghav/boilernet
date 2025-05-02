#!/usr/bin/env python3
import os
import sys
import socket
import struct
import time
import io
import hashlib
import random
from enum import Enum
from PIL import Image

INPUT_DIR   = "./input"
RESULTS_DIR = "./results"
DATA_PACKET_SIZE      = 2030
FILE_NOT_FOUND_ERR    = 0xFF

START_WRITE    = 0xA0
DATA_WRITE    = 0xA1
MID_WRITE    = 0xA2
END_WRITE    = 0xA3

FUC = 0xA4
WAY_BUSY = 0xA5

START_READ     = 0xB0
DATA_READ     = 0xB1
END_READ     = 0xB3

START_COMPUTE  = 0xC0
POLL_COMPUTE  = 0xC1
WAIT_COMPUTE = 0xC2
END_COMPUTE = 0xC3

DELETE         = 0xE0
DELETE_RET    = 0xE1

ACK_TIMEOUT_MS = 5000

response_times      = []
packet_response_ms  = {}

def record_packet_time(name, start, end):
    dt = (end - start)*1000
    response_times.append(dt)
    packet_response_ms.setdefault(name, []).append(dt)

# ### Helpers #################
def checksum32(data: bytes) -> int:
    return sum(data) & 0xFFFFFFFF

def recvall(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n-len(buf))
        if not chunk:
            raise RuntimeError("Socket closed")
        buf.extend(chunk)
    return bytes(buf)

def wait_for_ack(sock: socket.socket, exp_type: int, timeout: float=5.0) -> int:
    sock.settimeout(timeout)
    start = time.perf_counter()
    try:
        b = sock.recv(1)
        elapsed = time.perf_counter()
        record_packet_time(f"ACK_{exp_type:02X}", start, elapsed)
        if (b and b[0] == exp_type): return 0
        elif (b and b[0] == WAY_BUSY): return -1 
        else: return -2 
    except socket.timeout:
        record_packet_time(f"ACK_TIMEOUT_{exp_type:02X}", start, time.perf_counter())
        print("Timeout!!")
        return -2

# ### Packet Creators #################
def pad_filename(fn: str, length: int=80) -> bytes:
    b = fn.encode('utf-8')
    return b[:length].ljust(length, b'\0')

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

# ### FSMs #################
def fsm_write(sock, path, user_id):
    ext = os.path.splitext(path)[1].lower()
    img = Image.open(path).convert('RGB').resize((224,224))
    buf = io.BytesIO()
    img.save(buf, format='PNG', optimize=True, compress_level=9)
    data = buf.getvalue()

    print(f"STORE on FILENAME: {path} | ext: {ext}")

    total = len(data)
    n_full = total // DATA_PACKET_SIZE
    rem    = total % DATA_PACKET_SIZE
    n_pkts = n_full + bool(rem)
    mid_i  = n_pkts // 2
    fn     = f"{user_id}_{os.path.basename(path)}"

    print(f"Total Bytes: {total} | Packets: {n_pkts}")

    start_all = time.perf_counter()

    # START
    sock.sendall(create_start_write(fn))
    ret = wait_for_ack(sock, START_WRITE)
    if (ret == -2): raise RuntimeError("No START_WRITE ACK")
    elif (ret == -1): raise Exception("Way busy!")

    # DATA 
    for i in range(n_pkts):
        chunk = data[i*DATA_PACKET_SIZE:(i+1)*DATA_PACKET_SIZE]
        pkt_start = time.perf_counter()
        sock.sendall(create_data_write(chunk))
        pkt_end   = time.perf_counter()
        record_packet_time("DATA_WRITE", pkt_start, pkt_end)
        x_dur = pkt_end - pkt_start 
        print(f"[recv] DATA_WRITE_LEN: {len(chunk)} bytes -> {x_dur:.3f}s")

    # END
    chksum = checksum32(data)
    print(f"Sent END packet (write) with checksum {chksum:#010x}")
    sock.sendall(create_end_write(chksum))
    ret = wait_for_ack(sock, END_WRITE)
    if (ret == -2): raise RuntimeError("No END_WRITE ACK")
    elif (ret == -1): raise Exception("Way busy!") # should'nt hit

    end_all = time.perf_counter()
    duration = end_all - start_all
    rate     = total / duration if duration>0 else float('inf')

    print("WRITE complete")
    print(f"  Duration:   {duration:.3f}s")
    print(f"  Throughput: {rate:.2f} bytes/s")

def fsm_read(sock, out_path, user_id):
    fn = f"{user_id}_{os.path.basename(out_path)}"
    print(f"READ on FILENAME: {fn}")

    sock.sendall(create_start_read(fn))
    ret = wait_for_ack(sock, START_READ)
    if (ret == -2): raise RuntimeError("No START_READ ACK")
    elif (ret == -1): raise Exception("Way busy!")

    f = open(out_path,'wb')
    total_recv = 0
    start_all  = time.perf_counter()

    while True:
        header = recvall(sock,1)[0]
        if header == DATA_READ:
            length_bytes = recvall(sock,2)
            length = struct.unpack("!H", length_bytes)[0]

            t0 = time.perf_counter()
            chunk = recvall(sock, length)
            t1 = time.perf_counter()
            record_packet_time("DATA_READ", t0, t1)
            x_dur = t1 - t0 
            print(f"[recv] DATA_READ_LEN: {length} bytes -> {x_dur:.3f}s")

            total_recv += len(chunk)
            f.write(chunk)

        elif header == END_READ:
            checksum_bytes = recvall(sock,4)
            print(f"[recv] END_READ_CHECKSUM: {' '.join(f'0x{b:02X}' for b in checksum_bytes)}")
            remote = struct.unpack("!I", checksum_bytes)[0]
            if (remote>>24)&0xFF == FILE_NOT_FOUND_ERR:
                f.close()
                raise RuntimeError(fn)
            break

        else:
            raise RuntimeError(f"Unexpected READ pkt: {header:02X}")

    f.close()
    end_all  = time.perf_counter()
    duration = end_all - start_all
    rate     = total_recv / duration if duration>0 else float('inf')

    print("READ complete")
    print(f"  Total bytes: {total_recv}")
    print(f"  Duration:    {duration:.3f}s")
    print(f"  Throughput (including I/O + Control Logic):  {rate:.2f} bytes/s")
    print(f"  Checksum:    {remote:08X}")

def fsm_compute(sock, path, user_id):
    fn = f"{user_id}_{os.path.basename(path)}"
    print(f"COMPUTE on FILENAME: {fn}")

    sock.sendall(create_start_compute(fn))
    resp = sock.recv(2)
    print(f"[recv] START_COMPUTE_REPLY: {' '.join(f'0x{b:02X}' for b in resp)}")
    if not resp or resp[0] != START_COMPUTE:
        if resp and resp[0] == FUC:
            raise RuntimeError("FUC received instead of START_COMPUTE")
        if resp and resp[0] == WAY_BUSY: 
            raise Exception("WAY BUSY")
        raise RuntimeError("Failed to start compute")

def fsm_compute_poll(sock, path, user_id): 
    fn = f"{user_id}_{os.path.basename(path).split('.')[0]}_results.txt"
    print(f"COMPUTE POLL on filename: {fn}")

    sock.sendall(create_poll_compute(fn))
    resp = sock.recv(2)
    print(f"[recv] POLL_COMPUTE_REPLY: {' '.join(f'0x{b:02X}' for b in resp)}")
    if resp[0] == END_COMPUTE:
        flag = resp[1]
        print(flag)
        if flag&0xFF == FILE_NOT_FOUND_ERR:
            raise RuntimeError(fn)
        print("COMPUTE complete")
    elif resp[0] == WAIT_COMPUTE:
        print("COMPUTE in progress...")
    elif resp[0] == FUC:
        raise Exception("FUC received")
    elif resp[0] == WAY_BUSY:
        raise Exception("WAY BUSY")
    else:
        raise RuntimeError(f"Unexpected POLL pkt: {resp[0]:02X}")


def fsm_delete(sock, path, user_id):
    fn = f"{user_id}_{os.path.basename(path)}"
    print(f"DELETE on FILENAME: {path}")

    sock.sendall(create_delete_packet(fn))
    sock.settimeout(5)

    code = recvall(sock,2)
    print(f"[recv] DELETE_RET_CODE: {' '.join(f'0x{b:02X}' for b in code)}")
    ok, flag = code
    if flag != 0xAA:
        raise RuntimeError(f"DELETE_RET error, flag={flag:02X}")

    print("DELETE complete")

# ### Main #################
def main():
    if len(sys.argv)!=3:
        print("Usage: test_routines.py <file> <mode>")
        sys.exit(1)

    port    = 8080
    ip      = "192.168.0.50"
    user_id = "user01" # sys.argv[3]
    filearg = os.path.join(INPUT_DIR, sys.argv[1])
    mode    = sys.argv[2].lower()

    print(f"OPERATION: {mode} | USER_ID: {user_id}")

    print("Connecting over Socket")


    with socket.create_connection((ip,port), timeout=5) as sock:
        t0 = time.perf_counter()
        print("Opened Socket!")
        if mode=="write":
            assert os.path.exists(filearg), f"No such file or directory: {filearg}"
            fsm_write(sock, filearg, user_id)
        elif mode=="read":
            out = os.path.join(RESULTS_DIR, os.path.basename(filearg))
            fsm_read(sock, out, user_id)
        elif mode=="compute":
            fsm_compute(sock, filearg, user_id)
        elif mode=="compute_poll":
            fsm_compute_poll(sock, filearg, user_id)
        elif mode=="delete":
            fsm_delete(sock, filearg, user_id)
        else:
            raise ValueError("mode must be write/read/compute/compute_poll/delete")
        t1 = time.perf_counter()

    total = (t1-t0)*1000
    print(f"\n=== Timing Summary ===\nTotal: {total:.2f} ms")
    if response_times:
        avg = sum(response_times)/len(response_times)
        print(f"Avg pkt time: {avg:.2f} ms")
    for pkt, times in packet_response_ms.items():
        print(f"  {pkt}: {sum(times)/len(times):.2f} ms")

if __name__=="__main__":
    main()

# Even if it's a JPG, or JPEG, we convert it into PNG. 
    # python3 test_routines.py combined.png write 
    # python3 test_routines.py combined.png read 
    # python3 test_routines.py combined.png compute 
    # python3 test_routines.py combined.png compute_poll 
    # python3 test_routines.py combined.png delete 
