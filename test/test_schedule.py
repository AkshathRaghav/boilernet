import asyncio
import hashlib
import itertools
import random
import time
from heapq import heappush, heappop
from typing import Optional
import matplotlib.pyplot as plt

# ─── Priority mapping ──────────────────────────────────────────────────────────
PRIORITIES = {
    "store":   1,
    "fetch":   2,
    "compute": 3,
    "purge":   4,
}

# ─── Hash Function ──────────────────────────────────────────────────────────────
def stable_hash_way(filename: str) -> int:
    h = hashlib.md5(filename.encode('utf-8')).digest()
    return h[0] % 2

# ─── Global instrumentation lists ──────────────────────────────────────────────
scheduled_markers = []  # List of (real_time, command)
canceled_markers = []   # List of real_time when cancellation occurred

# ─── Event Class ────────────────────────────────────────────────────────────────
class Event:
    def __init__(self, event_id, task_id, command, filename, job_identifier):
        self.event_id       = event_id
        self.task_id        = task_id
        self.command        = command
        self.filename       = filename
        self.job_identifier = job_identifier
        self.status         = "In Queue"
        self.result         = None
        self.retry_count    = 0

# ─── Scheduler with RAM threshold and instrumentation ──────────────────────────
class Scheduler:
    def __init__(self, ram_limit=1000, ram_threshold_pct=0.3):
        self._counter            = itertools.count()
        self._queue              = []   # (priority, counter, Event)
        self.in_flight_compute   : Optional[Event] = None
        self.compute_way         : Optional[int]   = None
        self.completed           = {}  # event_id → Event
        # RAM simulation
        self.ram_limit           = ram_limit
        self.ram_threshold       = ram_limit * ram_threshold_pct
        self.current_ram         = 0
        self.event_ram           = {"store": 10, "fetch": 20, "compute": 50, "purge": 5}
        self.canceled_count      = 0

    def enqueue(self, event: Event) -> bool:
        """Return False if canceled due to RAM threshold."""
        ram_needed = self.event_ram[event.command]
        if self.current_ram + ram_needed > self.ram_threshold:
            self.canceled_count += 1
            canceled_markers.append(time.time())
            return False
        self.current_ram += ram_needed
        pri = PRIORITIES[event.command]
        cnt = next(self._counter)
        heappush(self._queue, (pri, cnt, event))
        return True

    async def start(self):
        asyncio.create_task(self._run_loop())

    async def _run_loop(self):
        while True:
            if self.in_flight_compute is None:
                await self._schedule_next()
            else:
                await self._side_then_poll()
            await asyncio.sleep(0)

    async def _schedule_next(self):
        if not self._queue:
            await asyncio.sleep(0.1)
            return

        pri, cnt, ev = heappop(self._queue)
        scheduled_markers.append((time.time(), ev.command))
        way = stable_hash_way(ev.filename)

        if ev.command == "compute":
            ret = await SUBMIT_COMPUTE(ev, way)
            if ret == 1:
                ev.status = "Processing"
                self.in_flight_compute = ev
                self.compute_way       = way
            else:
                await self._handle_ret(ev, ret)
        else:
            ret = await SUBMIT(ev, way)
            await self._handle_ret(ev, ret)

    async def _side_then_poll(self):
        # 1) side event if available
        for idx, (_, _, ev) in enumerate(self._queue):
            if ev.command != "compute":
                _, _, ev = self._queue.pop(idx)
                scheduled_markers.append((time.time(), ev.command))
                way = stable_hash_way(ev.filename)
                ret = await SUBMIT(ev, way)
                await self._handle_ret(ev, ret)
                break

        # 2) poll compute
        ev = self.in_flight_compute
        ret = await POLL_COMPUTE(ev, self.compute_way)
        if ret == 1:
            ev.status = "Completed"
            ev.result = await FETCH_COMPUTE_RESULT(ev, self.compute_way)
            self.completed[ev.event_id] = ev
            self.current_ram -= self.event_ram[ev.command]
            self.in_flight_compute = None
            self.compute_way       = None

    async def _handle_ret(self, ev: Event, ret: int):
        if ret == 1:
            ev.status = "Completed"
            ev.result = await FETCH_RESULT(ev)
            self.completed[ev.event_id] = ev
            self.current_ram -= self.event_ram[ev.command]
        elif ret in (0, -1):
            ev.status = "Failed"
            self.completed[ev.event_id] = ev
            self.current_ram -= self.event_ram[ev.command]
        elif ret == -2:
            ev.retry_count += 1
            await asyncio.sleep(0.1)
            self.enqueue(ev)

# ─── Simulated subroutines ──────────────────────────────────────────────────────
async def SUBMIT(ev: Event, way: int) -> int:
    delay_map = {"store": 0.2, "fetch": 0.1, "purge": 0.05}
    await asyncio.sleep(delay_map[ev.command] + random.uniform(0, 0.2))
    return random.choices([1, -2, 0, -1], weights=[0.7, 0.1, 0.1, 0.1])[0]

async def SUBMIT_COMPUTE(ev: Event, way: int) -> int:
    await asyncio.sleep(0.5 + random.uniform(0, 0.2))
    return random.choices([1, -2, 0], weights=[0.7, 0.2, 0.1])[0]

async def POLL_COMPUTE(ev: Event, way: int) -> int:
    await asyncio.sleep(0.1)
    return random.choices([1, -2], weights=[0.5, 0.5])[0]

async def FETCH_RESULT(ev: Event) -> bytes:
    return b"ok"

async def FETCH_COMPUTE_RESULT(ev: Event, way: int) -> bytes:
    return b"computed"

# ─── Test & Data Collection ────────────────────────────────────────────────────
async def run_simulation(duration=20.0):
    sched = Scheduler()
    await sched.start()

    times = []
    q_lengths = {cmd: [] for cmd in PRIORITIES}
    ram_usage = []

    inject_acc = 0.0
    start_real = time.time()
    for step in range(int(duration / 0.1)):
        t = step * 0.1
        rate = 5 if t < duration * 0.8 else 200
        inject_acc += rate * 0.1
        while inject_acc >= 1.0:
            inject_acc -= 1.0
            cmd = random.choice(list(PRIORITIES))
            filename = f"file_{step}_{random.randint(0,1000)}.png"
            sched.enqueue(Event(step, step, cmd, filename, "job1"))

        counts = {cmd:0 for cmd in PRIORITIES}
        for _, _, ev in sched._queue:
            counts[ev.command] += 1
        for cmd in PRIORITIES:
            q_lengths[cmd].append(counts[cmd])
        times.append(t)
        ram_usage.append(sched.current_ram)

        await asyncio.sleep(0.1)

    return start_real, times, q_lengths, ram_usage

# # ─── Run simulation and plot ───────────────────────────────────────────────────
# async def main():
#     start_real, times, q_lengths, ram_usage = await run_simulation()

#     plt.figure(figsize=(10, 6))
#     plt.rcParams['axes.labelsize'] = 18 
#     plt.rcParams.update({
#         "text.usetex": False,
#         "font.family": "serif",
#         "font.serif": ["Computer Modern Roman"]
#     })
#     color_map = {"store":"green","fetch":"blue","compute":"orange","purge":"red"}

#     if canceled_markers:
#         x_thresh = canceled_markers[0] - start_real
#         plt.axvline(x_thresh, color='black', linestyle='--', label="RAM Threshold Crossed")

#     for cmd, data in q_lengths.items():
#         c = color_map[cmd]
#         # use the same color for the line…
#         plt.plot(times, data, label=f"{cmd.upper()} Queue", color=c)
#         # …and later, for the markers, reuse c as well
#         xs = [(rt - start_real) for rt, c0 in scheduled_markers if c0 == cmd]
#         ys = [-1 * 1.05] * len(xs)
#         plt.scatter(xs, ys, marker='x', color=c, s=50, label=f"{cmd.upper()} Scheduled")

#     plt.xlabel("Time (s)")
#     plt.ylabel("Queue Size")
#     plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
#     plt.tight_layout()
#     plt.show()

if __name__ == "__main__":
    asyncio.run(main())