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
import logging 

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

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


# python3 test_routines.py combined.png write 
# python3 test_routines.py combined.png read 
# python3 test_routines.py combined.png compute 
# python3 test_routines.py combined.png compute_poll 
# python3 test_routines.py combined.png delete 
