#!/usr/bin/env python3
import os
import sys
import requests

# ─── Configuration ────────────────────────────────────────────────────────────
INPUT_DIR = "./input"
BASE_URL  = "http://127.0.0.1:8000"

def usage():
    print(f"Usage: {sys.argv[0]} <operation> <filename> <user_id>")
    print("  operation: store | read | compute | compute_poll | delete")
    sys.exit(1)

def main():
    if len(sys.argv) != 4:
        usage()

    op, filename, user_id = sys.argv[1], sys.argv[2], sys.argv[3]
    full_path = os.path.join(INPUT_DIR, filename)

    try:
        if op == "store":
            if not os.path.isfile(full_path):
                print(f"Error: file not found: {full_path}")
                sys.exit(1)
            with open(full_path, "rb") as f:
                files = {"file": (filename, f)}
                data  = {"user_id": user_id}
                resp  = requests.post(f"{BASE_URL}/store", files=files, data=data)

        elif op == "read":
            data = {"filename": filename, "user_id": user_id}
            resp = requests.post(f"{BASE_URL}/read", data=data)

        elif op in ("compute", "compute_poll"):
            data = {"filename": filename, "user_id": user_id}
            resp = requests.post(f"{BASE_URL}/{op}", data=data)

        elif op == "delete":
            data = {"filename": filename, "user_id": user_id}
            resp = requests.delete(f"{BASE_URL}/delete", data=data)

        else:
            print(f"Unknown operation: {op}")
            usage()

        print(f"[{resp.status_code}] {resp.json() if resp.headers.get('content-type','').startswith('application/json') else resp.text}")

    except requests.ConnectionError:
        print("Failed to connect to the server. Is FastAPI running on localhost:8000?")
        sys.exit(1)

if __name__ == "__main__":
    main()
