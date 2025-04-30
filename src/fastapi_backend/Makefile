VENV_DIR = ./venv
PYTHON = $(VENV_DIR)/bin/python
PIP = $(VENV_DIR)/bin/pip
UVICORN = $(VENV_DIR)/bin/uvicorn
APP_FILE = main
PORT = 8000

.DEFAULT_GOAL := run

venv:
	python3 -m venv $(VENV_DIR)
	$(PIP) install --upgrade pip
setup: venv
	$(PIP) install -r requirements.txt
run:
	uvicorn $(APP_FILE):app --host 127.0.0.1 --port $(PORT) --reload