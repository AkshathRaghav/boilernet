VENV_DIR = venv
PYTHON = $(VENV_DIR)/bin/python
PIP = $(VENV_DIR)/bin/pip
STREAMLIT = $(VENV_DIR)/bin/streamlit
APP_FILE = app.py

.DEFAULT_GOAL := run

venv:
	python3 -m venv $(VENV_DIR)
	$(PIP) install --upgrade pip
setup: venv
	$(PIP) install -r requirements.txt

run:
	venv
	$(STREAMLIT) run $(APP_FILE) --server.port=8501

