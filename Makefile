OS			:= $(shell uname -s)

# --- PATHS ---
ROOT			:= .
VENV			:= $(ROOT)/.venv
TEST_DIR		:= $(ROOT)/test
LOG_DIR			:= $(ROOT)/logs
PIO_DIR			:= $(ROOT)/.pio
PIO_ENV			?= esp32dev
PIO_BUILD_DIR	:= $(PIO_DIR)/build/$(PIO_ENV)


NAME			:= $(PIO_BUILD_DIR)/firmware.elf
RM				:= rm -rf

# ================================
# PlatformIO rules
# ================================
PIO			?= $(VENV)/bin/pio
PIO_ARGS	?= -e $(PIO_ENV)
PIO_RUN		:= $(PIO) run $(PIO_ARGS)
PIO_CLEAN	:= $(PIO) run -t clean $(PIO_ARGS)

# --- Upload Port ---
ifeq ($(OS), darwin)
	UP_PORT	:= /dev/tty.SLAB_USBtoUART
else ifeq ($(OS), Linux)
	UP_PORT	:= /dev/ttyUSB0
endif

# ================================
# Makefile Target
# ================================
.PHONY: all clean fclean re upload c f r clog

all:
	$(PIO_RUN)
clean:
	$(PIO_CLEAN)
fclean: clean
	$(RM) $(PIO_DIR)
re: fclean all

#
upload: all
	$(PIO) run $(PIO_ARGS) -t upload --upload-port #$(UP_PORT)

# Aliases
c: clog clean
f: clog fclean
r: f all

# Clean log files
clog:
	$(RM) $(LOG_DIR)/*.log*
$(LOG_DIR):
	@mkdir -p $(LOG_DIR)

# ============= Run Targets =============
.PHONY: test activate

test: $(PYTHON_LOCAL)
	@. $(VENV)/bin/activate && $(UV) run pytest -q

activate: $(PYTHON_LOCAL)
	@bash -lc 'source "$(VENV)/bin/activate" && exec $$SHELL -l'

# ================================
# Debugs
# ================================
.PHONY: debug

debug:
	$(PIO) debug $(PIO_ARGS)

# ================================
# Environment Setup
# ================================
.PHONY: init setuphooks pysync purge

# --- Local Binaries ---
UV_LOCAL     := $(VENV)/bin/uv
PIP_LOCAL    := $(VENV)/bin/pip
PYTHON_LOCAL := $(VENV)/bin/python

# --- Global Check ---
UV_GLOBAL    := $(shell command -v uv 2>/dev/null)
ifeq ($(strip $(UV_GLOBAL)),)
  UV := $(UV_LOCAL)
else
  UV := $(UV_GLOBAL)
endif

# --- Targets ---
init: setuphooks pysync

setuphooks:
	@git config --local core.hooksPath .githooks
	@chmod -R 744 .githooks/

# Python setup target. Creates venv and installs base dependencies only.
pysync: $(PYTHON_LOCAL)
	@$(UV) sync --all-extras

# Ensure the virtual environment exists
$(PYTHON_LOCAL):
	@if [ -n "$(UV_GLOBAL)" ]; then \
		"$(UV_GLOBAL)" venv "$(VENV)"; \
	else \
		echo "[Venv] Global 'uv' not found. Using 'python3 -m venv' as fallback..."; \
		python3 -m venv "$(VENV)"; \
		echo "[Venv] Installing 'uv' inside '$(VENV)' using pip..."; \
		"$(PIP_LOCAL)" install --upgrade pip uv; \
	fi
	@echo "[Venv] Virtual environment created successfully."

# Clean up the environment
purge: f
	$(RM) $(VENV)
	$(RM) uv.lock
	@echo "🧹 Clean up complete."

# ================================
# Misc
# ================================
.PHONY: nm nmbin printsrc printobj fill view submodule help

nm:
	@nm $(OBJ) | grep ' U ' | awk '{print $$2}' | sort | uniq

nmbin:
	@nm $(NAME) | grep ' U ' | awk '{print $$2}' | sort | uniq

FIRMWARE_DIR	:= $(ROOT)/firmware
SRC_DIR			:= $(FIRMWARE_DIR)/src
SRC				:= $(shell find $(SRC_DIR) -path '*/test' -prune -o -name '*.cpp' -print)
OBJ				:= $(shell find $(PIO_BUILD_DIR) -name '*.o' -print 2>/dev/null)

printsrc:
	@echo $(SRC) | tr ' ' '\n' | sort

printobj:
	@echo $(OBJ) | tr ' ' '\n' | sort

fill:
	@./tools/fillEmptyDir.sh

view:
	@./tools/rawCodeViewer.sh

submodule:
	git submodule update --init --recursive

help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Build Targets:"
	@echo "  all              Build firmware with PlatformIO (pio run)"
	@echo "  clean            Clean PlatformIO build artifacts (pio run -t clean)"
	@echo "  fclean           Fully clean (clean + remove .pio)"
	@echo "  re               Rebuild (fclean + all)"
	@echo "  clog             Clean log files"
	@echo "  c                Alias for 'clog' and 'clean'"
	@echo "  f                Alias for 'clog' and 'fclean'"
	@echo "  r                Alias for 're' (fclean + all) and 'clog'"
	@echo ""
	@echo "Run Targets:"
	@echo "  test             Run Python tests using pytest"
	@echo "  activate         Activate the Python virtual environment"
	@echo ""
	@echo "Debug Targets:"
	@echo "  debug            Build firmware with PlatformIO"
	@echo ""
	@echo "Environment Setup"
	@echo "  init             Initialize the environment"
	@echo "  setuphooks       Set up git hooks"
	@echo "  pysync           Initialize Python virtual environment with all extras"
	@echo "  purge            Purge the environment"
	@echo ""
	@echo "Misc:"
	@echo "  nm               List undefined symbols in object files"
	@echo "  nmbin            List undefined symbols in the executable"
	@echo "  printsrc         Print source files"
	@echo "  printobj         Print object files"
	@echo "  fill             Fill empty directories"
	@echo "  view             View source code"
	@echo "  submodule        Update and initialize git submodules"
	@echo "  help             Print this help message"
