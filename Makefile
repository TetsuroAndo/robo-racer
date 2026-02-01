OS			:= $(shell uname -s)
USER		:= $(shell whoami)
RM 			:= rm -rf

# --- PATHS ---
ROOT				:= .
VENV				:= $(ROOT)/.venv
TEST_DIR			:= $(ROOT)/test
LOG_DIR				:= $(ROOT)/logs

PIO_DIR				:= $(ROOT)/.pio
PIO_ENV				?= esp32dev
PIO_BUILD_DIR		:= $(PIO_DIR)/build/$(PIO_ENV)
PIO_BUILD_SRC_DIR	:= $(PIO_BUILD_DIR)/src

FIRMWARE_DIR		:= $(ROOT)/firmware
FIRMWARE_SRC_DIR	:= $(FIRMWARE_DIR)/src
FIRMWARE			:= $(PIO_BUILD_DIR)/firmware.elf

RPI_SRC_DIR			:= $(ROOT)/rpi/src
RPI_LIB_DIR			:= $(ROOT)/rpi/lib
RPI_BUILD_DIR		:= $(ROOT)/rpi/build

RPLIDAR_SDK_DIR		:= $(RPI_LIB_DIR)/rplidar_sdk
RPLIDAR_INC			:= $(RPLIDAR_SDK_DIR)/sdk/include
RPLIDAR_SRC			:= $(RPLIDAR_SDK_DIR)/sdk/src
RPLIDAR_LIB			:= $(RPLIDAR_SDK_DIR)/output/$(OS)/Release
RPLIDAR_SDK_MAKE	:= $(RPLIDAR_SDK_DIR)/Makefile

# ================================
# RPi build rules
# ================================

NAME		:= robo-racer
APP_SERIALD	:= seriald
CMAKE		?= cmake

# ================================
# PlatformIO rules / ESP32
# ================================
ifeq ($(OS), Linux)
  UP_PORT	:= /dev/ttyUSB0
else ifeq ($(OS), Darwin)
  UP_PORT	:= /dev/tty.SLAB_USBtoUART
else ifeq ($(OS), Windows_NT)
  UP_PORT	:= COM3
endif

PIO				?= $(VENV)/bin/pio
PIO_ARG_ENV		?= -e $(PIO_ENV)
PIO_ARG_CLEAN	:= -t clean
PIO_ARG_UPLOAD	:= $(PIO_ARG_ENV) -t upload --upload-port $(UP_PORT)
PIO_RUN			:= $(PIO) run -j $(shell nproc)

# ================================
# Makefile Target
# ================================
.PHONY: all

all: pio rpi
	@if [ "$(USER)" = "pi" ]; then $(MAKE) upload; fi

# === RPi build ===
.PHONY: rpi c-rpi
$(NAME): rpi
rpi: | rplidar_sdk $(LOG_DIR)
	$(CMAKE) -S $(ROOT)/rpi -B $(RPI_BUILD_DIR) -DCMAKE_BUILD_TYPE=Release -DROBO_RACER_NAME=$(NAME)
	$(CMAKE) --build $(RPI_BUILD_DIR) -j $(shell nproc)

c-rpi:
	$(RM) $(RPI_BUILD_DIR)

# === firmware build ===
.PHONY: pio upload monitor c-pio
pio:
	$(PIO_RUN) $(PIO_ARG_ENV)
upload:
	$(PIO_RUN) $(PIO_ARG_UPLOAD)
monitor:
	$(PIO) device monitor

c-pio:
	$(RM) $(PIO_BUILD_SRC_DIR)

fc-pio:
	$(PIO_RUN) $(PIO_ARG_CLEAN)

# === Clean / Rebuild ===
.PHONY: clean fclean re c f r clog
clean: c-rpi c-pio clog
fclean: clean
	$(MAKE) -C $(RPLIDAR_SDK_DIR) clean
	$(RM) $(PIO_DIR)
	$(RM) $(NAME)
re: fclean all

# Aliases
c: clog c-pio c-rpi
f: c
	$(RM) $(FIRMWARE)
	$(RM) $(NAME)
r: f all

# Clean log files
clog:
	$(RM) $(LOG_DIR)/*.log*
$(LOG_DIR):
	@mkdir -p $(LOG_DIR)

# ================================
# Runs
# ================================
.PHONY: test activate hils-build hils-local ros2-up ros2-shell ros2-build \
	ros2-bag-record ros2-bag-play ros2-session-up

hils-build:
	$(CMAKE) -S $(ROOT)/rpi -B $(RPI_BUILD_DIR) -DCMAKE_BUILD_TYPE=Release
	$(CMAKE) --build $(RPI_BUILD_DIR) --target seriald sim_esp32d -j $(shell nproc)

hils-local: hils-build $(PYTHON_LOCAL)
	$(PYTHON_LOCAL) $(ROOT)/tools/hils/run_local_e2e.py \
		--seriald $(RPI_BUILD_DIR)/apps/seriald/seriald \
		--sim $(RPI_BUILD_DIR)/apps/sim_esp32d/sim_esp32d

ros2-up:
	docker compose -f tools/ros2/compose.yml up -d

ros2-shell:
	docker compose -f tools/ros2/compose.yml run --rm ros2 bash

ros2-build:
	docker compose -f tools/ros2/compose.yml run --rm ros2 \
		bash /ws/tools/ros2/scripts/ros2_build.sh

ros2-bag-record:
	docker compose -f tools/ros2/compose.yml run --rm \
		-e RUN_ID -e TOPICS -e PROFILE -e OUT_DIR -e PUBLISH_RUN_ID \
		-e WAIT_SEC -e REQUIRE_AT_LEAST_ONE \
		ros2 \
		bash /ws/tools/ros2/scripts/bag_record.sh

ros2-bag-play:
	@if [ -z "$(BAG)" ]; then \
		echo "Error: BAG パラメータが未設定です。例: make ros2-bag-play BAG=/path/to/bag"; \
		exit 1; \
	fi
	docker compose -f tools/ros2/compose.yml run --rm ros2 \
		bash /ws/tools/ros2/scripts/bag_play.sh $(BAG)

ros2-session-up:
	docker compose -f tools/ros2/compose.yml run --rm \
		-e RUN_ID -e TOPICS -e PROFILE -e OUT_DIR -e PUBLISH_RUN_ID \
		-e WAIT_SEC -e REQUIRE_AT_LEAST_ONE -e SESSION_CMD -e SESSION_WAIT_SEC \
		ros2 \
		bash /ws/tools/ros2/scripts/session_up.sh

test: $(PYTHON_LOCAL)
	$(MAKE) all
	$(VENV)/bin/pytest -svv

activate: $(PYTHON_LOCAL)
	@bash -lc 'source "$(VENV)/bin/activate" && exec $$SHELL -l'

# ================================
# Debugs
# ================================
.PHONY: debug debug-gdb

debug: OPT		:= -O0 -g3 -ggdb
debug: fclean
	$(PIO) debug $(PIO_ARGS_ENV)
	$(MAKE) $(NAME) -j $(shell nproc) OPT="$(OPT)"

debug-gdb:
	$(PIO) debug -s $(PIO_BUILD_DIR) --interface gdb -p


# ================================
# SUBMODULE RULES
# ================================

.PHONY: rplidar_sdk

rplidar_sdk: $(RPLIDAR_SDK_MAKE)
	$(MAKE) -C $(RPLIDAR_SDK_DIR)/sdk CEXTRA=-w CXXEXTRA=-w

$(RPLIDAR_SDK_MAKE):
	@$(RM) $(RPLIDAR_SDK_DIR)
	@echo "[Submodule] Initializing rplidar_sdk..."
	@git submodule update --init --recursive $(RPLIDAR_SDK_DIR)

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
.PHONY: nm nmbin printsrc printobj fill view submodule rplidar_sdk help

nm:
	@nm $(OBJ) | grep ' U ' | awk '{print $$2}' | sort | uniq

nmbin:
	@nm $(NAME) | grep ' U ' | awk '{print $$2}' | sort | uniq

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
	@echo "  all              Build firmware (pio run) and RPi executable"
	@echo "  pio              Build firmware (PlatformIO)"
	@echo "  upload           Upload firmware to the device (pio run -t upload)"
	@echo "  rpi              Build RPi executable (auto-builds RPLIDAR SDK)"
	@echo "  c-rpi            Clean RPi build directory"
	@echo "  clean            Clean RPi builds, PlatformIO build sources, and logs"
	@echo "  c-pio            Clean PlatformIO build source outputs"
	@echo "  fc-pio           PlatformIO full clean (pio run -t clean)"
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
	@echo "  hils-build       Build seriald + sim_esp32d for HILS"
	@echo "  hils-local       Run local HILS (seriald + sim_esp32d)"
	@echo "  monitor          Open serial monitor (pio device monitor)"
	@echo "  ros2-up          Start ROS2 docker compose"
	@echo "  ros2-shell       Open ROS2 container shell"
	@echo "  ros2-build       Build ROS2 workspace in container"
	@echo "  ros2-bag-record  Record rosbag via container"
	@echo "  ros2-bag-play    Play rosbag via container (BAG=... required)"
	@echo "  ros2-session-up  Start ROS2 session + bag record"
	@echo ""
	@echo "Debug Targets:"
	@echo "  debug            Build debug"
	@echo "  debug-gdb        Build GDB debug session"
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
	@echo "  rplidar_sdk      Build the RPLIDAR SDK (auto-inits submodule)"
	@echo "  help             Print this help message"
