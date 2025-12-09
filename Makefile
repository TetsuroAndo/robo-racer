OS			:= $(shell uname -s)

# --- PATHS ---
ROOT			:= .
VENV			:= $(ROOT)/.venv
SRC_DIR			:= $(ROOT)/src
OBJ_DIR			:= $(ROOT)/obj
TEST_DIR		:= $(ROOT)/test
LOG_DIR			:= $(ROOT)/logs

SRC				:= $(shell find $(SRC_DIR) -path '*/test' -prune -o -name '*.cpp' -print)
OBJ				:= $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))

# ================================
# C++ Compiler rules
# ================================
NAME		:= bin
CXX			:= c++
CXXFLAG		:= -Wall -Wextra -Werror -std=c++23
OPT			:= -O3
RM			:= rm -rf
DEFINE		:= -D_GLIBCXX_USE_CXX23_ABI=0

# ================================
# Makefile Target
# ================================
.PHONY: all clean fclean re c f r clog

all:
	$(MAKE) $(NAME) -j $(shell nproc)
clean:
	$(RM) $(OBJ_DIR)
fclean: clean
	$(RM) $(NAME)
re: fclean all

# Aliases
c: clog
	$(RM) $(OBJ_DIR)
f: c
	$(RM) $(NAME)
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
.PHONY: debug debug-fa scan tidy check

debug: deubg-fa

# FSANITIZE BUILD
debug-fa: OPT		:= -g -O1 -fno-omit-frame-pointer -fsanitize=address
debug-fa: DEFINE	:= -DDEBUG_MODE=DEBUG_ALL
debug-fa: f
	$(MAKE) $(NAME) -j $(shell nproc)

# SCAN BUILD


# ============= STATIC ANALYSIS =============

# clang-tidy rule
TIDY := clang-tidy
TIDYFLAGS := --warnings-as-errors=* -checks=*

tidy: $(SRC)
	$(TIDY) $(TIDYFLAGS) $^ -- -std=c++98 $(CXXFLAG)
	@echo "================================"
	@echo "== Static Analysis Complete! =="
	@echo "================================"

# cppcheck rule
CPPCHECK := cppcheck
CPPCHECKFLAGS := --enable=all --inconclusive --std=c++03 --force --quiet

check:
	@$(CPPCHECK) $(CPPCHECKFLAGS) $(SRC)
	@echo "================================"
	@echo "== Static Analysis Complete! =="
	@echo "================================"

# ================================
# Build
# ================================

$(NAME): $(OBJ) | $(LOG_DIR)
	$(CXX) $(CXXFLAG) $(OPT) $(IDFLAG) $(LFLAG) $(DEFINE) -o  $@ $^
	@echo "====================="
	@echo "== Build Complete! =="
	@echo "====================="
	@echo "[Executable]: $(NAME)"
	@echo "[OS/Arch]: $(OS)"
	@echo "[Include]: $(INC_DIR)"
	@echo "[Compiler flags/CXXFLAG]: $(CXXFLAG)"
	@echo "[Linker flags/LFLAG]: $(LFLAG)"
	@echo "[Optimizer flags/OPT]: $(OPT)"
	@echo "[DEFINE]: $(DEFINE)"
	@echo "====================="

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAG) $(OPT) $(IDFLAG) $(DEFINE) -fPIC -MMD -MP  -c $< -o $@

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
	@echo "  all              Build all targets"
	@echo "  clean            Clean object files"
	@echo "  fclean           Fully clean (clean + remove executable)"
	@echo "  re               Rebuild (fclean + all)"
	@echo "  clog             Clean log files"
	@echo "  c                Alias for 'clean' and 'clog'"
	@echo "  f                Alias for 'fclean' and 'clog'"
	@echo "  r                Alias for 're' (fclean + all) and 'clog'"
	@echo ""
	@echo "Run Targets:"
	@echo "  test             Run Python tests using pytest"
	@echo "  activate         Activate the Python virtual environment"
	@echo ""
	@echo "Debug Targets:"
	@echo "  debug            Build with debug flags"
	@echo "  debug-fa         Build with debug flags and fsanitize"
	@echo "  "
	@echo "  tidy             Static analysis using clang-tidy"
	@echo "  check            Static analysis using cppcheck"
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
