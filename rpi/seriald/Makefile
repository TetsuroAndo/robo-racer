CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -Wno-psabi -pthread
LDFLAGS  := -pthread

BIN := seriald
SRC := \
  src/main.cpp \
  src/proto.cpp \
  src/uart_posix.cpp \
  src/async_logger.cpp \
  src/sink_stdout.cpp \
  src/sink_file.cpp \
  src/ipc.cpp

all: $(BIN)

$(BIN): $(SRC)
	$(CXX) $(CXXFLAGS) -Iinclude -Isrc -o $@ $(SRC) $(LDFLAGS)

clean:
	rm -f $(BIN)

.PHONY: all clean
