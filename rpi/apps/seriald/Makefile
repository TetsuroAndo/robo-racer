CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -Wno-psabi -pthread
LDFLAGS  := -pthread

BIN := seriald
SRC := \
  src/main.cpp \
  src/async_logger.cpp \
  src/sink_stdout.cpp \
  src/sink_file.cpp \
  ../../lib/mc_proto/src/mcproto.cpp \
  ../../lib/mc_ipc/src/UdsSeqPacket.cpp \
  ../../lib/mc_serial/src/Uart.cpp

all: $(BIN)

$(BIN): $(SRC)
	$(CXX) $(CXXFLAGS) -Iinclude -Isrc -I../../lib/mc_proto/include -I../../lib/mc_ipc/include -I../../lib/mc_serial/include -I../../lib/mc_core/include -o $@ $(SRC) $(LDFLAGS)

clean:
	rm -f $(BIN)

.PHONY: all clean
