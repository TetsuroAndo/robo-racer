# ===== configurable paths =====
RPLIDAR_SDK_DIR := rplidar_sdk
RPLIDAR_INC     := $(RPLIDAR_SDK_DIR)/sdk/include
RPLIDAR_SRC     := $(RPLIDAR_SDK_DIR)/sdk/src
RPLIDAR_LIB     := $(RPLIDAR_SDK_DIR)/output/Linux/Release

# ===== target / sources =====
TARGET := lidar_to_esp
SRCS   := main.cpp lidar_to_esp.cpp uart.c
OBJS   := $(SRCS:.cpp=.o)
OBJS   := $(OBJS:.c=.o)

# ===== compiler / flags =====
CXX      := g++
CXXFLAGS := -O2 -Wall -I$(RPLIDAR_INC) -I$(RPLIDAR_SRC)
LDFLAGS  := -L$(RPLIDAR_LIB)
LDLIBS   := -lsl_lidar_sdk -lpthread

# ===== rules =====
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS)

fclean: clean
	rm -f $(TARGET)

re: fclean all

.PHONY: all clean fclean re
