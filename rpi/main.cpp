// main.cpp

#include "lidar_to_esp.h"
#include <csignal>
#include <cstdlib>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

int main(int argc, char** argv) {
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    const char* lidar_dev = (argc >= 2) ? argv[1] : "/dev/ttyAMA2";
    int         lidar_baud = (argc >= 3) ? std::atoi(argv[2]) : 460800;
    const char* esp_dev   = (argc >= 4) ? argv[3] : "/dev/ttyAMA0";

    return run_lidar_to_esp(lidar_dev, lidar_baud, esp_dev);
}
