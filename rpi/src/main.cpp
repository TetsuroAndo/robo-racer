#include "LidarReceiver.h"
#include "Process.h"
#include "Sender.h"
#include "config/Config.h"
#include "lidar_to_esp.h"
#include <csignal>
#include <cstdlib>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

int main(int argc, char **argv) {
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	const char *lidar_dev = (argc >= 2) ? argv[1] : cfg::DEFAULT_LIDAR_DEVICE;
	int lidar_baud = (argc >= 3) ? std::atoi(argv[2]) : cfg::DEFAULT_LIDAR_BAUD;
	const char *esp_dev = (argc >= 4) ? argv[3] : cfg::DEFAULT_ESP_DEVICE;
	int esp_baud = (argc >= 5) ? std::atoi(argv[4]) : cfg::DEFAULT_ESP_BAUD;

	LidarReceiver lidarReceiver(lidar_dev, lidar_baud);
	Process process;
	Sender sender(esp_dev, esp_baud);

	while (!g_stop) {
		const std::vector< LidarData > &res = lidarReceiver.receive();
		const ProcResult procResult = process.proc(res);
		// usleep(1 * 1000); // 適度に空白を開けて送りすぎないようにする。
		sender.send(procResult.speed, procResult.angle);
	}
	return 0;
}
