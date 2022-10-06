#include "webdavserver.h"
#include "config.h"
#include "wificonf.h"
#include "cardreader.h"
#include <stdint.h>

WiFiServer dav_tcp_port(wifi_config.dav_port);
ESPWebDAV dav_server;
bool dav_ready = false;

bool initWebDAVServer() {
    if (sdcard_ready) {
        dav_tcp_port.begin(wifi_config.dav_port);
        dav_server.begin(&dav_tcp_port, &SD_DRV);
        dav_server.setTransferStatusCallback([](const char* name, int percent, bool receive)
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "%s: '%s': %d%%\n", receive ? "recv" : "send", name, percent);
            LOG (buf);
        });
        dav_ready = true;
        LOG ("DAV ready on port");
        LOG (CONFIG::intTostr(wifi_config.dav_port));
        LOG ("\n");
    } else {
        dav_ready = false;
        LOG ("DAV NOT ready!\n");
    }
    return dav_ready;
}

void handleDAVClient() {
    if (dav_ready) {
        dav_server.handleClient();
    }
}
