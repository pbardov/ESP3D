#ifndef WEBDAVSERVER_H
#define WEBDAVSERVER_H

#include <WiFiServer.h>
#include <ESPWebDAV.h>
#include "davhandler.h"

extern WiFiServer dav_tcp_port;
extern ESPWebDAV dav_server;

#endif
