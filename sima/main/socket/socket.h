#ifndef SOCKET_H
#define SOCKET_H

#include <string.h>

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "esp_netif.h"
#include "esp_log.h"

#include <netdb.h>

#include "credentials.h"

void init_socket();
void socket_connect(char* host_ip, int port);

#endif //SOCKET_H