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

#define SOCKET_BUFFER_MAX 128
#define SOCKET_TAG_MAX 32

typedef struct {
    int created;
    int desc;

    char tag[SOCKET_TAG_MAX];

    char rx_buff[SOCKET_BUFFER_MAX];
    char tx_buff[SOCKET_BUFFER_MAX];
} Socket;

void init_socket(Socket* sock);
void socket_close(Socket* sock);

void socket_connect(Socket* sock, const char* host_ip, const int port);

void socket_recv(Socket* sock);
void socket_tran(Socket* sock);

#endif //SOCKET_H