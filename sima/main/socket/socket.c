#include "socket.h"

int sock;

void init_socket(){
    int addr_family = 0;
    int ip_protocol = 0;

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE("Socket", "Unable to create socket: errno %d", errno);
            return;
        }
}

void socket_connect(char* host_ip, int port){
    char rx_buffer[128];

    struct sockaddr_in dest_addr;
    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE("Socket", "Socket unable to connect: errno %d", errno);
            return;
        }
    ESP_LOGI("Socket", "Successfully connected");


    recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
    ESP_LOGW("Server", "%s", rx_buffer);

    if (sock != -1) {
        ESP_LOGE("Socket", "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}