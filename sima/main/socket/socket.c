#include "socket.h"

void init_socket(Socket* sock){
    sock->created = 0;
    int addr_family = 0;
    int ip_protocol = 0;

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    sock->desc =  socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock->desc < 0) {
        ESP_LOGE(sock->tag, "Unable to create socket: errno %d", errno);
        return;
    }

    sock->created = 1;
}

void socket_connect(Socket* sock, const char* host_ip, const int port){
    struct sockaddr_in dest_addr;
    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    int err = connect(sock->desc, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    
    if (err != 0) {
        ESP_LOGE(sock->tag, "Socket unable to connect: errno %d", errno);
        return;
    }
    
    ESP_LOGI(sock->tag, "Successfully connected to server");
}

void socket_close(Socket* sock){
    if(!sock->created){
        return;
    }
    
    if (sock->desc != -1) {
        ESP_LOGE(sock->tag, "Shutting down socket...");
        shutdown(sock->desc, 0);
        close(sock->desc);
    }

    sock->created = 0;
}

void socket_recv(Socket* sock){
    if(!sock->created){
        return;
    }

    int err = recv(sock->desc, sock->rx_buff, SOCKET_BUFFER_MAX - 1, 0);
    if(err < 0){
        ESP_LOGE(sock->tag, "Error receiving data, errno %d", err);
    }else{
        sock->rx_buff[err] = '\0';
        ESP_LOGW(sock->tag, "%s", sock->rx_buff);
    }
}

void socket_tran(Socket* sock){
    if(!sock->created){
        return;
    }

    ESP_LOGW(sock->tag, "Sending: %s", sock->tx_buff);
    int err = write(sock->desc, sock->tx_buff, strlen(sock->tx_buff));
    if(err < 0){
        ESP_LOGE(sock->tag, "Error sending data errno %d", err);
    }
    
    memset(sock->tx_buff, 0, SOCKET_BUFFER_MAX);
}