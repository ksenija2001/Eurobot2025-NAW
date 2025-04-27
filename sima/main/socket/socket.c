#include "socket.h"

void init_socket(Socket* sock){

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
        ESP_LOGI(SOCKET_TAG, "%s Initializing socket...", sock->tag);
    #endif

    sock->created = 0;
    int addr_family = 0;
    int ip_protocol = 0;

    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
        ESP_LOGI(SOCKET_TAG, "%s: Creating socket...", sock->tag);
    #endif

    sock->desc =  socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock->desc < 0) {
        #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
            ESP_LOGE(SOCKET_TAG, "%s: Unable to create socket: errno %d", sock->tag, errno);
        #endif
        return;
    }

    sock->created = 1;
    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
        ESP_LOGI(SOCKET_TAG, "%s: Socket created", sock->tag);
    #endif
}

void socket_connect(Socket* sock, const char* host_ip, const int port){

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
        ESP_LOGI(SOCKET_TAG, "%s: Connecting to socket...", sock->tag);
    #endif

    struct sockaddr_in dest_addr;
    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    int err = connect(sock->desc, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    
    if (err != 0) {
        #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
            ESP_LOGE(SOCKET_TAG, "%s: Socket unable to connect: errno %d", sock->tag, errno);
        #endif
        return;
    }
    
    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
        ESP_LOGI(SOCKET_TAG, "%s: Successfully connected to server", sock->tag);
    #endif
}

void socket_close(Socket* sock){

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
        ESP_LOGI(SOCKET_TAG, "%s: Closing socket...", sock->tag);
    #endif

    if(!sock->created){
        return;
    }
    
    if (sock->desc != -1) {
        #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
            ESP_LOGE(SOCKET_TAG, "%s: Shutting down socket...", sock->tag);
        #endif
        shutdown(sock->desc, 0);
        close(sock->desc);
    }

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == HIGH_DEBUG_SOCKET_LEVEL
        ESP_LOGE(SOCKET_TAG, "%s: Socket closed", sock->tag);
    #endif
    sock->created = 0;
}

int32_t socket_recv(Socket* sock){

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
        ESP_LOGE(SOCKET_TAG, "%s: Receiving data...", sock->tag);
    #endif

    if(!sock->created){
        return -1;
    }

    int err = recv(sock->desc, sock->rx_buff, SOCKET_BUFFER_MAX - 1, 0);
    if(err < 0){
        #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
            ESP_LOGE(SOCKET_TAG, "%s: Error receiving data, errno %d", sock->tag, err);
        #endif
    }

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
        ESP_LOGE(SOCKET_TAG, "%s: Data received, length: %d", sock->tag, err);
    #endif

    return err;
}

int32_t socket_send(Socket* sock){

    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
        ESP_LOGE(SOCKET_TAG, "%s: Sending data...", sock->tag);
    #endif

    if(!sock->created){
        return -1;
    }

    int err = write(sock->desc, sock->tx_buff, strlen(sock->tx_buff));
    if(err < 0){
        #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
            ESP_LOGE(SOCKET_TAG, "%s: Error sending data errno %d", sock->tag, err);
        #endif
    }
    
    memset(sock->tx_buff, 0, SOCKET_BUFFER_MAX);
    #if defined(DEBUG_SOCKET) && DEBUG_SOCKET_LEVEL == LOW_DEBUG_SOCKET_LEVEL
        ESP_LOGE(SOCKET_TAG, "%s: Data sent, length %d", sock->tag, err);
    #endif

    return err;
}