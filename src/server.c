#include "server.h"

int server_socket_fd = -1;
int client_socket_fd = -1;

struct sockaddr_in address;

char        server_buffer[SERVER_BUFFER_SIZE] = {0};

void server_init(){
    printf("SERVER: Creating server socket... ");
    server_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_socket_fd < 0){
        printf("\nERROR: creating server socket");
        exit(1);
    }
    printf("OK\n");

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(SERVER_PORT);

    printf("SERVER: Binding server socket with port %d... ", SERVER_PORT);
    if(bind(server_socket_fd, (struct sockaddr*)&address, sizeof(address)) < 0){
        printf("\nERROR: binding server port");
        exit(1);
    }
    printf("OK\n");

    printf("SERVER: Listening for clients... ");
    if(listen(server_socket_fd, SERVER_MAX_CONNECTIONS) < 0){
        printf("\nERROR: listen for clients");
        exit(1);
    }
    printf("OK\n");
}

void server_deinit(){
    printf("SERVER: Closing client connection...");
    if(close(client_socket_fd) < 0){
        printf("\nERROR: Closing client socket");
    }else
        printf("OK\n");

    printf("SERVER: Closing server connection...");
    if(close(server_socket_fd) < 0){
        printf("\nERROR: Closing server socket");
    }else
        printf("OK\n");
}

void server_accept(){
    socklen_t len = sizeof(address);
    client_socket_fd = accept(server_socket_fd, (struct sockaddr*)&address,  &len);

    printf("\nSERVER: Accepting new client... ");
    if(client_socket_fd == -1){
        printf("\nERROR: accepting client");
    }else
        printf("OK\n");
}

void server_read(){
    uint32_t size = read(client_socket_fd, server_buffer, SERVER_BUFFER_SIZE - 1);
    server_buffer[size] = '\0';

    printf("\nCLIENT: %s\n", server_buffer);
}

void server_write(char* msg, const uint32_t len){
    printf("\nSERVER [server_write]: %s", msg);

    write(client_socket_fd, msg, len);
}