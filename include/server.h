#ifndef SERVER_H
#define SERVER_H

#include <stdio.h> 
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h> // read(), write(), close()

#include "credentials.h"

#define SERVER_MAX_CONNECTIONS 1
#define SERVER_BUFFER_SIZE 1024

extern int server_socket_fd;
extern int client_socket_fd;

extern char         server_buffer[];

void server_init();
void server_deinit();

void server_accept();

void server_read();
void server_write(char* msg, const uint32_t len);

#endif  //SERVER_H