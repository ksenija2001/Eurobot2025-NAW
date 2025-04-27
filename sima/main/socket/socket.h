#ifndef SOCKET_H
#define SOCKET_H

#include <string.h>

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "esp_netif.h"

#include <netdb.h>

#include "credentials.h"

//#define DEBUG_SOCKET
#define HIGH_DEBUG_SOCKET_LEVEL 1
#define LOW_DEBUG_SOCKET_LEVEL 0

#if defined(DEBUG_SOCKET)
    #include "esp_log.h"

    #define SOCKET_TAG "Socket"
#endif

#if !defined(DEBUG_SOCKET_LEVEL)
    #define DEBUG_SOCKET_LEVEL HIGH_DEBUG_SOCKET_LEVEL
#endif

#define SOCKET_BUFFER_MAX 128
#define SOCKET_TAG_MAX 32

typedef struct {
    int created;
    int desc;

    char tag[SOCKET_TAG_MAX];

    char rx_buff[SOCKET_BUFFER_MAX];
    char tx_buff[SOCKET_BUFFER_MAX];
} Socket;

/***
 * @brief Function used to create and initialize socket
 * 
 * @param sock Pointer to `Socket` struct variable
 * 
 * @retval None
 */
void init_socket(Socket* sock);

/***
 * @brief Function used to destroy and close socket
 * 
 * @param sock Pointer to `Socket` struct variable
 * 
 * @retval None
 */
void socket_close(Socket* sock);

/***
 * @brief Function used to connect to socket
 * 
 * @param sock      Pointer to `Socket` struct variable
 * @param host_ip   String containing host IP address
 * @param port      Host port to connect to
 * 
 * @retval None
 */
void socket_connect(Socket* sock, const char* host_ip, const int port);

/***
 * @brief   Function used to receive max 128 bytes from socket and store them
 *          in `Socket` buffer (`rx_buff`)
 * 
 * @param sock Pointer to `Socket` struct variable
 * 
 * @retval Number of bytes received from host
 */
uint32_t socket_recv(Socket* sock);

/***
 * @brief Function used to send max 128 bytes from `Socket` buffer (`tx_buff`)
 * 
 * @param sock Pointer to `Socket` struct variable
 * 
 * @retval None
 */
uint32_t socket_send(Socket* sock);

#endif //SOCKET_H