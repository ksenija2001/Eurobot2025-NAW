#include "server.h"
#include <sys/select.h>

char* msg = "Sima 1";
char input_buff[128];

int main(){

    printf("APP: starting...\n");
    server_init();
    
    server_accept();
    uint8_t running = 1;
    int32_t size;

    while(running){
        fd_set read_fds;
        FD_ZERO(&read_fds);

        FD_SET(0, &read_fds);           // stdin (fd 0)
        FD_SET(client_socket_fd, &read_fds);   // socket fd

        int max_fd = client_socket_fd > 0 ? client_socket_fd : 0;

        int ret = select(max_fd + 1, &read_fds, NULL, NULL, NULL);
        if(ret < 0) break;

        if (FD_ISSET(0, &read_fds)) {
            if (fgets(input_buff, sizeof(input_buff), stdin)) {
                // Remove newline
                input_buff[strcspn(input_buff, "\n")] = 0;
                printf("You typed: %s\n", input_buff);

                if (strcmp(input_buff, "exit") == 0) {
                    running = 0;
                    break;
                }
            }
        }

        if (FD_ISSET(client_socket_fd, &read_fds)) {
            size = server_read();

            if(size < 0) {
                running = 0;
                break;
            }
        }
    }
    server_deinit();

    return 0;
}