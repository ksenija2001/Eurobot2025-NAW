#include "server.h"

char* msg = "Sima 1";

int main(){

    printf("APP: starting...\n");
    server_init();

     uint8_t running = 1;
     while(running){
        server_accept();

        //char* msg = "Test, radi li ?";
        uint32_t len = strlen(msg);
        server_write(msg, len);

        //server_read();

        if(!strcmp(server_buffer, "quit")){
            running = 0;
        }
    }
    server_deinit();

    return 0;
}