import socket
import threading

server_sock = None
client_sock = None

def client_loop(client, addr):

    print("Sima joined", addr)
    
    while True:

        msg = b""

        while True:
            part = client.recv(128)

            msg += part

            if part.endswith(b'\n'):
                break

        if msg.decode() == "close":
            break

        print("Sima [", addr, "]:", msg.decode())

    client.close()

def main():
    global server_sock, client_socket

    print("test")

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # enable address reuse
    server_sock.bind(('', 9999))
    server_sock.listen(5)

    while(True):

        client_socket, addr = server_sock.accept()

        thread = threading.Thread(target=client_loop, args=(client_socket, addr))

        thread.start()

if __name__ == "__main__":
    main()
