import socket

port = 9999
sock = None

def main():
    global sock

    sock = socket.socket()
    sock.connect(('127.0.0.1', port)) 

    sock.send(b"Ovo je test sa test klijenta\n")

    sock.close()

if __name__ == "__main__":
    main()