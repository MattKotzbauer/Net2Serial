import logging
import socket
import serial
from threading import Thread

# (logging config)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# TCP settings
TCP_IP = '127.0.0.1'
TCP_PORT = 5000
BUFFER_SIZE = 1024

# SSL / TLS settings
CERT_FILE = "/home/matt/codes/Net2Serial/cert.crt"
KEY_FILE = "/home/matt/codes/Net2Serial/key_file.key"

# UART settings
VIRTUAL_UART = '/tmp/virtual-uart0'
BAUD_RATE = 9600

# tcp receive -> uart write
def tcp_to_uart(tcp_socket, uart_connection):
    while True:
        data = tcp_socket.recv(BUFFER_SIZE)
        # (close connection if we don't receive data)
        if not data:
            break
        # (otherwise write)
        uart_connection.write(data)

# uart read -> tcp send
def uart_to_tcp(tcp_socket, uart_connection):
    while True:
        data = uart_connection.read(BUFFER_SIZE)
        if not data:
            break
        tcp_socket.send(data)

def handle_client(client_socket, addr, uart_connection):
    print(f"Connection from: {addr}")

    # (t1 writes to uart from tcp, t2 sends tcp from uart)
    t1 = Thread(target=tcp_to_uart, args=(client_socket, uart_connection))
    t2 = Thread(target=uart_to_tcp, args=(client_socket, uart_connection))
    
    # (initialize)
    t1.start()
    t2.start()

    # (wait for both threads)
    t1.join()
    t2.join()

    client_socket.close()
    print(f"Connection with {addr} closed.")

if __name__ == '__main__':
    uart = serial.Serial(VIRTUAL_UART, BAUD_RATE)

    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    context.load_cert_chain(certfile=CERT_FILE, keyfile=KEY_FILE)

    tcp_server.bind((TCP_IP, TCP_PORT))
    tcp_server.listen(1)
    logging.info("TCP server listening on %s:%s", TCP_IP, TCP_PORT)

    try:
        while True:
            conn, addr = tcp_server.accept()
            secure_conn = context.wrap_socket(conn, server_side=True)
            client_thread = Thread(target=handle_client, args=(secure_conn, addr, uart))
            client_thread.start()
    except KeyboardInterrupt:
        print("Shutting down server.")
    finally:
        tcp_server.close()
        uart.close()

