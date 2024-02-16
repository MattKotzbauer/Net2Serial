#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int init_uart(const char* uart_device) {
    int uart_fd = open(uart_device, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd < 0) {
        perror("Error opening UART");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(uart_fd, &tty) != 0) {
        perror("Error from tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return -1;
    }
    return uart_fd;
}


int init_tcp_server(int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("ERROR opening socket");
        exit(1);
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR on binding");
        exit(1);
    }

    listen(sockfd, 5);
    return sockfd;
}

void* tcp_to_uart(void* args) {
    connection_args *conn_args = (connection_args*)args;
    char buffer[1024];
    ssize_t bytes_read;

    while ((bytes_read = read(conn_args->client_fd, buffer, sizeof(buffer))) > 0) {
        write(conn_args->uart_fd, buffer, bytes_read);
    }

    // (todo: error handling)
    return NULL;
}

void* uart_to_tcp(void* args) {
    connection_args *conn_args = (connection_args*)args;
    char buffer[1024];
    ssize_t bytes_read;

    while ((bytes_read = read(conn_args->uart_fd, buffer, sizeof(buffer))) > 0) {
        write(conn_args->client_fd, buffer, bytes_read);
    }

    // (todo: error handling)
    return NULL;
}

int main() {
    int uart_fd = init_uart("/dev/ttyUSB0"); // adjust
    int server_fd = init_tcp_server(5000);   // adjust port

    while (1) {
        struct sockaddr_in cli_addr;
        socklen_t clilen = sizeof(cli_addr);

        int client_fd = accept(server_fd, (struct sockaddr *)&cli_addr, &clilen);
        if (client_fd < 0) {
            perror("ERROR on accept");
            continue;
        }

        connection_args *args = malloc(sizeof(connection_args));
        if (!args) {
            perror("Failed to allocate memory for connection args");
            close(client_fd);
            continue;
        }

        args->client_fd = client_fd;
        args->uart_fd = uart_fd;

        pthread_t thread1, thread2;

        if(pthread_create(&thread1, NULL, tcp_to_uart, (void*)args) != 0) {
            perror("Failed to create tcp_to_uart thread");
        }

        if(pthread_create(&thread2, NULL, uart_to_tcp, (void*)args) != 0) {
            perror("Failed to create uart_to_tcp thread");
        }

        pthread_detach(thread1);
        pthread_detach(thread2); 
    }

    close(server_fd);
    close(uart_fd);

    return 0;
}

