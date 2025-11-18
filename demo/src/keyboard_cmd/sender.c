// Compilar: gcc -O2 -Wall -o cmd cmd.c
// Uso:      ./cmd 127.0.0.1 9000
// Escribe:  #WL:1;   #XY:10,20,30;   #R;

#define _POSIX_C_SOURCE 200112L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Uso: %s <host> <puerto>\n", argv[0]);
        return 1;
    }
    const char *host = argv[1], *port = argv[2];

    struct addrinfo hints = {0}, *res = NULL;
    hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
    int rc = getaddrinfo(host, port, &hints, &res);
    if (rc != 0) { fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rc)); return 1; }

    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) { perror("socket"); freeaddrinfo(res); return 1; }

    if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) { perror("connect"); close(fd); freeaddrinfo(res); return 1; }
    freeaddrinfo(res);

    printf("Conectado a %s:%s\n", host, port);
    printf("Escribe comandos (#WL:1;  #XY:10,20,30;  #R;). Ctrl+D para salir.\n");

    char *line = NULL; size_t cap = 0; ssize_t n;
    while ((n = getline(&line, &cap, stdin)) != -1) {
        ssize_t sent = 0;
        while (sent < n) {
            ssize_t m = send(fd, line + sent, (size_t)(n - sent), 0);
            if (m < 0) { if (errno == EINTR) continue; perror("send"); close(fd); free(line); return 1; }
            sent += m;
        }
    }
    free(line);
    close(fd);
    return 0;
}
