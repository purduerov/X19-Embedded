/**
 * @file sil_bridge_server.c
 * @brief Software-in-the-Loop (SIL) Cross-Platform CAN Bus TCP Bridge Server.
 * @organization Purdue ROV
 *
 * Runs the X19 subsea vehicle node state machines (Node 1 Pi Shield,
 * Node 2 Control Board, Node 3 Power Slab) at 100 Hz virtual or real-time clock.
 * Exposes a lightweight TCP framing protocol on 127.0.0.1:8765 allowing the
 * Raspberry Pi companion computer software (ZMQ bridges, Python scripts, or ROS2)
 * to transmit CAN ID 0x100 / 0x110 frames and receive 0x200 / 0x210 / 0x300 telemetry.
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"
#include "x19_can_protocol.h"
#include "x19_parameters.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  typedef SOCKET socket_t;
  #define IS_INVALID_SOCKET(s) ((s) == INVALID_SOCKET)
  #define CLOSE_SOCKET(s) closesocket(s)
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  typedef int socket_t;
  #define INVALID_SOCKET (-1)
  #define IS_INVALID_SOCKET(s) ((s) < 0)
  #define CLOSE_SOCKET(s) close(s)
#endif

/* Forward declarations of node lifecycle functions */
extern void node1_app_init(void);
extern void node1_app_step(void);

extern void node2_app_init(void);
extern void node2_app_step(void);

extern void node3_app_init(void);
extern void node3_app_step(void);

#define SIL_BRIDGE_DEFAULT_PORT 8765
#define SIL_MAGIC_HEADER        0x58313943 /* "X19C" in ASCII */

/* Frame packet: 4 bytes magic, 4 bytes id, 1 byte len, up to 64 bytes data */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t id;
    uint8_t  len;
    uint8_t  data[64];
} sil_can_packet_t;
#pragma pack(pop)

static void set_nonblocking(socket_t sock) {
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);
#else
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif
}

static void platform_sleep_ms(uint32_t ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

int main(int argc, char **argv) {
    int port = SIL_BRIDGE_DEFAULT_PORT;
    int max_cycles = 0; /* 0 = run indefinitely until client disconnects */

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            max_cycles = atoi(argv[++i]);
        }
    }

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "SIL Bridge: WSAStartup failed.\n");
        return 1;
    }
#endif

    socket_t server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (IS_INVALID_SOCKET(server_fd)) {
        fprintf(stderr, "SIL Bridge: Failed to create server socket.\n");
        return 1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons((uint16_t)port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        fprintf(stderr, "SIL Bridge: Failed to bind to 127.0.0.1:%d\n", port);
        CLOSE_SOCKET(server_fd);
        return 1;
    }

    if (listen(server_fd, 1) < 0) {
        fprintf(stderr, "SIL Bridge: Listen failed on port %d\n", port);
        CLOSE_SOCKET(server_fd);
        return 1;
    }

    printf("===============================================================\n");
    printf(" X19 Software-in-the-Loop (SIL) Multi-Node Bus Server Active\n");
    printf(" Listening for Pi Companion / Test Client on 127.0.0.1:%d\n", port);
    printf("===============================================================\n");
    fflush(stdout);

    /* Initialize Subsea Microcontroller Firmware Nodes */
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();

    mock_can_set_current_node(X19_NODE_PI_SHIELD);
    node1_app_init();

    mock_can_set_current_node(X19_NODE_CONTROL_BOARD);
    node2_app_init();

    mock_can_set_current_node(X19_NODE_POWER_SLAB);
    node3_app_init();

    /* Accept connection from Pi Core Python Bridge or Test Harness */
    struct sockaddr_in client_addr;
    int addrlen = sizeof(client_addr);
    socket_t client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &addrlen);
    if (IS_INVALID_SOCKET(client_fd)) {
        fprintf(stderr, "SIL Bridge: Client accept failed.\n");
        CLOSE_SOCKET(server_fd);
        return 1;
    }

    printf("SIL Bridge: Pi Companion Client connected from %s:%d\n",
           inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
    fflush(stdout);

    set_nonblocking(client_fd);

    uint32_t cycle_count = 0;
    uint8_t rx_stream_buf[sizeof(sil_can_packet_t) * 4];
    size_t rx_stream_len = 0;

    bool running = true;
    while (running) {
        /* 1. Advance discrete simulation clock by 10 ms (100 Hz tick) */
        mock_bsp_advance_time_ms(10);

        /* 2. Read any incoming CAN packets from Pi Core companion over TCP */
        int bytes_read = recv(client_fd, (char *)(rx_stream_buf + rx_stream_len),
                              (int)(sizeof(rx_stream_buf) - rx_stream_len), 0);
        if (bytes_read > 0) {
            rx_stream_len += (size_t)bytes_read;
            while (rx_stream_len >= sizeof(sil_can_packet_t)) {
                sil_can_packet_t *pkt = (sil_can_packet_t *)rx_stream_buf;
                if (pkt->magic == SIL_MAGIC_HEADER) {
                    /* Injected into CAN bus as if transmitted by Raspberry Pi 5 / Pi Shield */
                    mock_can_set_current_node(X19_NODE_PI_CORE);
                    can_send(pkt->id, pkt->data, pkt->len);
                    size_t consumed = sizeof(sil_can_packet_t);
                    memmove(rx_stream_buf, rx_stream_buf + consumed, rx_stream_len - consumed);
                    rx_stream_len -= consumed;
                } else {
                    /* Corrupted magic, advance 1 byte */
                    memmove(rx_stream_buf, rx_stream_buf + 1, rx_stream_len - 1);
                    rx_stream_len--;
                }
            }
        } else if (bytes_read == 0) {
            printf("SIL Bridge: Companion client closed connection.\n");
            break;
        }

        /* 3. Step Node 2 (Control Board): Process CAN commands, execute 1kHz ramp, stream 0x200 */
        mock_can_set_current_node(X19_NODE_CONTROL_BOARD);
        node2_app_step();

        /* 4. Step Node 1 (Pi Shield): Evaluate environmental sensors, stream 0x210, leak safety */
        mock_can_set_current_node(X19_NODE_PI_SHIELD);
        node1_app_step();

        /* 5. Step Node 3 (Power Slab): Evaluate PMBus converter telemetry, stream 0x300 */
        mock_can_set_current_node(X19_NODE_POWER_SLAB);
        node3_app_step();

        /* 6. Drain any frames addressed to or broadcast to Pi Core and forward over TCP */
        mock_can_set_current_node(X19_NODE_PI_CORE);
        uint32_t rx_id;
        uint8_t rx_data[64];
        uint8_t rx_len;
        while (can_receive(&rx_id, rx_data, &rx_len)) {
            sil_can_packet_t tx_pkt;
            tx_pkt.magic = SIL_MAGIC_HEADER;
            tx_pkt.id = rx_id;
            tx_pkt.len = rx_len;
            if (rx_len > 0) {
                memcpy(tx_pkt.data, rx_data, rx_len);
            }
            send(client_fd, (const char *)&tx_pkt, sizeof(sil_can_packet_t), 0);
        }

        cycle_count++;
        if (max_cycles > 0 && (int)cycle_count >= max_cycles) {
            break;
        }

        /* Sleep 10 ms to throttle host CPU to true 100 Hz wall clock */
        platform_sleep_ms(10);
    }

    printf("SIL Bridge: Simulation ended after %u cycles (PWM0: %u us, PWM7: %u us).\n",
           cycle_count, mock_bsp_get_pwm_us(0), mock_bsp_get_pwm_us(7));

    CLOSE_SOCKET(client_fd);
    CLOSE_SOCKET(server_fd);

#ifdef _WIN32
    WSACleanup();
#endif

    return 0;
}
