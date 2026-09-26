/**
 * @file sil_bridge_server.c
 * @brief Software-in-the-Loop (SIL) Cross-Platform CAN Bus TCP Bridge Server.
 * @organization Purdue ROV
 *
 * Runs the X19 subsea vehicle node state machines (Node 1 Pi Shield,
 * Node 2 Control Board, Node 3 Power Slab) at 100 Hz virtual or real-time clock.
 * Exposes a lightweight TCP framing protocol on 127.0.0.1:8765 allowing the
 * Raspberry Pi companion computer software (ZMQ bridges or Python scripts)
 * to transmit CAN ID 0x100 / 0x110 frames and receive 0x200 / 0x210 / 0x300 telemetry.
 *
 * Automatically keeps the simulation running and accepts new client connections
 * when a client reconnects or a dashboard refreshes.
 *
 * CLI flags:
 *   --port <N>            Listen on port N instead of default 8765.
 *   --cycles <N>          Exit after N simulation cycles (0 = run indefinitely).
 *   --fast-forward <N>    Run N cycles as fast as possible (no real-time sleep).
 *                         Physics engine is enabled. Useful for CI burn-in.
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_physics.h"
#include "mocks/mock_sensors.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET socket_t;
#define IS_INVALID_SOCKET(s) ((s) == INVALID_SOCKET)
#define CLOSE_SOCKET(s)      closesocket(s)
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int socket_t;
#define INVALID_SOCKET       (-1)
#define IS_INVALID_SOCKET(s) ((s) < 0)
#define CLOSE_SOCKET(s)      close(s)
#endif

/* Forward declarations of node lifecycle functions */
extern void node1_app_init(void);
extern void node1_app_step(void);

extern void node2_app_init(void);
extern void node2_app_step(void);

extern void node3_app_init(void);
extern void node3_app_step(void);

#define SIL_BRIDGE_DEFAULT_PORT 8765
#define SIL_CAN_ID_OUTPUT_STATUS 0x7FEU /* SIL-only snapshot of mocked BSP outputs */
#define SIL_MAGIC_HEADER_LEGACY 0x58313943 /* "X19C" in ASCII */
#define SIL_MAGIC_HEADER        0x524F5643 /* "ROVC" in ASCII */

/* Frame packet: 4 bytes magic, 4 bytes id, 1 byte len, up to 64 bytes data */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
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

static bool send_sil_frame(socket_t sock, uint32_t id, const uint8_t *data, uint8_t len) {
    if (len > 64U) {
        return false;
    }

    sil_can_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.magic = SIL_MAGIC_HEADER;
    packet.id = id;
    packet.len = len;
    if (len > 0U && data != NULL) {
        memcpy(packet.data, data, len);
    }
    return send(sock, (const char *)&packet, (int)sizeof(packet), 0) == (int)sizeof(packet);
}

int main(int argc, char **argv) {
    int port = SIL_BRIDGE_DEFAULT_PORT;
    int max_cycles = 0;   /* 0 = run indefinitely */
    int fast_forward = 0; /* 0 = disabled; > 0 = run N cycles headless and exit */

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            max_cycles = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--fast-forward") == 0 && i + 1 < argc) {
            fast_forward = atoi(argv[++i]);
        }
    }

    /* --fast-forward mode: headless, no TCP, full physics, no real-time sleep */
    if (fast_forward > 0) {
        printf("================================================================\n");
        printf(" X19 SIL Fast-Forward Mode: %d cycles (no TCP, no real-time)\n", fast_forward);
        printf("================================================================\n");
        fflush(stdout);

        mock_bsp_reset();
        mock_bsp_set_auto_advance_delay(false);
        mock_can_reset();
        mock_sensors_reset();
        mock_physics_reset();
        mock_physics_set_enabled(true);

        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_init();
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_init();
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_init();

        for (int ff = 0; ff < fast_forward; ff++) {
            mock_bsp_advance_time_ms(10);
            mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
            node2_app_step();
            mock_can_set_current_node(ROV_NODE_PI_SHIELD);
            node1_app_step();
            mock_can_set_current_node(ROV_NODE_POWER_SLAB);
            node3_app_step();

            if ((ff + 1) % 10000 == 0) {
                printf("[FF] Cycle %d / %d | SimTime=%u ms | PWMs: [%u, %u, %u, %u, %u, %u, %u, %u]\n", ff + 1,
                       fast_forward, time_get_ms(), mock_bsp_get_pwm_us(0), mock_bsp_get_pwm_us(1),
                       mock_bsp_get_pwm_us(2), mock_bsp_get_pwm_us(3), mock_bsp_get_pwm_us(4), mock_bsp_get_pwm_us(5),
                       mock_bsp_get_pwm_us(6), mock_bsp_get_pwm_us(7));
                fflush(stdout);
            }
        }

        printf("[FF] Done: %d cycles in %u ms virtual time. TX frames: %u\n", fast_forward, time_get_ms(),
               mock_can_get_tx_count());
        return 0;
    }

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "SIL Bridge: WSAStartup failed.\n");
        return 1;
    }
#else
    /* A departing client must not take the simulator down with it.
     *
     * The observable failure this prevents: the engine streams 0x7FE and 0x200
     * at 100 Hz, so any client that stops reading accumulates a backlog in its
     * receive buffer.  If such a client then goes away, the kernel sends RST
     * rather than FIN, and on Linux the next send() to that peer fails *and*
     * raises SIGPIPE, whose default action is to terminate this process.  The
     * engine therefore died on an ordinary client disconnect -- a browser tab
     * closed mid-session, or a dashboard hitting "Restart Engine" -- and because
     * the listener went with it, the client's next connect() was refused with
     * ECONNREFUSED.  The disconnect handling further down (a send() returning
     * <= 0 closes client_fd and keeps serving) is already correct; it simply
     * never got to run.
     *
     * Ignoring the signal process-wide is preferred over MSG_NOSIGNAL on the two
     * send() calls because those are not the only descriptors at risk: if stdout
     * is ever a pipe whose reader has gone, a plain printf() would kill the
     * engine the same way.  Windows has no SIGPIPE at all -- send() there just
     * returns SOCKET_ERROR/WSAECONNRESET -- which is why this defect only ever
     * appeared on the Linux CI runner.
     */
    signal(SIGPIPE, SIG_IGN);
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

    if (listen(server_fd, 5) < 0) {
        fprintf(stderr, "SIL Bridge: Listen failed on port %d\n", port);
        CLOSE_SOCKET(server_fd);
        return 1;
    }

    set_nonblocking(server_fd);

    printf("===============================================================\n");
    printf(" X19 Software-in-the-Loop (SIL) Multi-Node Bus Server Active\n");
    printf(" Listening for Dashboard / Pi Companion on 127.0.0.1:%d\n", port);
    printf("===============================================================\n");
    fflush(stdout);

    /* Initialize Subsea Microcontroller Firmware Nodes */
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true); /* Run 6-DOF physics plant model in real-time mode */

    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
    node1_app_init();

    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_init();

    mock_can_set_current_node(ROV_NODE_POWER_SLAB);
    node3_app_init();

    socket_t client_fd = INVALID_SOCKET;
    uint32_t cycle_count = 0;
    uint8_t rx_stream_buf[sizeof(sil_can_packet_t) * 4];
    size_t rx_stream_len = 0;

    static uint32_t s_last_tx_env = 0;
    static uint32_t s_last_tx_pwr = 0;
    static bool s_env_initial_sent = false;
    static bool s_pwr_initial_sent = false;

    bool running = true;
    while (running) {
        /* Check for new client connections */
        struct sockaddr_in client_addr;
#ifdef _WIN32
        int addrlen = sizeof(client_addr);
#else
        socklen_t addrlen = sizeof(client_addr);
#endif
        socket_t new_client = accept(server_fd, (struct sockaddr *)&client_addr, &addrlen);
        if (!IS_INVALID_SOCKET(new_client)) {
            if (!IS_INVALID_SOCKET(client_fd)) {
                CLOSE_SOCKET(client_fd);
            }
            client_fd = new_client;
            set_nonblocking(client_fd);
            rx_stream_len = 0;
            s_env_initial_sent = false;
            s_pwr_initial_sent = false;
            printf("SIL Bridge: Client connected from %s:%d\n", inet_ntoa(client_addr.sin_addr),
                   ntohs(client_addr.sin_port));
            fflush(stdout);
        }

        /* 1. Advance discrete simulation clock by 10 ms (100 Hz tick) */
        mock_bsp_advance_time_ms(10);

        /* 2. Read any incoming CAN packets from companion client */
        if (!IS_INVALID_SOCKET(client_fd)) {
            int bytes_read = recv(client_fd, (char *)(rx_stream_buf + rx_stream_len),
                                  (int)(sizeof(rx_stream_buf) - rx_stream_len), 0);
            if (bytes_read > 0) {
                rx_stream_len += (size_t)bytes_read;
                while (rx_stream_len >= sizeof(sil_can_packet_t)) {
                    sil_can_packet_t *pkt = (sil_can_packet_t *)rx_stream_buf;
                    if (pkt->magic == SIL_MAGIC_HEADER || pkt->magic == SIL_MAGIC_HEADER_LEGACY) {
                        /* Security: Validate payload length to prevent buffer over-read */
                        if (pkt->len > 64) {
                            printf("SIL Bridge: WARNING: Dropped malformed packet with length %u (exceeds maximum 64)\n", pkt->len);
                            fflush(stdout);
                            memmove(rx_stream_buf, rx_stream_buf + 1, rx_stream_len - 1);
                            rx_stream_len--;
                            continue;
                        }

                        mock_can_set_current_node(ROV_NODE_PI_CORE);
                        can_send(pkt->id, pkt->data, pkt->len);

                        /* Log C-level execution for real-time verification */
                        if (pkt->id == ROV_CAN_ID_THRUSTER_CMD && pkt->len >= 16) {
                            const uint16_t *pwms = (const uint16_t *)pkt->data;
                            printf("[C STM32 CAN-RX] ID=0x%03X (THRUSTER_CMD) -> Targets: [%u, %u, %u, %u, %u, %u, %u, "
                                   "%u] us\n",
                                   pkt->id, pwms[0], pwms[1], pwms[2], pwms[3], pwms[4], pwms[5], pwms[6], pwms[7]);
                            fflush(stdout);
                        } else if (pkt->id == ROV_CAN_ID_SOLENOID_CMD && pkt->len >= 2) {
                            const uint16_t *mask = (const uint16_t *)pkt->data;
                            printf("[C STM32 CAN-RX] ID=0x%03X (SOLENOID_CMD) -> Mask: 0x%04X\n", pkt->id, *mask);
                            fflush(stdout);
                        } else if (pkt->id == ROV_CAN_ID_EMERGENCY_BREAK) {
                            printf(
                                "[C STM32 CAN-RX] ID=0x%03X (EMERGENCY_BREAK) -> TIMx_BDTR Hardware Clamp Tripped!\n",
                                pkt->id);
                            fflush(stdout);
                        }

                        size_t consumed = sizeof(sil_can_packet_t);
                        memmove(rx_stream_buf, rx_stream_buf + consumed, rx_stream_len - consumed);
                        rx_stream_len -= consumed;
                    } else {
                        memmove(rx_stream_buf, rx_stream_buf + 1, rx_stream_len - 1);
                        rx_stream_len--;
                    }
                }
            } else if (bytes_read == 0) {
                /* Disconnected */
                CLOSE_SOCKET(client_fd);
                client_fd = INVALID_SOCKET;
                rx_stream_len = 0;
                printf("SIL Bridge: Client disconnected, waiting for reconnection...\n");
                fflush(stdout);
                if (max_cycles > 0) {
                    break;
                }
            }
        }

        /* 3. Step Node 2 (Control Board): Process CAN commands, execute 1kHz ramp, stream 0x200 */
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();

        /* 4. Step Node 1 (Pi Shield): Evaluate environmental sensors, stream 0x210, leak safety */
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();

        /* 5. Step Node 3 (Power Slab): Evaluate PMBus converter telemetry, stream 0x300 */
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_step();

        /* SIL-only feedback lets integration tests inspect actual mocked outputs. */
        if (!IS_INVALID_SOCKET(client_fd)) {
            uint8_t status[23];
            for (uint8_t channel = 0; channel < ROV_NUM_THRUSTERS; channel++) {
                uint16_t pwm = mock_bsp_get_pwm_us(channel);
                status[channel * 2U] = (uint8_t)(pwm & 0xFFU);
                status[channel * 2U + 1U] = (uint8_t)(pwm >> 8U);
            }
            status[16] = mock_bsp_is_emergency_brake_tripped() ? 1U : 0U;
            uint16_t solenoids = mock_bsp_get_solenoid_mask();
            status[17] = (uint8_t)(solenoids & 0xFFU);
            status[18] = (uint8_t)(solenoids >> 8U);
            uint32_t sim_time_ms = time_get_ms();
            status[19] = (uint8_t)(sim_time_ms & 0xFFU);
            status[20] = (uint8_t)((sim_time_ms >> 8U) & 0xFFU);
            status[21] = (uint8_t)((sim_time_ms >> 16U) & 0xFFU);
            status[22] = (uint8_t)(sim_time_ms >> 24U);
            if (!send_sil_frame(client_fd, SIL_CAN_ID_OUTPUT_STATUS, status, sizeof(status))) {
                CLOSE_SOCKET(client_fd);
                client_fd = INVALID_SOCKET;
            }
        }

        /* 6. Drain frames addressed to Pi Core and forward over TCP (rate-limited telemetry to avoid flooding) */
        mock_can_set_current_node(ROV_NODE_PI_CORE);
        uint32_t rx_id;
        uint8_t rx_data[64];
        uint8_t rx_len;
        uint32_t now_ms = time_get_ms();

        while (can_receive(&rx_id, rx_data, &rx_len)) {
            if (!IS_INVALID_SOCKET(client_fd)) {
                bool should_send = false;

                if (rx_id == ROV_CAN_ID_EMERGENCY_BREAK || rx_id == ROV_CAN_ID_EFUSE_FAULT_ALERT ||
                    rx_id == ROV_CAN_ID_NAV_TELEMETRY) {
                    /* Critical safety alerts and 100 Hz Nav telemetry: ALWAYS send immediately */
                    should_send = true;
                } else if (rx_id == ROV_CAN_ID_ENV_TELEMETRY) {
                    /* Enclosure telemetry: instant if leak, otherwise 1 Hz (initial frame instant) */
                    if ((rx_len >= 13 && rx_data[12] != 0) || !s_env_initial_sent || (now_ms - s_last_tx_env >= 1000)) {
                        s_last_tx_env = now_ms;
                        s_env_initial_sent = true;
                        should_send = true;
                    }
                } else if (rx_id == ROV_CAN_ID_POWER_TELEMETRY) {
                    /* Power telemetry: initial frame immediately, then 1 Hz */
                    if (!s_pwr_initial_sent || (now_ms - s_last_tx_pwr >= 1000)) {
                        s_last_tx_pwr = now_ms;
                        s_pwr_initial_sent = true;
                        should_send = true;
                    }
                } else {
                    /* Any command, ACK, or other frame: send immediately */
                    should_send = true;
                }

                if (should_send) {
                    sil_can_packet_t tx_pkt;
                    memset(&tx_pkt, 0, sizeof(tx_pkt));
                    tx_pkt.magic = SIL_MAGIC_HEADER;
                    tx_pkt.id = rx_id;
                    uint8_t copy_len = (rx_len > sizeof(tx_pkt.data)) ? (uint8_t)sizeof(tx_pkt.data) : rx_len;
                    tx_pkt.len = copy_len;
                    if (copy_len > 0) {
                        memcpy(tx_pkt.data, rx_data, copy_len);
                    }
                    int sent = send(client_fd, (const char *)&tx_pkt, sizeof(sil_can_packet_t), 0);
                    if (sent <= 0) {
                        CLOSE_SOCKET(client_fd);
                        client_fd = INVALID_SOCKET;
                    }
                }
            }
        }

        cycle_count++;
        if (cycle_count % 200 == 0) {
            printf("[C Engine Tick] SimTime=%u ms | Active TIMx PWMs: [%u, %u, %u, %u, %u, %u, %u, %u] us\n",
                   time_get_ms(), mock_bsp_get_pwm_us(0), mock_bsp_get_pwm_us(1), mock_bsp_get_pwm_us(2),
                   mock_bsp_get_pwm_us(3), mock_bsp_get_pwm_us(4), mock_bsp_get_pwm_us(5), mock_bsp_get_pwm_us(6),
                   mock_bsp_get_pwm_us(7));
            fflush(stdout);
        }
        if (max_cycles > 0 && (int)cycle_count >= max_cycles) {
            break;
        }

        /* Sleep 10 ms to throttle host CPU to true 100 Hz wall clock */
        platform_sleep_ms(10);
    }

    printf("SIL Bridge: Server exiting after %u cycles.\n", cycle_count);

    if (!IS_INVALID_SOCKET(client_fd)) {
        CLOSE_SOCKET(client_fd);
    }
    CLOSE_SOCKET(server_fd);

#ifdef _WIN32
    WSACleanup();
#endif

    return 0;
}
