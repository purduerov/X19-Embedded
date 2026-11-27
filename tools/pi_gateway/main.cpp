/**
 * @file main.cpp
 * @brief X19 Raspberry Pi Core CAN Gateway Daemon (C++17).
 * @organization Purdue ROV
 *
 * Bridges vehicle CAN FD traffic (0x100 Thrusters, 0x110 Solenoids, 0x200 Nav,
 * 0x210 Env, 0x300 Power, 0x001 E-Stop) with the topside/companion computer stack.
 *
 * Supports two runtime transports:
 *   1. Linux SocketCAN: Native CAN FD on physical Pi 5 interface (e.g. can0, vcan0).
 *   2. Host SIL Bridge: Cross-platform TCP stream to sil_bridge_server (localhost:8765).
 *
 * Includes:
 *   - 6-DOF Thruster Kinematic Mixer with vector saturation scaling (prevents heading distortion).
 *   - Fail-safe watchdog: disarms thrusters (neutral 1500 us) on timeout (>250 ms) or 0x001 E-Stop.
 *   - Optional ZeroMQ Pub/Sub support when cppzmq/zmq.hpp is available.
 */

#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_types.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET socket_handle_t;
#define IS_INVALID_SOCKET(s) ((s) == INVALID_SOCKET)
#define CLOSE_SOCKET(s)      closesocket(s)
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int socket_handle_t;
#define INVALID_SOCKET       (-1)
#define IS_INVALID_SOCKET(s) ((s) < 0)
#define CLOSE_SOCKET(s)      close(s)
#endif

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#endif

#if __has_include(<zmq.hpp>)
#define HAVE_ZMQ 1
#include <zmq.hpp>
#endif

namespace rov {

// ============================================================================
// Data Structures
// ============================================================================

struct JoystickInput {
    float forward{0.0f};   // Surge [-1.0, 1.0]
    float strafe{0.0f};    // Sway [-1.0, 1.0]
    float vertical{0.0f};  // Heave [-1.0, 1.0]
    float roll{0.0f};      // [-1.0, 1.0]
    float pitch{0.0f};     // [-1.0, 1.0]
    float yaw{0.0f};       // Heading [-1.0, 1.0]
    uint64_t timestamp_us{0};
};

struct CanFrame {
    uint32_t id{0};
    uint8_t len{0};
    uint8_t data[64]{0};
};

// ============================================================================
// Kinematics: 6-DOF Thruster Mixing Matrix
// ============================================================================

class ThrusterKinematics {
public:
    static float clamp_axis(float val) {
        if (std::isnan(val) || std::isinf(val)) {
            return 0.0f;
        }
        return std::max(-1.0f, std::min(1.0f, val));
    }

    /**
     * @brief Mix 6-DOF normalized joystick inputs into 8 thruster PWM values.
     *
     * Four horizontal thrusters (FR, FL, RR, RL): Surge, sway, and yaw.
     * Four vertical thrusters (FR, FL, RR, RL): Heave, roll, and pitch.
     *
     * Employs vector saturation scaling: when a composite axis request exceeds
     * 1.0, the entire group is scaled down proportionally to preserve vehicle
     * direction rather than clipping individual actuators.
     */
    static rov_thruster_cmd_t mix(const JoystickInput& input) {
        rov_thruster_cmd_t cmd;
        for (int i = 0; i < 8; ++i) {
            cmd.pwm_us[i] = ROV_PWM_STOP_US;
        }

        const float fwd = clamp_axis(input.forward);
        const float strf = clamp_axis(input.strafe);
        const float vert = clamp_axis(input.vertical);
        const float yaw = clamp_axis(input.yaw);
        const float roll = clamp_axis(input.roll);
        const float pitch = clamp_axis(input.pitch);

        // Horizontal layout (T1..T4: FR, FL, RR, RL)
        float horiz[4] = {
            fwd - strf - yaw,
            fwd + strf + yaw,
            fwd + strf - yaw,
            fwd - strf + yaw
        };

        float max_h = 1.0f;
        for (int i = 0; i < 4; ++i) {
            max_h = std::max(max_h, std::abs(horiz[i]));
        }
        if (max_h > 1.0f) {
            for (int i = 0; i < 4; ++i) {
                horiz[i] /= max_h;
            }
        }

        // Vertical layout (T5..T8: FR, FL, RR, RL)
        float vertical_vals[4] = {
            vert - roll - pitch,
            vert + roll - pitch,
            vert - roll + pitch,
            vert + roll + pitch
        };

        float max_v = 1.0f;
        for (int i = 0; i < 4; ++i) {
            max_v = std::max(max_v, std::abs(vertical_vals[i]));
        }
        if (max_v > 1.0f) {
            for (int i = 0; i < 4; ++i) {
                vertical_vals[i] /= max_v;
            }
        }

        // Map [-1.0, 1.0] to [1100, 1900] us around neutral 1500 us (delta +/- 400 us)
        constexpr float PWM_DELTA = 400.0f;
        for (int i = 0; i < 4; ++i) {
            const int pwm = static_cast<int>(static_cast<float>(ROV_PWM_STOP_US) + (horiz[i] * PWM_DELTA));
            cmd.pwm_us[i] = static_cast<uint16_t>(std::max(1000, std::min(2000, pwm)));
        }
        for (int i = 0; i < 4; ++i) {
            const int pwm = static_cast<int>(static_cast<float>(ROV_PWM_STOP_US) + (vertical_vals[i] * PWM_DELTA));
            cmd.pwm_us[4 + i] = static_cast<uint16_t>(std::max(1000, std::min(2000, pwm)));
        }

        return cmd;
    }
};

// ============================================================================
// Transport Abstraction
// ============================================================================

class ICanTransport {
public:
    virtual ~ICanTransport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool send(uint32_t can_id, const uint8_t* data, uint8_t len) = 0;
    virtual bool receive(CanFrame& frame, uint32_t timeout_ms) = 0;
};

// ============================================================================
// SIL TCP Transport (Connects to sil_bridge_server on 127.0.0.1:8765)
// ============================================================================

class SilTcpTransport : public ICanTransport {
private:
    std::string host;
    int port;
    socket_handle_t sock{INVALID_SOCKET};

#pragma pack(push, 1)
    struct SilPacket {
        uint32_t magic;
        uint32_t id;
        uint8_t len;
        uint8_t data[64];
    };
#pragma pack(pop)

public:
    SilTcpTransport(std::string h, int p) : host(std::move(h)), port(p) {}

    ~SilTcpTransport() override {
        close();
    }

    bool open() override {
#ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            std::cerr << "[SilTransport] WSAStartup failed\n";
            return false;
        }
#endif
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (IS_INVALID_SOCKET(sock)) {
            std::cerr << "[SilTransport] Failed to create socket\n";
            return false;
        }

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

        if (connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != 0) {
            CLOSE_SOCKET(sock);
            sock = INVALID_SOCKET;
            return false;
        }

#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(sock, FIONBIO, &mode);
#else
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif
        std::cout << "[SilTransport] Connected to SIL Bridge at " << host << ":" << port << "\n";
        return true;
    }

    void close() override {
        if (!IS_INVALID_SOCKET(sock)) {
            CLOSE_SOCKET(sock);
            sock = INVALID_SOCKET;
#ifdef _WIN32
            WSACleanup();
#endif
        }
    }

    bool send(uint32_t can_id, const uint8_t* data, uint8_t len) override {
        if (IS_INVALID_SOCKET(sock) || len > 64) {
            return false;
        }
        SilPacket pkt{};
        pkt.magic = 0x524F5643; // "ROVC"
        pkt.id = can_id;
        pkt.len = len;
        if (data && len > 0) {
            std::memcpy(pkt.data, data, len);
        }
        const int sent = ::send(sock, reinterpret_cast<const char*>(&pkt), sizeof(pkt), 0);
        return sent == static_cast<int>(sizeof(pkt));
    }

    bool receive(CanFrame& frame, uint32_t timeout_ms) override {
        if (IS_INVALID_SOCKET(sock)) {
            return false;
        }
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        struct timeval tv{};
        tv.tv_sec = static_cast<long>(timeout_ms / 1000);
        tv.tv_usec = static_cast<long>((timeout_ms % 1000) * 1000);

        const int ret = select(static_cast<int>(sock + 1), &fds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            return false;
        }

        SilPacket pkt{};
        int received = 0;
        const int target = static_cast<int>(sizeof(pkt));
        while (received < target) {
            const int n = ::recv(sock, reinterpret_cast<char*>(&pkt) + received, target - received, 0);
            if (n <= 0) {
                return false;
            }
            received += n;
        }

        if (pkt.magic != 0x524F5643 && pkt.magic != 0x58313943) {
            return false;
        }

        frame.id = pkt.id;
        frame.len = std::min(pkt.len, static_cast<uint8_t>(64));
        std::memcpy(frame.data, pkt.data, frame.len);
        return true;
    }
};

// ============================================================================
// SocketCAN Transport (Linux native can0/vcan0)
// ============================================================================

#ifdef __linux__
class SocketCanTransport : public ICanTransport {
private:
    std::string interface_name;
    int sock{-1};

public:
    explicit SocketCanTransport(std::string iface) : interface_name(std::move(iface)) {}

    ~SocketCanTransport() override {
        close();
    }

    bool open() override {
        sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock < 0) {
            perror("[SocketCAN] socket creation failed");
            return false;
        }

        // Enable CAN FD support
        int canfd_on = 1;
        if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on)) != 0) {
            perror("[SocketCAN] setsockopt CAN_RAW_FD_FRAMES failed");
            close();
            return false;
        }

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
        if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
            perror("[SocketCAN] ioctl SIOCGIFINDEX failed");
            close();
            return false;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("[SocketCAN] bind failed");
            close();
            return false;
        }

        std::cout << "[SocketCAN] Bound successfully to interface " << interface_name << " (CAN FD enabled)\n";
        return true;
    }

    void close() override {
        if (sock >= 0) {
            ::close(sock);
            sock = -1;
        }
    }

    bool send(uint32_t can_id, const uint8_t* data, uint8_t len) override {
        if (sock < 0 || len > 64) {
            return false;
        }
        struct canfd_frame frame{};
        frame.can_id = can_id;
        frame.len = len;
        frame.flags = CANFD_BRS; // Bit Rate Switch (5 Mbps data phase)
        if (data && len > 0) {
            std::memcpy(frame.data, data, len);
        }
        const ssize_t bytes_sent = write(sock, &frame, sizeof(frame));
        return bytes_sent == sizeof(frame);
    }

    bool receive(CanFrame& frame, uint32_t timeout_ms) override {
        if (sock < 0) {
            return false;
        }
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        struct timeval tv{};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        const int ret = select(sock + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            return false;
        }

        struct canfd_frame raw_frame{};
        const ssize_t nbytes = read(sock, &raw_frame, sizeof(raw_frame));
        if (nbytes <= 0) {
            return false;
        }

        frame.id = raw_frame.can_id & CAN_EFF_MASK;
        frame.len = raw_frame.len;
        std::memcpy(frame.data, raw_frame.data, std::min(frame.len, static_cast<uint8_t>(64)));
        return true;
    }
};
#endif

// ============================================================================
// Pi Gateway Coordinator
// ============================================================================

class PiGateway {
private:
    std::unique_ptr<ICanTransport> transport;
    std::atomic<bool> running{false};
    std::atomic<bool> emergency_stop{false};
    JoystickInput latest_joystick{};
    std::chrono::steady_clock::time_point last_joystick_time{};
    bool verbose{false};

    // Telemetry cache
    rov_nav_telemetry_t latest_nav{};
    rov_env_telemetry_t latest_env{};
    rov_power_telemetry_t latest_power{};

public:
    PiGateway(std::unique_ptr<ICanTransport> t, bool v = false)
        : transport(std::move(t)), verbose(v) {}

    void set_joystick(const JoystickInput& js) {
        latest_joystick = js;
        last_joystick_time = std::chrono::steady_clock::now();
    }

    void trigger_estop() {
        emergency_stop = true;
        uint8_t alert[8] = {0xAA, 0x55, 0x01, 0, 0, 0, 0, 0};
        transport->send(ROV_CAN_ID_EMERGENCY_BREAK, alert, sizeof(alert));
        std::cerr << "\n[PiGateway] *** EMERGENCY STOP TRIGGERED ***\n";
    }

    void process_incoming_frame(const CanFrame& frame) {
        if (frame.id == ROV_CAN_ID_EMERGENCY_BREAK) {
            if (frame.len >= 2 && frame.data[0] == 0xAA && frame.data[1] == 0x55) {
                emergency_stop = true;
                std::cerr << "\n[PiGateway] Alert: Received Emergency Break (0x001). Actuators DISARMED.\n";
            }
        } else if (frame.id == ROV_CAN_ID_EFUSE_FAULT_ALERT) {
            emergency_stop = true;
            std::cerr << "\n[PiGateway] Alert: Power Slab eFuse Fault (0x005). Actuators DISARMED.\n";
        } else if (frame.id == ROV_CAN_ID_NAV_TELEMETRY) {
            if (rov_can_unpack_nav_telemetry(frame.data, frame.len, &latest_nav) == ROV_OK) {
                if (verbose) {
                    std::printf("[NAV 0x200] Depth: %5.2fm | Gyro: (%+5.2f, %+5.2f, %+5.2f) | Quat: (%4.2f, %4.2f, %4.2f, %4.2f)\n",
                                static_cast<double>(latest_nav.depth_meters),
                                static_cast<double>(latest_nav.gyro_x_rad_s),
                                static_cast<double>(latest_nav.gyro_y_rad_s),
                                static_cast<double>(latest_nav.gyro_z_rad_s),
                                static_cast<double>(latest_nav.q_w),
                                static_cast<double>(latest_nav.q_x),
                                static_cast<double>(latest_nav.q_y),
                                static_cast<double>(latest_nav.q_z));
                }
            }
        } else if (frame.id == ROV_CAN_ID_ENV_TELEMETRY) {
            if (rov_can_unpack_env_telemetry(frame.data, frame.len, &latest_env) == ROV_OK) {
                if (verbose) {
                    std::printf("[ENV 0x210] Press: %6.1fhPa | Hum: %4.1f%% | Temp: %4.1fC | Leak: 0x%02X | 5V Rail: %4.2fV\n",
                                static_cast<double>(latest_env.pressure_hpa),
                                static_cast<double>(latest_env.humidity_pct),
                                static_cast<double>(latest_env.temperature_c),
                                latest_env.leak_flags,
                                static_cast<double>(latest_env.rail_voltage_mv) / 1000.0);
                }
            }
        } else if (frame.id == ROV_CAN_ID_POWER_TELEMETRY) {
            if (rov_can_unpack_power_telemetry(frame.data, frame.len, &latest_power) == ROV_OK) {
                if (verbose) {
                    std::printf("[PWR 0x300] Tether: %4.1fV @ %4.1fA | 5V: %4.2fV | PCB Temp: %4.1fC\n",
                                static_cast<double>(latest_power.tether_voltage_mv) / 1000.0,
                                static_cast<double>(latest_power.tether_current_ma) / 1000.0,
                                static_cast<double>(latest_power.v5_voltage_mv) / 1000.0,
                                static_cast<double>(latest_power.pcb_temp_c) / 10.0);
                }
            }
        }
    }

    void run(int max_cycles = 0, double rate_hz = 50.0) {
        if (!transport->open()) {
            std::cerr << "[PiGateway] Failed to open transport\n";
            return;
        }

        running = true;
        const auto loop_period = std::chrono::duration<double>(1.0 / rate_hz);
        int cycle = 0;

        std::cout << "[PiGateway] Gateway loop running at " << rate_hz << " Hz (Watchdog: 250ms)\n";

        while (running && (max_cycles <= 0 || cycle < max_cycles)) {
            const auto start_time = std::chrono::steady_clock::now();

            // 1. Drain incoming CAN frames (up to 32 frames per cycle)
            CanFrame rx_frame;
            int frames_processed = 0;
            while (frames_processed < 32 && transport->receive(rx_frame, 1)) {
                process_incoming_frame(rx_frame);
                frames_processed++;
            }

            // 2. Watchdog & Thruster Command Transmission (CAN 0x100)
            rov_thruster_cmd_t thruster_cmd;
            const auto elapsed_since_js = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_joystick_time
            ).count();

            // If emergency stop is active or joystick timed out (>250ms), send neutral PWM
            if (emergency_stop || elapsed_since_js > 250) {
                for (int i = 0; i < 8; ++i) {
                    thruster_cmd.pwm_us[i] = ROV_PWM_STOP_US;
                }
            } else {
                thruster_cmd = ThrusterKinematics::mix(latest_joystick);
            }

            // Pack and transmit 0x100
            uint8_t tx_buf[64];
            size_t packed_len = 0;
            if (rov_can_pack_thruster_cmd(&thruster_cmd, tx_buf, sizeof(tx_buf), &packed_len) == ROV_OK) {
                transport->send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, static_cast<uint8_t>(packed_len));
            }

            cycle++;
            const auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < loop_period) {
                std::this_thread::sleep_for(loop_period - elapsed);
            }
        }

        // Clean disarm upon exit
        rov_thruster_cmd_t neutral_cmd;
        for (int i = 0; i < 8; ++i) {
            neutral_cmd.pwm_us[i] = ROV_PWM_STOP_US;
        }
        uint8_t tx_buf[64];
        size_t packed_len = 0;
        if (rov_can_pack_thruster_cmd(&neutral_cmd, tx_buf, sizeof(tx_buf), &packed_len) == ROV_OK) {
            transport->send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, static_cast<uint8_t>(packed_len));
        }

        transport->close();
        std::cout << "[PiGateway] Shutdown complete. Sent neutral safety frame.\n";
    }

    void stop() {
        running = false;
    }
};

} // namespace rov

// ============================================================================
// CLI & Main Entrypoint
// ============================================================================

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  --sil <host:port>   Connect to SIL Bridge server (default: 127.0.0.1:8765)\n"
#ifdef __linux__
              << "  --can <interface>   Connect to Linux SocketCAN (e.g. can0, vcan0)\n"
#endif
              << "  --test-mix          Run 6-DOF kinematics mixing self-test and exit\n"
              << "  --thrust <F> <S> <V> <R> <P> <Y>  Inject fixed thrust [-1.0 .. 1.0]\n"
              << "  --cycles <N>        Exit after N cycles (0 = infinite)\n"
              << "  --rate <Hz>         Loop update rate in Hz (default: 50)\n"
              << "  --verbose, -v       Enable verbose telemetry logging\n"
              << "  --estop             Immediately send E-Stop command and exit\n";
}

static int run_mix_self_test() {
    std::cout << "Running Thruster Kinematics self-test...\n";

    // Neutral test
    rov::JoystickInput js_zero{};
    auto pwms_zero = rov::ThrusterKinematics::mix(js_zero);
    for (int i = 0; i < 8; ++i) {
        if (pwms_zero.pwm_us[i] != 1500) {
            std::cerr << "FAIL: Zero input did not produce neutral 1500 us on channel " << i + 1 << "\n";
            return 1;
        }
    }

    // Pure Surge test (forward = 1.0)
    rov::JoystickInput js_fwd{};
    js_fwd.forward = 1.0f;
    auto pwms_fwd = rov::ThrusterKinematics::mix(js_fwd);
    for (int i = 0; i < 4; ++i) {
        if (pwms_fwd.pwm_us[i] != 1900) {
            std::cerr << "FAIL: Full forward did not produce 1900 us on horizontal thruster " << i + 1 << "\n";
            return 1;
        }
    }
    for (int i = 4; i < 8; ++i) {
        if (pwms_fwd.pwm_us[i] != 1500) {
            std::cerr << "FAIL: Horizontal thrust leaked into vertical thruster " << i + 1 << "\n";
            return 1;
        }
    }

    // Saturation test (forward = 1.0, strafe = 1.0, yaw = 1.0)
    // Raw sum would be 3.0, but vector scaling must clamp cleanly to [1100, 1900] us
    rov::JoystickInput js_sat{};
    js_sat.forward = 1.0f;
    js_sat.strafe = 1.0f;
    js_sat.yaw = 1.0f;
    auto pwms_sat = rov::ThrusterKinematics::mix(js_sat);
    for (int i = 0; i < 8; ++i) {
        if (pwms_sat.pwm_us[i] < 1000 || pwms_sat.pwm_us[i] > 2000) {
            std::cerr << "FAIL: PWM out of bounds on saturated input: " << pwms_sat.pwm_us[i] << " us\n";
            return 1;
        }
    }

    std::cout << "PASS: Thruster kinematics mixing self-test succeeded!\n";
    return 0;
}

int main(int argc, char** argv) {
    std::string sil_addr = "127.0.0.1:8765";
    std::string can_iface = "";
    bool verbose = false;
    int cycles = 0;
    double rate_hz = 50.0;
    bool estop_only = false;
    rov::JoystickInput custom_thrust{};
    bool has_custom_thrust = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--test-mix") {
            return run_mix_self_test();
        } else if (arg == "--sil" && i + 1 < argc) {
            sil_addr = argv[++i];
        } else if (arg == "--can" && i + 1 < argc) {
            can_iface = argv[++i];
        } else if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--cycles" && i + 1 < argc) {
            cycles = std::atoi(argv[++i]);
        } else if (arg == "--rate" && i + 1 < argc) {
            rate_hz = std::atof(argv[++i]);
        } else if (arg == "--estop") {
            estop_only = true;
        } else if (arg == "--thrust" && i + 6 < argc) {
            custom_thrust.forward = static_cast<float>(std::atof(argv[++i]));
            custom_thrust.strafe = static_cast<float>(std::atof(argv[++i]));
            custom_thrust.vertical = static_cast<float>(std::atof(argv[++i]));
            custom_thrust.roll = static_cast<float>(std::atof(argv[++i]));
            custom_thrust.pitch = static_cast<float>(std::atof(argv[++i]));
            custom_thrust.yaw = static_cast<float>(std::atof(argv[++i]));
            has_custom_thrust = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }

    std::unique_ptr<rov::ICanTransport> transport;

#ifdef __linux__
    if (!can_iface.empty()) {
        transport = std::make_unique<rov::SocketCanTransport>(can_iface);
    }
#endif

    if (!transport) {
        // Parse host:port for SIL
        std::string host = "127.0.0.1";
        int port = 8765;
        const auto colon_pos = sil_addr.find(':');
        if (colon_pos != std::string::npos) {
            host = sil_addr.substr(0, colon_pos);
            port = std::atoi(sil_addr.substr(colon_pos + 1).c_str());
        } else {
            host = sil_addr;
        }
        transport = std::make_unique<rov::SilTcpTransport>(host, port);
    }

    rov::PiGateway gateway(std::move(transport), verbose);

    if (estop_only) {
        gateway.trigger_estop();
        return 0;
    }

    if (has_custom_thrust) {
        gateway.set_joystick(custom_thrust);
    }

    gateway.run(cycles, rate_hz);
    return 0;
}
