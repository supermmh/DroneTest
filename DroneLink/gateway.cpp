#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <sys/mman.h>
#include <iomanip>

#pragma pack(push, 1)
struct CommControlFrame
{
    uint8_t head[2];
    uint8_t msg_id, ctrl_mode;
    float target_vx, target_vy, target_thrust, target_yaw;
    uint16_t crc16;
};
struct CommTelemetryFrame
{
    uint8_t head[2];
    uint8_t msg_id, _pad;
    float roll, pitch, yaw, pos_z;
    uint16_t crc16;
};
#pragma pack(pop)

std::atomic<CommControlFrame> g_latest_cmd;
std::atomic<uint64_t> g_last_udp_time_ms(0), g_last_fc_time_ms(0);
std::atomic<uint32_t> g_stat_udp_rx_cnt(0), g_stat_udp_tx_cnt(0);
std::atomic<uint32_t> g_stat_uart_rx_cnt(0), g_stat_uart_tx_cnt(0);

// === 🚀 终极物理探针：捕获原始总线字节 ===
std::atomic<uint32_t> g_stat_uart_raw_rx_bytes(0);

sockaddr_in g_pc_addr{};
bool g_pc_addr_valid = false;

uint64_t get_time_ms() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

uint16_t calc_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc & 0xFFFF;
}

void udp_rx_thread(int udp_sock)
{
    CommControlFrame frame;
    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    while (true)
    {
        if (recvfrom(udp_sock, &frame, sizeof(frame), MSG_DONTWAIT, (sockaddr *)&client_addr, &addr_len) > 0)
        {
            if (frame.head[0] == 0xAA && frame.head[1] == 0x55 && calc_crc16((uint8_t *)&frame.msg_id, 18) == frame.crc16)
            {
                g_latest_cmd.store(frame, std::memory_order_relaxed);
                g_last_udp_time_ms.store(get_time_ms(), std::memory_order_release);
                g_stat_udp_rx_cnt.fetch_add(1, std::memory_order_relaxed);

                // 🌟 核心修复 1：移除只赋值一次的限制！
                // 只要收到心跳包，就更新 PC 的地址和端口。
                // 这样 PC 断开后重新连接，底层 Socket 发送目标能实时更新。
                g_pc_addr = client_addr;
                g_pc_addr.sin_port = htons(8889);
                g_pc_addr_valid = true;
            }
        }
        else
        {
            usleep(100);
        }
    }
}

void uart_tx_thread(int uart_fd)
{
    while (true)
    {
        auto start = std::chrono::steady_clock::now();
        CommControlFrame cmd = g_latest_cmd.load(std::memory_order_relaxed);

        // 这里的 150ms 失控判定可以保持不变，它代表在 500Hz 下丢失了连续 75 帧后触发迫降，非常安全
        if (get_time_ms() - g_last_udp_time_ms.load(std::memory_order_acquire) > 150)
        {
            cmd.ctrl_mode = 0x02;
            cmd.target_vx = 0.0f;
            cmd.target_vy = 0.0f;
            cmd.target_thrust = 0.0f;
            cmd.target_yaw = 0.0f;
            cmd.crc16 = calc_crc16((uint8_t *)&cmd.msg_id, 18);
            g_latest_cmd.store(cmd, std::memory_order_relaxed);
        }

        if (write(uart_fd, &cmd, sizeof(cmd)) > 0)
            g_stat_uart_tx_cnt.fetch_add(1, std::memory_order_relaxed);

        // 🌟 将 5000 微秒改为 2000 微秒 (2ms = 500Hz)
        std::this_thread::sleep_until(start + std::chrono::microseconds(2000));
    }
}

void uart_rx_thread(int uart_fd, int udp_sock)
{
    uint8_t buf[256];
    uint8_t frame_buf[sizeof(CommTelemetryFrame)];
    int sync_state = 0, idx = 0;
    while (true)
    {
        int n = read(uart_fd, buf, sizeof(buf));
        if (n > 0)
        {
            // 【物理层探针】：哪怕是乱码，只要收到字节就涨计数
            g_stat_uart_raw_rx_bytes.fetch_add(n, std::memory_order_relaxed);

            for (int i = 0; i < n; i++)
            {
                uint8_t b = buf[i];
                if (sync_state == 0)
                {
                    if (b == 0xBB)
                    {
                        frame_buf[idx++] = b;
                        sync_state = 1;
                    }
                }
                else if (sync_state == 1)
                {
                    if (b == 0x66)
                    {
                        frame_buf[idx++] = b;
                        sync_state = 2;
                    }
                    else if (b == 0xBB)
                    {
                        idx = 1;
                    }
                    else
                    {
                        idx = 0;
                        sync_state = 0;
                    }
                }
                else if (sync_state == 2)
                {
                    frame_buf[idx++] = b;
                    if (idx == sizeof(CommTelemetryFrame))
                    {
                        CommTelemetryFrame *rx = (CommTelemetryFrame *)frame_buf;
                        if (calc_crc16((uint8_t *)&rx->msg_id, sizeof(CommTelemetryFrame) - 4) == rx->crc16)
                        {
                            g_stat_uart_rx_cnt.fetch_add(1, std::memory_order_relaxed);
                            g_last_fc_time_ms.store(get_time_ms(), std::memory_order_relaxed);

                            // 🌟 核心修复 2：拯救 Wi-Fi 驱动的防洪阀门！
                            // 只有在最近 100ms 内收到了 PC 的心跳包，才允许向外发送 UDP 数据。
                            if (g_pc_addr_valid && (get_time_ms() - g_last_udp_time_ms.load(std::memory_order_relaxed) < 100))
                            {
                                if (sendto(udp_sock, frame_buf, sizeof(CommTelemetryFrame), 0, (sockaddr *)&g_pc_addr, sizeof(g_pc_addr)) > 0)
                                {
                                    g_stat_udp_tx_cnt.fetch_add(1, std::memory_order_relaxed);
                                }
                            }
                        }
                        idx = 0;
                        sync_state = 0;
                    }
                }
            }
        }
        else
        {
            usleep(100);
        }
    }
}

void monitor_thread()
{
    uint32_t last_urx = 0, last_utx = 0, last_srx = 0, last_stx = 0, last_raw = 0;
    auto last_time = std::chrono::steady_clock::now();
    uint32_t urx_rate = 0, utx_rate = 0, srx_rate = 0, stx_rate = 0, raw_rate = 0;

    std::cout << "\033[2J\033[H"; // 首次清屏
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz 界面刷新率
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();

        uint32_t urx = g_stat_udp_rx_cnt.load(std::memory_order_relaxed);
        uint32_t utx = g_stat_udp_tx_cnt.load(std::memory_order_relaxed);
        uint32_t srx = g_stat_uart_rx_cnt.load(std::memory_order_relaxed);
        uint32_t stx = g_stat_uart_tx_cnt.load(std::memory_order_relaxed);
        uint32_t raw = g_stat_uart_raw_rx_bytes.load(std::memory_order_relaxed);

        // 速率统计依然按秒刷新，防止数值乱跳看不清
        if (dt >= 1.0)
        {
            urx_rate = (urx - last_urx) / dt;
            utx_rate = (utx - last_utx) / dt;
            srx_rate = (srx - last_srx) / dt;
            stx_rate = (stx - last_stx) / dt;
            raw_rate = (raw - last_raw) / dt;

            last_urx = urx;
            last_utx = utx;
            last_srx = srx;
            last_stx = stx;
            last_raw = raw;
            last_time = now;
        }

        bool pc_conn = (get_time_ms() - g_last_udp_time_ms.load(std::memory_order_relaxed)) < 200;
        bool fc_conn = (get_time_ms() - g_last_fc_time_ms.load(std::memory_order_relaxed)) < 250;

        // 获取来自 PC 的最新控制指令 (线程安全)
        CommControlFrame cmd = g_latest_cmd.load(std::memory_order_relaxed);

        // 使用 \033[H 将光标放回左上角覆盖输出，实现彻底无闪屏刷新
        std::cout << "\033[H================= Pi Gateway Diagnostics =================\n";
        std::cout << " [Network] PC UDP Link  : " << (pc_conn ? "\033[32mCONNECTED\033[0m   " : "\033[31mDISCONNECTED\033[0m") << "      \n";
        std::cout << " [Serial]  FC UART Link : " << (fc_conn ? "\033[32mCONNECTED\033[0m   " : "\033[31mDISCONNECTED\033[0m") << "      \n";
        std::cout << "----------------------------------------------------------\n";
        std::cout << " [Downlink] UDP RX(PC->Pi) : " << std::setw(5) << urx_rate << " Hz  (Target: 500)  \n";
        std::cout << " [Downlink] UART TX(Pi->FC): " << std::setw(5) << stx_rate << " Hz  (Target: 500)  \n";
        std::cout << "----------------------------------------------------------\n";

        // === 新增：打印 PC 端传下来的最新控制量 ===
        std::cout << " [PC Input] Mode  : " << (cmd.ctrl_mode == 0x01 ? "\033[32mNORMAL (0x01)\033[0m" : "\033[31mFAILSAFE (0x02)\033[0m") << "       \n";
        std::cout << std::fixed << std::setprecision(2) << std::showpos; // 强制显示加减号并对齐小数点
        std::cout << "            Thrust: " << std::setw(6) << cmd.target_thrust << "  |  Yaw : " << std::setw(6) << cmd.target_yaw << "   \n";
        std::cout << "            Pitch : " << std::setw(6) << cmd.target_vx << "  |  Roll: " << std::setw(6) << cmd.target_vy << "   \n";
        std::cout << std::noshowpos << std::setprecision(0); // 恢复流格式

        std::cout << "----------------------------------------------------------\n";
        std::cout << " [Uplink]   UART RX(FC->Pi): " << std::setw(5) << srx_rate << " Hz  (Valid Frames)\n";
        std::cout << " \033[33m[Uplink]   RAW UART BYTES : " << std::setw(5) << raw_rate << " B/s (Target: ~11000)\033[0m   \n";
        std::cout << " [Uplink]   UDP TX(Pi->PC) : " << std::setw(5) << utx_rate << " Hz  (Target: 500)  \n";
        std::cout << "==========================================================\n"
                  << std::flush;
    }
}

int main()
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(8888);
    addr.sin_addr.s_addr = INADDR_ANY;
    bind(udp_sock, (sockaddr *)&addr, sizeof(addr));
    int uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd < 0)
        return -1;
    struct termios tty;
    tcgetattr(uart_fd, &tty);
    cfmakeraw(&tty);
    cfsetspeed(&tty, B2000000);
    tty.c_cflag |= (CS8 | CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    tcsetattr(uart_fd, TCSANOW, &tty);
    struct serial_struct serial;
    ioctl(uart_fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(uart_fd, TIOCSSERIAL, &serial);

    CommControlFrame init_cmd = {{0xAA, 0x55}, 0x01, 0x02, 0, 0, 0, 0, 0};
    init_cmd.crc16 = calc_crc16((uint8_t *)&init_cmd.msg_id, 18);
    g_latest_cmd.store(init_cmd);

    std::thread t1(udp_rx_thread, udp_sock);
    std::thread t2(uart_tx_thread, uart_fd);
    std::thread t3(uart_rx_thread, uart_fd, udp_sock);
    std::thread t4(monitor_thread);
    sched_param param;
    param.sched_priority = 85;
    pthread_setschedparam(t2.native_handle(), SCHED_FIFO, &param);
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    return 0;
}