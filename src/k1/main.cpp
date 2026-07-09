#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <glob.h>

// ==================== 1. 硬件通信与底层系统调用头文件 ====================
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

// ==================== 2. 机器视觉与 AI 推理框架头文件 ====================
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include "spacemit_ort_env.h"

const float PI = 3.1415926f;

// ==================== 串口与播报策略配置 ====================
static const char* UART_DEVICE = "/dev/ttyS10";
static const int UART_BAUD_FLAG = B9600;
static const int UART_REPEAT_TIMES = 1;
static const int UART_REPEAT_GAP_MS = 80;
static const int BAD_CONFIRM_FRAMES = 2;
static const int GOOD_CONFIRM_FRAMES = 10;
static const int SEND_MIN_GAP_MS = 1200;

static const float SCORE_TH = 0.45f;
static const float SHOULDER_TH = 10.0f;
static const float SHOULDER_SEVERE_TH = 18.0f;
static const float SLOUCH_TH = 25.0f;
static const float SLOUCH_SEVERE_TH = 40.0f;

// 调试时 true；正式提高帧率时可以改成 false
static const bool SHOW_WINDOW = true;

static const bool PERSON_LOST_ENABLE = false;
static const int PERSON_LOST_CONFIRM_FRAMES = 120;

// ==================== STM32 face 表情板串口配置 ====================
// face 板通过 USB-TTL 插到 K1 的 USB 口。通常识别为 /dev/ttyACM0 或 /dev/ttyUSB0。
// 运行时可手动指定：FACE_UART_DEVICE=/dev/ttyACM0 ./pose_detector
static const speed_t FACE_UART_BAUD = B115200;
static const bool FACE_ENABLE = true;
static const int FACE_SCAN_RETRY = 10;
static const int FACE_SCAN_GAP_MS = 500;
static const char* FACE_DEFAULT_EXPR = "loving";      // face 默认表情
static const int FACE_DEFAULT_AFTER_MS = 5000;          // 同一检测状态持续 5 秒后回默认表情

// ==================== K1 -> ASRPRO 语音协议 ====================
// 数据包格式：AA cmd 55 0D 0A
// 0x01 正常恢复
// 0x02 轻度驼背
// 0x03 轻度高低肩
// 0x04 明显驼背
// 0x05 明显高低肩
// 0x06 背部和肩膀同时异常
// 0x07 人离开/画面中无人，可选
//
// 环境温湿度扩展协议：
// ASRPRO -> K1：AB 20 55，表示用户语音请求“查询环境”。
// K1 -> ASRPRO：AA 20 温度整数 湿度整数 55 0D 0A。
// K1 -> ASRPRO：AA 21 55 0D 0A，表示温湿度读取失败。
static const uint8_t ENV_QUERY_HEAD = 0xAB;
static const uint8_t ENV_QUERY_CMD = 0x20;
static const uint8_t ENV_REPORT_CMD = 0x20;
static const uint8_t ENV_FAIL_CMD = 0x21;
static const int ENV_READ_TIMEOUT_MS = 8000;

enum class PostureState {
    Good,
    SlouchMild,
    SlouchSevere,
    ShoulderMild,
    ShoulderSevere,
    BothBad,
    PersonLost
};

// ---------------- 几何解剖学三点角度计算 ----------------
float angle3(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
    float r = atan2(c.y - b.y, c.x - b.x) - atan2(a.y - b.y, a.x - b.x);
    float d = fabs(r * 180.0f / PI);
    return d > 180.0f ? 360.0f - d : d;
}

// ---------------- 线程安全帧缓冲区：只保留最新帧 ----------------
class FrameBuffer {
public:
    cv::Mat frame;
    std::mutex mtx;
    std::condition_variable cv;
    bool ok = false;

    void push(const cv::Mat& f) {
        std::lock_guard<std::mutex> l(mtx);
        f.copyTo(frame);
        ok = true;
        cv.notify_one();
    }

    bool pop(cv::Mat& out) {
        std::unique_lock<std::mutex> l(mtx);
        cv.wait(l, [this] { return ok; });
        out = frame.clone();
        ok = false;
        return true;
    }
};

FrameBuffer gbuf;

// ---------------- V4L2 摄像头采集线程 ----------------
void cam() {
    cv::VideoCapture cap(20, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] 无法打开摄像头物理设备 /dev/video20\n";
        return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 15);

    cv::Mat f;
    while (true) {
        cap >> f;
        if (!f.empty()) {
            gbuf.push(f);
        }
    }
}

// ---------------- 串口初始化：9600, 8N1, 无流控, raw 模式 ----------------
int init_uart(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        std::cerr << "\n========================================================\n"
                  << "[UART ERROR] 无法打开串口设备: " << device << "\n"
                  << "[原因排查] 1. sudo chmod 666 " << device << "\n"
                  << "          2. 确认物理线接 R_UART0_TXD/RXD，不是 R_UART1\n"
                  << "          3. 确认没有其他程序占用该串口\n"
                  << "[errno] " << errno << ": " << std::strerror(errno) << "\n"
                  << "========================================================\n\n";
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        std::cerr << "[UART ERROR] tcgetattr 失败: " << std::strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    cfsetispeed(&options, UART_BAUD_FLAG);
    cfsetospeed(&options, UART_BAUD_FLAG);

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= (CLOCAL | CREAD);

#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | INPCK | ISTRIP);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        std::cerr << "[UART ERROR] tcsetattr 失败: " << std::strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    std::cout << "[UART OK] 已打开 " << device << "，参数：9600 8N1 raw，无流控\n";
    return fd;
}


const char* baud_to_text(speed_t baud) {
    if (baud == B9600) return "9600";
    if (baud == B115200) return "115200";
    return "unknown";
}

void add_glob_matches(const std::string& pattern, std::vector<std::string>& out, std::set<std::string>& seen) {
    glob_t g;
    std::memset(&g, 0, sizeof(g));
    if (glob(pattern.c_str(), 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; ++i) {
            std::string p = g.gl_pathv[i];
            if (seen.insert(p).second) {
                out.push_back(p);
            }
        }
    }
    globfree(&g);
}

std::vector<std::string> build_face_port_candidates() {
    std::vector<std::string> ports;
    std::set<std::string> seen;

    const char* env_port = std::getenv("FACE_UART_DEVICE");
    if (env_port && env_port[0] != '\0') {
        ports.push_back(env_port);
        seen.insert(env_port);
    }

    add_glob_matches("/dev/serial/by-id/*", ports, seen);
    add_glob_matches("/dev/ttyACM*", ports, seen);
    add_glob_matches("/dev/ttyUSB*", ports, seen);
    add_glob_matches("/dev/ttyCH*", ports, seen);

    return ports;
}

// face 表情板串口初始化：115200, 8N1, raw 模式
int init_face_one_port(const std::string& device, bool verbose_error = true) {
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        if (verbose_error) {
            std::cerr << "[FACE ERROR] 无法打开串口设备: " << device
                      << " errno=" << errno << " " << std::strerror(errno) << "\n";
        }
        return -1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        if (verbose_error) {
            std::cerr << "[FACE ERROR] tcgetattr 失败: " << std::strerror(errno) << "\n";
        }
        close(fd);
        return -1;
    }

    cfsetispeed(&options, FACE_UART_BAUD);
    cfsetospeed(&options, FACE_UART_BAUD);

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= (CLOCAL | CREAD);

#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | INPCK | ISTRIP);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 5;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        if (verbose_error) {
            std::cerr << "[FACE ERROR] tcsetattr 失败: " << std::strerror(errno) << "\n";
        }
        close(fd);
        return -1;
    }

    std::cout << "[FACE OK] " << device << " 已打开，参数：" << baud_to_text(FACE_UART_BAUD) << " 8N1 raw\n";
    return fd;
}

int init_face_serial(std::string& opened_port) {
    if (!FACE_ENABLE) return -1;

    for (int attempt = 1; attempt <= FACE_SCAN_RETRY; ++attempt) {
        std::vector<std::string> candidates = build_face_port_candidates();

        if (candidates.empty()) {
            std::cerr << "[FACE WARN] 第 " << attempt << "/" << FACE_SCAN_RETRY
                      << " 次扫描：没有发现 face USB 串口。\n";
        } else {
            std::cout << "[FACE SCAN] 第 " << attempt << "/" << FACE_SCAN_RETRY << " 次扫描候选串口: ";
            for (const auto& p : candidates) std::cout << p << " ";
            std::cout << "\n";
        }

        for (const auto& port : candidates) {
            int fd = init_face_one_port(port, true);
            if (fd >= 0) {
                opened_port = port;
                return fd;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(FACE_SCAN_GAP_MS));
    }

    std::cerr << "[FACE WARN] face 表情板串口仍未打开。\n"
              << "[FACE CHECK] 你的 face 板现在一般是 /dev/ttyACM0，请先执行：sudo chmod 666 /dev/ttyACM0\n"
              << "[FACE CHECK] 推荐运行：FACE_UART_DEVICE=/dev/ttyACM0 ./pose_detector\n";
    return -1;
}

bool write_all(int fd, const uint8_t* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(fd, data + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[UART WRITE ERROR] " << std::strerror(errno) << "\n";
            return false;
        }
        if (n == 0) {
            std::cerr << "[UART WRITE ERROR] write 返回 0，串口可能异常\n";
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    tcdrain(fd);
    return true;
}

bool send_voice_cmd(int voice_fd, uint8_t cmd_code, int repeat_times = UART_REPEAT_TIMES) {
    if (voice_fd < 0) {
        std::cerr << "[UART SKIP] 串口未打开，无法发送语音指令 0x"
                  << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(cmd_code) << std::dec << std::setfill(' ') << "\n";
        return false;
    }

    uint8_t packet[5] = {0xAA, cmd_code, 0x55, 0x0D, 0x0A};
    bool ok = true;

    for (int i = 0; i < repeat_times; ++i) {
        tcflush(voice_fd, TCIFLUSH);

        bool one_ok = write_all(voice_fd, packet, sizeof(packet));
        ok = ok && one_ok;

        std::cout << "[UART TX] AA "
                  << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(cmd_code)
                  << " 55 0D 0A  repeat=" << std::dec << (i + 1) << "/" << repeat_times
                  << (one_ok ? " OK" : " FAIL") << std::setfill(' ') << "\n";

        if (i + 1 < repeat_times) {
            std::this_thread::sleep_for(std::chrono::milliseconds(UART_REPEAT_GAP_MS));
        }
    }

    return ok;
}

bool send_env_report(int voice_fd, float temperature, float humidity) {
    if (voice_fd < 0) {
        std::cerr << "[ENV SKIP] 语音串口未打开，无法发送温湿度。\n";
        return false;
    }

    int temp_i = static_cast<int>(std::round(temperature));
    int hum_i = static_cast<int>(std::round(humidity));
    temp_i = std::max(0, std::min(99, temp_i));
    hum_i = std::max(0, std::min(99, hum_i));

    uint8_t packet[7] = {
        0xAA,
        ENV_REPORT_CMD,
        static_cast<uint8_t>(temp_i),
        static_cast<uint8_t>(hum_i),
        0x55,
        0x0D,
        0x0A
    };

    tcflush(voice_fd, TCIFLUSH);
    bool ok = write_all(voice_fd, packet, sizeof(packet));

    std::cout << "[ENV TX] AA 20 "
              << std::dec << temp_i << " " << hum_i
              << " 55 0D 0A  T=" << temperature << "C H=" << humidity << "% "
              << (ok ? "OK" : "FAIL") << "\n";
    return ok;
}

bool poll_env_query_from_asr(int voice_fd) {
    if (voice_fd < 0) return false;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(voice_fd, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int ready = select(voice_fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ready <= 0 || !FD_ISSET(voice_fd, &rfds)) {
        return false;
    }

    uint8_t buf[64];
    ssize_t n = read(voice_fd, buf, sizeof(buf));
    if (n <= 0) return false;

    static uint8_t state = 0;
    static uint8_t cmd = 0;

    for (ssize_t i = 0; i < n; ++i) {
        uint8_t b = buf[i];
        if (state == 0) {
            if (b == ENV_QUERY_HEAD) state = 1;
        } else if (state == 1) {
            cmd = b;
            state = 2;
        } else if (state == 2) {
            if (b == 0x55 && cmd == ENV_QUERY_CMD) {
                std::cout << "[ENV RX] 收到 ASRPRO 查询环境请求：AB 20 55\n";
                state = 0;
                return true;
            }
            state = 0;
        }
    }

    return false;
}


// ---------------- STM32 face 表情板：发送 ASCII 命令 + CRLF ----------------
bool send_face_cmd(int face_fd, const std::string& cmd) {
    if (face_fd < 0) {
        std::cerr << "[FACE SKIP] face 串口未打开，无法发送: " << cmd << "\n";
        return false;
    }

    std::string data = cmd + "\r\n";
    bool ok = write_all(face_fd, reinterpret_cast<const uint8_t*>(data.data()), data.size());
    std::cout << "[FACE TX] " << cmd << "\\r\\n " << (ok ? "OK" : "FAIL") << "\n";
    return ok;
}

struct EnvReading {
    float temperature = 0.0f;
    float humidity = 0.0f;
    bool ok = false;
};

bool parse_temp_hum_line(const std::string& line, float& temperature, float& humidity) {
    // STM32 face 资料中的 DHT11 数据格式通常是：T:25.3 H:60.2 或 T:25.3 H:60.2%
    // 这里做成宽松解析，只要一行里包含 T: 和 H: 就尽量提取后面的数字。
    size_t tpos = line.find("T:");
    size_t hpos = line.find("H:");
    if (tpos == std::string::npos || hpos == std::string::npos || hpos <= tpos) {
        return false;
    }

    try {
        std::string t_part = line.substr(tpos + 2, hpos - (tpos + 2));
        std::string h_part = line.substr(hpos + 2);
        temperature = std::stof(t_part);
        humidity = std::stof(h_part);
        return true;
    } catch (...) {
        return false;
    }
}

bool read_temperature_humidity_from_face(int face_fd, EnvReading& out) {
    if (face_fd < 0) {
        std::cerr << "[ENV ERROR] face 串口未打开，无法读取温湿度。\n";
        return false;
    }

    std::cout << "[ENV READ] 切换 STM32 face 到 sensor 模式，并打开 sensoron 自动采集...\n";
    tcflush(face_fd, TCIOFLUSH);

    // 重要：v2 固件里传感器默认 disabled。只发 sensor 可能只切屏，不一定开始采集。
    // 因此这里先 sensor 切到传感器显示，再 sensoron 打开自动检测，等待一轮 3 秒采样。
    send_face_cmd(face_fd, "sensor");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    send_face_cmd(face_fd, "sensoron");
    std::this_thread::sleep_for(std::chrono::milliseconds(3300));

    std::string line;
    bool saw_dht_error = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ENV_READ_TIMEOUT_MS);

    while (std::chrono::steady_clock::now() < deadline) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(face_fd, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;  // 200 ms

        int ready = select(face_fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ready <= 0 || !FD_ISSET(face_fd, &rfds)) {
            continue;
        }

        char buf[256];
        ssize_t n = read(face_fd, buf, sizeof(buf));
        if (n <= 0) {
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            char c = buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                if (!line.empty()) {
                    std::cout << "[FACE RX] " << line << "\n";
                    float t = 0.0f, h = 0.0f;
                    if (parse_temp_hum_line(line, t, h)) {
                        out.temperature = t;
                        out.humidity = h;
                        out.ok = true;
                        send_face_cmd(face_fd, "sensoroff");
                        send_face_cmd(face_fd, "face");
                        send_face_cmd(face_fd, FACE_DEFAULT_EXPR);
                        std::cout << "[ENV READ OK] T=" << t << "C H=" << h << "%\n";
                        return true;
                    }
                    if (line.find("DHT11 error") != std::string::npos) {
                        saw_dht_error = true;
                        std::cerr << "[ENV ERROR] STM32 返回 DHT11 error，可能是 DHT11 没接好或固件没读到传感器。\n";
                    }
                    line.clear();
                }
            } else {
                if (line.size() < 240) {
                    line.push_back(c);
                } else {
                    std::cout << "[FACE RX PARTIAL] " << line << "\n";
                    line.clear();
                }
            }
        }
    }

    if (!line.empty()) {
        std::cout << "[FACE RX PARTIAL] " << line << "\n";
    }

    send_face_cmd(face_fd, "sensoroff");
    send_face_cmd(face_fd, "face");
    send_face_cmd(face_fd, FACE_DEFAULT_EXPR);

    if (saw_dht_error) {
        std::cerr << "[ENV ERROR] 已收到 DHT11 error，没有有效温湿度。\n";
    } else {
        std::cerr << "[ENV ERROR] " << ENV_READ_TIMEOUT_MS << " ms 内没有读到 T:XX.X H:XX.X 数据。\n";
    }
    return false;
}

void query_environment_and_report(int voice_fd, int face_fd) {
    send_face_cmd(face_fd, "thinking");

    EnvReading env;
    bool ok = read_temperature_humidity_from_face(face_fd, env);
    if (ok) {
        send_env_report(voice_fd, env.temperature, env.humidity);
    } else {
        std::cerr << "[ENV REPORT] 温湿度读取失败，通知 ASRPRO 播报失败提示。\n";
        send_voice_cmd(voice_fd, ENV_FAIL_CMD);
    }
}

uint8_t state_to_cmd(PostureState state) {
    switch (state) {
        case PostureState::Good: return 0x01;
        case PostureState::SlouchMild: return 0x02;
        case PostureState::ShoulderMild: return 0x03;
        case PostureState::SlouchSevere: return 0x04;
        case PostureState::ShoulderSevere: return 0x05;
        case PostureState::BothBad: return 0x06;
        case PostureState::PersonLost: return 0x07;
    }
    return 0x01;
}

std::string state_to_en(PostureState state) {
    switch (state) {
        case PostureState::Good: return "Good";
        case PostureState::SlouchMild: return "SlouchMild";
        case PostureState::SlouchSevere: return "SlouchSevere";
        case PostureState::ShoulderMild: return "ShoulderMild";
        case PostureState::ShoulderSevere: return "ShoulderSevere";
        case PostureState::BothBad: return "BothBad";
        case PostureState::PersonLost: return "PersonLost";
    }
    return "Good";
}

std::string state_to_cn(PostureState state) {
    switch (state) {
        case PostureState::Good: return "正常";
        case PostureState::SlouchMild: return "轻度驼背";
        case PostureState::SlouchSevere: return "明显驼背";
        case PostureState::ShoulderMild: return "轻度高低肩";
        case PostureState::ShoulderSevere: return "明显高低肩";
        case PostureState::BothBad: return "背部和肩膀同时异常";
        case PostureState::PersonLost: return "画面中未检测到人";
    }
    return "正常";
}


std::string state_to_face_expr(PostureState state) {
    switch (state) {
        case PostureState::Good:
            return "happy";
        case PostureState::SlouchMild:
        case PostureState::SlouchSevere:
            return "sad";
        case PostureState::ShoulderMild:
        case PostureState::ShoulderSevere:
            return "angry";   // 高低肩：用 angry，替代原先的惊讶表情
        case PostureState::BothBad:
            return "angry";
        case PostureState::PersonLost:
            return "neutral";
    }
    return "happy";
}

// 状态联动：先改变 face 表情，再发送 ASRPRO 语音命令。
void send_robot_feedback(int voice_fd, int face_fd, PostureState state) {
    uint8_t voice_cmd = state_to_cmd(state);
    std::string face_expr = state_to_face_expr(state);

    std::cout << "[ROBOT FEEDBACK] 状态=" << state_to_cn(state)
              << " -> voice cmd=0x" << std::hex << std::uppercase
              << std::setw(2) << std::setfill('0') << static_cast<int>(voice_cmd)
              << std::dec << std::setfill(' ')
              << " , face=" << face_expr << "\n";

    send_face_cmd(face_fd, face_expr);
    send_voice_cmd(voice_fd, voice_cmd);
}

bool is_bad_state(PostureState state) {
    return state == PostureState::SlouchMild ||
           state == PostureState::SlouchSevere ||
           state == PostureState::ShoulderMild ||
           state == PostureState::ShoulderSevere ||
           state == PostureState::BothBad;
}

PostureState classify_posture(float raw_sh, float raw_ne) {
    bool slouch = raw_ne >= SLOUCH_TH;
    bool shoulder = raw_sh >= SHOULDER_TH;

    if (slouch && shoulder) {
        return PostureState::BothBad;
    }

    if (slouch) {
        return raw_ne >= SLOUCH_SEVERE_TH ? PostureState::SlouchSevere : PostureState::SlouchMild;
    }

    if (shoulder) {
        return raw_sh >= SHOULDER_SEVERE_TH ? PostureState::ShoulderSevere : PostureState::ShoulderMild;
    }

    return PostureState::Good;
}


PostureState key_to_state(int key) {
    switch (key) {
        case '1': return PostureState::Good;
        case '2': return PostureState::SlouchMild;
        case '3': return PostureState::ShoulderMild;
        case '4': return PostureState::SlouchSevere;
        case '5': return PostureState::ShoulderSevere;
        case '6': return PostureState::BothBad;
        case '7': return PostureState::PersonLost;
    }
    return PostureState::Good;
}

// -------------------------------- 主程序 --------------------------------
int main() {
    cv::setNumThreads(0);

    std::cout << "[INFO] === K1 YOLOv8-pose + ASRPRO 语音 + STM32 face + 温湿度联动版启动 ===\n";

    // ===== 1. 串口初始化 =====
    int voice_fd = init_uart(UART_DEVICE);

    std::string face_port;
    int face_fd = init_face_serial(face_port);
    if (face_fd >= 0) {
        // 进入手动表情模式，关闭传感器自动表情，避免覆盖 K1 下发的姿态表情。
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        send_face_cmd(face_fd, "face");
        send_face_cmd(face_fd, "sensoroff");
        send_face_cmd(face_fd, "loving");
        std::cout << "[FACE INIT] face 表情板已接入: " << face_port << "，初始 loving；同一检测状态持续5秒后自动回loving。\n";
    }

    // ===== 2. ONNX Runtime & SpacemiT 硬件加速提供商初始化 =====
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "pose_prod_320");

    std::cout << "[ORT Available Providers BEFORE SpaceMIT Init] ";
    auto providers_before = Ort::GetAvailableProviders();
    for (const auto& provider : providers_before) {
        std::cout << provider << " ";
    }
    std::cout << std::endl;

    Ort::SessionOptions opt;

    // 不设置 affinity，避免 K1 上出现 Invalid intra_thread_affinity id
    opt.SetIntraOpNumThreads(4);
    opt.SetInterOpNumThreads(1);
    opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    std::unordered_map<std::string, std::string> p;
    p["SPACEMIT_EP_INTRA_THREAD_NUM"] = "4";

    std::cout << "[SpaceMIT] 正在调用 SessionOptionsSpaceMITEnvInit..." << std::endl;
    SessionOptionsSpaceMITEnvInit(opt, p);
    std::cout << "[SpaceMIT] SessionOptionsSpaceMITEnvInit 调用完成" << std::endl;

    std::cout << "[ORT Available Providers AFTER SpaceMIT Init] ";
    auto providers_after = Ort::GetAvailableProviders();
    for (const auto& provider : providers_after) {
        std::cout << provider << " ";
    }
    std::cout << std::endl;

    std::string model_path = "/home/lu/Horse/Q_S/model/yolov8n-pose-320.onnx";
    std::cout << "[MODEL PATH] " << model_path << std::endl;

    Ort::Session session(env, model_path.c_str(), opt);

    // ===== 2.1 获取模型输入输出名 =====
    Ort::AllocatorWithDefaultOptions alloc;

    auto input_name_alloc = session.GetInputNameAllocated(0, alloc);
    auto output_name_alloc = session.GetOutputNameAllocated(0, alloc);

    std::string input_name = input_name_alloc.get();
    std::string output_name = output_name_alloc.get();

    const char* input_names[] = { input_name.c_str() };
    const char* output_names[] = { output_name.c_str() };

    std::cout << "[MODEL INPUT] " << input_name << std::endl;
    std::cout << "[MODEL OUTPUT] " << output_name << std::endl;

    // ===== 3. 摄像头线程 =====
    std::thread(cam).detach();

    // ===== 4. 显示窗口 =====
    const std::string window_name = "Spacemit K1 Pose Monitor - Voice Expanded";
    if (SHOW_WINDOW) {
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    }

    // ===== 5. 模型参数 =====
    const int W = 320;
    const int H = 320;
    const int ANCHORS = 2100;

    std::vector<float> input_data(1 * 3 * W * H);
    std::vector<cv::Mat> chw(3);
    std::vector<int64_t> shape = {1, 3, H, W};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    cv::Mat frame, rgb, resized, f32;

    // ===== 6. 状态锁存策略 =====
    PostureState global_state = PostureState::Good;
    PostureState raw_state_last = PostureState::Good;
    int same_state_counter = 0;
    int person_lost_counter = 0;
    bool abnormal_latched = false;

    // face 自动回默认状态计时：同一摄像头检测状态持续 2 秒后，表情回到默认 loving。
    std::string face_stable_expr = state_to_face_expr(PostureState::Good);
    auto face_stable_since = std::chrono::steady_clock::now();
    bool face_default_sent_for_current_expr = false;

    auto last_send_time = std::chrono::steady_clock::now() - std::chrono::milliseconds(SEND_MIN_GAP_MS);

    if (voice_fd < 0) {
        std::cout << "[SYSTEM RUNTIME] 纯视觉模式：串口未打开。\n";
    } else {
        std::cout << "[SYSTEM RUNTIME] 联动模式：语音 + face 表情；同一次异常只提醒一次，恢复正常后解除锁存。\n";
        if (SHOW_WINDOW) {
            std::cout << "[KEY TEST] 窗口聚焦后：1=正常happy，2=轻度驼背sad，3=轻度高低肩angry，4=明显驼背sad，5=明显高低肩angry，6=综合异常angry，7=人离开neutral，8=查询温湿度，ESC=退出；同一检测状态持续5秒后face回loving。\n";
        }
    }

    // ===== 7. 核心循环 =====
    while (gbuf.pop(frame)) {
        int64 start_tick = cv::getTickCount();

        // 7.0 处理 ASRPRO 通过语音命令发来的“查询环境”请求。
        // ASRPRO 发送 AB 20 55，K1 读取 face 板 DHT11 数据后回传 AA 20 温度 湿度 55。
        if (poll_env_query_from_asr(voice_fd)) {
            query_environment_and_report(voice_fd, face_fd);
        }

        // 7.1 前处理
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
        cv::resize(rgb, resized, cv::Size(W, H));
        resized.convertTo(f32, CV_32FC3, 1.0 / 255.0);

        for (int i = 0; i < 3; i++) {
            chw[i] = cv::Mat(H, W, CV_32FC1, input_data.data() + i * H * W);
        }
        cv::split(f32, chw);

        // 7.2 推理
        Ort::Value input_tensor = Ort::Value::CreateTensor(
            mem,
            input_data.data(),
            input_data.size(),
            shape.data(),
            shape.size()
        );

        auto infer_t0 = std::chrono::steady_clock::now();

        auto result = session.Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1
        );

        auto infer_t1 = std::chrono::steady_clock::now();
        double infer_ms = std::chrono::duration<double, std::milli>(infer_t1 - infer_t0).count();

        float* data = result[0].GetTensorMutableData<float>();

        // 7.3 取最高置信度人体
        int best = 0;
        float score = 0.0f;

        for (int i = 0; i < ANCHORS; i++) {
            float s = data[4 * ANCHORS + i];
            if (s > score) {
                score = s;
                best = i;
            }
        }

        if (score < SCORE_TH) {
            person_lost_counter++;

            if (PERSON_LOST_ENABLE && person_lost_counter >= PERSON_LOST_CONFIRM_FRAMES && !abnormal_latched) {
                auto now = std::chrono::steady_clock::now();
                auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count();

                if (gap_ms >= SEND_MIN_GAP_MS) {
                    std::cout << "[PERSON LOST] 长时间未检测到人 -> send AA 07 55\n";
                    send_robot_feedback(voice_fd, face_fd, PostureState::PersonLost);
                    abnormal_latched = true;
                    global_state = PostureState::PersonLost;
                    last_send_time = now;
                }
            }

            if (SHOW_WINDOW) {
                cv::putText(frame, "No reliable person", cv::Point(20, 70),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

                std::string txt_infer = "Infer: " + std::to_string(infer_ms).substr(0, 5) + " ms";
                cv::putText(frame, txt_infer, cv::Point(20, 110),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                cv::imshow(window_name, frame);

                int key = cv::waitKey(1);
                if (key == 27) {
                    break;
                }

                if (key >= '1' && key <= '7') {
                    PostureState test_state = key_to_state(key);
                    std::cout << "[KEY TEST] 手动联动状态=" << state_to_cn(test_state) << "\n";
                    send_robot_feedback(voice_fd, face_fd, test_state);
                } else if (key == '8') {
                    std::cout << "[KEY TEST] 手动查询环境温湿度\n";
                    query_environment_and_report(voice_fd, face_fd);
                }
            }

            continue;
        }

        person_lost_counter = 0;

        // 7.4 坐标还原
        float sx = frame.cols / 320.0f;
        float sy = frame.rows / 320.0f;

        cv::Point2f nose(data[5 * ANCHORS + best] * sx,  data[6 * ANCHORS + best] * sy);
        cv::Point2f ls(data[20 * ANCHORS + best] * sx, data[21 * ANCHORS + best] * sy);
        cv::Point2f rs(data[23 * ANCHORS + best] * sx, data[24 * ANCHORS + best] * sy);
        cv::Point2f mid = (ls + rs) * 0.5f;

        float raw_sh = angle3(rs, ls, cv::Point2f(rs.x, ls.y));
        float raw_ne = angle3(nose, mid, cv::Point2f(mid.x, mid.y - 100));

        // 7.5 姿态分类
        PostureState raw_state = classify_posture(raw_sh, raw_ne);

        // 7.6 连续帧确认
        if (raw_state == raw_state_last) {
            same_state_counter++;
        } else {
            raw_state_last = raw_state;
            same_state_counter = 1;
        }

        int need_frames = (raw_state == PostureState::Good) ? GOOD_CONFIRM_FRAMES : BAD_CONFIRM_FRAMES;
        auto now = std::chrono::steady_clock::now();
        auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count();

        // 7.6.1 face 自动回默认表情策略
        // 只看摄像头当前检测出来的状态对应的表情：
        // Good=happy，驼背=sad，高低肩/综合异常=angry。
        // 如果同一个表情状态持续 2 秒，就让 face 回到默认 loving，避免长时间保持警示表情。
        std::string current_detected_face_expr = state_to_face_expr(raw_state);
        if (current_detected_face_expr != face_stable_expr) {
            face_stable_expr = current_detected_face_expr;
            face_stable_since = now;
            face_default_sent_for_current_expr = false;
        } else if (!face_default_sent_for_current_expr) {
            auto face_stable_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - face_stable_since).count();
            if (face_stable_ms >= FACE_DEFAULT_AFTER_MS) {
                std::cout << "[FACE DEFAULT] 摄像头检测到同一状态持续 " << face_stable_ms
                          << " ms，face 回默认表情: " << FACE_DEFAULT_EXPR << "\n";
                send_face_cmd(face_fd, FACE_DEFAULT_EXPR);
                face_default_sent_for_current_expr = true;
            }
        }

        // 7.7 语音下发策略
        if (same_state_counter >= need_frames && gap_ms >= SEND_MIN_GAP_MS) {
            if (raw_state == PostureState::Good) {
                if (abnormal_latched && global_state != PostureState::Good) {
                    std::cout << "[STATUS RECOVER] 坐姿稳定恢复正常 raw_sh=" << raw_sh
                              << " raw_ne=" << raw_ne << " -> send AA 01 55\n";

                    send_robot_feedback(voice_fd, face_fd, PostureState::Good);
                    global_state = PostureState::Good;
                    abnormal_latched = false;
                    last_send_time = now;
                } else if (!abnormal_latched) {
                    global_state = PostureState::Good;
                }
            } else if (is_bad_state(raw_state)) {
                if (!abnormal_latched) {
                    uint8_t cmd = state_to_cmd(raw_state);

                    std::cout << "[STATUS ALERT] 当前状态=" << state_to_cn(raw_state)
                              << " raw_sh=" << raw_sh
                              << " raw_ne=" << raw_ne
                              << " -> send AA "
                              << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                              << static_cast<int>(cmd)
                              << " 55" << std::dec << std::setfill(' ') << "\n";

                    send_robot_feedback(voice_fd, face_fd, raw_state);
                    global_state = raw_state;
                    abnormal_latched = true;
                    last_send_time = now;
                }
            }
        }

        int64 end_tick = cv::getTickCount();
        double frame_time = (end_tick - start_tick) / cv::getTickFrequency();
        double real_fps = frame_time > 0 ? 1.0 / frame_time : 0.0;

        // 7.8 绘图与窗口显示
        if (SHOW_WINDOW) {
            cv::circle(frame, nose, 6, cv::Scalar(0, 255, 0), -1);
            cv::circle(frame, ls, 6, cv::Scalar(0, 0, 255), -1);
            cv::circle(frame, rs, 6, cv::Scalar(255, 0, 0), -1);
            cv::line(frame, ls, rs, cv::Scalar(255, 255, 255), 2);
            cv::line(frame, nose, mid, cv::Scalar(0, 255, 255), 2);

            std::string txt_score = "Score: " + std::to_string(score).substr(0, 4);
            std::string txt_sh = "Raw Shoulder: " + std::to_string(raw_sh).substr(0, 5) + " deg";
            std::string txt_ne = "Raw Neck: " + std::to_string(raw_ne).substr(0, 5) + " deg";
            std::string txt_raw = "Raw: " + state_to_en(raw_state) + "  Cnt: " +
                                  std::to_string(same_state_counter) + "/" + std::to_string(need_frames);
            std::string txt_decision = "Decision: [" + state_to_en(global_state) + "] Latch: " +
                                       (abnormal_latched ? "ON" : "OFF");
            std::string txt_fps = "FPS: " + std::to_string(real_fps).substr(0, 4);
            std::string txt_infer = "Infer: " + std::to_string(infer_ms).substr(0, 5) + " ms";

            cv::putText(frame, txt_score, cv::Point(20, 35),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(frame, txt_sh, cv::Point(20, 70),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
            cv::putText(frame, txt_ne, cv::Point(20, 100),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, txt_raw, cv::Point(20, 130),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            cv::putText(frame, txt_decision, cv::Point(20, 165),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            cv::putText(frame, txt_fps, cv::Point(20, 200),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            cv::putText(frame, txt_infer, cv::Point(20, 230),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            cv::imshow(window_name, frame);

            int key = cv::waitKey(1);
            if (key == 27) {
                break;
            } else if (key >= '1' && key <= '7') {
                PostureState test_state = key_to_state(key);
                std::cout << "[KEY TEST] 手动联动状态=" << state_to_cn(test_state)
                          << "，face=" << state_to_face_expr(test_state) << "\n";
                send_robot_feedback(voice_fd, face_fd, test_state);
            } else if (key == '8') {
                std::cout << "[KEY TEST] 手动查询环境温湿度\n";
                query_environment_and_report(voice_fd, face_fd);
            }
        }
    }

    if (voice_fd >= 0) {
        close(voice_fd);
        std::cout << "[INFO] 语音串口资源已释放。\n";
    }

    if (face_fd >= 0) {
        close(face_fd);
        std::cout << "[INFO] face 串口资源已释放。\n";
    }

    if (SHOW_WINDOW) {
        cv::destroyAllWindows();
    }

    return 0;
}
