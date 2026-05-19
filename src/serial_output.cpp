#include "serial_output.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <cstring>

namespace omt {

OutputWriter::OutputWriter() = default;

OutputWriter::~OutputWriter() {
    if (fd_ >= 0) close(fd_);
    if (udpSocket_ >= 0) close(udpSocket_);
}

bool OutputWriter::initSerial(const std::string& device, int baudrate) {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) return false;
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
        close(fd_); fd_ = -1;
        return false;
    }
    
    speed_t baud = B115200;
    switch (baudrate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
    }
    
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close(fd_); fd_ = -1;
        return false;
    }
    
    isSerial_ = true;
    return true;
}

bool OutputWriter::initUDP(const std::string& ip, int port) {
    udpSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket_ < 0) return false;
    
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
    
    if (connect(udpSocket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(udpSocket_); udpSocket_ = -1;
        return false;
    }
    
    isUDP_ = true;
    return true;
}

bool OutputWriter::initFile(const std::string& path) {
    // Handled in write() via stream redirect or fopen
    return true;
}

void OutputWriter::setFormat(OutputFormat fmt) {
    format_ = fmt;
}

void OutputWriter::setRateLimit(int fps) {
    rateLimitFps_ = fps;
}

bool OutputWriter::isOk() const {
    return isSerial_ || isUDP_ || fd_ < 0; // stdout always OK
}

double OutputWriter::now() {
    auto tp = std::chrono::steady_clock::now();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch()).count();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
        tp.time_since_epoch() % std::chrono::seconds(1)).count();
    return static_cast<double>(sec) + us * 1e-6;
}

void OutputWriter::write(const Pose3D& pose, int targetId) {
    if (!enabled_) return;
    
    if (rateLimitFps_ > 0) {
        double dt = now() - lastWriteTime_;
        if (dt < 1.0 / rateLimitFps_) return;
    }
    lastWriteTime_ = now();
    
    std::string data;
    switch (format_) {
        case OutputFormat::JSON: data = formatJSON(pose, targetId); break;
        case OutputFormat::CSV: data = formatCSV(pose, targetId); break;
        case OutputFormat::BINARY: data = formatBinary(pose, targetId); break;
        case OutputFormat::CUSTOM: data = formatCustom(pose, targetId); break;
    }
    
    send(data);
}

void OutputWriter::writeRaw(const std::string& data) {
    if (!enabled_) return;
    send(data);
}

void OutputWriter::send(const std::string& data) {
    const char* buf = data.c_str();
    size_t len = data.length();
    
    if (isSerial_ && fd_ >= 0) {
        write(fd_, buf, len);
        write(fd_, "\n", 1);
    }
    if (isUDP_ && udpSocket_ >= 0) {
        send(udpSocket_, buf, len, 0);
    }
    
    // Always echo to stdout for debugging
    std::cout << data << std::endl;
}

std::string OutputWriter::formatJSON(const Pose3D& p, int id) {
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    char buf[512];
    snprintf(buf, sizeof(buf),
        "{"
        "\"id\":%d,"
        "\"dx\":%.2f,\"dy\":%.2f,"
        "\"yaw\":%.4f,\"pitch\":%.4f,"
        "\"yaw_deg\":%.2f,\"pitch_deg\":%.2f,"
        "\"cam\":[%.4f,%.4f,%.4f],"
        "\"world\":[%.4f,%.4f,%.4f],"
        "\"has_world\":%s,"
        "\"dist\":%.4f,"
        "\"valid\":%s,"
        "\"t\":%lld"
        "}",
        id,
        p.pixelOffsetX, p.pixelOffsetY,
        p.yaw, p.pitch,
        p.yaw * 180.0f / CV_PI, p.pitch * 180.0f / CV_PI,
        p.cameraCoord.x, p.cameraCoord.y, p.cameraCoord.z,
        p.worldCoord.x, p.worldCoord.y, p.worldCoord.z,
        p.hasWorldCoord ? "true" : "false",
        p.distance,
        p.valid ? "true" : "false",
        static_cast<long long>(now_ms)
    );
    return std::string(buf);
}

std::string OutputWriter::formatCSV(const Pose3D& p, int id) {
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    char buf[256];
    snprintf(buf, sizeof(buf),
        "%d,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%s,%lld",
        id,
        p.pixelOffsetX, p.pixelOffsetY,
        p.yaw, p.pitch,
        p.cameraCoord.x, p.cameraCoord.y, p.cameraCoord.z,
        p.worldCoord.x, p.worldCoord.y, p.worldCoord.z,
        p.distance,
        p.hasWorldCoord ? "1" : "0",
        p.valid ? "1" : "0",
        static_cast<long long>(now_ms)
    );
    return std::string(buf);
}

std::string OutputWriter::formatBinary(const Pose3D& p, int id) {
    // Compact binary packet: 40 bytes
    struct __attribute__((packed)) Packet {
        uint8_t sync = 0xAA;
        int16_t id;
        float dx, dy;
        float yaw, pitch;
        float cx, cy, cz;
        float wx, wy, wz;
        float dist;
        uint8_t flags;
        uint16_t crc;
    } pkt;
    
    pkt.id = static_cast<int16_t>(id);
    pkt.dx = p.pixelOffsetX;
    pkt.dy = p.pixelOffsetY;
    pkt.yaw = p.yaw;
    pkt.pitch = p.pitch;
    pkt.cx = p.cameraCoord.x;
    pkt.cy = p.cameraCoord.y;
    pkt.cz = p.cameraCoord.z;
    pkt.wx = p.worldCoord.x;
    pkt.wy = p.worldCoord.y;
    pkt.wz = p.worldCoord.z;
    pkt.dist = p.distance;
    pkt.flags = (p.valid ? 0x01 : 0) | (p.hasWorldCoord ? 0x02 : 0);
    pkt.crc = 0; // Simplified, add CRC16 in production
    
    return std::string(reinterpret_cast<const char*>(&pkt), sizeof(pkt));
}

std::string OutputWriter::formatCustom(const Pose3D& p, int id) {
    // Compact text: $ID,dx,dy,yaw,pitch,cx,cy,cz,wx,wy,wz,dist,valid*
    char buf[256];
    snprintf(buf, sizeof(buf),
        "$%d,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d*",
        id,
        p.pixelOffsetX, p.pixelOffsetY,
        p.yaw, p.pitch,
        p.cameraCoord.x, p.cameraCoord.y, p.cameraCoord.z,
        p.worldCoord.x, p.worldCoord.y, p.worldCoord.z,
        p.distance,
        p.valid ? 1 : 0
    );
    return std::string(buf);
}

} // namespace omt
