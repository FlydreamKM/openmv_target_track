#ifndef SERIAL_OUTPUT_HPP
#define SERIAL_OUTPUT_HPP

#include <string>
#include <sstream>
#include "pose_estimator.hpp"

namespace omt {

/**
 * @brief 输出格式枚举
 */
enum class OutputFormat {
    JSON,       // JSON格式，易于上位机解析
    CSV,        // CSV格式，易于记录
    BINARY,     // 二进制，最小带宽
    CUSTOM      // 自定义协议
};

/**
 * @brief 串口/网络输出器
 * 
 * 可输出到：
 * - stdout（默认，用于调试）
 * - 串口（/dev/ttyUSB0等）
 * - UDP/TCP socket（网络传输）
 * - 文件
 */
class OutputWriter {
public:
    OutputWriter();
    ~OutputWriter();
    
    /**
     * @brief 初始化串口输出
     * @param device 设备路径（如"/dev/ttyS0"）
     * @param baudrate 波特率
     * @return 是否成功
     */
    bool initSerial(const std::string& device, int baudrate = 115200);
    
    /**
     * @brief 初始化UDP输出
     * @param ip 目标IP
     * @param port 目标端口
     * @return 是否成功
     */
    bool initUDP(const std::string& ip, int port);
    
    /**
     * @brief 初始化文件输出
     */
    bool initFile(const std::string& path);
    
    /**
     * @brief 设置输出格式
     */
    void setFormat(OutputFormat fmt);
    
    /**
     * @brief 输出一帧位姿数据
     */
    void write(const Pose3D& pose, int targetId = -1);
    
    /**
     * @brief 输出自定义字符串
     */
    void writeRaw(const std::string& data);
    
    /**
     * @brief 是否启用输出
     */
    void setEnabled(bool en) { enabled_ = en; }
    
    /**
     * @brief 设置帧率限制（0 = 无限制）
     */
    void setRateLimit(int fps);
    
    bool isOk() const;
    
private:
    int fd_ = -1;
    int udpSocket_ = -1;
    bool isSerial_ = false;
    bool isUDP_ = false;
    bool isFile_ = false;
    bool enabled_ = true;
    OutputFormat format_ = OutputFormat::JSON;
    
    // 帧率限制
    int rateLimitFps_ = 0;
    double lastWriteTime_ = 0.0;
    
    std::string formatJSON(const Pose3D& pose, int targetId);
    std::string formatCSV(const Pose3D& pose, int targetId);
    std::string formatBinary(const Pose3D& pose, int targetId);
    std::string formatCustom(const Pose3D& pose, int targetId);
    
    void send(const std::string& data);
    double now();
};

} // namespace omt

#endif // SERIAL_OUTPUT_HPP
