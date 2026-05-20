#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

#include <opencv2/opencv.hpp>

namespace omt {

/**
 * @brief 外部IMU抽象接口
 *
 * 用户需继承此类并实现具体IMU驱动（如串口读取MPU6050/BMI088、I2C、SPI、UDP等）。
 * 典型调用流程：
 *   1. 在 main() 中实例化具体实现类
 *   2. 每帧调用 update() 拉取最新数据
 *   3. 通过 getQuaternion() / getEulerAnglesDeg() 获取相机姿态
 *
 * 坐标系约定：
 *   - 四元数与欧拉角表示的是 **世界坐标系 -> IMU(相机)坐标系** 的旋转
 *   - IMU坐标系与相机坐标系默认重合；若存在安装偏差，请在外部标定后补偿
 */
class ImuInterface {
public:
    virtual ~ImuInterface() = default;

    /**
     * @brief 从硬件/数据源读取最新IMU数据
     * @return true 表示成功读到新一帧数据
     */
    virtual bool update() = 0;

    /** @brief 当前数据是否有效（已成功update至少一次且未超时） */
    virtual bool isValid() const = 0;

    /** @brief 获取最近一次成功update的时间戳（秒，steady_clock） */
    virtual double getTimestamp() const = 0;

    /**
     * @brief 获取姿态四元数 (w, x, y, z)
     * @return 归一化四元数，表示世界 -> IMU的旋转
     */
    virtual cv::Vec4f getQuaternion() const = 0;

    /**
     * @brief 获取欧拉角（度），顺序为 roll, pitch, yaw
     * @details 采用 ZYX 内旋顺序（先绕Z偏航yaw，再绕Y俯仰pitch，再绕X横滚roll）
     */
    virtual cv::Point3f getEulerAnglesDeg() const = 0;

    /** @brief 获取角速度（度/秒），IMU坐标系下 */
    virtual cv::Point3f getGyroscopeDeg() const = 0;

    /** @brief 获取加速度（m/s^2），IMU坐标系下 */
    virtual cv::Point3f getAccelerometer() const = 0;
};

/**
 * @brief 四元数转旋转矩阵
 * @param q (w, x, y, z)
 * @return 3x3 CV_64F 旋转矩阵
 */
cv::Mat quatToRotationMatrix(const cv::Vec4f& q);

/**
 * @brief 旋转矩阵转欧拉角（ZYX，度）
 * @param R 3x3 CV_64F 旋转矩阵
 * @return (roll, pitch, yaw) 单位：度
 */
cv::Point3f rotationMatrixToEulerDeg(const cv::Mat& R);

/**
 * @brief IMU数据占位实现（用于测试或IMU未接入时）
 *
 * update() 始终返回 false，isValid() 始终返回 false。
 * 姿态固定为 identity（无旋转）。
 */
class DummyImu : public ImuInterface {
public:
    bool update() override { return false; }
    bool isValid() const override { return false; }
    double getTimestamp() const override { return 0.0; }
    cv::Vec4f getQuaternion() const override { return cv::Vec4f(1.0f, 0.0f, 0.0f, 0.0f); }
    cv::Point3f getEulerAnglesDeg() const override { return cv::Point3f(0.0f, 0.0f, 0.0f); }
    cv::Point3f getGyroscopeDeg() const override { return cv::Point3f(0.0f, 0.0f, 0.0f); }
    cv::Point3f getAccelerometer() const override { return cv::Point3f(0.0f, 0.0f, 0.0f); }
};

} // namespace omt

#endif // IMU_INTERFACE_HPP
