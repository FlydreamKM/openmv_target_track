#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <opencv2/opencv.hpp>
#include "target_detector.hpp"

namespace omt {

/**
 * @brief 三维位姿结果
 */
struct Pose3D {
    // 像素偏移（相对于画面中心）
    float pixelOffsetX = 0.0f;    // 像素，右正左负
    float pixelOffsetY = 0.0f;    // 像素，下正上负
    
    // 角度（弧度）
    float yaw = 0.0f;             // 水平偏角（绕Y轴）
    float pitch = 0.0f;           // 俯仰角（绕X轴）
    
    // 相机坐标系下的3D坐标（相机原点为(0,0,0)，Z轴向前）
    cv::Point3f cameraCoord;      // (X, Y, Z) 单位：米
    
    // 世界坐标系下的3D坐标（如果提供了外参）
    cv::Point3f worldCoord;       // (Xw, Yw, Zw) 单位：米
    bool hasWorldCoord = false;
    
    // 距离
    float distance = 0.0f;        // 到靶标中心的直线距离（米）
    
    // 像素坐标
    cv::Point2f imageCenter;      // 画面中心
    cv::Point2f targetCenter;     // 靶标中心
    
    bool valid = false;
};

/**
 * @brief 相机标定参数
 */
struct CameraCalibration {
    cv::Mat cameraMatrix;     // 3x3 内参矩阵
    cv::Mat distCoeffs;       // 畸变系数（1x4或1x5，无畸变可全0）
    int imageWidth = 640;
    int imageHeight = 480;
    float hfov_deg = 90.0f;   // 水平视场角（度）
    float vfov_deg = 0.0f;    // 垂直视场角（度，0表示自动计算）
    
    /**
     * @brief 从FOV和分辨率生成标定参数（无畸变假设）
     */
    static CameraCalibration fromFOV(int width, int height, float hfov_deg);
    
    /**
     * @brief 从OpenCV YAML文件加载
     */
    static CameraCalibration fromFile(const std::string& path);
    
    /**
     * @brief 保存到YAML文件
     */
    void save(const std::string& path) const;
};

/**
 * @brief 相机外参（世界坐标系到相机坐标系的变换）
 */
struct Extrinsics {
    cv::Mat R;  // 3x3 旋转矩阵
    cv::Mat t;  // 3x1 平移向量
    
    Extrinsics();
    
    /**
     * @brief 从欧拉角（度）和平移构建
     */
    static Extrinsics fromEulerXYZ(float rx_deg, float ry_deg, float rz_deg,
                                     float tx, float ty, float tz);
    
    /**
     * @brief 从YAML加载
     */
    static Extrinsics fromFile(const std::string& path);
};

/**
 * @brief 三维位姿估计器
 */
class PoseEstimator {
public:
    explicit PoseEstimator(const CameraCalibration& calib);
    
    /**
     * @brief 设置外参（用于输出世界坐标）
     */
    void setExtrinsics(const Extrinsics& ext);
    
    /**
     * @brief 估计ArUco标记位姿（使用solvePnP）
     * @param target 检测结果
     * @param markerSize 标记实际边长（米）
     * @return 三维位姿
     */
    Pose3D estimateAruco(const TargetResult& target, float markerSize);
    
    /**
     * @brief 估计圆形靶标位姿（使用相似三角形）
     * @param target 检测结果
     * @param realDiameter 靶标实际直径（米）
     * @return 三维位姿
     */
    Pose3D estimateCircle(const TargetResult& target, float realDiameter);
    
    /**
     * @brief 通用的单点估计（已知深度Z时）
     * @param pixel 像素坐标
     * @param depthZ 深度距离（米，Z轴正向）
     * @return 三维位姿
     */
    Pose3D estimateAtDepth(const cv::Point2f& pixel, float depthZ);
    
    /**
     * @brief 获取相机参数
     */
    const CameraCalibration& getCalibration() const { return calib_; }
    
    /**
     * @brief 像素坐标转归一化平面坐标
     */
    cv::Point2f pixelToNormalized(const cv::Point2f& pixel) const;
    
    /**
     * @brief 归一化坐标 + 深度 -> 相机3D坐标
     */
    cv::Point3f normalizedToCamera(const cv::Point2f& norm, float depthZ) const;
    
private:
    CameraCalibration calib_;
    Extrinsics extrinsics_;
    bool hasExtrinsics_ = false;
    
    // ArUco标记3D模型点（单位正方形，中心在原点，Z=0平面）
    static std::vector<cv::Point3f> getArucoObjectPoints(float size);
};

} // namespace omt

#endif // POSE_ESTIMATOR_HPP
