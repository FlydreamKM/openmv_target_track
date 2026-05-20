#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <opencv2/opencv.hpp>
#include "target_detector.hpp"
#include "imu_interface.hpp"

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

    // ========== 光点误差信息 ==========
    bool hasBrightSpot = false;   // 是否检测到光点
    cv::Point2f brightSpotPixel;  // 光点像素坐标
    float spotOffsetX = 0.0f;     // 光点相对靶标中心的像素误差 X（右正）
    float spotOffsetY = 0.0f;     // 光点相对靶标中心的像素误差 Y（下正）
    float spotYaw = 0.0f;         // 光点水平偏角（弧度）
    float spotPitch = 0.0f;       // 光点俯仰偏角（弧度）

    // ========== IMU姿态信息 ==========
    bool hasImu = false;          // 是否有IMU数据
    cv::Vec4f imuQuaternion = cv::Vec4f(1.0f, 0.0f, 0.0f, 0.0f); // (w,x,y,z)
    cv::Point3f imuEulerDeg;      // roll, pitch, yaw（度）
};

/**
 * @brief 相机标定参数
 */
struct CameraCalibration {
    cv::Mat cameraMatrix;     // 3x3 内参矩阵
    cv::Mat distCoeffs;       // 畸变系数（1x4或1x5，无畸变可全0）
    int imageWidth = 640;
    int imageHeight = 480;
    float hfov_deg = 136.0f;   // 水平视场角（度）
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
     * @brief 估计方形靶标位姿（使用solvePnP）
     * @param target 检测结果（要求 corners 为4个角点）
     * @param realSize 方形靶标实际边长（米）
     * @return 三维位姿
     */
    Pose3D estimateSquare(const TargetResult& target, float realSize);
    
    /**
     * @brief 通用的单点估计（已知深度Z时）
     * @param pixel 像素坐标
     * @param depthZ 深度距离（米，Z轴正向）
     * @return 三维位姿
     */
    Pose3D estimateAtDepth(const cv::Point2f& pixel, float depthZ);

    /**
     * @brief 计算光点相对靶标中心的误差角
     * @param target 靶标检测结果（用于获取中心与深度）
     * @param spotPixel 光点像素坐标
     * @param depthZ 靶标深度（米），可由 estimateSquare/ArUco 得到
     * @return 包含 spotOffsetX/Y、spotYaw/Pitch 的 Pose3D 结构
     */
    Pose3D estimateSpotError(const TargetResult& target, const cv::Point2f& spotPixel, float depthZ);

    /**
     * @brief 将IMU数据注入Pose3D
     * @param pose 待填充的位姿结构
     * @param imu IMU接口指针（若为nullptr则不做任何事）
     */
    static void injectImuData(Pose3D& pose, const ImuInterface* imu);
    
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

    // 方形靶标3D模型点（与ArUco一致）
    static std::vector<cv::Point3f> getSquareObjectPoints(float size);

    // 通用后处理：相机坐标 -> 世界坐标、角度计算等
    void fillPoseAnglesAndWorld(Pose3D& pose) const;
};

} // namespace omt

#endif // POSE_ESTIMATOR_HPP
