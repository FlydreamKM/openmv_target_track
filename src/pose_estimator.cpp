#include "pose_estimator.hpp"
#include <cmath>

namespace omt {

// ========== CameraCalibration ==========
CameraCalibration CameraCalibration::fromFOV(int width, int height, float hfov_deg) {
    CameraCalibration cal;
    cal.imageWidth = width;
    cal.imageHeight = height;
    cal.hfov_deg = hfov_deg;
    
    float cx = width / 2.0f;
    float cy = height / 2.0f;
    
    // fx = cx / tan(HFOV/2)
    float hfov_rad = hfov_deg * CV_PI / 180.0f;
    float fx = cx / std::tan(hfov_rad / 2.0f);
    
    // 根据宽高比计算垂直FOV和fy
    float aspect = static_cast<float>(width) / height;
    float vfov_rad = 2.0f * std::atan(std::tan(hfov_rad / 2.0f) / aspect);
    cal.vfov_deg = vfov_rad * 180.0f / CV_PI;
    float fy = cy / std::tan(vfov_rad / 2.0f);
    
    cal.cameraMatrix = (cv::Mat_<double>(3,3) << 
        fx,  0, cx,
         0, fy, cy,
         0,  0,  1);
    
    // 无畸变
    cal.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    
    return cal;
}

CameraCalibration CameraCalibration::fromFile(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Cannot open calibration file: " + path);
    }
    
    CameraCalibration cal;
    fs["image_width"] >> cal.imageWidth;
    fs["image_height"] >> cal.imageHeight;
    fs["camera_matrix"] >> cal.cameraMatrix;
    fs["distortion_coefficients"] >> cal.distCoeffs;
    fs["hfov_deg"] >> cal.hfov_deg;
    fs["vfov_deg"] >> cal.vfov_deg;
    
    return cal;
}

void CameraCalibration::save(const std::string& path) const {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << "image_width" << imageWidth;
    fs << "image_height" << imageHeight;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "hfov_deg" << hfov_deg;
    fs << "vfov_deg" << vfov_deg;
}

// ========== Extrinsics ==========
Extrinsics::Extrinsics() {
    R = cv::Mat::eye(3, 3, CV_64F);
    t = cv::Mat::zeros(3, 1, CV_64F);
}

Extrinsics Extrinsics::fromEulerXYZ(float rx_deg, float ry_deg, float rz_deg,
                                      float tx, float ty, float tz) {
    Extrinsics ext;
    cv::Mat R_x = (cv::Mat_<double>(3,3) << 
        1, 0, 0,
        0, std::cos(rx_deg*CV_PI/180), -std::sin(rx_deg*CV_PI/180),
        0, std::sin(rx_deg*CV_PI/180), std::cos(rx_deg*CV_PI/180));
    cv::Mat R_y = (cv::Mat_<double>(3,3) << 
        std::cos(ry_deg*CV_PI/180), 0, std::sin(ry_deg*CV_PI/180),
        0, 1, 0,
        -std::sin(ry_deg*CV_PI/180), 0, std::cos(ry_deg*CV_PI/180));
    cv::Mat R_z = (cv::Mat_<double>(3,3) << 
        std::cos(rz_deg*CV_PI/180), -std::sin(rz_deg*CV_PI/180), 0,
        std::sin(rz_deg*CV_PI/180), std::cos(rz_deg*CV_PI/180), 0,
        0, 0, 1);
    ext.R = R_z * R_y * R_x;
    ext.t = (cv::Mat_<double>(3,1) << tx, ty, tz);
    return ext;
}

Extrinsics Extrinsics::fromFile(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Cannot open extrinsics file: " + path);
    }
    Extrinsics ext;
    fs["rotation_matrix"] >> ext.R;
    fs["translation_vector"] >> ext.t;
    return ext;
}

// ========== PoseEstimator ==========
PoseEstimator::PoseEstimator(const CameraCalibration& calib) : calib_(calib) {}

void PoseEstimator::setExtrinsics(const Extrinsics& ext) {
    extrinsics_ = ext;
    hasExtrinsics_ = true;
}

cv::Point2f PoseEstimator::pixelToNormalized(const cv::Point2f& pixel) const {
    double cx = calib_.cameraMatrix.at<double>(0, 2);
    double cy = calib_.cameraMatrix.at<double>(1, 2);
    double fx = calib_.cameraMatrix.at<double>(0, 0);
    double fy = calib_.cameraMatrix.at<double>(1, 1);
    
    return cv::Point2f(
        (pixel.x - cx) / fx,
        (pixel.y - cy) / fy
    );
}

cv::Point3f PoseEstimator::normalizedToCamera(const cv::Point2f& norm, float depthZ) const {
    return cv::Point3f(
        norm.x * depthZ,
        norm.y * depthZ,
        depthZ
    );
}

std::vector<cv::Point3f> PoseEstimator::getArucoObjectPoints(float size) {
    float h = size / 2.0f;
    return {
        cv::Point3f(-h,  h, 0),  // 左上
        cv::Point3f( h,  h, 0),  // 右上
        cv::Point3f( h, -h, 0),  // 右下
        cv::Point3f(-h, -h, 0)   // 左下
    };
}

Pose3D PoseEstimator::estimateAruco(const TargetResult& target, float markerSize) {
    Pose3D pose;
    if (!target.valid || target.corners.size() != 4) {
        return pose;
    }
    
    pose.imageCenter = cv::Point2f(calib_.imageWidth / 2.0f, calib_.imageHeight / 2.0f);
    pose.targetCenter = target.center;
    pose.pixelOffsetX = target.center.x - pose.imageCenter.x;
    pose.pixelOffsetY = target.center.y - pose.imageCenter.y; // 图像Y向下为正
    
    // solvePnP
    auto objectPoints = getArucoObjectPoints(markerSize);
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objectPoints, target.corners, 
                           calib_.cameraMatrix, calib_.distCoeffs,
                           rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    
    if (!ok) return pose;
    
    // tvec是相机坐标系下标记中心的位置
    pose.cameraCoord = cv::Point3f(
        static_cast<float>(tvec.at<double>(0)),
        static_cast<float>(tvec.at<double>(1)),
        static_cast<float>(tvec.at<double>(2))
    );
    
    pose.distance = cv::norm(tvec);
    
    // 计算角度：直接用像素归一化坐标更稳定
    cv::Point2f norm = pixelToNormalized(target.center);
    pose.yaw = std::atan2(pose.cameraCoord.x, pose.cameraCoord.z);   // 绕Y轴
    pose.pitch = std::atan2(pose.cameraCoord.y, pose.cameraCoord.z); // 绕X轴
    
    // 世界坐标转换
    if (hasExtrinsics_) {
        cv::Mat p_cam = (cv::Mat_<double>(3,1) << 
            pose.cameraCoord.x, pose.cameraCoord.y, pose.cameraCoord.z);
        cv::Mat p_world = extrinsics_.R.t() * (p_cam - extrinsics_.t);
        pose.worldCoord = cv::Point3f(
            static_cast<float>(p_world.at<double>(0)),
            static_cast<float>(p_world.at<double>(1)),
            static_cast<float>(p_world.at<double>(2))
        );
        pose.hasWorldCoord = true;
    }
    
    pose.valid = true;
    return pose;
}

Pose3D PoseEstimator::estimateCircle(const TargetResult& target, float realDiameter) {
    Pose3D pose;
    if (!target.valid || target.radius <= 0) {
        return pose;
    }
    
    pose.imageCenter = cv::Point2f(calib_.imageWidth / 2.0f, calib_.imageHeight / 2.0f);
    pose.targetCenter = target.center;
    pose.pixelOffsetX = target.center.x - pose.imageCenter.x;
    pose.pixelOffsetY = target.center.y - pose.imageCenter.y;
    
    // 相似三角形法估算深度
    // 实际直径 / 像素直径 = Z / fx  =>  Z = (实际直径 * fx) / 像素直径
    double fx = calib_.cameraMatrix.at<double>(0, 0);
    float pixelDiameter = target.radius * 2.0f;
    float depthZ = (realDiameter * fx) / pixelDiameter;
    
    // 归一化坐标 -> 相机坐标
    cv::Point2f norm = pixelToNormalized(target.center);
    pose.cameraCoord = normalizedToCamera(norm, depthZ);
    pose.distance = cv::norm(pose.cameraCoord);
    
    // 角度
    pose.yaw = std::atan2(pose.cameraCoord.x, pose.cameraCoord.z);
    pose.pitch = std::atan2(pose.cameraCoord.y, pose.cameraCoord.z);
    
    // 世界坐标
    if (hasExtrinsics_) {
        cv::Mat p_cam = (cv::Mat_<double>(3,1) << 
            pose.cameraCoord.x, pose.cameraCoord.y, pose.cameraCoord.z);
        cv::Mat p_world = extrinsics_.R.t() * (p_cam - extrinsics_.t);
        pose.worldCoord = cv::Point3f(
            static_cast<float>(p_world.at<double>(0)),
            static_cast<float>(p_world.at<double>(1)),
            static_cast<float>(p_world.at<double>(2))
        );
        pose.hasWorldCoord = true;
    }
    
    pose.valid = true;
    return pose;
}

Pose3D PoseEstimator::estimateAtDepth(const cv::Point2f& pixel, float depthZ) {
    Pose3D pose;
    
    pose.imageCenter = cv::Point2f(calib_.imageWidth / 2.0f, calib_.imageHeight / 2.0f);
    pose.targetCenter = pixel;
    pose.pixelOffsetX = pixel.x - pose.imageCenter.x;
    pose.pixelOffsetY = pixel.y - pose.imageCenter.y;
    
    cv::Point2f norm = pixelToNormalized(pixel);
    pose.cameraCoord = normalizedToCamera(norm, depthZ);
    pose.distance = cv::norm(pose.cameraCoord);
    
    pose.yaw = std::atan2(pose.cameraCoord.x, pose.cameraCoord.z);
    pose.pitch = std::atan2(pose.cameraCoord.y, pose.cameraCoord.z);
    
    if (hasExtrinsics_) {
        cv::Mat p_cam = (cv::Mat_<double>(3,1) << 
            pose.cameraCoord.x, pose.cameraCoord.y, pose.cameraCoord.z);
        cv::Mat p_world = extrinsics_.R.t() * (p_cam - extrinsics_.t);
        pose.worldCoord = cv::Point3f(
            static_cast<float>(p_world.at<double>(0)),
            static_cast<float>(p_world.at<double>(1)),
            static_cast<float>(p_world.at<double>(2))
        );
        pose.hasWorldCoord = true;
    }
    
    pose.valid = true;
    return pose;
}

} // namespace omt
