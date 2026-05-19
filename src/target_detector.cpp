#include "target_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

namespace omt {

// ========== 基类 ==========
void TargetDetector::drawResults(cv::Mat& frame, const std::vector<TargetResult>& results) {
    for (const auto& r : results) {
        if (!r.valid) continue;
        
        // 绘制角点连线
        if (r.corners.size() == 4) {
            for (size_t i = 0; i < 4; ++i) {
                cv::line(frame, r.corners[i], r.corners[(i+1)%4], cv::Scalar(0, 255, 0), 2);
            }
        }
        
        // 绘制中心十字
        const int len = 15;
        cv::Point c(r.center.x, r.center.y);
        cv::line(frame, cv::Point(c.x - len, c.y), cv::Point(c.x + len, c.y), cv::Scalar(0, 0, 255), 2);
        cv::line(frame, cv::Point(c.x, c.y - len), cv::Point(c.x, c.y + len), cv::Scalar(0, 0, 255), 2);
        
        // ID标签
        std::string label = (r.id >= 0) 
            ? cv::format("ID:%d (%.0fpx)", r.id, r.radius * 2)
            : cv::format("Target (%.0fpx)", r.radius * 2);
        cv::putText(frame, label, cv::Point(c.x + 20, c.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    }
}

// ========== ArUco检测器 ==========
ArucoDetector::ArucoDetector(int dictionary, float markerSizeMeters)
    : markerSizeMeters_(markerSizeMeters) {
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary);
    params_ = cv::aruco::DetectorParameters::create();
    params_->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin_;
    params_->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax_;
    params_->adaptiveThreshConstant = adaptiveThreshConstant_;
}

std::vector<TargetResult> ArucoDetector::detect(const cv::Mat& frame) {
    std::vector<TargetResult> results;
    if (frame.empty()) return results;
    
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, params_, rejected);
    
    for (size_t i = 0; i < ids.size(); ++i) {
        TargetResult r;
        r.id = ids[i];
        r.corners = corners[i];
        r.valid = true;
        
        // 计算中心（四边形质心）
        cv::Moments m = cv::moments(corners[i]);
        r.center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
        
        // 估算半径（对角线一半）
        float d = cv::norm(corners[i][0] - corners[i][2]);
        r.radius = d * 0.5f;
        r.area = d * d * 0.5f; // 近似面积
        
        results.push_back(r);
    }
    return results;
}

void ArucoDetector::setParam(const std::string& key, double value) {
    if (key == "adaptiveThreshWinSizeMin") {
        adaptiveThreshWinSizeMin_ = static_cast<int>(value);
        params_->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin_;
    } else if (key == "adaptiveThreshWinSizeMax") {
        adaptiveThreshWinSizeMax_ = static_cast<int>(value);
        params_->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax_;
    } else if (key == "adaptiveThreshConstant") {
        adaptiveThreshConstant_ = value;
        params_->adaptiveThreshConstant = adaptiveThreshConstant_;
    } else if (key == "markerSize") {
        markerSizeMeters_ = static_cast<float>(value);
    }
}

// ========== 圆形靶标检测器 ==========
CircleTargetDetector::CircleTargetDetector(Mode mode, float realDiameterMeters)
    : mode_(mode), realDiameterMeters_(realDiameterMeters) {}

void CircleTargetDetector::setColorRange(const cv::Scalar& lower, const cv::Scalar& upper) {
    hsvLower_ = lower;
    hsvUpper_ = upper;
}

void CircleTargetDetector::setHoughParams(double dp, double minDist, 
                                          double param1, double param2,
                                          int minRadius, int maxRadius) {
    houghDp_ = dp; houghMinDist_ = minDist;
    houghParam1_ = param1; houghParam2_ = param2;
    houghMinRadius_ = minRadius; houghMaxRadius_ = maxRadius;
}

std::vector<TargetResult> CircleTargetDetector::detect(const cv::Mat& frame) {
    if (mode_ == Mode::CONTOUR) {
        return detectContour_(frame);
    }
    return detectHough_(frame);
}

std::vector<TargetResult> CircleTargetDetector::detectContour_(const cv::Mat& frame) {
    std::vector<TargetResult> results;
    if (frame.empty()) return results;
    
    cv::Mat hsv, mask, blurred;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(hsv, blurred, cv::Size(5, 5), 0);
    cv::inRange(blurred, hsvLower_, hsvUpper_, mask);
    
    // 形态学去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& cnt : contours) {
        if (cnt.size() < 5) continue;
        
        double area = cv::contourArea(cnt);
        if (area < 100.0) continue; // 过滤噪声
        
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(cnt, center, radius);
        
        // 圆度检查：面积与外接圆面积比
        double circleArea = CV_PI * radius * radius;
        double circularity = area / circleArea;
        if (circularity < 0.6) continue; // 不够圆则跳过
        
        TargetResult r;
        r.center = center;
        r.radius = radius;
        r.area = static_cast<float>(area);
        r.valid = true;
        
        // 拟合椭圆获取角点近似
        if (cnt.size() >= 5) {
            cv::RotatedRect ell = cv::fitEllipse(cnt);
            r.bounding_box = ell;
            
            // 生成4个角点（椭圆外接矩形）
            cv::Point2f pts[4];
            ell.points(pts);
            r.corners.assign(pts, pts + 4);
        }
        
        results.push_back(r);
    }
    
    // 按面积从大到小排序
    std::sort(results.begin(), results.end(), 
              [](const TargetResult& a, const TargetResult& b) { return a.area > b.area; });
    return results;
}

std::vector<TargetResult> CircleTargetDetector::detectHough_(const cv::Mat& frame) {
    std::vector<TargetResult> results;
    if (frame.empty()) return results;
    
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2);
    
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT,
                     houghDp_, houghMinDist_,
                     houghParam1_, houghParam2_,
                     houghMinRadius_, houghMaxRadius_);
    
    for (const auto& c : circles) {
        TargetResult r;
        r.center = cv::Point2f(c[0], c[1]);
        r.radius = c[2];
        r.area = CV_PI * r.radius * r.radius;
        r.valid = true;
        
        // 为Hough结果构造虚拟角点
        float x = c[0], y = c[1], rad = c[2];
        r.corners = {
            cv::Point2f(x - rad, y - rad),
            cv::Point2f(x + rad, y - rad),
            cv::Point2f(x + rad, y + rad),
            cv::Point2f(x - rad, y + rad)
        };
        
        results.push_back(r);
    }
    return results;
}

void CircleTargetDetector::setParam(const std::string& key, double value) {
    if (key == "houghDp") houghDp_ = value;
    else if (key == "houghMinDist") houghMinDist_ = value;
    else if (key == "houghParam1") houghParam1_ = value;
    else if (key == "houghParam2") houghParam2_ = value;
    else if (key == "houghMinRadius") houghMinRadius_ = static_cast<int>(value);
    else if (key == "houghMaxRadius") houghMaxRadius_ = static_cast<int>(value);
    else if (key == "realDiameter") realDiameterMeters_ = static_cast<float>(value);
}

// ========== 工厂函数 ==========
std::unique_ptr<TargetDetector> createDefaultDetector() {
    return std::make_unique<ArucoDetector>(cv::aruco::DICT_4X4_50, 0.165f);
}

} // namespace omt
