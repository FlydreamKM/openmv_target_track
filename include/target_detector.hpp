#ifndef TARGET_DETECTOR_HPP
#define TARGET_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace omt {

/**
 * @brief 靶标检测结果结构体
 */
struct TargetResult {
    int id = -1;                          // 靶标ID（ArUco时有效）
    cv::Point2f center;                    // 靶标中心像素坐标
    std::vector<cv::Point2f> corners;    // 靶标角点（4个）
    float area = 0.0f;                    // 靶标像素面积
    bool valid = false;                   // 是否有效检测
    
    // 用于圆形/矩形靶标的额外信息
    float radius = 0.0f;                  // 等效半径（像素）
    cv::RotatedRect bounding_box;        // 旋转外接矩形
};

/**
 * @brief 靶标检测器基类
 */
class TargetDetector {
public:
    virtual ~TargetDetector() = default;
    
    /**
     * @brief 检测图像中的靶标
     * @param frame 输入图像（BGR或灰度）
     * @return 检测结果列表
     */
    virtual std::vector<TargetResult> detect(const cv::Mat& frame) = 0;
    
    /**
     * @brief 绘制检测结果
     * @param frame 输入/输出图像
     * @param results 检测结果
     */
    virtual void drawResults(cv::Mat& frame, const std::vector<TargetResult>& results);
    
    /**
     * @brief 设置检测参数
     */
    virtual void setParam(const std::string& key, double value) = 0;
};

/**
 * @brief ArUco标记检测器（推荐用于三维跟踪）
 */
class ArucoDetector : public TargetDetector {
public:
    /**
     * @brief 构造函数
     * @param dictionary 标记字典类型（默认DICT_4X4_50）
     * @param markerSizeMeters 标记实际边长（米）
     */
    explicit ArucoDetector(int dictionary = cv::aruco::DICT_4X4_50, 
                           float markerSizeMeters = 0.165f);
    
    std::vector<TargetResult> detect(const cv::Mat& frame) override;
    void setParam(const std::string& key, double value) override;
    
    float getMarkerSize() const { return markerSizeMeters_; }
    void setMarkerSize(float size) { markerSizeMeters_ = size; }
    
private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    float markerSizeMeters_;
    
    // 可调参数
    int adaptiveThreshWinSizeMin_ = 3;
    int adaptiveThreshWinSizeMax_ = 23;
    double adaptiveThreshConstant_ = 7.0;
};

/**
 * @brief 圆形靶标检测器（基于轮廓/Hough圆）
 */
class CircleTargetDetector : public TargetDetector {
public:
    /**
     * @brief 检测模式
     */
    enum class Mode {
        CONTOUR,    // 基于颜色阈值+轮廓（推荐）
        HOUGH       // Hough圆变换（对光照敏感）
    };
    
    /**
     * @brief 构造函数
     * @param mode 检测模式
     * @param realDiameterMeters 靶标实际直径（米）
     */
    explicit CircleTargetDetector(Mode mode = Mode::CONTOUR, 
                                  float realDiameterMeters = 0.10f);
    
    std::vector<TargetResult> detect(const cv::Mat& frame) override;
    void setParam(const std::string& key, double value) override;
    
    // 颜色阈值设置（HSV空间）- 仅CONTOUR模式
    void setColorRange(const cv::Scalar& lower, const cv::Scalar& upper);
    
    // Hough参数 - 仅HOUGH模式
    void setHoughParams(double dp, double minDist, 
                        double param1, double param2,
                        int minRadius, int maxRadius);
    
    float getRealDiameter() const { return realDiameterMeters_; }
    
private:
    Mode mode_;
    float realDiameterMeters_;
    
    // 颜色阈值（CONTOUR模式）
    cv::Scalar hsvLower_ = cv::Scalar(0, 100, 100);   // 默认红色下限
    cv::Scalar hsvUpper_ = cv::Scalar(10, 255, 255); // 默认红色上限
    
    // Hough参数
    double houghDp_ = 1.0;
    double houghMinDist_ = 50.0;
    double houghParam1_ = 100.0;  // Canny阈值
    double houghParam2_ = 30.0;   // 累加器阈值
    int houghMinRadius_ = 10;
    int houghMaxRadius_ = 200;
    
    std::vector<TargetResult> detectContour_(const cv::Mat& frame);
    std::vector<TargetResult> detectHough_(const cv::Mat& frame);
};

/**
 * @brief 创建默认检测器（优先ArUco）
 */
std::unique_ptr<TargetDetector> createDefaultDetector();

} // namespace omt

#endif // TARGET_DETECTOR_HPP
