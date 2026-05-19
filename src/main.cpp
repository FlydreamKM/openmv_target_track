#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "target_detector.hpp"
#include "pose_estimator.hpp"
#include "serial_output.hpp"

using namespace omt;

static volatile bool g_running = true;

void signal_handler(int sig) {
    g_running = false;
}

void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [options]\n"
              << "Options:\n"
              << "  -m, --mode <aruco|circle>    Detection mode (default: aruco)\n"
              << "  -d, --device <index|path>    Camera device (default: 0)\n"
              << "  -c, --calib <path>           Calibration file (YAML)\n"
              << "  -e, --ext <path>             Extrinsics file for world coords\n"
              << "  -s, --size <meters>          Target real size (default: 0.165)\n"
              << "  -o, --output <serial|udp>    Output method\n"
              << "  --serial <device>            Serial port (e.g., /dev/ttyUSB0)\n"
              << "  --udp <ip:port>              UDP target (e.g., 192.168.1.100:5005)\n"
              << "  -f, --format <json|csv|bin|custom> Output format (default: json)\n"
              << "  -r, --res <WxH>              Resolution (default: 640x480)\n"
              << "  --fov <degrees>              Horizontal FOV (default: 90)\n"
              << "  --show                       Show debug window\n"
              << "  --headless                   No GUI, text only\n"
              << "  -h, --help                   This help\n"
              << "\nExamples:\n"
              << "  # ArUco marker tracking, 90deg FOV, output JSON to stdout\n"
              << "  " << prog << " -m aruco --fov 90\n"
              << "\n"
              << "  # Circle target (red), output compact format to serial\n"
              << "  " << prog << " -m circle -s 0.10 --serial /dev/ttyS0 -f custom\n"
              << "\n"
              << "  # With world coordinates (camera extrinsics known)\n"
              << "  " << prog << " -m aruco -e extrinsics.yaml --udp 192.168.1.100:5005\n";
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Default parameters
    std::string mode = "aruco";
    std::string device = "0";
    std::string calibPath;
    std::string extPath;
    float targetSize = 0.165f;  // meters
    std::string outputMethod = "stdout";
    std::string serialDev;
    std::string udpTarget;
    std::string formatStr = "json";
    int width = 640, height = 480;
    float hfov = 90.0f;
    bool showWindow = false;
    bool headless = false;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-m" || arg == "--mode") && i + 1 < argc) {
            mode = argv[++i];
        } else if ((arg == "-d" || arg == "--device") && i + 1 < argc) {
            device = argv[++i];
        } else if ((arg == "-c" || arg == "--calib") && i + 1 < argc) {
            calibPath = argv[++i];
        } else if ((arg == "-e" || arg == "--ext") && i + 1 < argc) {
            extPath = argv[++i];
        } else if ((arg == "-s" || arg == "--size") && i + 1 < argc) {
            targetSize = std::stof(argv[++i]);
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            outputMethod = argv[++i];
        } else if (arg == "--serial" && i + 1 < argc) {
            serialDev = argv[++i];
            outputMethod = "serial";
        } else if (arg == "--udp" && i + 1 < argc) {
            udpTarget = argv[++i];
            outputMethod = "udp";
        } else if ((arg == "-f" || arg == "--format") && i + 1 < argc) {
            formatStr = argv[++i];
        } else if ((arg == "-r" || arg == "--res") && i + 1 < argc) {
            std::string res = argv[++i];
            size_t x = res.find('x');
            if (x != std::string::npos) {
                width = std::stoi(res.substr(0, x));
                height = std::stoi(res.substr(x + 1));
            }
        } else if (arg == "--fov" && i + 1 < argc) {
            hfov = std::stof(argv[++i]);
        } else if (arg == "--show") {
            showWindow = true;
        } else if (arg == "--headless") {
            headless = true;
        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    // Load or generate calibration
    CameraCalibration calib;
    if (!calibPath.empty()) {
        try {
            calib = CameraCalibration::fromFile(calibPath);
            std::cout << "[INFO] Loaded calibration from " << calibPath << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[WARN] Failed to load calibration: " << e.what() << std::endl;
            calib = CameraCalibration::fromFOV(width, height, hfov);
        }
    } else {
        calib = CameraCalibration::fromFOV(width, height, hfov);
        std::cout << "[INFO] Using synthetic calibration: " << width << "x" << height
                  << ", HFOV=" << hfov << "deg" << std::endl;
    }

    // Load extrinsics (optional, for world coordinates)
    Extrinsics extrinsics;
    bool hasExtrinsics = false;
    if (!extPath.empty()) {
        try {
            extrinsics = Extrinsics::fromFile(extPath);
            hasExtrinsics = true;
            std::cout << "[INFO] Loaded extrinsics from " << extPath << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[WARN] Failed to load extrinsics: " << e.what() << std::endl;
        }
    }

    // Create detector
    std::unique_ptr<TargetDetector> detector;
    if (mode == "aruco") {
        detector = std::make_unique<ArucoDetector>(cv::aruco::DICT_4X4_50, targetSize);
        std::cout << "[INFO] Mode: ArUco (4x4_50), marker size=" << targetSize << "m" << std::endl;
    } else if (mode == "circle") {
        detector = std::make_unique<CircleTargetDetector>(CircleTargetDetector::Mode::CONTOUR, targetSize);
        // Default red color range for circle mode
        static_cast<CircleTargetDetector*>(detector.get())->setColorRange(
            cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255)
        );
        std::cout << "[INFO] Mode: Circle (red), diameter=" << targetSize << "m" << std::endl;
    } else {
        std::cerr << "[ERROR] Unknown mode: " << mode << std::endl;
        return 1;
    }

    // Create pose estimator
    PoseEstimator estimator(calib);
    if (hasExtrinsics) {
        estimator.setExtrinsics(extrinsics);
    }

    // Create output writer
    OutputWriter writer;
    if (outputMethod == "serial" && !serialDev.empty()) {
        if (writer.initSerial(serialDev, 115200)) {
            std::cout << "[INFO] Serial output: " << serialDev << std::endl;
        } else {
            std::cerr << "[WARN] Failed to open serial port " << serialDev << std::endl;
        }
    } else if (outputMethod == "udp" && !udpTarget.empty()) {
        size_t col = udpTarget.find(':');
        if (col != std::string::npos) {
            std::string ip = udpTarget.substr(0, col);
            int port = std::stoi(udpTarget.substr(col + 1));
            if (writer.initUDP(ip, port)) {
                std::cout << "[INFO] UDP output: " << ip << ":" << port << std::endl;
            }
        }
    }

    // Set output format
    if (formatStr == "json") writer.setFormat(OutputFormat::JSON);
    else if (formatStr == "csv") writer.setFormat(OutputFormat::CSV);
    else if (formatStr == "bin") writer.setFormat(OutputFormat::BINARY);
    else if (formatStr == "custom") writer.setFormat(OutputFormat::CUSTOM);

    // Open camera
    cv::VideoCapture cap;
    if (device.find_first_not_of("0123456789") == std::string::npos) {
        cap.open(std::stoi(device));
    } else {
        cap.open(device); // GStreamer pipeline or V4L2 path
    }

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Cannot open camera: " << device << std::endl;
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    std::cout << "[INFO] Camera opened. Running... (Ctrl+C to stop)" << std::endl;

    // Main loop
    cv::Mat frame;
    int frameCount = 0;
    double t0 = cv::getTickCount();

    while (g_running) {
        if (!cap.read(frame)) {
            usleep(10000);
            continue;
        }

        if (frame.empty()) continue;

        // Detect targets
        auto results = detector->detect(frame);

        // Process each target
        for (const auto& target : results) {
            if (!target.valid) continue;

            Pose3D pose;
            if (mode == "aruco") {
                pose = estimator.estimateAruco(target, targetSize);
            } else {
                pose = estimator.estimateCircle(target, targetSize);
            }

            if (pose.valid) {
                writer.write(pose, target.id);
            }
        }

        // Debug visualization
        if (showWindow && !headless) {
            detector->drawResults(frame, results);

            // Draw crosshair at image center
            cv::Point center(calib.imageWidth / 2, calib.imageHeight / 2);
            cv::line(frame, cv::Point(center.x - 20, center.y), cv::Point(center.x + 20, center.y),
                     cv::Scalar(0, 255, 255), 1);
            cv::line(frame, cv::Point(center.x, center.y - 20), cv::Point(center.x, center.y + 20),
                     cv::Scalar(0, 255, 255), 1);

            // FPS
            frameCount++;
            double t1 = cv::getTickCount();
            double fps = frameCount / ((t1 - t0) / cv::getTickFrequency());
            if (frameCount % 30 == 0) {
                cv::putText(frame, cv::format("FPS: %.1f", fps), cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }

            cv::imshow("OpenMV Target Track", frame);
            if (cv::waitKey(1) == 27) break; // ESC
        }
    }

    std::cout << "[INFO] Shutting down." << std::endl;
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
