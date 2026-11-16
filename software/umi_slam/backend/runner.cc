// Offline stereo-inertial adapter for the locally patched ORB-SLAM3 library.
#include <System.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

struct Frame { long long ns; std::string left, right; };
struct Measurement { double t; cv::Point3f gyro, acc; };

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: umi_slam_runner vocabulary settings prepared_dir output_dir\n";
        return 2;
    }
    try {
        const std::string data = argv[3], out = argv[4];
        std::ifstream images(data + "/frames.csv"), imu(data + "/imu.csv");
        if (!images || !imu) throw std::runtime_error("Cannot read prepared inputs");
        std::vector<Frame> frames;
        std::vector<Measurement> measurements;
        std::string line;
        while (std::getline(images, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream s(line); std::string ts, l, r;
            std::getline(s, ts, ','); std::getline(s, l, ','); std::getline(s, r);
            frames.push_back({std::stoll(ts), data + "/" + l, data + "/" + r});
        }
        while (std::getline(imu, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::replace(line.begin(), line.end(), ',', ' ');
            std::stringstream s(line); long long ns; double gx, gy, gz, ax, ay, az;
            if (!(s >> ns >> gx >> gy >> gz >> ax >> ay >> az))
                throw std::runtime_error("Invalid IMU row");
            measurements.push_back({ns * 1e-9, cv::Point3f(gx,gy,gz), cv::Point3f(ax,ay,az)});
        }
        if (frames.size() < 2 || measurements.size() < 2 ||
            measurements.front().t > frames.front().ns * 1e-9 ||
            measurements.back().t < frames.back().ns * 1e-9)
            throw std::runtime_error("IMU does not cover image interval");
        ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, false);
        std::ofstream states(out + "/tracking.csv");
        states << "timestamp_ns,state,imu_initialized,map_id\n";
        size_t k = 0;
        while (k + 1 < measurements.size() && measurements[k+1].t <= frames[0].ns * 1e-9) ++k;
        bool initialized = false;
        for (size_t i = 0; i < frames.size(); ++i) {
            auto start = std::chrono::steady_clock::now();
            const auto& f = frames[i];
            cv::Mat l = cv::imread(f.left, cv::IMREAD_GRAYSCALE);
            cv::Mat r = cv::imread(f.right, cv::IMREAD_GRAYSCALE);
            if (l.empty() || r.empty()) throw std::runtime_error("Cannot decode image");
            std::vector<ORB_SLAM3::IMU::Point> points;
            if (i > 0) {
                while (k < measurements.size() && measurements[k].t <= f.ns * 1e-9) {
                    const auto& m = measurements[k++];
                    points.emplace_back(m.acc.x,m.acc.y,m.acc.z,m.gyro.x,m.gyro.y,m.gyro.z,m.t);
                }
            }
            slam.TrackStereo(l, r, f.ns * 1e-9, points);
            bool ready = slam.OfflineImuInitialized();
            initialized = initialized || ready;
            states << f.ns << ',' << slam.GetTrackingState() << ',' << ready << ','
                   << slam.OfflineMapId() << '\n';
            if (i % 25 == 0 || i + 1 == frames.size()) {
                states.flush();
                std::cout << "UMI_PROGRESS " << i+1 << '/' << frames.size() << std::endl;
            }
            // Preserve sensor pacing: local mapping must have time to optimize.
            if (i + 1 < frames.size()) {
                auto delay = std::chrono::duration<double>((frames[i+1].ns-f.ns)*1e-9);
                std::this_thread::sleep_until(start + delay);
            }
        }
        slam.Shutdown();  // Packaged source waits for mapping, loop closure and GBA.
        states.close();
        if (!initialized) throw std::runtime_error("IMU initialization never completed");
        slam.SaveTrajectoryEuRoC(out + "/imu_euroc.txt");
        slam.SaveKeyFrameTrajectoryEuRoC(out + "/keyframes_euroc.txt");
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "UMI_ERROR " << e.what() << std::endl;
        return 1;
    }
}
