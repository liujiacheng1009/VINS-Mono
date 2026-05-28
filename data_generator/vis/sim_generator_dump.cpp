/**
 * Export DataGenerator ground-truth trajectory, IMU, and observations to JSON.
 */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/data_generator.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace {

std::string quatToJson(const Eigen::Quaterniond &q)
{
    std::ostringstream os;
    os << std::setprecision(17)
       << "[" << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "]";
    return os.str();
}

std::string vec3ToJson(const Eigen::Vector3d &v)
{
    std::ostringstream os;
    os << std::setprecision(17) << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
    return os.str();
}

std::string mat3ToJson(const Eigen::Matrix3d &R)
{
    std::ostringstream os;
    os << std::setprecision(17) << "[";
    for (int r = 0; r < 3; ++r)
    {
        if (r > 0)
            os << ",";
        os << "[" << R(r, 0) << "," << R(r, 1) << "," << R(r, 2) << "]";
    }
    os << "]";
    return os.str();
}

} // namespace

int main(int argc, char **argv)
{
    const std::string out_path = (argc > 1) ? argv[1] : std::string("sim_dump.json");
    const double duration_scale = (argc > 2) ? std::stod(argv[2]) : 3.0;

    DataGenerator generator(false);
    generator.setQuiet(true);

    const int num_cam = generator.numCameras();
    const double t_end = duration_scale * DataGenerator::MAX_TIME;

    std::vector<double> imu_t;
    std::vector<Eigen::Vector3d> imu_pos, imu_vel_body, imu_acc, imu_gyr;
    std::vector<Eigen::Quaterniond> imu_quat;

    struct FrameRecord
    {
        double t;
        Eigen::Vector3d position;
        Eigen::Quaterniond quat;
        std::vector<std::tuple<int, int, int, Eigen::Vector3d>> observations;
    };
    std::vector<FrameRecord> frames;

    int publish_count = 0;
    while (generator.getTime() <= t_end)
    {
        const double t = generator.getTime();
        const Eigen::Vector3d pos = generator.getPosition();
        const Eigen::Matrix3d rot = generator.getRotation();
        const Eigen::Quaterniond quat(rot);

        imu_t.push_back(t);
        imu_pos.push_back(pos);
        imu_vel_body.push_back(generator.getVelocity());
        imu_acc.push_back(generator.getLinearAcceleration());
        imu_gyr.push_back(generator.getAngularVelocity());
        imu_quat.push_back(quat);

        if (publish_count % DataGenerator::IMU_PER_IMG == 0)
        {
            const auto raw = generator.getImage();
            FrameRecord frame;
            frame.t = t;
            frame.position = pos;
            frame.quat = quat;
            for (const auto &id_pts : raw)
            {
                const int packed_id = id_pts.first;
                const int feature_id = packed_id / num_cam;
                const int camera_id = packed_id % num_cam;
                frame.observations.emplace_back(packed_id, feature_id, camera_id, id_pts.second);
            }
            frames.push_back(std::move(frame));
        }

        generator.update();
        publish_count++;
    }

    const auto cloud = generator.getCloud();
    const Eigen::Vector3d acc_bias = generator.getAccelerometerBias();
    const Eigen::Vector3d gyr_bias = generator.getGyroscopeBias();

    std::ofstream out(out_path);
    if (!out)
    {
        std::cerr << "Failed to open " << out_path << "\n";
        return 1;
    }

    out << std::setprecision(17);
    out << "{\n";
    out << "\"version\":1,\n";
    out << "\"num_cam\":" << num_cam << ",\n";
    out << "\"freq\":" << DataGenerator::FREQ << ",\n";
    out << "\"imu_per_img\":" << DataGenerator::IMU_PER_IMG << ",\n";
    out << "\"max_time\":" << DataGenerator::MAX_TIME << ",\n";
    out << "\"duration\":" << t_end << ",\n";

    out << "\"extrinsics\":[\n";
    for (int k = 0; k < num_cam; ++k)
    {
        if (k > 0)
            out << ",\n";
        out << "{\"camera_id\":" << k
            << ",\"ric\":" << mat3ToJson(generator.getRic(k))
            << ",\"tic\":" << vec3ToJson(generator.getTic(k)) << "}";
    }
    out << "],\n";

    out << "\"acc_bias\":" << vec3ToJson(acc_bias) << ",\n";
    out << "\"gyr_bias\":" << vec3ToJson(gyr_bias) << ",\n";

    out << "\"landmarks\":[\n";
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        if (i > 0)
            out << ",\n";
        out << vec3ToJson(cloud[i]);
    }
    out << "],\n";

    out << "\"imu\":{\n";
    out << "\"t\":[";
    for (size_t i = 0; i < imu_t.size(); ++i)
    {
        if (i > 0)
            out << ",";
        out << imu_t[i];
    }
    out << "],\n\"position\":[";
    for (size_t i = 0; i < imu_pos.size(); ++i)
    {
        if (i > 0)
            out << ",";
        out << vec3ToJson(imu_pos[i]);
    }
    out << "],\n\"velocity_body\":[";
    for (size_t i = 0; i < imu_vel_body.size(); ++i)
    {
        if (i > 0)
            out << ",";
        out << vec3ToJson(imu_vel_body[i]);
    }
    out << "],\n\"quaternion_wxyz\":[";
    for (size_t i = 0; i < imu_quat.size(); ++i)
    {
        if (i > 0)
            out << ",";
        out << quatToJson(imu_quat[i]);
    }
    out << "],\n\"acc\":[";
    for (size_t i = 0; i < imu_acc.size(); ++i)
    {
        if (i > 0)
            out << ",";
        out << vec3ToJson(imu_acc[i]);
    }
    out << "],\n\"gyr\":[";
    for (size_t i = 0; i < imu_gyr.size(); ++i)
    {
        if (i > 0)
            out << ",";
        out << vec3ToJson(imu_gyr[i]);
    }
    out << "]\n},\n";

    out << "\"image_frames\":[\n";
    for (size_t fi = 0; fi < frames.size(); ++fi)
    {
        const auto &f = frames[fi];
        if (fi > 0)
            out << ",\n";
        out << "{\"t\":" << f.t
            << ",\"position\":" << vec3ToJson(f.position)
            << ",\"quaternion_wxyz\":" << quatToJson(f.quat)
            << ",\"observations\":[";
        for (size_t oi = 0; oi < f.observations.size(); ++oi)
        {
            int packed_id, feature_id, camera_id;
            Eigen::Vector3d ray;
            std::tie(packed_id, feature_id, camera_id, ray) = f.observations[oi];
            if (oi > 0)
                out << ",";
            out << "{\"packed_id\":" << packed_id
                << ",\"feature_id\":" << feature_id
                << ",\"camera_id\":" << camera_id
                << ",\"ray_cam\":" << vec3ToJson(ray) << "}";
        }
        out << "]}";
    }
    out << "\n]\n}\n";

    std::cout << "Wrote " << out_path << " (imu samples=" << imu_t.size()
              << ", image frames=" << frames.size() << ", landmarks=" << cloud.size() << ")\n";
    return 0;
}
