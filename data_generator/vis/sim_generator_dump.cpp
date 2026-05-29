/**
 * Export DataGenerator ground-truth trajectory, IMU, and observations to JSON.
 */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "../src/data_generator.h"
#include "../src/data_generator_config.h"
#include "../../vins_estimator/src/parameters.h"
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
    const std::string out_path = (argc > 1) ? argv[1] : std::string("data_generator/vis/output/sim_dump.json");
    const double duration_scale = (argc > 2) ? std::stod(argv[2]) : 3.0;
    const std::string config_path =
        (argc > 3) ? argv[3] : std::string("config/simulation/simulation_config.yaml");

    try
    {
        readParameters(config_path);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to readParameters(" << config_path << "): " << e.what() << "\n";
        return 1;
    }

    DataGeneratorOptions opts;
    try
    {
        opts = dataGeneratorOptionsFromVinsParameters();
        dataGeneratorLoadDefaultConfig(opts);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to build DataGenerator options: " << e.what() << "\n";
        return 1;
    }

    DataGenerator generator(opts, false);
    generator.setQuiet(true);

    const int num_cam = generator.numCameras();
    const int imu_per_img = generator.imuPerImage();
    const double t_end = duration_scale * DataGenerator::MAX_TIME;

    std::vector<double> imu_t;
    std::vector<Eigen::Vector3d> imu_pos, imu_vel_body, imu_acc, imu_gyr;
    std::vector<Eigen::Quaterniond> imu_quat;

    struct FrameRecord
    {
        double t;
        Eigen::Vector3d position;
        Eigen::Quaterniond quat;
        std::vector<int> observed_landmark_ids;
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

        if (publish_count % imu_per_img == 0)
        {
            const auto raw = generator.getImage();
            FrameRecord frame;
            frame.t = t;
            frame.position = pos;
            frame.quat = quat;
            std::unordered_set<int> union_lm;
            for (const auto &ids : generator.output_gr_ids)
                for (int lm : ids)
                    union_lm.insert(lm);
            frame.observed_landmark_ids.assign(union_lm.begin(), union_lm.end());
            for (const auto &id_pts : raw)
            {
                const int packed_id = id_pts.first;
                const int feature_id = packed_id / num_cam;
                const int slot = packed_id % num_cam;
                const int cam_id = generator.cameraId(slot);
                frame.observations.emplace_back(packed_id, feature_id, cam_id, id_pts.second);
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
    out << "\"camera_ids\":[";
    for (int k = 0; k < num_cam; ++k)
    {
        if (k > 0)
            out << ",";
        out << generator.cameraId(k);
    }
    out << "],\n";
    out << "\"freq\":" << DataGenerator::FREQ << ",\n";
    out << "\"imu_per_img\":" << imu_per_img << ",\n";
    out << "\"max_time\":" << DataGenerator::MAX_TIME << ",\n";
    out << "\"duration\":" << t_end << ",\n";
    out << "\"config_file\":\"" << config_path << "\",\n";

    out << "\"extrinsics\":[\n";
    for (int k = 0; k < num_cam; ++k)
    {
        if (k > 0)
            out << ",\n";
        out << "{\"slot\":" << k
            << ",\"camera_id\":" << generator.cameraId(k)
            << ",\"ric\":" << mat3ToJson(generator.getRic(generator.cameraId(k)))
            << ",\"tic\":" << vec3ToJson(generator.getTic(generator.cameraId(k))) << "}";
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
            << ",\"observed_landmark_ids\":[";
        for (size_t li = 0; li < f.observed_landmark_ids.size(); ++li)
        {
            if (li > 0)
                out << ",";
            out << f.observed_landmark_ids[li];
        }
        out << "],\"observations\":[";
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

    std::cout << "Wrote " << out_path << " (config=" << config_path << ", imu=" << imu_t.size()
              << ", frames=" << frames.size() << ", cam=" << num_cam << ")\n";
    return 0;
}
