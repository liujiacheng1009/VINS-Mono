#include <iostream>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "../data_generator/src/data_generator.h"
#include "../vins_estimator/src/estimator.h"
#include "../vins_estimator/src/parameters.h"

int main(int argc, char **argv)
{
    const std::string config_path =
        (argc > 1) ? argv[1] : std::string("../config/simulation/simulation_config.yaml");

    readParameters(config_path);

    Estimator estimator;
    estimator.setParameter();

    DataGenerator generator;
    int publish_count = 0;
    bool init_feature = false;
    double current_time = -1.0;

    std::unordered_map<int, Eigen::Vector2d> prev_uv;
    double prev_image_time = -1.0;
    std::vector<double> abs_pos_errors_raw;
    std::vector<double> abs_pos_errors_aligned;
    bool has_align_offset = false;
    Eigen::Vector3d align_offset = Eigen::Vector3d::Zero();

    while (generator.getTime() <= 3.0 * DataGenerator::MAX_TIME)
    {
        const double t = generator.getTime();
        const Eigen::Vector3d gt_position = generator.getPosition();
        const Eigen::Vector3d acc = generator.getLinearAcceleration();
        const Eigen::Vector3d gyr = generator.getAngularVelocity();

        if (current_time < 0)
            current_time = t;
        const double dt = t - current_time;
        if (dt > 0)
        {
            estimator.processIMU(dt, acc, gyr);
            current_time = t;
        }

        if (publish_count % DataGenerator::IMU_PER_IMG == 0)
        {
            const auto raw_features = generator.getImage();
            if (!init_feature)
            {
                init_feature = true;
            }
            else
            {
                SimpleHeader header;
                header.stamp = SimpleTime(t);
                header.frame_id = "world";

                std::unordered_map<int, Eigen::Vector2d> cur_uv;
                std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> image;
                for (const auto &id_pts : raw_features)
                {
                    const int packed_id = id_pts.first;
                    const int feature_id = packed_id / NUM_OF_CAM;
                    const int camera_id = packed_id % NUM_OF_CAM;
                    const double x = id_pts.second.x();
                    const double y = id_pts.second.y();
                    const double z = id_pts.second.z();
                    if (z == 0.0)
                        continue;

                    const Eigen::Vector2d uv(x / z, y / z);
                    Eigen::Vector2d vel(0.0, 0.0);
                    const auto it = prev_uv.find(feature_id);
                    if (it != prev_uv.end() && prev_image_time > 0.0 && t > prev_image_time)
                        vel = (uv - it->second) / (t - prev_image_time);

                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, uv.x(), uv.y(), vel.x(), vel.y();
                    image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
                    cur_uv[feature_id] = uv;
                }

                estimator.processImage(image, header);
                prev_uv.swap(cur_uv);
                prev_image_time = t;

                if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                {
                    const auto &p = estimator.Ps[WINDOW_SIZE];
                    const double raw_err = (p - gt_position).norm();
                    abs_pos_errors_raw.push_back(raw_err);

                    if (!has_align_offset)
                    {
                        align_offset = gt_position - p;
                        has_align_offset = true;
                    }
                    const double aligned_err = ((p + align_offset) - gt_position).norm();
                    abs_pos_errors_aligned.push_back(aligned_err);
                    std::cout << "t=" << t << " p=(" << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
                }
            }
        }

        generator.update();
        publish_count++;
    }

    std::cout << "Simulation done.\n";
    auto print_metrics = [](const std::vector<double> &errors, const std::string &tag)
    {
        double sum = 0.0;
        double sum_sq = 0.0;
        double max_err = 0.0;
        for (double e : errors)
        {
            sum += e;
            sum_sq += e * e;
            if (e > max_err)
                max_err = e;
        }
        const double mae = sum / errors.size();
        const double rmse = std::sqrt(sum_sq / errors.size());
        const double final_err = errors.back();
        std::cout << "[metrics][" << tag << "] samples=" << errors.size()
                  << " mae=" << mae
                  << " rmse=" << rmse
                  << " max=" << max_err
                  << " final=" << final_err << "\n";
    };

    if (!abs_pos_errors_raw.empty())
    {
        print_metrics(abs_pos_errors_raw, "raw");
        print_metrics(abs_pos_errors_aligned, "aligned");
    }
    else
    {
        std::cout << "[metrics] no NON_LINEAR samples, metrics unavailable.\n";
    }
    return 0;
}
