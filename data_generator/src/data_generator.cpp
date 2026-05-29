#include "data_generator.h"

#include <stdexcept>

// ODR definitions for static members exported to pybind (bindings.cpp).
const int DataGenerator::FREQ;
const int DataGenerator::MAX_TIME;

#define SEED 1
#define Y_COS 2
#define Z_COS 2 * 2
#define IMU_NOISE 0
#define IMG_NOISE 0
#define BIAS_ACC 0
#define BIAS_GYR 1

DataGenerator::DataGenerator(bool verbose)
    : DataGenerator(DataGeneratorOptions::legacyDefaults(), verbose)
{
}

DataGenerator::DataGenerator(const DataGeneratorOptions &options, bool verbose)
    : num_cam_(options.num_cam),
      camera_ids_(options.camera_ids),
      ric_(options.ric),
      tic_(options.tic),
      fov_deg_(options.fov_deg),
      num_points_(options.num_points),
      imu_per_img_(options.imu_per_img),
      before_feature_id_(static_cast<size_t>(options.num_cam)),
      current_feature_id_(static_cast<size_t>(options.num_cam)),
      quiet_(!verbose)
{
    if (num_cam_ < 1)
        num_cam_ = 1;
    if (static_cast<int>(camera_ids_.size()) != num_cam_)
    {
        camera_ids_.resize(static_cast<size_t>(num_cam_));
        for (int i = 0; i < num_cam_; ++i)
            camera_ids_[static_cast<size_t>(i)] = i;
    }
    for (int cam_id : camera_ids_)
    {
        if (ric_.find(cam_id) == ric_.end() || tic_.find(cam_id) == tic_.end())
            throw std::runtime_error("DataGenerator: missing ric/tic for camera_id " + std::to_string(cam_id));
    }

    srand(SEED);
    t = 0;
    current_id = 0;

    initLandmarks(verbose);
    initAxis();

    if (NUMBER_OF_AP > 0)
        ap[0] = Vector3d(MAX_BOX, -MAX_BOX, MAX_BOX);
    if (NUMBER_OF_AP > 1)
        ap[1] = Vector3d(-MAX_BOX, MAX_BOX, MAX_BOX);
    if (NUMBER_OF_AP > 2)
        ap[2] = Vector3d(-MAX_BOX, -MAX_BOX, -MAX_BOX);
    if (NUMBER_OF_AP > 3)
        ap[3] = Vector3d(MAX_BOX, MAX_BOX, -MAX_BOX);

    acc_cov = 0.01 * 0.01 * Matrix3d::Identity();
    gyr_cov = 0.001 * 0.001 * Matrix3d::Identity();
    pts_cov = (0.3 / 460) * (0.3 / 460) * Matrix2d::Identity();

    generator = default_random_engine(SEED);
    distribution = normal_distribution<double>(0.0, 1);
}

void DataGenerator::initLandmarks(bool verbose)
{
    pts_.resize(static_cast<size_t>(num_points_ * 3));
    for (int i = 0; i < num_points_; i++)
    {
        pts_[static_cast<size_t>(i * 3 + 0)] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts_[static_cast<size_t>(i * 3 + 1)] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts_[static_cast<size_t>(i * 3 + 2)] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        if (verbose)
            cout << "pts i " << i << " " << pts_[static_cast<size_t>(i * 3 + 0)] << " "
                 << pts_[static_cast<size_t>(i * 3 + 1)] << " " << pts_[static_cast<size_t>(i * 3 + 2)] << endl;
    }
}

void DataGenerator::initAxis()
{
    Axis[0] = Vector3d(10, 0, 0);
    Axis[1] = Vector3d(0, 10, 0);
    Axis[2] = Vector3d(0, 0, 10);
    Axis[3] = Vector3d(-10, 0, 0);
    Axis[4] = Vector3d(0, -10, 0);
    Axis[5] = Vector3d(0, 0, -10);
}

void DataGenerator::update()
{
    t += 1.0 / FREQ;
}

double DataGenerator::getTime()
{
    return t;
}

Vector3d DataGenerator::getPoint(int i)
{
    return Vector3d(pts_[static_cast<size_t>(3 * i)], pts_[static_cast<size_t>(3 * i + 1)],
                    pts_[static_cast<size_t>(3 * i + 2)]);
}

Vector3d DataGenerator::getAP(int i)
{
    return ap[i];
}

Vector3d DataGenerator::getPosition()
{
    double x, y, z;
    if (t < MAX_TIME)
    {
        x = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * Y_COS);
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        x = MAX_BOX / 2.0 - MAX_BOX / 2.0;
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0;
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        x = -MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * Y_COS);
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * Z_COS);
    }

    return Vector3d(x, y, z);
}

Matrix3d DataGenerator::getRotation()
{
    return (AngleAxisd(30.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitX()) *
            AngleAxisd(40.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitY()) *
            AngleAxisd(0, Vector3d::UnitZ()))
        .toRotationMatrix();
}

Vector3d DataGenerator::getAngularVelocity()
{
    const double delta_t = 0.00001;
    Matrix3d rot = getRotation();
    t += delta_t;
    Matrix3d drot = (getRotation() - rot) / delta_t;
    t -= delta_t;
    Matrix3d skew = rot.inverse() * drot;
    if (IMU_NOISE)
    {
        Vector3d disturb = Vector3d(distribution(generator) * sqrt(gyr_cov(0, 0)),
                                    distribution(generator) * sqrt(gyr_cov(1, 1)),
                                    distribution(generator) * sqrt(gyr_cov(2, 2)));
        return disturb + Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
    }
    else
    {
#if BIAS_GYR
        return Vector3d(skew(2, 1) + 0.02, -skew(2, 0) + 0.03, skew(1, 0) + 0.04);
#endif
        return Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
    }
}

Vector3d DataGenerator::getVelocity()
{
    double dx, dy, dz;
    if (t < MAX_TIME)
    {
        dx = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        dz = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        dx = 0.0;
        dy = 0.0;
        dz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        dx = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        dz = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }

    return getRotation().inverse() * Vector3d(dx, dy, dz);
}

Vector3d DataGenerator::getLinearAcceleration()
{
    double ddx, ddy, ddz;
    if (t < MAX_TIME)
    {
        ddx = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS) *
              (1.0 / MAX_TIME * M_PI * Y_COS);
        ddz = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS) *
              (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        ddx = 0.0;
        ddy = 0.0;
        ddz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        ddx = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS) *
              (1.0 / MAX_TIME * M_PI * Y_COS);
        ddz = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS) *
              (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    if (IMU_NOISE)
    {
        Vector3d disturb = Vector3d(distribution(generator) * sqrt(acc_cov(0, 0)),
                                    distribution(generator) * sqrt(acc_cov(1, 1)),
                                    distribution(generator) * sqrt(acc_cov(2, 2)));
        return getRotation().inverse() * (disturb + Vector3d(ddx, ddy, ddz + 9.805));
    }
    else
    {
#if BIAS_ACC
        return getRotation().inverse() * Vector3d(ddx, ddy, ddz + 9.805) + Vector3d(0.01, 0.02, 0.03);
#endif
        return getRotation().inverse() * Vector3d(ddx, ddy, ddz + 9.805);
    }
}

Vector3d DataGenerator::getAccelerometerBias()
{
#if BIAS_ACC
    return Vector3d(0.01, 0.02, 0.03);
#else
    return Vector3d::Zero();
#endif
}

Vector3d DataGenerator::getGyroscopeBias()
{
#if BIAS_GYR
    return Vector3d(0.02, 0.03, 0.04);
#else
    return Vector3d::Zero();
#endif
}

vector<pair<int, Vector3d>> DataGenerator::getImage()
{
    vector<pair<int, Vector3d>> image;
    Vector3d position = getPosition();
    Matrix3d quat = getRotation();
    if (!quiet_)
        printf("max: %d\n", current_id);

    const double fov_rad = M_PI * fov_deg_ / 2.0 / 180.0;

    vector<vector<int>> ids(static_cast<size_t>(num_cam_));
    vector<vector<int>> gr_ids(static_cast<size_t>(num_cam_));
    vector<vector<Vector3d>> cur_pts(static_cast<size_t>(num_cam_));

    for (int k = 0; k < num_cam_; k++)
    {
        const int cam_id = camera_ids_[static_cast<size_t>(k)];
        const Matrix3d &R_ic = ric_.at(cam_id);
        const Vector3d &t_ic = tic_.at(cam_id);

        for (int i = 0; i < num_points_; i++)
        {
            double xx = pts_[static_cast<size_t>(i * 3 + 0)] - position(0);
            double yy = pts_[static_cast<size_t>(i * 3 + 1)] - position(1);
            double zz = pts_[static_cast<size_t>(i * 3 + 2)] - position(2);
            Vector3d local_point = R_ic.inverse() * (quat.inverse() * Vector3d(xx, yy, zz) - t_ic);
            xx = local_point(0);
            yy = local_point(1);
            zz = local_point(2);

            if (zz > 0.0 && std::fabs(atan2(xx, zz)) <= fov_rad && std::fabs(atan2(yy, zz)) <= fov_rad)
            {
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
#if IMG_NOISE
                xx += distribution(generator) * sqrt(pts_cov(0, 0));
                yy += distribution(generator) * sqrt(pts_cov(1, 1));
#endif

                const int n_id = before_feature_id_[static_cast<size_t>(k)].count(i)
                                     ? before_feature_id_[static_cast<size_t>(k)][i]
                                     : -1;
                ids[static_cast<size_t>(k)].push_back(n_id);
                gr_ids[static_cast<size_t>(k)].push_back(i);
                cur_pts[static_cast<size_t>(k)].push_back(Vector3d(xx, yy, zz));
            }
        }

        for (int i = 0; i < 6; i++)
        {
            output_Axis[i].clear();
            Vector3d local_point = R_ic.inverse() * (quat.inverse() * (Axis[i] - position) - t_ic);
            double xx = local_point(0);
            double yy = local_point(1);
            double zz = local_point(2);
            if (zz > 0.0 && std::fabs(atan2(xx, zz)) <= fov_rad && std::fabs(atan2(yy, zz)) <= fov_rad)
            {
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
                local_point = R_ic.inverse() * (quat.inverse() * (Axis[i] + Vector3d(1, 0, 0) - position) - t_ic);
                xx = local_point(0);
                yy = local_point(1);
                zz = local_point(2);
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
                local_point = R_ic.inverse() * (quat.inverse() * (Axis[i] + Vector3d(0, 1, 0) - position) - t_ic);
                xx = local_point(0);
                yy = local_point(1);
                zz = local_point(2);
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
                local_point = R_ic.inverse() * (quat.inverse() * (Axis[i] + Vector3d(0, 0, 1) - position) - t_ic);
                xx = local_point(0);
                yy = local_point(1);
                zz = local_point(2);
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
            }
        }
    }

    output_gr_pts.clear();
    if (!gr_ids.empty())
    {
        for (auto i : gr_ids[0])
            output_gr_pts.emplace_back(pts_[static_cast<size_t>(i * 3 + 0)], pts_[static_cast<size_t>(i * 3 + 1)],
                                       pts_[static_cast<size_t>(i * 3 + 2)]);
    }

    for (int k = 0; k < num_cam_; k++)
    {
        auto &id_list = ids[static_cast<size_t>(k)];
        auto &gr = gr_ids[static_cast<size_t>(k)];
        for (size_t i = 0; i < id_list.size(); i++)
        {
            if (id_list[i] == -1)
                id_list[i] = current_id++;
            current_feature_id_[static_cast<size_t>(k)][gr[i]] = id_list[i];
        }
        std::swap(before_feature_id_[static_cast<size_t>(k)], current_feature_id_[static_cast<size_t>(k)]);
        current_feature_id_[static_cast<size_t>(k)].clear();
    }

    for (int k = 0; k < num_cam_; k++)
    {
        const auto &id_list = ids[static_cast<size_t>(k)];
        const auto &pts_list = cur_pts[static_cast<size_t>(k)];
        for (size_t i = 0; i < id_list.size(); i++)
            image.push_back(make_pair(id_list[i] * num_cam_ + k, pts_list[i]));
    }

    return image;
}

vector<Vector3d> DataGenerator::getCloud()
{
    vector<Vector3d> cloud;
    for (int i = 0; i < num_points_; i++)
        cloud.push_back(getPoint(i));
    return cloud;
}
