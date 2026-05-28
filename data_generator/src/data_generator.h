#include <cstdlib>
#include <cmath>
#include <vector>
#include <tuple>
#include <map>
#include <algorithm>
#include <random>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

class DataGenerator
{
  public:
    explicit DataGenerator(bool verbose = true);
    void update();

    double getTime();

    Vector3d getPoint(int i);
    Vector3d getAP(int i);
    vector<Vector3d> getCloud();
    Vector3d getPosition();
    Matrix3d getRotation();
    Vector3d getVelocity();

    Vector3d getAngularVelocity();
    Vector3d getLinearAcceleration();
    Vector3d getAccelerometerBias();
    Vector3d getGyroscopeBias();

    vector<pair<int, Vector3d>> getImage();

    int numCameras() const { return NUMBER_OF_CAMERA; }
    Matrix3d getRic(int k) const { return Ric[k]; }
    Vector3d getTic(int k) const { return Tic[k]; }
    void setQuiet(bool quiet) { quiet_ = quiet; }

    static int const FREQ = 500;
    //static int const MAX_TIME = 10;
    static int const MAX_TIME = 40;
    static int const FOV = 90;

    static int const NUMBER_OF_CAMERA = 1;
    static int const NUMBER_OF_AP = 1;
    static int const NUM_POINTS = 500;
    static int const MAX_BOX = 10;
    static int const IMU_PER_IMG = 50;
    static int const IMU_PER_WIFI = 5;

    vector<Vector3d> output_gr_pts;
    vector<Vector3d> output_Axis[6];

  private:
    int pts[NUM_POINTS * 3];
    double t;
    map<int, int> before_feature_id[NUMBER_OF_CAMERA];
    map<int, int> current_feature_id[NUMBER_OF_CAMERA];
    int current_id;

    Matrix3d Ric[NUMBER_OF_CAMERA];
    Vector3d Tic[NUMBER_OF_CAMERA];
    Vector3d ap[NUMBER_OF_AP];
    Matrix3d acc_cov, gyr_cov;
    Matrix2d pts_cov;
    default_random_engine generator;
    normal_distribution<double> distribution;

    Vector3d Axis[6];
    bool quiet_ = false;
};
