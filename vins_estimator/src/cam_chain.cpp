#include "cam_chain.h"

#include <log_value/log_macros.h>

#include <cmath>
#include <cctype>
#include <fstream>
#include <sstream>

namespace {

std::string trim(const std::string &s)
{
    size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])))
        ++b;
    size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
        --e;
    return s.substr(b, e - b);
}

bool parseBracketDoubles(const std::string &line, std::vector<double> &out)
{
    const size_t lb = line.find('[');
    const size_t rb = line.find(']');
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb)
        return false;

    out.clear();
    std::string inner = line.substr(lb + 1, rb - lb - 1);
    std::stringstream ss(inner);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        token = trim(token);
        if (token.empty())
            continue;
        out.push_back(std::stod(token));
    }
    return !out.empty();
}

bool parseBracketInts(const std::string &line, std::vector<int> &out)
{
    std::vector<double> tmp;
    if (!parseBracketDoubles(line, tmp))
        return false;
    out.clear();
    for (double v : tmp)
        out.push_back(static_cast<int>(std::lround(v)));
    return !out.empty();
}

int camIndexFromLine(const std::string &line)
{
    const std::string t = trim(line);
    if (t.size() < 5 || t.compare(0, 3, "cam") != 0)
        return -1;
    size_t colon = t.find(':');
    if (colon == std::string::npos)
        return -1;
    try
    {
        return std::stoi(t.substr(3, colon - 3));
    }
    catch (...)
    {
        return -1;
    }
}

bool readMatrix4x4FromStream(std::istream &in, Eigen::Matrix4d &T)
{
    T.setIdentity();
    std::string line;
    int row = 0;
    while (row < 4 && std::getline(in, line))
    {
        if (!line.empty() && line[0] == '#')
            continue;
        const std::string t = trim(line);
        if (t.empty())
            continue;
        std::vector<double> vals;
        if (parseBracketDoubles(t, vals) && vals.size() >= 4)
        {
            for (int c = 0; c < 4; ++c)
                T(row, c) = vals[static_cast<size_t>(c)];
            ++row;
        }
    }
    return row == 4;
}

void rtFromT(const Eigen::Matrix4d &T, Eigen::Matrix3d &R, Eigen::Vector3d &t)
{
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
    Eigen::Quaterniond Q(R);
    R = Q.normalized().toRotationMatrix();
}

const CamNode *findNode(const CamChainFile &chain, int cam_id)
{
    const auto it = chain.cameras.find(cam_id);
    if (it == chain.cameras.end())
        return nullptr;
    return &it->second;
}

const CamIntrinsics *intrinsicsForCamera(const CamChainFile &imu_chain, const CamChainFile &stereo_chain, int cam_id)
{
    if (const CamNode *n = findNode(stereo_chain, cam_id); n && n->has_intrinsics)
        return &n->intrinsics;
    if (const CamNode *n = findNode(imu_chain, cam_id); n && n->has_intrinsics)
        return &n->intrinsics;
    return nullptr;
}

} // namespace

std::string resolveCamChainPath(const std::string &config_file, const std::string &cam_chain_file_key)
{
    std::string config_dir;
    const size_t pos = config_file.find_last_of("/\\");
    if (pos != std::string::npos)
        config_dir = config_file.substr(0, pos + 1);

    std::string chain_file = cam_chain_file_key;
    if (chain_file.empty())
        chain_file = "cam_chain.yaml";

    if (!chain_file.empty() && chain_file[0] != '/')
        chain_file = config_dir + chain_file;
    return chain_file;
}

std::string resolveCamChainImuPath(const std::string &config_file, const std::string &cam_chain_imu_file_key)
{
    std::string config_dir;
    const size_t pos = config_file.find_last_of("/\\");
    if (pos != std::string::npos)
        config_dir = config_file.substr(0, pos + 1);

    std::string chain_file = cam_chain_imu_file_key;
    if (chain_file.empty())
        chain_file = "cam_chain-imucam.yaml";

    if (!chain_file.empty() && chain_file[0] != '/')
        chain_file = config_dir + chain_file;
    return chain_file;
}

bool loadCamChainYaml(const std::string &path, CamChainFile &out)
{
    out = CamChainFile{};
    std::ifstream in(path);
    if (!in.good())
        return false;

    int current_cam = -1;
    CamIntrinsics pending_intr;
    std::string line;

    while (std::getline(in, line))
    {
        if (!line.empty() && line[0] == '#')
            continue;

        const int cam_idx = camIndexFromLine(line);
        if (cam_idx >= 0)
        {
            current_cam = cam_idx;
            pending_intr = CamIntrinsics{};
            if (out.cameras.find(cam_idx) == out.cameras.end())
                out.cameras[cam_idx] = CamNode{};
            continue;
        }

        if (current_cam < 0)
            continue;

        CamNode &node = out.cameras[current_cam];
        const std::string t = trim(line);

        if (t.find("intrinsics:") != std::string::npos)
        {
            std::vector<double> v;
            if (parseBracketDoubles(t, v) && v.size() >= 4)
            {
                pending_intr.fu = v[0];
                pending_intr.fv = v[1];
                pending_intr.cu = v[2];
                pending_intr.cv = v[3];
            }
            continue;
        }

        if (t.find("resolution:") != std::string::npos)
        {
            std::vector<int> v;
            if (parseBracketInts(t, v) && v.size() >= 2)
            {
                pending_intr.width = v[0];
                pending_intr.height = v[1];
                node.intrinsics = pending_intr;
                node.has_intrinsics = pending_intr.width > 0 && pending_intr.fu > 0.0;
            }
            continue;
        }

        if (t.find("T_cam_imu:") != std::string::npos || t.find("T_cn_cnm1:") != std::string::npos)
        {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            if (!readMatrix4x4FromStream(in, T))
                return false;
            if (t.find("T_cam_imu:") != std::string::npos)
            {
                node.T_cam_imu = T;
                node.has_T_cam_imu = true;
            }
            else
            {
                node.T_cn_cnm1 = T;
                node.has_T_cn_cnm1 = true;
            }
            continue;
        }
    }

    if (out.cameras.empty())
    {
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain: no cameras in ", path);
        return false;
    }
    return true;
}

bool ricTicFromTCamImu(const Eigen::Matrix4d &T_cam_imu, Eigen::Matrix3d &R_ic, Eigen::Vector3d &t_ic)
{
    const Eigen::Matrix4d T_imu_cam = T_cam_imu.inverse();
    rtFromT(T_imu_cam, R_ic, t_ic);
    return true;
}

bool imuExtrinsicForCamera(int cam_id,
                           const Eigen::Matrix4d &T_cam_imu_cam0,
                           const CamChainFile &stereo_chain,
                           Eigen::Matrix3d &R_ic,
                           Eigen::Vector3d &t_ic)
{
    if (cam_id < 0)
        return false;

    if (cam_id == 0)
        return ricTicFromTCamImu(T_cam_imu_cam0, R_ic, t_ic);

    Eigen::Matrix4d T_ck_c0 = Eigen::Matrix4d::Identity();
    for (int n = 1; n <= cam_id; ++n)
    {
        const CamNode *node = findNode(stereo_chain, n);
        if (!node || !node->has_T_cn_cnm1)
            return false;
        T_ck_c0 = node->T_cn_cnm1 * T_ck_c0;
    }

    const Eigen::Matrix4d T_imu_c0 = T_cam_imu_cam0.inverse();
    const Eigen::Matrix4d T_imu_ck = T_imu_c0 * T_ck_c0.inverse();
    rtFromT(T_imu_ck, R_ic, t_ic);
    return true;
}

bool applyCameraSetup(int num_cam,
                      const std::vector<int> &camera_ids,
                      const CamChainFile &imu_cam_chain,
                      const CamChainFile &stereo_chain,
                      double &focal_length_out,
                      double &image_width_out,
                      double &image_height_out,
                      std::vector<Eigen::Matrix3d> &ric_out,
                      std::vector<Eigen::Vector3d> &tic_out)
{
    if (num_cam < 1 || static_cast<int>(camera_ids.size()) != num_cam)
        return false;

    const CamNode *imu_cam0 = findNode(imu_cam_chain, 0);
    if (!imu_cam0 || !imu_cam0->has_T_cam_imu)
    {
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain-imucam: missing cam0 T_cam_imu");
        return false;
    }

    const Eigen::Matrix4d &T_cam_imu_cam0 = imu_cam0->T_cam_imu;

    ric_out.clear();
    tic_out.clear();
    ric_out.reserve(static_cast<size_t>(num_cam));
    tic_out.reserve(static_cast<size_t>(num_cam));

    for (int slot = 0; slot < num_cam; ++slot)
    {
        const int cam_id = camera_ids[static_cast<size_t>(slot)];
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        if (!imuExtrinsicForCamera(cam_id, T_cam_imu_cam0, stereo_chain, R, t))
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "failed IMU extrinsic for cam", cam_id);
            return false;
        }
        ric_out.push_back(R);
        tic_out.push_back(t);
    }

    const int primary_id = camera_ids[0];
    const CamIntrinsics *intr = intrinsicsForCamera(imu_cam_chain, stereo_chain, primary_id);
    if (!intr)
    {
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "missing intrinsics for cam", primary_id);
        return false;
    }

    focal_length_out = 0.5 * (intr->fu + intr->fv);
    image_width_out = static_cast<double>(intr->width);
    image_height_out = static_cast<double>(intr->height);
    return focal_length_out > 0.0 && image_width_out > 0.0 && image_height_out > 0.0;
}
