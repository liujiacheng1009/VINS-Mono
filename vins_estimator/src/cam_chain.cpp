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

Eigen::Matrix4d TImuCam(const Eigen::Matrix3d &R_ic, const Eigen::Vector3d &t_ic)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R_ic;
    T.block<3, 1>(0, 3) = t_ic;
    return T;
}

void rtFromT(const Eigen::Matrix4d &T, Eigen::Matrix3d &R, Eigen::Vector3d &t)
{
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
    Eigen::Quaterniond Q(R);
    R = Q.normalized().toRotationMatrix();
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

bool loadCamChainFile(const std::string &path, CamChainData &out)
{
    out = CamChainData{};
    std::ifstream in(path);
    if (!in.good())
        return false;

    int current_cam = -1;
    bool reading_T = false;
    int T_row = 0;
    Eigen::Matrix4d T_accum = Eigen::Matrix4d::Identity();

    auto flush_cam = [&](int cam_idx, const CamIntrinsics &intr) {
        if (cam_idx < 0)
            return;
        while (static_cast<int>(out.cameras.size()) <= cam_idx)
            out.cameras.push_back(CamIntrinsics{});
        out.cameras[static_cast<size_t>(cam_idx)] = intr;
    };

    CamIntrinsics pending_intr;
    std::string line;
    while (std::getline(in, line))
    {
        if (!line.empty() && line[0] == '#')
            continue;

        const int cam_idx = camIndexFromLine(line);
        if (cam_idx >= 0)
        {
            if (reading_T && T_row == 4 && current_cam >= 1)
                out.T_cn_cnm1.push_back(T_accum);
            reading_T = false;
            T_row = 0;
            current_cam = cam_idx;
            pending_intr = CamIntrinsics{};
            continue;
        }

        if (current_cam < 0)
            continue;

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
                flush_cam(current_cam, pending_intr);
            }
            continue;
        }

        if (t.find("T_cn_cnm1:") != std::string::npos)
        {
            reading_T = true;
            T_row = 0;
            T_accum.setIdentity();
            continue;
        }

        if (reading_T && T_row < 4)
        {
            std::vector<double> row;
            if (parseBracketDoubles(t, row) && row.size() >= 4)
            {
                for (int c = 0; c < 4; ++c)
                    T_accum(T_row, c) = row[static_cast<size_t>(c)];
                ++T_row;
                if (T_row == 4 && current_cam >= 1)
                {
                    out.T_cn_cnm1.push_back(T_accum);
                    reading_T = false;
                    T_row = 0;
                }
            }
        }
    }

    if (reading_T && T_row == 4 && current_cam >= 1)
        out.T_cn_cnm1.push_back(T_accum);

    if (out.cameras.empty())
    {
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain: no cameras in ", path);
        return false;
    }

    for (const auto &intr : out.cameras)
    {
        if (intr.width <= 0 || intr.height <= 0 || intr.fu <= 0.0)
        {
            LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain: incomplete intrinsics in ", path);
            return false;
        }
    }

    if (out.cameras.size() > 1 && out.T_cn_cnm1.size() != out.cameras.size() - 1)
    {
        LOG_TXT_LEVEL(logging::ValueLogger::Level::WARNING, "cam_chain: T_cn_cnm1 count mismatch in ", path);
        return false;
    }
    return true;
}

bool applyCamChain(int num_cam,
                   const CamChainData &chain,
                   const Eigen::Matrix3d &R_ic0,
                   const Eigen::Vector3d &t_ic0,
                   double &focal_length_out,
                   double &image_width_out,
                   double &image_height_out,
                   std::vector<Eigen::Matrix3d> &ric_out,
                   std::vector<Eigen::Vector3d> &tic_out)
{
    if (num_cam < 1 || chain.cameras.empty())
        return false;

    const int use_cam = std::min(num_cam, static_cast<int>(chain.cameras.size()));
    const CamIntrinsics &cam0 = chain.cameras[0];
    focal_length_out = 0.5 * (cam0.fu + cam0.fv);
    image_width_out = static_cast<double>(cam0.width);
    image_height_out = static_cast<double>(cam0.height);

    ric_out.clear();
    tic_out.clear();
    ric_out.reserve(static_cast<size_t>(use_cam));
    tic_out.reserve(static_cast<size_t>(use_cam));

    ric_out.push_back(R_ic0);
    tic_out.push_back(t_ic0);

    if (use_cam == 1)
        return true;

    if (static_cast<int>(chain.T_cn_cnm1.size()) < use_cam - 1)
        return false;

    Eigen::Matrix4d T_imu_c = TImuCam(R_ic0, t_ic0);
    for (int k = 1; k < use_cam; ++k)
    {
        const Eigen::Matrix4d &T_cnm1_cn = chain.T_cn_cnm1[static_cast<size_t>(k - 1)];
        T_imu_c = T_imu_c * T_cnm1_cn.inverse();
        Eigen::Matrix3d R_k;
        Eigen::Vector3d t_k;
        rtFromT(T_imu_c, R_k, t_k);
        ric_out.push_back(R_k);
        tic_out.push_back(t_k);
    }
    return true;
}
