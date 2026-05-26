#include "state_manager.h"
#include "parameters.h"
#include "utility/utility.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
bool near(double a, double b, double tol = 1e-9) { return std::abs(a - b) < tol; }

bool check(const char *name, bool ok)
{
    if (!ok)
        std::cerr << "FAIL: " << name << '\n';
    return ok;
}
} // namespace

int main()
{
    vinsParameters().setWindowSize(10);
    vinsParameters().setNumOfCam(1);
    vinsParameters().setMaxFeatureCount(1000);

    StateManager sm;
    sm.clear();
    bool ok = true;
    ok &= check("clear P0", sm.positionAtSlot(0).norm() < 1e-12);
    ok &= check("clear slot", sm.slotCount() == 0);

    SimpleHeader hdr;
    hdr.stamp = SimpleTime(1.0);
    sm.initializeFrameAtSlot(0, 1, hdr, Vector3d(1, 2, 3), Matrix3d::Identity(), Vector3d::Zero(),
                             Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero());
    sm.setSlotCount(0);
    ok &= check("contains", sm.contains(1));
    ok &= check("slotOf", sm.slotOf(1).value() == 0);

    sm.syncToParameters();
    const Vector3d p_before = sm.positionAtSlot(0);
    sm.syncFromParameters();
    ok &= check("sync roundtrip P", (sm.positionAtSlot(0) - p_before).norm() < 1e-10);

    sm.setSlotCount(windowSize());
    for (int i = 0; i <= windowSize(); i++)
    {
        SimpleHeader h;
        h.stamp = SimpleTime(static_cast<double>(i));
        sm.bindFrame(i, static_cast<FrameId>(100 + i), h);
        sm.positionAtSlot(i) = Vector3d(i, 0, 0);
    }
    const FrameId moved_id = 101;
    sm.snapshotOldestFrame();
    sm.slideWindowOld();
    ok &= check("slideWindowOld contains moved", sm.contains(moved_id) && sm.slotOf(moved_id).value() == 0);

    const auto blocks = sm.parameterBlocks();
    ok &= check("parameterBlocks pose", blocks.pose != nullptr);
    ok &= check("extrinsic ric", sm.extrinsic(0).ric.isApprox(Matrix3d::Identity(), 1e-9));

    // yaw 对齐：优化后 frame(0) 的 yaw 应与优化前一致
    sm.clear();
    sm.setSlotCount(1);
    for (int i = 0; i <= 1; i++)
    {
        SimpleHeader h;
        h.stamp = SimpleTime(static_cast<double>(i));
        const double yaw = 0.3 * i;
        const Matrix3d R = Utility::ypr2R(Vector3d(yaw, 0.1, 0.0));
        sm.bindFrame(i, static_cast<FrameId>(i + 1), h);
        sm.positionAtSlot(i) = Vector3d(i, 0, 0);
        sm.rotationAtSlot(i) = R;
        sm.velocityAtSlot(i) = Vector3d::Zero();
    }
    const Vector3d ypr_before = Utility::R2ypr(sm.rotationAtSlot(0));
    sm.syncToParameters();
    sm.poseParameter(1)[0] += 0.5; // 扰动 Ceres 参数
    sm.syncFromParameters();
    const Vector3d ypr_after = Utility::R2ypr(sm.rotationAtSlot(0));
    ok &= check("yaw alignment frame0", near(ypr_before.x(), ypr_after.x(), 1e-8));

    std::cout << (ok ? "state_manager_smoke_test: PASS\n" : "state_manager_smoke_test: FAIL\n");
    return ok ? 0 : 1;
}
