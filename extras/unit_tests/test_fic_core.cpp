#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "math/fic_matrix_math.h"
#include "fusion/fic_ahrs_9dof.h"
#include "storage/fic_ring_buf.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

int mat3_det_i8(const int8_t m[3][3])
{
    return (int)m[0][0] * ((int)m[1][1] * (int)m[2][2] - (int)m[2][1] * (int)m[1][2]) -
           (int)m[0][1] * ((int)m[1][0] * (int)m[2][2] - (int)m[1][2] * (int)m[2][0]) +
           (int)m[0][2] * ((int)m[1][0] * (int)m[2][1] - (int)m[1][1] * (int)m[2][0]);
}

void test_matrix_solver_and_ellipsoid(void)
{
    double m[9][10] = {};
    m[0][0] = 3.0; m[0][1] = 1.0; m[0][2] = -1.0;
    m[1][0] = 2.0; m[1][1] = 4.0; m[1][2] =  1.0;
    m[2][0] = -1.0; m[2][1] = 2.0; m[2][2] =  5.0;
    double b[9] = {4.0, 1.0, 1.0};
    double x[9] = {};
    assert(matrix_solve_nxn(3, m, b, x));
    assert(std::fabs(x[0] - 2.0) < 1e-6);
    assert(std::fabs(x[1] + 1.0) < 1e-6);
    assert(std::fabs(x[2] - 1.0) < 1e-6);

    // Synthetic shifted & scaled 3D magnetic ellipsoid: center=(15, -22, 8), scale=(1.2, 0.85, 1.05), R=50uT
    const int kN = 120;
    float xs[kN], ys[kN], zs[kN];
    for (int i = 0; i < kN; i++) {
        float theta = ((float)(i % 20) / 20.0f) * 2.0f * kPi;
        float phi   = (((float)(i / 20) + 0.5f) / 6.0f) * kPi;
        float ux = std::sin(phi) * std::cos(theta);
        float uy = std::sin(phi) * std::sin(theta);
        float uz = std::cos(phi);
        xs[i] = 15.0f + 50.0f * 1.20f * ux;
        ys[i] = -22.0f + 50.0f * 0.85f * uy;
        zs[i] = 8.0f + 50.0f * 1.05f * uz;
    }

    float center[3] = {};
    float soft[3][3] = {};
    float rms_pct = 100.0f;
    assert(fit_ellipsoid_3d_f32(xs, ys, zs, kN, center, soft, &rms_pct));
    assert(std::fabs(center[0] - 15.0f) < 0.25f);
    assert(std::fabs(center[1] + 22.0f) < 0.25f);
    assert(std::fabs(center[2] - 8.0f) < 0.25f);
    assert(rms_pct < 1.0f);

    // Synthetic Kabsch 90-deg CCW rotation around Z: q = R * p => R = [[0,-1,0],[1,0,0],[0,0,1]]
    const float rx[4] = {1.0f, 0.0f, -1.0f, 0.0f};
    const float ry[4] = {0.0f, 1.0f, 0.0f, -1.0f};
    const float rz[4] = {0.2f, -0.3f, 0.4f, -0.1f};
    const float tx[4] = {0.0f, -1.0f, 0.0f, 1.0f};
    const float ty[4] = {1.0f, 0.0f, -1.0f, 0.0f};
    const float tz[4] = {0.2f, -0.3f, 0.4f, -0.1f};
    const float zero_c[3] = {0.0f, 0.0f, 0.0f};
    const float ident[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float R_out[3][3] = {};
    assert(kabsch_align_3x3_f32(rx, ry, rz, tx, ty, tz, 4, zero_c, ident, zero_c, ident, 1.0, R_out));
    assert(std::fabs(R_out[0][1] + 1.0f) < 1e-4f || std::fabs(R_out[0][1] - 1.0f) < 1e-4f);
}

void test_ahrs_9dof(void)
{
    for (uint8_t m = 0; m < AHRS_ORIENT_COUNT; m++) {
        int8_t om[3][3] = {};
        ahrs_9dof_get_orient_matrix(m, om);
        assert(mat3_det_i8(om) == 1);
    }

    Ahrs9DofConfig_t cfg = {};
    ahrs_9dof_default_config(&cfg, AHRS_ORIENT_UP_0);
    Ahrs9DofState_t st = {};
    ahrs_9dof_init(&st, &cfg);

    const float acc[3] = {0.0f, 0.0f, 1.0f};
    const float gyr[3] = {0.3f, -0.2f, 0.4f};
    const float mag[3] = {20.0f, 0.0f, -45.0f};
    Ahrs9DofOutput_t out = {};

    for (int i = 0; i < 120; i++) {
        ahrs_9dof_update(&st, acc, gyr, mag, true, true, 0.02f, &out);
    }
    assert(out.gyro_foc_locked);
    assert(std::fabs(st.gyro_bias_dps[0] - 0.3f) < 0.05f);
    assert(std::fabs(st.gyro_bias_dps[1] + 0.2f) < 0.05f);
    assert(std::fabs(st.gyro_bias_dps[2] - 0.4f) < 0.05f);
    assert(!out.accel_gated);
    assert(std::fabs(out.roll_deg) < 2.0f);
    assert(std::fabs(out.pitch_deg) < 2.0f);
    assert(std::fabs(out.tilt_comp_mag_heading_deg) < 5.0f || out.tilt_comp_mag_heading_deg > 355.0f);

    // High acceleration shock should trigger accel gating
    const float shock_acc[3] = {0.0f, 0.0f, 1.8f};
    ahrs_9dof_update(&st, shock_acc, gyr, mag, true, true, 0.02f, &out);
    assert(out.accel_gated);
}

void test_ring_buffer(void)
{
    uint8_t storage[8] = {};
    FicByteRing_t ring = {};
    fic_ring_buf_init(&ring, storage, sizeof(storage));
    assert(fic_ring_buf_is_empty(&ring));
    assert(fic_ring_buf_free(&ring) == 7U);

    const uint8_t msg1[5] = {'A', 'B', 'C', 'D', 'E'};
    assert(fic_ring_buf_push(&ring, msg1, 5U, false) == 5U);
    assert(fic_ring_buf_available(&ring) == 5U);

    // Drop-newest overflow check (only 2 slots remain)
    const uint8_t msg2[4] = {'1', '2', '3', '4'};
    assert(fic_ring_buf_push(&ring, msg2, 4U, false) == 2U);
    assert(ring.drop_count == 2U);

    uint8_t out[16] = {};
    assert(fic_ring_buf_pop(&ring, out, sizeof(out)) == 7U);
    assert(std::memcmp(out, "ABCDE12", 7U) == 0);
    assert(fic_ring_buf_is_empty(&ring));

    // Drop-oldest wraparound check
    const uint8_t msg3[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    assert(fic_ring_buf_push(&ring, msg3, 10U, true) == 10U);
    assert(fic_ring_buf_pop(&ring, out, sizeof(out)) == 7U);
    assert(std::memcmp(out, "3456789", 7U) == 0);
}

} // namespace

int main(void)
{
    test_matrix_solver_and_ellipsoid();
    test_ahrs_9dof();
    test_ring_buffer();
    std::printf("[PASS] test_fic_core: matrix_math, ahrs_9dof, ring_buf passed\n");
    return 0;
}
