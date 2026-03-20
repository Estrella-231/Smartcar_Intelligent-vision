#include "gyroscope.h"

IMU_Data_t g_imu = {0};

/*
 * Read one fresh IMU sample from the vendor driver and cache the raw Z-axis
 * gyro value into the project-owned IMU state.
 *
 * Keeping the raw value is useful during debug because it lets us separate:
 * - "SPI communication is alive"
 * - "bias removal and yaw integration are correct"
 */
void gyro_read_data(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    g_imu.gyro_z_raw = gyro_z;
}

/*
 * Reset all runtime yaw state without touching the already estimated static
 * bias. This is useful after calibration when the team wants to redefine the
 * current heading as zero again.
 */
void imu_reset_yaw(void)
{
    g_imu.gyro_z_calib = 0;
    g_imu.gyro_z_dps = 0;
    g_imu.gyro_acc_filtered = 0;
    g_imu.gyro_z_dps2 = 0;
    g_imu.rotate_angle = 0;
    g_imu.rotate_angle_total = 0;
    g_imu.last_gyro_z_dps = 0;
}

/*
 * Bring up the IMU and estimate static Z-axis gyro bias while the car is kept
 * completely still.
 */
uint8_t imu_init_and_calibrate(void)
{
    int64_t gyro_bias_sum = 0;

    memset(&g_imu, 0, sizeof(g_imu));

    if(imu963ra_init())
    {
        g_imu.init_ok = 0;
        return 0;
    }

    /*
     * Average multiple static samples instead of using a hard-coded bias.
     * Real sensor bias changes with board, temperature, and mounting stress.
     */
    for(uint32_t i = 0; i < IMU_CALIB_SAMPLE_COUNT; i++)
    {
        imu963ra_get_gyro();
        gyro_bias_sum += imu963ra_gyro_z;
        system_delay_ms(IMU_CALIB_SAMPLE_DELAY_MS);
    }

    g_imu.gyro_z_bias_raw = (int32_t)(gyro_bias_sum / IMU_CALIB_SAMPLE_COUNT);
    g_imu.init_ok = 1;
    imu_reset_yaw();

    return 1;
}

uint8_t imu_is_ready(void)
{
    return g_imu.init_ok;
}

int32_t imu_get_gyro_z_raw(void)
{
    return g_imu.gyro_z_raw;
}

int32_t imu_get_yaw_cd(void)
{
    return g_imu.rotate_angle;
}

int32_t imu_get_gyro_z_cdps(void)
{
    return g_imu.gyro_z_dps;
}

/*
 * Remove the static bias estimated at startup.
 *
 * The result is intentionally kept in raw LSB here. Physical-unit conversion is
 * done in a separate step so communication, calibration, and unit conversion can
 * each be debugged independently.
 */
void IMU_calib_data(void)
{
    g_imu.gyro_z_calib = g_imu.gyro_z_raw - g_imu.gyro_z_bias_raw;
}

/*
 * Convert the calibrated raw gyro value into 0.01 degree/s.
 *
 * Scale:
 * - raw_lsb / 14.3 = deg/s
 * - deg/s * 100    = centi-deg/s
 *
 * To avoid floating-point math in the fast path:
 * - centi-deg/s = raw_lsb * 1000 / 143
 */
void IMU_Convert_Physical(void)
{
    g_imu.gyro_z_dps = (g_imu.gyro_z_calib * 1000) / GYRO_LSB_PER_DPS_X10;

    /*
     * Small residual bias and sensor noise should not keep integrating forever
     * while the car is standing still.
     *
     * Important:
     * - only the angular rate is dead-banded
     * - the accumulated yaw angle is not cleared here
     *
     * That preserves heading while still stopping tiny noise from accumulating.
     */
    if(abs(g_imu.gyro_z_dps) < GYRO_RATE_DEADBAND_CDPS)
    {
        g_imu.gyro_z_dps = 0;
    }
}

/*
 * Integrate yaw with an explicit sampling period.
 *
 * Task 8 adds a dedicated 5 ms PIT interrupt, so the integration code must use
 * the real interrupt period instead of assuming one fixed compile-time value.
 *
 * Units:
 * - gyro_z_dps   : 0.01 deg/s
 * - period_ms    : ms
 * - rotate_angle : 0.01 deg
 *
 * Formula:
 * - angle += rate * dt
 * - cdeg  += cdeg/s * ms / 1000
 */
void IMU_Calc_Rotate_Angle_Period(uint32_t period_ms)
{
    g_imu.rotate_angle_total += (g_imu.gyro_z_dps * (int32_t)period_ms) / 1000;
    g_imu.rotate_angle = g_imu.rotate_angle_total;
}

/*
 * Differentiate angular rate using the real update period.
 *
 * The filtered angular-acceleration value is not the main target of Task 8, but
 * keeping it period-correct avoids silently breaking later control code that
 * reuses this field.
 */
void IMU_Calc_Rotate_Acc_Period(uint32_t period_ms)
{
    if(period_ms == 0)
    {
        g_imu.gyro_z_dps2 = 0;
        return;
    }

    g_imu.gyro_z_dps2 =
        ((g_imu.gyro_z_dps - g_imu.last_gyro_z_dps) * 1000) / (int32_t)period_ms;
    g_imu.gyro_acc_filtered = (g_imu.gyro_acc_filtered * 4 + g_imu.gyro_z_dps2) / 5;
    g_imu.gyro_z_dps2 = g_imu.gyro_acc_filtered;
    g_imu.last_gyro_z_dps = g_imu.gyro_z_dps;
}

/*
 * Full IMU processing chain with explicit period:
 * raw sample -> bias removal -> physical conversion -> yaw integration.
 */
void imu_deal_data_period(uint32_t period_ms)
{
    if(!g_imu.init_ok || period_ms == 0)
    {
        return;
    }

    gyro_read_data();
    IMU_calib_data();
    IMU_Convert_Physical();
    IMU_Calc_Rotate_Angle_Period(period_ms);
    IMU_Calc_Rotate_Acc_Period(period_ms);
}

/*
 * Legacy wrappers kept for earlier code paths that still assume one fixed
 * PID_CONTROL_PERIOD.
 */
void IMU_Calc_Rotate_Angle(void)
{
    IMU_Calc_Rotate_Angle_Period(PID_CONTROL_PERIOD);
}

void IMU_Calc_Rotate_Acc(void)
{
    IMU_Calc_Rotate_Acc_Period(PID_CONTROL_PERIOD);
}

void imu_deal_data(void)
{
    imu_deal_data_period(PID_CONTROL_PERIOD);
}
