#include "mpu6050_kalman.h"
#include <math.h>

// 平台相关的时间函数 - 需要根据具体平台实现
#ifndef PLATFORM_SPECIFIC_TIMER
// 默认实现，使用标准C库（可能需要根据平台调整）
#include <time.h>
uint32_t get_timestamp_ms(void) {
    return (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC);
}
#endif

// ============================================================================
// 卡尔曼滤波器实现
// ============================================================================

void kalman_init(KalmanFilter* kf, float Q_angle, float Q_gyro, float R_angle) {
    kf->Q_angle = Q_angle;
    kf->Q_gyro = Q_gyro;
    kf->R_angle = R_angle;
    
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    
    // 初始化误差协方差矩阵为零矩阵
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

float kalman_update(KalmanFilter* kf, float newAngle, float newRate, float dt) {
    // 1. 预测步骤（时间更新）
    // 更新角度估计：角度 = 角度 + dt * (角速度 - 偏差)
    kf->angle += dt * (newRate - kf->bias);
    
    // 更新误差协方差矩阵 P
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_gyro * dt;
    
    // 2. 更新步骤（测量更新）
    // 计算卡尔曼增益
    float S = kf->P[0][0] + kf->R_angle;  // 创新协方差
    float K[2];  // 卡尔曼增益
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    // 计算角度残差
    float y = newAngle - kf->angle;
    
    // 更新状态估计
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    
    // 更新误差协方差
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}

// ============================================================================
// 低通滤波器实现
// ============================================================================

void low_pass_filter_init(LowPassFilter* lpf, float alpha) {
    lpf->alpha = alpha;
    lpf->prev_value = 0.0f;
    lpf->is_initialized = 0;
}

float low_pass_filter_update(LowPassFilter* lpf, float new_value) {
    // 如果是第一次更新，直接使用新值
    if (!lpf->is_initialized) {
        lpf->prev_value = new_value;
        lpf->is_initialized = 1;
        return new_value;
    }
    
    // 应用一阶低通滤波公式: y[n] = α * x[n] + (1-α) * y[n-1]
    float filtered_value = lpf->alpha * new_value + (1.0f - lpf->alpha) * lpf->prev_value;
    lpf->prev_value = filtered_value;
    
    return filtered_value;
}

// ============================================================================
// 工具函数
// ============================================================================

void calculate_angles_from_accel_2(int16_t accel[3], float* roll, float* pitch) {
    // 转换为g单位
    float ax = accel[0] / ACCEL_SENSITIVITY;
    float ay = accel[1] / ACCEL_SENSITIVITY;
    float az = accel[2] / ACCEL_SENSITIVITY;
    
    // 计算滚转角（绕X轴旋转）
    *roll = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
    
    // 计算俯仰角（绕Y轴旋转）
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

void get_gyro_rates(int16_t gyro[3], float* roll_rate, float* pitch_rate, float* yaw_rate) {
    *roll_rate = gyro[0] / GYRO_SENSITIVITY;   // X轴角速度 (°/s)
    *pitch_rate = gyro[1] / GYRO_SENSITIVITY;  // Y轴角速度 (°/s)
    *yaw_rate = gyro[2] / GYRO_SENSITIVITY;    // Z轴角速度 (°/s)
}

void normalize_angles(EulerAngles* angles) {
    // 将角度限制在-180°到180°范围内
    while (angles->roll > 180.0f) angles->roll -= 360.0f;
    while (angles->roll < -180.0f) angles->roll += 360.0f;
    
    while (angles->pitch > 180.0f) angles->pitch -= 360.0f;
    while (angles->pitch < -180.0f) angles->pitch += 360.0f;
    
    while (angles->yaw > 180.0f) angles->yaw -= 360.0f;
    while (angles->yaw < -180.0f) angles->yaw += 360.0f;
}

// ============================================================================
// 姿态估计器实现
// ============================================================================

void attitude_estimator_init(AttitudeEstimator* estimator) {
    attitude_estimator_init_with_params(estimator, 
                                       KF_Q_ANGLE_DEFAULT, 
                                       KF_Q_GYRO_DEFAULT, 
                                       KF_R_ANGLE_DEFAULT,
                                       LP_FILTER_ALPHA_DEFAULT);
}

void attitude_estimator_init_with_params(AttitudeEstimator* estimator, 
                                        float Q_angle, float Q_gyro, float R_angle,
                                        float lpf_alpha) {
    // 初始化卡尔曼滤波器
    kalman_init(&estimator->roll_filter, Q_angle, Q_gyro, R_angle);
    kalman_init(&estimator->pitch_filter, Q_angle, Q_gyro, R_angle);
    
    // 初始化低通滤波器
    low_pass_filter_init(&estimator->roll_lpf, lpf_alpha);
    low_pass_filter_init(&estimator->pitch_lpf, lpf_alpha);
    low_pass_filter_init(&estimator->yaw_lpf, lpf_alpha);
    
    // 初始化角度
    estimator->angles.roll = 0.0f;
    estimator->angles.pitch = 0.0f;
    estimator->angles.yaw = 0.0f;
    
    estimator->filtered_angles.roll = 0.0f;
    estimator->filtered_angles.pitch = 0.0f;
    estimator->filtered_angles.yaw = 0.0f;
    
    estimator->last_update_time = 0;
    estimator->is_initialized = 0;
}

void update_attitude(AttitudeEstimator* estimator, int16_t accel[3], int16_t gyro[3], uint32_t current_time) {
    // 如果是第一次更新，只初始化时间戳
    if (!estimator->is_initialized) {
        estimator->last_update_time = current_time;
        estimator->is_initialized = 1;
        return;
    }
    
    // 计算时间差（秒）
    float dt = (current_time - estimator->last_update_time) / 1000.0f;
    estimator->last_update_time = current_time;
    
    // 确保时间差合理（防止异常值）
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;  // 默认10ms
    }
    
    // 从加速度计计算角度（作为测量值）
    float accel_roll, accel_pitch;
    calculate_angles_from_accel_2(accel, &accel_roll, &accel_pitch);
    
    // 从陀螺仪获取角速度（作为预测值）
    float gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate;
    get_gyro_rates(gyro, &gyro_roll_rate, &gyro_pitch_rate, &gyro_yaw_rate);
    
    // 使用卡尔曼滤波融合数据（滚转角和俯仰角）
    estimator->angles.roll = kalman_update(&estimator->roll_filter, accel_roll, gyro_roll_rate, dt);
    estimator->angles.pitch = kalman_update(&estimator->pitch_filter, accel_pitch, gyro_pitch_rate, dt);
    
    // 偏航角只能从陀螺仪积分得到（会漂移，实际应用中需要磁力计校准）
    estimator->angles.yaw += gyro_yaw_rate * dt;
    
    // 规范化角度范围
    normalize_angles(&estimator->angles);
    
    // 对卡尔曼滤波后的角度应用低通滤波
    estimator->filtered_angles.roll = low_pass_filter_update(&estimator->roll_lpf, estimator->angles.roll);
    estimator->filtered_angles.pitch = low_pass_filter_update(&estimator->pitch_lpf, estimator->angles.pitch);
    estimator->filtered_angles.yaw = low_pass_filter_update(&estimator->yaw_lpf, estimator->angles.yaw);
    
    // 确保滤波后的角度也在合理范围内
    normalize_angles(&estimator->filtered_angles);
}

EulerAngles get_filtered_angles(AttitudeEstimator* estimator) {
    return estimator->filtered_angles;
}