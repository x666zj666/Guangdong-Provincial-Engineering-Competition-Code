#ifndef MPU6050_KALMAN_H
#define MPU6050_KALMAN_H

#include <stdint.h>

// MPU6050传感器灵敏度定义
#define ACCEL_SENSITIVITY  16384.0f  // ±2g量程, LSB/g
#define GYRO_SENSITIVITY  131.0f    // ±250°/s量程, LSB/°/s

// 单位转换常量
#define RAD_TO_DEG        57.295779513f
#define DEG_TO_RAD        0.01745329251f
#define GRAVITY           9.80665f

// 卡尔曼滤波器默认参数
#define KF_Q_ANGLE_DEFAULT  0.001f
#define KF_Q_GYRO_DEFAULT   0.003f
#define KF_R_ANGLE_DEFAULT  0.03f

// 低通滤波器默认参数
#define LP_FILTER_ALPHA_DEFAULT 0.2f  // 默认滤波系数

// 欧拉角结构体
typedef struct {
    float roll;   // 横滚角 (度)
    float pitch;  // 俯仰角 (度) 
    float yaw;    // 偏航角 (度)
} EulerAngles;

// 卡尔曼滤波器结构体
typedef struct {
    float Q_angle;   // 过程噪声协方差（角度）
    float Q_gyro;    // 过程噪声协方差（陀螺仪）
    float R_angle;   // 测量噪声协方差
    
    float angle;     // 估计的角度
    float bias;      // 估计的陀螺仪偏差
    float P[2][2];   // 误差协方差矩阵
} KalmanFilter;

// 一阶低通滤波器结构体
typedef struct {
    float alpha;     // 滤波系数 (0-1)
    float prev_value; // 上一次的滤波值
    int is_initialized; // 是否已初始化
} LowPassFilter;

// 姿态估计器结构体
typedef struct {
    KalmanFilter roll_filter;
    KalmanFilter pitch_filter;
    LowPassFilter roll_lpf;
    LowPassFilter pitch_lpf;
    LowPassFilter yaw_lpf;
    EulerAngles angles;           // 卡尔曼滤波后的角度
    EulerAngles filtered_angles;  // 低通滤波后的角度
    uint32_t last_update_time;
    int is_initialized;
} AttitudeEstimator;

// 函数声明

// 卡尔曼滤波器函数
void kalman_init(KalmanFilter* kf, float Q_angle, float Q_gyro, float R_angle);
float kalman_update(KalmanFilter* kf, float newAngle, float newRate, float dt);

// 低通滤波器函数
void low_pass_filter_init(LowPassFilter* lpf, float alpha);
float low_pass_filter_update(LowPassFilter* lpf, float new_value);

// 姿态估计器函数
void attitude_estimator_init(AttitudeEstimator* estimator);
void attitude_estimator_init_with_params(AttitudeEstimator* estimator, 
                                        float Q_angle, float Q_gyro, float R_angle,
                                        float lpf_alpha);
void update_attitude(AttitudeEstimator* estimator, int16_t accel[3], int16_t gyro[3], uint32_t current_time);
EulerAngles get_filtered_angles(AttitudeEstimator* estimator);

// 工具函数
void calculate_angles_from_accel_2(int16_t accel[3], float* roll, float* pitch);
void get_gyro_rates(int16_t gyro[3], float* roll_rate, float* pitch_rate, float* yaw_rate);
void normalize_angles(EulerAngles* angles);
uint32_t get_timestamp_ms(void);

#endif // MPU6050_KALMAN_H