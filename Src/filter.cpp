
#include <filter.hpp>

static const filter::vec3 ACCELEROMETER_VARIANCE = filter::vec3(1.5696e-3f, 1.5696e-3f, 1.8639e-3f);
static const filter::vec3 ACCELEROMETER_BIAS_VARIANCE = filter::vec3(1.962e-3f, 1.962e-3f, 1.962e-3f);
static const float BARO_VARIANCE = 0.09f;
static const float BARO_BIAS_VARIANCE = 8.0e-2f;
static const filter::vec3 GRAVITY = filter::vec3(0, 0, -9.81f);

filter::filter()
{
    // assume we are still
    estimate = vec10::Zero();

    // we know nothing
    estimate_covariance = mat10::Identity() * 10.0f;
}

void filter::reinitialize(float altitude_m)
{
    estimate = vec10::Zero();
    estimate(2) = altitude_m;
    estimate_covariance = mat10::Identity() * 0.5f;
}

void filter::predict(float delta_time, const vec3 &acceleration_mps2)
{
    // Basic integration for position using acceleration corrected with bias
    vec3 filtered_acceleration = acceleration_mps2 + GRAVITY;
    estimate.block<3, 1>(3, 0) += filtered_acceleration * delta_time;

    // \phi_k=I+F\Delta t
    mat10 phi_k = mat10::Identity();
    phi_k.block<3, 3>(0, 3) = mat3::Identity() * delta_time;
    phi_k.block<3, 3>(3, 6) = mat3::Identity() * -delta_time;

    estimate = phi_k * estimate;

    mat10 process_noise = calculate_process_noise(delta_time);
    estimate_covariance = phi_k * estimate_covariance * phi_k.transpose() + process_noise;
}

filter::mat10 filter::calculate_process_noise(float delta_time)
{
    mat10 Q = mat10::Zero();
    Q.block<3, 3>(0, 0) = ((ACCELEROMETER_VARIANCE.asDiagonal() 
        * powf(delta_time, 3) * (1.0f / 3.0f)) + (ACCELEROMETER_BIAS_VARIANCE.asDiagonal()
        * powf(delta_time, 5) * (1.0f / 20.0f)));
    Q.block<3, 3>(0, 3) = ((ACCELEROMETER_VARIANCE.asDiagonal() 
        * powf(delta_time, 2) * (1.0f / 2.0f) + (ACCELEROMETER_BIAS_VARIANCE.asDiagonal()
        * powf(delta_time, 4) * (1.0f / 8.0f))));
    Q.block<3, 3>(3, 0) = Q.block<3, 3>(0, 3);
    Q.block<3, 3>(3, 3) = ((ACCELEROMETER_VARIANCE.asDiagonal() * delta_time) +
        (ACCELEROMETER_BIAS_VARIANCE.asDiagonal() * powf(delta_time, 3) * (1.0f / 3.0f)));
    Q.block<3, 3>(0, 6) = (ACCELEROMETER_BIAS_VARIANCE.asDiagonal() *
        -powf(delta_time, 3.0f) * (1.0f / 6.0f));
    Q.block<3, 3>(3, 6) = (ACCELEROMETER_BIAS_VARIANCE.asDiagonal() *
        -powf(delta_time, 2.0f) * (1.0f / 2.0f));
    Q.block<3, 3>(6, 6) = (ACCELEROMETER_BIAS_VARIANCE.asDiagonal() * delta_time);
    Q.block<3, 3>(6, 0) = Q.block<3, 3>(0, 6);
    Q.block<3, 3>(6, 3) = Q.block<3, 3>(3, 6);
    Q(9, 9) = BARO_BIAS_VARIANCE * delta_time;
    return Q;
}

void filter::correct_barometer(float altitude_m)
{
    vec10 HT = vec10::Zero();
    HT(2) = 1;
    HT(9) = 1;
    
    auto gain = estimate_covariance * HT / (HT.transpose() * estimate_covariance * HT + BARO_VARIANCE);
    estimate = estimate + gain * (altitude_m - HT.transpose() * estimate);
    estimate_covariance = (mat10::Identity() - gain * HT.transpose()) * estimate_covariance;
}

void filter::correct_accelerometer(const vec3 &acceleration_mps2)
{
    vec3 filtered_acceleration = acceleration_mps2 + GRAVITY;

    mat3x10 H = mat3x10::Zero();
    H.block<3, 3>(0, 6) = mat3::Identity();

    auto gain = estimate_covariance * H.transpose() * (H * estimate_covariance *
            H.transpose() + ACCELEROMETER_VARIANCE.asDiagonal().toDenseMatrix()).inverse();
    estimate = estimate + gain * (filtered_acceleration - H * estimate);
    estimate_covariance = (mat10::Identity() - gain * H) * estimate_covariance;
}

