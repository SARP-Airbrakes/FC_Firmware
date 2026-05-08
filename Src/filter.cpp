
#include <filter.hpp>

static const filter::vec3 ACCELEROMETER_VARIANCE = filter::vec3(1.5696e-3f, 1.5696e-3f, 1.8639e-3f);
static const float BARO_VARIANCE = 0.08f;

static const filter::vec3 ACCELEROMETER_BIAS_VARIANCE = filter::vec3(3.85e-6f, 3.85e-6f, 3.85e-6f);
static const float BARO_BIAS_VARIANCE = 8e-4f;
static const float BARO_MOVING_BIAS_VARIANCE = 1e-8f;

static const filter::vec3 GRAVITY = filter::vec3(0, 0, -9.81f);

filter::filter()
{
    // assume we are still
    estimate = vecS::Zero();

    // we know nothing
    estimate_covariance = matS::Identity() * 10.0f;
}

filter::vec3 filter::get_filtered_acceleration(const vec3 &raw_acceleration_mps2) const
{
    return raw_acceleration_mps2 - estimate.segment<3>(6) + GRAVITY;
}

void filter::reinitialize(float altitude_m)
{
    estimate = vecS::Zero();
    estimate(2) = altitude_m;
    estimate_covariance = matS::Identity() * 0.5f;
}

void filter::predict(float delta_time, const vec3 &acceleration_mps2)
{
    // Basic integration for position using acceleration corrected with bias
    vec3 filtered_acceleration = acceleration_mps2 + GRAVITY;
    estimate.segment<3>(3) += filtered_acceleration * delta_time;

    // \phi_k=I+F\Delta t
    matS phi_k = matS::Identity();
    phi_k.block<3, 3>(0, 3) = mat3::Identity() * delta_time;
    phi_k.block<3, 3>(3, 6) = mat3::Identity() * -delta_time;

    // ignore the fact that this is really stupid
    phi_k(9, 10) = filtered_acceleration(2) - estimate(8);

    estimate = phi_k * estimate;

    matS process_noise = calculate_process_noise(delta_time, acceleration_mps2);
    estimate_covariance = phi_k * estimate_covariance * phi_k.transpose() + process_noise;
}

filter::matS filter::calculate_process_noise(float delta_time, const vec3 &acceleration_mps2)
{
    float moving_bias = estimate(10);
    float upward_velocity = estimate(5);
    float filtered_acceleration = acceleration_mps2(2) - estimate(8) + GRAVITY(2);

    matS Q = matS::Zero();
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
    Q(2, 9) = moving_bias * ACCELEROMETER_VARIANCE(2) * powf(delta_time, 2.0f) / 2.0f;
    Q(9, 2) = Q(2, 9);
    Q(5, 9) = moving_bias * ACCELEROMETER_VARIANCE(2) * delta_time;
    Q(9, 5) = Q(5, 9);

    // variance of the barometer bias
    Q(9, 9) = powf(moving_bias, 2.0f) * ACCELEROMETER_VARIANCE(2) * delta_time +
        BARO_BIAS_VARIANCE * delta_time + BARO_MOVING_BIAS_VARIANCE *
        powf(upward_velocity, 2.0f) * delta_time + BARO_MOVING_BIAS_VARIANCE *
        filtered_acceleration * powf(delta_time, 2.0f) +
        BARO_MOVING_BIAS_VARIANCE * powf(filtered_acceleration * delta_time,
                2.0f) * delta_time / 3.0f;

    Q(9, 10) = BARO_MOVING_BIAS_VARIANCE * upward_velocity * delta_time +
        BARO_MOVING_BIAS_VARIANCE * filtered_acceleration * powf(delta_time,
                2.0f) / 2.0f;
    Q(10, 9) = Q(9, 10);
    Q(10, 10) = BARO_MOVING_BIAS_VARIANCE * delta_time;
    return Q;
}

void filter::correct_barometer(float altitude_m)
{
    vecS HT = vecS::Zero();
    HT(2) = 1;
    HT(9) = 1;
    
    auto gain = estimate_covariance * HT / (HT.transpose() * estimate_covariance * HT + BARO_VARIANCE);
    estimate = estimate + gain * (altitude_m - HT.transpose() * estimate);
    estimate_covariance = (matS::Identity() - gain * HT.transpose()) * estimate_covariance;
}

void filter::correct_accelerometer(const vec3 &acceleration_mps2)
{
    vec3 filtered_acceleration = acceleration_mps2 + GRAVITY;

    mat3xS H = mat3xS::Zero();
    H.block<3, 3>(0, 6) = mat3::Identity();

    auto gain = estimate_covariance * H.transpose() * (H * estimate_covariance *
            H.transpose() + ACCELEROMETER_VARIANCE.asDiagonal().toDenseMatrix()).inverse();
    estimate = estimate + gain * (filtered_acceleration - H * estimate);
    estimate_covariance = (matS::Identity() - gain * H) * estimate_covariance;
}

