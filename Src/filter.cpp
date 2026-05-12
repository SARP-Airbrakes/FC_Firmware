
#include <filter.hpp>

static const float ACCELEROMETER_VARIANCE = 1.8639e-3f;
static const float BARO_VARIANCE = 0.08f;

static const float ACCELEROMETER_BIAS_VARIANCE = 3.85e-6f;
static const float BARO_BIAS_VARIANCE = 8e-4f;
static const float BARO_MOVING_BIAS_VARIANCE = 1e-8f;

static const float GRAVITY = -9.81f;

filter::filter()
{
    // assume we are still
    estimate = vecS::Zero();

    // we know nothing
    estimate_covariance = matS::Identity();
}

filter::vec3 filter::get_filtered_acceleration(const vec3 &raw_acceleration_mps2) const
{
    vec3 out = raw_acceleration_mps2;
    out(2) += -estimate(2) + GRAVITY;
    return out;
}

void filter::reinitialize(float altitude_m)
{
    estimate = vecS::Zero();
    estimate(0) = altitude_m;
    estimate_covariance = matS::Identity();
}

void filter::predict(float delta_time, const vec3 &acceleration_mps2)
{
    // Basic integration for position using acceleration corrected with bias
    float filtered_acceleration = acceleration_mps2(2) + GRAVITY;
    estimate(1) += filtered_acceleration * delta_time;

    // \phi_k=I+F\Delta t
    matS phi_k = matS::Identity();
    phi_k(0, 1) = delta_time;
    phi_k(1, 2) = -delta_time;

    // ignore the fact that this is really stupid
    phi_k(3, 4) = filtered_acceleration - estimate(2);

    estimate = phi_k * estimate;

    matS process_noise = calculate_process_noise(delta_time, acceleration_mps2);
    estimate_covariance = phi_k * estimate_covariance * phi_k.transpose() + process_noise;
}

filter::matS filter::calculate_process_noise(float delta_time, const vec3 &acceleration_mps2)
{
    float moving_bias = estimate(4);
    float upward_velocity = estimate(1);
    float filtered_acceleration = acceleration_mps2(2) - estimate(2) + GRAVITY;

    matS Q = matS::Zero();
    Q(0, 0) = ((ACCELEROMETER_VARIANCE 
        * powf(delta_time, 3) * (1.0f / 3.0f)) + (ACCELEROMETER_BIAS_VARIANCE
        * powf(delta_time, 5) * (1.0f / 20.0f)));
    Q(0, 1) = ((ACCELEROMETER_VARIANCE 
        * powf(delta_time, 2) * (1.0f / 2.0f) + (ACCELEROMETER_BIAS_VARIANCE
        * powf(delta_time, 4) * (1.0f / 8.0f))));
    Q(1, 0) = Q(0, 1);
    Q(1, 1) = ((ACCELEROMETER_VARIANCE * delta_time) +
        (ACCELEROMETER_BIAS_VARIANCE * powf(delta_time, 3) * (1.0f / 3.0f)));
    Q(0, 2) = (ACCELEROMETER_BIAS_VARIANCE *
        -powf(delta_time, 3.0f) * (1.0f / 6.0f));
    Q(1, 2) = (ACCELEROMETER_BIAS_VARIANCE *
        -powf(delta_time, 2.0f) * (1.0f / 2.0f));
    Q(2, 2) = (ACCELEROMETER_BIAS_VARIANCE * delta_time);
    Q(2, 0) = Q(0, 2);
    Q(2, 1) = Q(1, 2);
    Q(0, 3) = moving_bias * ACCELEROMETER_VARIANCE * powf(delta_time, 2.0f) / 2.0f;
    Q(3, 0) = Q(0, 3);
    Q(1, 3) = moving_bias * ACCELEROMETER_VARIANCE * delta_time;
    Q(3, 1) = Q(1, 3);

    // variance of the barometer bias
    Q(3, 3) = powf(moving_bias, 2.0f) * ACCELEROMETER_VARIANCE * delta_time +
        BARO_BIAS_VARIANCE * delta_time + BARO_MOVING_BIAS_VARIANCE *
        powf(upward_velocity, 2.0f) * delta_time + BARO_MOVING_BIAS_VARIANCE *
        filtered_acceleration * powf(delta_time, 2.0f) +
        BARO_MOVING_BIAS_VARIANCE * powf(filtered_acceleration * delta_time,
                2.0f) * delta_time / 3.0f;

    Q(3, 4) = BARO_MOVING_BIAS_VARIANCE * upward_velocity * delta_time +
        BARO_MOVING_BIAS_VARIANCE * filtered_acceleration * powf(delta_time,
                2.0f) / 2.0f;
    Q(4, 3) = Q(3, 4);
    Q(4, 4) = BARO_MOVING_BIAS_VARIANCE * delta_time;
    return Q;
}

void filter::correct_barometer(float altitude_m)
{
    vecS HT = vecS::Zero();
    HT(0) = 1;
    HT(3) = 1;
    
    auto gain = estimate_covariance * HT / (HT.transpose() * estimate_covariance * HT + BARO_VARIANCE);
    estimate = estimate + gain * (altitude_m - HT.transpose() * estimate);
    estimate_covariance = (matS::Identity() - gain * HT.transpose()) * estimate_covariance;
}

void filter::correct_accelerometer(const vec3 &acceleration_mps2)
{
    float filtered_acceleration = acceleration_mps2(2) + GRAVITY;

    vecS HT = vecS::Zero();
    HT(2) = 1;

    auto gain = estimate_covariance * HT / (HT.transpose() * estimate_covariance
            * HT + ACCELEROMETER_VARIANCE);
    estimate = estimate + gain * (filtered_acceleration - HT.transpose() * estimate);
    estimate_covariance = (matS::Identity() - gain * HT.transpose()) * estimate_covariance;
}

