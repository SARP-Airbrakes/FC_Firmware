
#ifndef AIRBRAKES_FILTER_HPP_
#define AIRBRAKES_FILTER_HPP_

#include <Eigen/Eigen>

class filter {
public:
    static constexpr int STATE_DIMENSION = 11;

    using vec3 = Eigen::Vector3f;
    using vecS = Eigen::Matrix<float, STATE_DIMENSION, 1>;
    using mat3 = Eigen::Matrix3f;
    using mat3xS = Eigen::Matrix<float, 3, STATE_DIMENSION>;
    using matS = Eigen::Matrix<float, STATE_DIMENSION, STATE_DIMENSION>;

public:

    filter();

    // Initialize with an initial altitude
    void reinitialize(float altitude_m);

    void predict(float delta_time, const vec3 &acceleration_mps2);
    void correct_barometer(float altitude_m);

    // When we are on the pad, we can assume that we are not accelerating and
    // thus that any acceleration on the pad measured is bias.
    void correct_accelerometer(const vec3 &acceleration_mps2);

    auto get_position() const {
        return estimate.segment<3>(0);
    }

    auto get_velocity() const {
        return estimate.segment<3>(3);
    }

    const auto &get_estimated_state() const {
        return estimate;
    }

    const auto &get_estimated_state_covariance() const {
        return estimate_covariance;
    }

private:
    vecS estimate;
    matS estimate_covariance;

    matS calculate_process_noise(float delta_time, const vec3 &acceleration_mps2);

};

#endif // AIRBRAKES_FILTER_HPP_

