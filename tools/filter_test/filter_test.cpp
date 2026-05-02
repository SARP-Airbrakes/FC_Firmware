
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <utility>
#include <iostream>
#include <filter.hpp>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " [data]\n";
        return 1;
    }

    std::string path(argv[1]);
    std::ifstream stream(path, std::ios::binary);

    if (!stream.is_open()) {
        std::cout << "Failed to open " << path << "\n";
        return 1;
    }

    std::string buf;
    filter test_filter;

    float last_time = -1.0f;
    bool flying = false;

    std::ofstream outs("out.csv");
    if (!outs.is_open()) {
        return 2;
    }

    outs << "packet_id,time_s,altitude_m,vertical_velocity_mps,accelerometer_bias_mps2,baro_bias_m,altitude_stddev,vertical_velocity_stddev,accelerometer_bias_stddev,baro_bias_stddev\n";

    while (std::getline(stream, buf)) {
        std::istringstream line_stream(buf);
        std::vector<std::string> line;
        while (std::getline(line_stream, buf, ',')) {
            line.push_back(buf);
        }
        
        try {
            int packet_id(std::stoi(line[0]));
            float time_s(std::stof(line[1]));
            float accel_x_mps2(std::stof(line[2]));
            float accel_y_mps2(std::stof(line[3]));
            float accel_z_mps2(std::stof(line[4]));
            float baro_altitude_m(std::stof(line[9]));

            if (last_time == -1.0f) {
                test_filter.reinitialize(baro_altitude_m);
                last_time = time_s;
            } else {
                float delta_time = time_s - last_time;
                last_time = time_s;

                if (accel_z_mps2 > 12.0f) {
                    flying = true;
                }

                filter::vec3 accel_mps2(accel_x_mps2, accel_y_mps2, accel_z_mps2);

                test_filter.predict(delta_time, accel_mps2);
                test_filter.correct_barometer(baro_altitude_m);

                if (!flying)
                    test_filter.correct_accelerometer(accel_mps2);
                if (flying)
                    std::cout << "FLYING!!\n";

                std::cout << "packet_id: " << packet_id << "\n";
                std::cout << "estimate:\n" << test_filter.get_estimated_state() << "\n";
                std::cout << "estimate_covariance:\n" << test_filter.get_estimated_state_covariance() << "\n";
                std::cout << "\n";

                outs << packet_id << ",";
                outs << time_s << ",";
                outs << test_filter.get_position()(2) << ",";
                outs << test_filter.get_velocity()(2) << ",";
                outs << test_filter.get_estimated_state()(8) << ",";
                outs << test_filter.get_estimated_state()(9) << ",";
                outs << std::sqrtf(test_filter.get_estimated_state_covariance()(2, 2)) << ",";
                outs << std::sqrtf(test_filter.get_estimated_state_covariance()(5, 5)) << ",";
                outs << std::sqrtf(test_filter.get_estimated_state_covariance()(8, 8)) << ",";
                outs << std::sqrtf(test_filter.get_estimated_state_covariance()(9, 9)) << "\n";
            }
        } catch (std::invalid_argument const& ex) {
            std::cerr << "std::invalid_argument::what(): " << ex.what() << "\n";
        }
    }

    return 0;
}
