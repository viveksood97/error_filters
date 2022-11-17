#include <iostream>
#include <vector>
#include "matplotlib.h"
#include "data.h"
#include "error_filters.h"


namespace plt = matplotlibcpp;
int main() {
    std::vector<double> new_data_points_avg;
    std::vector<double> new_data_points_moving_avg;
    std::vector<double> new_data_points_exp_moving_avg;

    ErrorFilters error_filter;

    for(double raw_data_point: imu_data) {
        new_data_points_avg.push_back(error_filter.recursive_average_filter(raw_data_point));

        error_filter.set_window_size(10);
        new_data_points_moving_avg.push_back(error_filter.moving_average_filter(raw_data_point));

        error_filter.set_alpha(0.95);
        new_data_points_exp_moving_avg.push_back(error_filter.exp_moving_average_filter(raw_data_point));

    }
    std::cout << "Hello" << std::endl;
    plt::plot(new_data_points_avg, {{"label","Mean Filter"}});
    plt::plot(new_data_points_moving_avg, {{"label","Moving Avg Filter"}});
    plt::plot(new_data_points_exp_moving_avg, {{"label","Exponential Moving Avg Filter"}});
    plt::plot(imu_data, {{"label","Raw Data"}});
    plt::legend();
    plt::show();
    return 0;
}