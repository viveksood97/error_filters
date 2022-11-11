#include <iostream>
#include <vector>
#include "matplotlib.h"
#include "data.h"
#include "error_filters.h"


namespace plt = matplotlibcpp;
int main() {
    std::vector<double> new_data_points_avg;
    std::vector<double> new_data_points_moving_avg;
    ErrorFilters error_filter;
    for(double raw_data_point: imu_data) {
        new_data_points_avg.push_back(error_filter.recursive_average_filter(raw_data_point));

        error_filter.set_window_size(10);
        new_data_points_moving_avg.push_back(error_filter.moving_average_filter(raw_data_point));
    }
    plt::plot(new_data_points_avg);
    plt::plot(new_data_points_moving_avg);
    plt::plot(imu_data);
    plt::show();
    return 0;
}