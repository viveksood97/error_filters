#include <iostream>
#include <vector>
#include "matplotlib.h"
#include "data.h"
#include "error_filters.h"


namespace plt = matplotlibcpp;
int main() {
    std::vector<double> new_data_points;
    ErrorFilters error_filter;
    for(double raw_data_point: imu_data) {
        new_data_points.push_back(error_filter.recursive_average_filter(raw_data_point));
    }
    plt::plot(new_data_points);
    plt::plot(imu_data);
    plt::show();
    return 0;
}