#include "error_filters.h"
#include <numeric>

ErrorFilters::ErrorFilters()
{
    recursive_average_count = 1;

    for (int i = 0; i != window_size; ++i){
        past_data_points.push_back(0.0);
    }
}

ErrorFilters::~ErrorFilters()
{
}

double ErrorFilters::recursive_average_filter(double data_point)
{
    double filtered_point = ((recursive_average_count - 1) * recursive_average_past_mean / recursive_average_count) + data_point / recursive_average_count;
    recursive_average_past_mean = filtered_point;
    ++recursive_average_count;
    return filtered_point;
}

double ErrorFilters::moving_average_filter(double data_point)
{
    past_data_points.pop_front();
    past_data_points.push_back(data_point);
    return std::accumulate(past_data_points.begin(), past_data_points.end(), 0.000f)/window_size;
    // return filtered_point;
}


void ErrorFilters::set_window_size(int size) {
    this->window_size = size;
}
