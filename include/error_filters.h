#ifndef INCLUDE_ERROR_FILTERS_HPP_
#define INCLUDE_ERROR_FILTERS_HPP_

#include <queue>
#include <vector>

class ErrorFilters {
 private:
    double recursive_average_count;
    double recursive_average_past_mean;

    double window_size;
    double moving_average_count;
    double moving_average_past_mean;
    std::deque<double> past_data_points;


 public:
    ErrorFilters();
    ~ErrorFilters();

    void set_window_size(int size);
    double recursive_average_filter(double data_point);
    double moving_average_filter(double data_point);
};


#endif  // INCLUDE_ERROR_FILTERS_HPP_