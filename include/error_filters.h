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

    double alpha = 0.9;
    double exp_moving_avg_past_mean = 0;


 public:
    ErrorFilters();
    ~ErrorFilters();

    
    double recursive_average_filter(double data_point);

    void set_window_size(int size);
    double moving_average_filter(double data_point);

    void set_alpha(double alp);
    double exp_moving_average_filter(double data_point);
};


#endif  // INCLUDE_ERROR_FILTERS_HPP_