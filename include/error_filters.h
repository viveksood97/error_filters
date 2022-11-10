#ifndef INCLUDE_ERROR_FILTERS_HPP_
#define INCLUDE_ERROR_FILTERS_HPP_

class ErrorFilters {
 private:
    double recursive_average_count;
    double recursive_average_past_mean;

    double window_size;
    double moving_average_count;
    double moving_average_past_mean;

 public:
    ErrorFilters();
    ~ErrorFilters();
    double recursive_average_filter(double data_point);
    double moving_average_filter(double data_point);
};


#endif  // INCLUDE_ERROR_FILTERS_HPP_