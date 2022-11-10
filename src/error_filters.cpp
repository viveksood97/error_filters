#include "error_filters.h"


ErrorFilters::ErrorFilters() {
    recursive_average_count = 1;
}

ErrorFilters::~ErrorFilters() {
}

double ErrorFilters::recursive_average_filter(double data_point) {
    double filtered_point = ((recursive_average_count - 1) * recursive_average_past_mean / recursive_average_count) + data_point/recursive_average_count;
    recursive_average_past_mean = filtered_point;
    ++recursive_average_count;
    return filtered_point;
}


double ErrorFilters::moving_average_filter(double data_point) {
    // double filtered_point = moving_average_past_mean + (data_point - )
    // return filtered_point;
}

