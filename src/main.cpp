#include <iostream>
#include "../include/data.h"


int main() {
    for(int i = 0; i != imu_data.size(); ++i) {
        std::cout << imu_data[i] << std::endl;
    }

    return 0;
}