//
// Created by miles on 7/12/21.
//

// For reasons I don't really understand, c++ only understands gettimeofday when Transmission.h is included
//    Transmission is built automatically by catkin so it must include some sort of secret sauce?

#include <ros/time.h>
//#include <time.h>

long millitime() {
    // i love oop but this snippet belongs in the tenth circle of hell (verbosity)
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

