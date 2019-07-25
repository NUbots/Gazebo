#ifndef GAZEBO_CLOCK_UTILS_H
#define GAZEBO_CLOCK_UTILS_H

#include <chrono>

template <typename period>
struct Per;

template <typename Unit, std::intmax_t num, std::intmax_t den>
struct Per<std::chrono::duration<Unit, std::ratio<num, den>>> : public std::chrono::steady_clock::duration {
    Per(int ticks)
        : std::chrono::steady_clock::duration(std::lround(
              (double(num) / double(ticks* den))
              * (double(std::chrono::steady_clock::period::den) / double(std::chrono::steady_clock::period::num)))) {}
};

template <typename T>
inline std::chrono::steady_clock::time_point time_point_cast(const T& time_point) {
    // Get our seconds and nanos in c++ land
    auto seconds = std::chrono::seconds(time_point.seconds());
    auto nanos   = std::chrono::nanoseconds(time_point.nanos());

    // Make a timestamp out of the summation of them
    return std::chrono::steady_clock::time_point(seconds + nanos);
}


#endif  // GAZEBO_CLOCK_UTILS_H
