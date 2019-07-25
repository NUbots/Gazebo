#ifndef GAZEBO_CLOCK_UTILS_H
#define GAZEBO_CLOCK_UTILS_H

#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <chrono>
#include <cmath>

// Convert a frequency into a duration
template <typename period>
struct Per;

template <typename Unit, std::intmax_t num, std::intmax_t den>
struct Per<std::chrono::duration<Unit, std::ratio<num, den>>> : public std::chrono::steady_clock::duration {
    Per(int ticks)
        : std::chrono::steady_clock::duration(std::lround(
              (double(num) / double(ticks* den))
              * (double(std::chrono::steady_clock::period::den) / double(std::chrono::steady_clock::period::num)))) {}
};

// Cast a google::protobuf::Timestamp to a std::chrono::steady_clock::time_point
inline std::chrono::steady_clock::time_point time_point_cast(const ::google::protobuf::Timestamp& time_point) {
    // Get our seconds and nanos in c++ land
    auto seconds = std::chrono::seconds(time_point.seconds());
    auto nanos   = std::chrono::nanoseconds(time_point.nanos());

    // Make a timestamp out of the summation of them
    return std::chrono::steady_clock::time_point(seconds + nanos);
}

// Cast a std::chrono::steady_clock::time_point to a google::protobuf::Timestamp
inline ::google::protobuf::Timestamp time_point_cast(const std::chrono::steady_clock::time_point& t) {
    // Get the epoch timestamp
    auto d = t.time_since_epoch();

    // Get our seconds and the remainder nanoseconds
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
    auto nanos   = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);

    // Set our seconds and nanoseconds
    ::google::protobuf::Timestamp proto;
    proto.set_seconds(seconds.count());
    proto.set_nanos(nanos.count());
    return proto;
}

#endif  // GAZEBO_CLOCK_UTILS_H
