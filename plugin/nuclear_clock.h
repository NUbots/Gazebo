#ifndef GAZEBO_NUCLEAR_CLOCK_H
#define GAZEBO_NUCLEAR_CLOCK_H

#include <chrono>
#include <nuclear>

template <typename period>
struct Per;

template <typename Unit, std::intmax_t num, std::intmax_t den>
struct Per<std::chrono::duration<Unit, std::ratio<num, den>>> : public NUClear::clock::duration {
    Per(int ticks)
        : NUClear::clock::duration(
              std::lround((double(num) / double(ticks* den))
                          * (double(NUClear::clock::period::den) / double(NUClear::clock::period::num)))) {}
};

#endif  // GAZEBO_NUCLEAR_CLOCK_H
