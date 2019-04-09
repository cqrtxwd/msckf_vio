#ifndef TIMER_H_
#define TIMER_H_

#include <chrono>

namespace ultimate_msckf_vio {
class Timer {
 public:
  Timer() {
    StartTimer();
  }

  void StartTimer() {
    start_ = std::chrono::system_clock::now();
  }

  double EndTimer() {
    end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = end_ - start_;
    return duration.count() * 1000;
  }


private:
std::chrono::time_point<std::chrono::system_clock> start_, end_;

};

}

#endif // TIMER_H_
