#ifndef _H_TIMING__
#define _H_TIMING__

#include <chrono>

class Timing {

	std::chrono::high_resolution_clock::time_point start_time_;
	std::chrono::high_resolution_clock::time_point stop_time_;
public: 
	Timing() {
	}

	void start() {
		start_time_ = std::chrono::high_resolution_clock::now();
	}

	void stop() {
		stop_time_ = std::chrono::high_resolution_clock::now();
	}

	float duration() {
		return std::chrono::duration_cast<std::chrono::microseconds>(stop_time_ - start_time_).count();
	}
};

#endif 