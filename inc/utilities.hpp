#pragma once
#include <chrono>
namespace std::chrono {
template <typename Duration>
struct timeval to_timeval(Duration&& d) {
	std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(d);
	struct timeval tv;
	tv.tv_sec = sec.count();
	tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(d - sec).count();
	return tv;
}
}  // namespace std::chrono