#pragma once

// A minimal parallel-for over [0, n) on std::thread, for the distance
// field and lattice sampling loops. Chunks are handed out dynamically so
// uneven work (blocks near a body's surface cost far more than blocks
// deep inside or outside it) balances across threads.

#include <algorithm>
#include <atomic>
#include <functional>
#include <thread>
#include <vector>

namespace aerotelic {

inline int DefaultThreadCount() {
	unsigned n = std::thread::hardware_concurrency();
	return n == 0 ? 4 : static_cast<int>(n);
}

// Calls fn(i) for every i in [0, n), on up to `threads` threads (0 = all
// cores). Exceptions thrown by fn are swallowed per call and the first is
// rethrown on the calling thread once every worker has finished.
inline void ParallelFor(size_t n, const std::function<void(size_t)>& fn, int threads = 0, size_t chunk = 1) {
	if (n == 0) return;
	if (threads <= 0) threads = DefaultThreadCount();
	threads = static_cast<int>(std::min<size_t>(static_cast<size_t>(threads), n));
	if (threads <= 1) {
		for (size_t i = 0; i < n; ++i) fn(i);
		return;
	}
	std::atomic<size_t> next{0};
	std::exception_ptr first_error;
	std::atomic<bool> failed{false};
	auto worker = [&]() {
		for (;;) {
			size_t start = next.fetch_add(chunk);
			if (start >= n) break;
			size_t end = std::min(n, start + chunk);
			for (size_t i = start; i < end; ++i) {
				if (failed.load()) return;
				try {
					fn(i);
				} catch (...) {
					if (!failed.exchange(true)) first_error = std::current_exception();
					return;
				}
			}
		}
	};
	std::vector<std::thread> pool;
	pool.reserve(threads - 1);
	for (int t = 1; t < threads; ++t) pool.emplace_back(worker);
	worker();
	for (auto& th : pool) th.join();
	if (first_error) std::rethrow_exception(first_error);
}

} // namespace aerotelic
