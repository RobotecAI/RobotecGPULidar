	#pragma once

#include <chrono>
#include <fmt/color.h>
#include <fmt/os.h>
#include <functional>
#include <numeric>
#include <map>
#include <vector>

#define PROFILE_GPU_LIDAR 0

struct Clock
{
	using InternalClock = std::chrono::high_resolution_clock;

	static std::map<std::string, InternalClock::time_point> clock_start;
	static std::map<std::string, std::vector<double>> clock_measures;


	Clock(std::string tag) : tag(tag) {
		if (!PROFILE_GPU_LIDAR) {
			return;
		}
		clock_start[tag] = InternalClock::now();
	}

	~Clock() {
		if (!PROFILE_GPU_LIDAR) {
			return;
		}
		auto diff = static_cast<std::chrono::duration<double>>(InternalClock::now() - clock_start[tag]).count() * 1000.0;
		clock_measures[tag].push_back(diff);
	}

	static void printReset() {
		if (!PROFILE_GPU_LIDAR) {
			return;
		}
		auto out = fmt::output_file("/tmp/results.txt");
		out.print("tag,mean,median,min,max,samples,sum\n");
		for (auto&& kv : clock_measures) {
			auto& tag = kv.first;
			auto& measures = kv.second;
			std::sort(measures.begin(), measures.end());
			auto sum = std::accumulate(measures.begin(), measures.end(), 0.0);

			out.print("{},{},{},{},{},{},{}\n",
					   tag, sum/measures.size(),measures[measures.size()/2], *measures.begin(), *measures.rbegin(), measures.size(), sum);
			measures.clear();
		}
	}

	std::string tag;
};
