#pragma once

#include <chrono>
#include <string>
#include <map>
#include <iostream>

namespace cuda_slam {

class Timer {
public:
    Timer() : is_running_(false) {}

    void start() {
        start_time_ = std::chrono::high_resolution_clock::now();
        is_running_ = true;
    }

    void stop() {
        if (is_running_) {
            end_time_ = std::chrono::high_resolution_clock::now();
            is_running_ = false;
        }
    }

    double getElapsedMs() const {
        if (is_running_) {
            auto now = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double, std::milli>(now - start_time_).count();
        }
        return std::chrono::duration<double, std::milli>(end_time_ - start_time_).count();
    }

    double getElapsedSeconds() const {
        return getElapsedMs() / 1000.0;
    }

private:
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point end_time_;
    bool is_running_;
};

class ScopedTimer {
public:
    ScopedTimer(const std::string& name) : name_(name) {
        timer_.start();
    }

    ~ScopedTimer() {
        timer_.stop();
        std::cout << "[Timer] " << name_ << ": "
                  << timer_.getElapsedMs() << " ms" << std::endl;
    }

private:
    std::string name_;
    Timer timer_;
};

class PerformanceMonitor {
public:
    static PerformanceMonitor& getInstance() {
        static PerformanceMonitor instance;
        return instance;
    }

    void recordTime(const std::string& operation, double time_ms) {
        timings_[operation].push_back(time_ms);
        if (timings_[operation].size() > max_samples_) {
            timings_[operation].erase(timings_[operation].begin());
        }
    }

    double getAverageTime(const std::string& operation) const {
        auto it = timings_.find(operation);
        if (it == timings_.end() || it->second.empty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double t : it->second) {
            sum += t;
        }
        return sum / it->second.size();
    }

    void printStatistics() const {
        std::cout << "\n=== Performance Statistics ===" << std::endl;
        for (const auto& [op, times] : timings_) {
            double avg = 0.0;
            for (double t : times) {
                avg += t;
            }
            avg /= times.size();
            std::cout << op << ": " << avg << " ms (avg over "
                      << times.size() << " samples)" << std::endl;
        }
        std::cout << "==============================\n" << std::endl;
    }

    void clear() {
        timings_.clear();
    }

    void setMaxSamples(size_t max_samples) {
        max_samples_ = max_samples;
    }

private:
    PerformanceMonitor() : max_samples_(100) {}
    std::map<std::string, std::vector<double>> timings_;
    size_t max_samples_;
};

#define SCOPED_TIMER(name) ScopedTimer __timer##__LINE__(name)

} // namespace cuda_slam
