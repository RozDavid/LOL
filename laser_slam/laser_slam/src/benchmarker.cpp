#include "laser_slam/benchmarker.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <glog/logging.h>

namespace fs = boost::filesystem;

namespace laser_slam {

//=================================================================================================
//    Benchmarker implementation
//=================================================================================================

#define ALIGN_MODIFIERS std::setw(70) << std::setfill(' ') << std::left
#define FLOAT_MODIFIERS std::fixed << std::setprecision(2)

void Benchmarker::notifyNewStepStart() {
  current_timestamp_ = Clock::now();
  ++current_step_id_;
}

void Benchmarker::startMeasurement(const std::string& topic_name) {
  CHECK_NE(topic_name, "");
  std::lock_guard<std::mutex> lock(started_measurements_mutex_);
  auto result = started_mesurements_.emplace(topic_name, TimePoint());
  if (result.second == false) {
    LOG(WARNING) << "Starting a measurement for topic '" << topic_name << "' twice.";
  }
  result.first->second = Clock::now();
}

void Benchmarker::stopMeasurement(const std::string& topic_name, const bool ignore_measurement) {
  CHECK_NE(topic_name, "");
  const TimePoint end = Clock::now();
  std::lock_guard<std::mutex> lock(started_measurements_mutex_);
  const auto start_it = started_mesurements_.find(topic_name);
  if (start_it != started_mesurements_.end()) {
    if (!ignore_measurement) {
      addMeasurement(topic_name, start_it->second, end);
    }
    started_mesurements_.erase(start_it);
  } else {
    LOG(WARNING) << "Trying to finish a measurement for topic '" << topic_name << "' which has "
        "not been started.";
  }
}

void Benchmarker::addMeasurement(const std::string& topic_name, const TimePoint start,
                                 const TimePoint end) {
  CHECK_NE(topic_name, "");
  double milliseconds = durationToMilliseconds(end - start);

  if (params_.enable_live_output) {
    LOG(INFO) << topic_name << " took " << FLOAT_MODIFIERS << milliseconds << "ms.";
  }

  std::lock_guard<std::mutex> lock(value_topics_mutex_);
  value_topics_["Times." + topic_name].addValue(current_step_id_, current_timestamp_,
                                                milliseconds);
}

void Benchmarker::addValue(const std::string& topic_name, const double value) {
  CHECK_NE(topic_name, "");
  std::lock_guard<std::mutex> lock(value_topics_mutex_);
  value_topics_["Values." + topic_name].addValue(current_step_id_, current_timestamp_, value);
}

void Benchmarker::resetTopic(const std::string& topic_prefix) {
  std::lock_guard<std::mutex> lock(value_topics_mutex_);
  for (auto topic_it = value_topics_.begin(); topic_it != value_topics_.end();) {
    // Delete topics that have the specified prefix.
    if (topic_it->first.find("Times." + topic_prefix) == 0 ||
        topic_it->first.find("Values." + topic_prefix) == 0) {
      value_topics_.erase(topic_it++);
    } else {
      ++topic_it;
    }
  }

  if (topic_prefix == "") {
    current_timestamp_ = Clock::now();
    current_step_id_ = 0u;
  }
}

void Benchmarker::saveData() {
  fs::path root(setupAndGetResultsRootDirectory());

  // Write statistics
  fs::path statistics_file_path = root / fs::path("statistics.txt");
  std::ofstream out_file(statistics_file_path.string());
  if (out_file.is_open()) {
    logStatistics(out_file);
    out_file.close();
  } else {
    LOG(INFO) << "Failed to save statistics results to " << statistics_file_path.string();
  }

  // Write data for each topic
  if (!params_.save_statistics_only) {
    std::lock_guard<std::mutex> lock(value_topics_mutex_);
    TimePoint first_timepoint = getFirstValueTimepoint();
    for (const auto& topic : value_topics_) {
      // Build file name for the topic.
      std::vector<std::string> tokens;
      boost::split(tokens, topic.first, boost::is_any_of(".:-/\\, "));
      fs::path subdir;
      for (auto token_it = tokens.begin(); token_it != tokens.end() - 1; ++token_it) {
        subdir = subdir / *token_it;
      }

      // Create directory and open file.
      fs::create_directories(root / subdir);
      fs::path topic_file_path = root / subdir / fs::path(tokens.back() + ".txt");
      std::ofstream out_file(topic_file_path.string());

      // Write data. Write times in milliseconds relative to the start of the measurements.
      if (out_file.is_open()) {
        for (const auto& value : topic.second.getValues()) {
          out_file << value.step_id << " "
                   << durationToMilliseconds(value.timestamp - first_timepoint) << " "
                   << value.value << std::endl;
        }
        out_file.close();
      } else {
        LOG(INFO) << "Failed to save results to " << topic_file_path.string();
      }
    }
  }

  LOG(INFO) << "Benchmark results saved to " << root.string();
}

void Benchmarker::logStatistics(std::ostream& stream) {
  std::lock_guard<std::mutex> lock(value_topics_mutex_);
  stream << std::endl;
  stream << "Statistics:" << std::endl;
  stream << " " << ALIGN_MODIFIERS << "Topic: " << "Mean (SD)" << std::endl;

  for (const auto& topic : value_topics_) {
    stream << " " << ALIGN_MODIFIERS << (topic.first + ": ")
                  << FLOAT_MODIFIERS << topic.second.getMean() << " ("
                  << FLOAT_MODIFIERS << topic.second.getStandardDeviation() << ")" << std::endl;
  }
  stream << "" << std::endl;
}

std::string Benchmarker::setupAndGetResultsRootDirectory() {
  // Get the current time and use it for creating the directory name
  std::time_t now = Clock::to_time_t(Clock::now());
  char time_buffer[100];
  std::strftime(time_buffer, sizeof(time_buffer), "%F_%H-%M-%S", std::localtime(&now));

  // Create directory
  fs::path root = fs::path(params_.results_directory) / fs::path(std::string(time_buffer));
  fs::create_directories(root);

  return root.string();
}

Benchmarker::TimePoint Benchmarker::getFirstValueTimepoint() {
  TimePoint first_timepoint = TimePoint::max();
  for (const auto& topic : value_topics_) {
    if (!topic.second.getValues().empty()) {
      first_timepoint = std::min(first_timepoint, topic.second.getValues().front().timestamp);
    }
  }
  return first_timepoint;
}

double Benchmarker::durationToMilliseconds(const Duration duration) {
  constexpr double mus_to_ms = 1.0 / 1000.0;
  auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
  return static_cast<double>(microseconds) * mus_to_ms;
}

//=================================================================================================
//    TimerTopic implementation
//=================================================================================================

void Benchmarker::ValueTopic::addValue(const size_t step_id, const TimePoint timestamp,
                                       const double value) {
  sum_ += value;
  sum_of_squares_ += value * value;
  values_count_++;

  if (!Benchmarker::getParameters().save_statistics_only) {
    values_.emplace_back(step_id, timestamp, value);
  }
}

double Benchmarker::ValueTopic::getMean() const {
  return sum_ / static_cast<double>(values_count_);
}

double Benchmarker::ValueTopic::getStandardDeviation() const {
  return sqrt(sum_of_squares_ / static_cast<double>(values_count_) -
              pow(sum_ / static_cast<double>(values_count_), 2.0));
}

//=================================================================================================
//    Benchmarker fields
//=================================================================================================

std::mutex Benchmarker::value_topics_mutex_;
std::mutex Benchmarker::started_measurements_mutex_;
std::map<std::string, Benchmarker::ValueTopic> Benchmarker::value_topics_;
std::unordered_map<std::string, Benchmarker::TimePoint> Benchmarker::started_mesurements_;
Benchmarker::TimePoint Benchmarker::current_timestamp_;
size_t Benchmarker::current_step_id_ = 0u;
BenchmarkerParams Benchmarker::params_;

//=================================================================================================
//    ScopedTimer implementation
//=================================================================================================

ScopedTimer::~ScopedTimer() {
  Benchmarker::addMeasurement(topic_name_, start_, Benchmarker::Clock::now());
}

} // namespace laser_slam

