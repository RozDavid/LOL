#ifndef LASER_SLAM_BENCHMARKER_HPP_
#define LASER_SLAM_BENCHMARKER_HPP_

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace laser_slam {

// In order to use the benchmarker define the BENCHMARK_ENABLE macro in your project.

#ifdef BENCHMARK_ENABLE
/// \brief Notifies the benchmarker that a new step started. All measurements and values added
/// after this call will have the same timestamp.
#define BENCHMARK_START_NEW_STEP() laser_slam::Benchmarker::notifyNewStepStart();
/// \brief Measure the time elapsed from this line to the end of the scope and record it to the
/// \c topic_name topic.
#define BENCHMARK_BLOCK(topic_name) laser_slam::ScopedTimer __timer_ ## __LINE__(topic_name);
/// \brief Starts a measurement for the \c topic_name topic.
#define BENCHMARK_START(topic_name) laser_slam::Benchmarker::startMeasurement(topic_name);
/// \brief Stops a measurement for the \c topic_name topic and store the result.
#define BENCHMARK_STOP(topic_name) laser_slam::Benchmarker::stopMeasurement(topic_name);
/// \brief Stops a measurement for the \c topic_name topic and ignore the result.
#define BENCHMARK_STOP_AND_IGNORE(topic_name) \
  laser_slam::Benchmarker::stopMeasurement(topic_name, true);
/// \brief Records a value in the \c topic_name topic. Only values of type \c double are supported.
#define BENCHMARK_RECORD_VALUE(topic_name, value) \
    laser_slam::Benchmarker::addValue(topic_name, static_cast<double>(value));
/// \brief Reset the topics with name starting with \c topic_prefix.
#define BENCHMARK_RESET(topic_prefix) laser_slam::Benchmarker::resetTopic(topic_prefix);
/// \brief Reset all topics.
#define BENCHMARK_RESET_ALL() laser_slam::Benchmarker::resetTopic("");
#else
#define BENCHMARK_START_NEW_STEP()
#define BENCHMARK_BLOCK(topic_name)
#define BENCHMARK_START(topic_name)
#define BENCHMARK_STOP(topic_name)
#define BENCHMARK_STOP_AND_IGNORE(topic_name)
#define BENCHMARK_RECORD_VALUE(topic_name, value)
#define BENCHMARK_RESET(topic_prefix)
#define BENCHMARK_RESET_ALL()
#endif

/// \brief Parameters of the benchmarker.
struct BenchmarkerParams {
  /// \brief If true, the benchmarker will collect only statistics, without storing the actual
  /// measurements.
  bool save_statistics_only;
  /// \brief If true, every collected measurement or value will printed to the logger.
  bool enable_live_output;
  /// \brief Path where the results will be saved.
  std::string results_directory;
};

/// \brief Benchmark helper class. Allows collecting data and statistics about execution times and
/// metrics (values).
/// \remark This class is thread-safe, but simultaneously collecting measurements for the same
/// topic from multiple threads is not supported.
class Benchmarker {

 public:
  /// \brief The clock used by the benchmarker.
  typedef std::chrono::high_resolution_clock Clock;
  typedef Clock::time_point TimePoint;
  typedef Clock::duration Duration;

  /// \brief Prevent static class from being instantiated.
  Benchmarker() = delete;

  /// \brief Notifies the benchmarker that a new step started. All measurements and values added
  /// after this call will have the same timestamp.
  static void notifyNewStepStart();

  /// \brief Starts a measurement for the \c topic_name topic.
  /// \param topic_name Name of the topic to which the measurement belongs.
  static void startMeasurement(const std::string& topic_name);

  /// \brief Stops a measurement for the \c topic_name topic.
  /// \param topic_name Name of the topic to which the measurement belongs.
  /// \param ignore_measurement If true, the timer will be stopped but the result is ignored.
  static void stopMeasurement(const std::string& topic_name, bool ignore_measurement = false);

  /// \brief Add a measurement for the \c topic_name topic.
  /// \param topic_name Name of the topic to which the measurement belongs.
  /// \param start The timestamp representing the start of the measurement.
  /// \param end The timestamp representing the end of the measurement.
  static void addMeasurement(const std::string& topic_name, const TimePoint start,
                             const TimePoint end);

  /// \brief Add a value for the \c topic_name topic.
  /// \param topic_name Name of the topic to which the value belongs.
  /// \param value The value that must be added.
  static void addValue(const std::string& topic_name, double value);

  /// \brief Reset all collected data for all the topics with the given prefix. If \e topic_prefix
  /// is an empty string, all data will be reset
  /// \param topic_prefix Prefix for the names of the topics that must be reset.
  static void resetTopic(const std::string& topic_prefix);

  /// \brief Save the recorded data for all the topics.
  static void saveData();

  /// \brief Print the statistics to the specified string.
  /// \param stream The destination stream.
  static void logStatistics(std::ostream& stream);

  /// \brief Set the parameters of the benchmarker.
  /// \param parameters The new parameters for the benchmarker.
  static inline void setParameters(const BenchmarkerParams& parameters) { params_ = parameters; }

  /// \brief Get the parameters of the benchmarker.
  /// \return The parameters of the benchmarker.
  static inline const BenchmarkerParams& getParameters() { return params_; }

 private:
  // Represents a value and information about the step from which it was collected.
  struct ValueEntry {
    ValueEntry(size_t step_id, const TimePoint timestamp, double value)
      : step_id(step_id), timestamp(timestamp), value(value) {
    }
    size_t step_id;
    TimePoint timestamp;
    double value;
  };

  // Helper class for collecting data and statistics about values.
  class ValueTopic {

   public:
    // Adds a value.
    void addValue(size_t step_id, const TimePoint timestamp, double value);

    // Computes the mean of the measurements.
    double getMean() const;

    // Computes the standard deviation of the values.
    double getStandardDeviation() const;

    // Gets a reference to the stored data.
    inline const std::vector<ValueEntry>& getValues() const { return values_; }

   private:
    // Sum of the values, needed for computing mean and standard deviations.
    double sum_ = 0.0;

    // Sum of squares of the values, needed for computing the standard deviation.
    double sum_of_squares_ = 0.0;

    // The number of values recorded.
    uint64_t values_count_ = 0u;

    // The collected values.
    std::vector<ValueEntry> values_;
  };

  // Helper functions.
  static std::string setupAndGetResultsRootDirectory();
  static TimePoint getFirstValueTimepoint();
  static double durationToMilliseconds(Duration duration);

  // Mutexes for synchronizing accesses to the benchmarker.
  static std::mutex value_topics_mutex_;
  static std::mutex started_measurements_mutex_;

  // Map containing the values sorted in alphabetical order.
  static std::map<std::string, ValueTopic> value_topics_;

  // Starting time points of the measurements currently in progress.
  static std::unordered_map<std::string, TimePoint> started_mesurements_;

  // Timestamp of the step being currently benchmarked.
  static TimePoint current_timestamp_;

  // ID of the step being currently benchmarked.
  static size_t current_step_id_;

  // Parameters of the benchmarker.
  static BenchmarkerParams params_;
};

/// \brief A timer that measures the time elapsed between its creation and destruction. This is
/// useful for timing whole functions or any scoped block of code. The measurement is automatically
/// committed to the benchmarker.
class ScopedTimer {

 public:
  /// \brief Default constructor. Starts the timer.
  /// \param topic_name Name of the topic to which the measurement belongs.
  ScopedTimer(const std::string& topic_name)
   : topic_name_(topic_name), start_(Benchmarker::Clock::now()) {
  }

  /// \brief Destructor. Stops the timer and commits the result to the benchmarker.
  ~ScopedTimer();

 private:
  // Name of the timed block.
  const std::string topic_name_;

  // The time point when the timer was started.
  const std::chrono::time_point<Benchmarker::Clock> start_;
};

} // namespace laser_slam

#endif /* LASER_SLAM_BENCHMARKER_HPP_ */
