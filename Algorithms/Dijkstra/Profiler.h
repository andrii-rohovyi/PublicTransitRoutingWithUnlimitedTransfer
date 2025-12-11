#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Vector.h"

namespace TDD {

inline constexpr static int MetricWidth = 16;
inline constexpr static int TimeWidth = 15;

typedef enum {
    PHASE_CLEAR,
    PHASE_INITIALIZATION,
    PHASE_MAIN_LOOP,
    PHASE_PRUNING, // Optional, if we want to measure pruning time specifically
    NUM_PHASES
} Phase;

constexpr const char* PhaseNames[] = {
    "Clear",
    "Init",
    "Main Loop",
    "Pruning"
};

typedef enum {
    METRIC_SETTLES,
    METRIC_RELAXES_TRANSIT,
    METRIC_RELAXES_WALKING,
    METRIC_ENQUEUES,
    METRIC_PRUNED_LABELS,
    NUM_METRICS
} Metric;

constexpr const char* MetricNames[] = {
    "Settles",
    "Relaxes (Bus)",
    "Relaxes (Walk)",
    "Enqueues",
    "Pruned"
};

class NoProfiler {
public:
    inline void start() const noexcept {}
    inline void done() const noexcept {}
    inline void startPhase(const Phase) const noexcept {}
    inline void donePhase(const Phase) const noexcept {}
    inline void countMetric(const Metric) const noexcept {}
    inline void printStatistics() const noexcept {}
};

class AggregateProfiler {
public:
    AggregateProfiler() :
        numQueries(0),
        totalTime(0.0),
        phaseTime(NUM_PHASES, 0.0),
        metricValue(NUM_METRICS, 0) {
    }

    inline void start() noexcept {
        totalTimer.restart();
    }

    inline void done() noexcept {
        totalTime += totalTimer.elapsedMicroseconds();
        numQueries++;
    }

    inline void startPhase(const Phase) noexcept {
        phaseTimer.restart();
    }

    inline void donePhase(const Phase phase) noexcept {
        phaseTime[phase] += phaseTimer.elapsedMicroseconds();
    }

    inline void countMetric(const Metric metric) noexcept {
        metricValue[metric]++;
    }

    inline void printStatistics() const noexcept {
        std::cout << std::endl << "=== TD-Dijkstra Statistics ===" << std::endl;
        std::cout << "Total Queries: " << numQueries << std::endl;
        
        // Print Metrics
        std::cout << std::endl << "Metrics (Avg per query):" << std::endl;
        for (int i = 0; i < NUM_METRICS; ++i) {
            std::cout << std::setw(MetricWidth) << MetricNames[i] << ": " 
                      << String::prettyDouble((double)metricValue[i] / numQueries) << std::endl;
        }

        // Print Phases
        std::cout << std::endl << "Phases (Avg time per query):" << std::endl;
        for (int i = 0; i < NUM_PHASES; ++i) {
            std::cout << std::setw(MetricWidth) << PhaseNames[i] << ": " 
                      << String::musToString(phaseTime[i] / numQueries) << std::endl;
        }

        std::cout << std::endl << "Total Time (Avg): " << String::musToString(totalTime / numQueries) << std::endl;
    }

private:
    size_t numQueries;
    Timer totalTimer;
    double totalTime;
    
    Timer phaseTimer;
    std::vector<double> phaseTime;
    
    std::vector<long long> metricValue;
};

}
