#pragma once

#include <iostream>
#include <vector>
#include <thread>

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/Types.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/String/String.h"

#include "../../../DataStructures/Intermediate/Data.h"
#include "../../../DataStructures/Graph/Graph.h"
#include "../../../DataStructures/Graph/TimeDependentGraph.h"

#include "ShortcutSearchJTS.h"

namespace RAPTOR::ULTRA {

template<bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class BuilderJTS {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool CountOptimalCandidates = COUNT_OPTIMAL_CANDIDATES;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = BuilderJTS<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

public:
    BuilderJTS(const Intermediate::Data& data, const TimeDependentGraph& tdGraph) :
        data(data),
        tdGraph(tdGraph),
        numberOfStops(data.numberOfStops()) {
        shortcutGraph.addVertices(data.transferGraph.numVertices());
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int witnessLimit,
                          const int minTime = -never, const int maxTime = never) noexcept {
        Progress progress(numberOfStops);

        const size_t numThreads = threadPinning.numberOfThreads;
        const size_t pinMultiplier = threadPinning.pinMultiplier;

        std::vector<DynamicTransferGraph> threadGraphs(numThreads);
        std::vector<size_t> threadOptimalCandidates(numThreads, 0);

        Timer timer;

        omp_set_num_threads(numThreads);
        #pragma omp parallel
        {
            const size_t threadId = omp_get_thread_num();
            pinThreadToCoreId((threadId * pinMultiplier) % numberOfCores());

            DynamicTransferGraph& localGraph = threadGraphs[threadId];
            localGraph.addVertices(data.transferGraph.numVertices());

            ShortcutSearchJTS<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates> search(
                data, tdGraph, localGraph, witnessLimit);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < numberOfStops; i++) {
                search.run(StopId(i), minTime, maxTime);
                progress++;
            }

            if constexpr (CountOptimalCandidates) {
                threadOptimalCandidates[threadId] = search.getNumberOfOptimalCandidates();
            }
        }

        progress.finished();

        std::cout << "Merging thread-local graphs..." << std::flush;
        for (size_t t = 0; t < numThreads; t++) {
            for (const Vertex from : threadGraphs[t].vertices()) {
                for (const Edge edge : threadGraphs[t].edgesFrom(from)) {
                    const Vertex to = threadGraphs[t].get(ToVertex, edge);
                    const int travelTime = threadGraphs[t].get(TravelTime, edge);

                    if (!shortcutGraph.hasEdge(from, to)) {
                        shortcutGraph.addEdge(from, to).set(TravelTime, travelTime);
                    }
                }
            }
        }
        std::cout << " done." << std::endl;

        std::cout << "Time: " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
        std::cout << "Number of shortcuts: " << String::prettyInt(shortcutGraph.numEdges()) << std::endl;

        if constexpr (CountOptimalCandidates) {
            size_t totalOptimal = 0;
            for (size_t count : threadOptimalCandidates) {
                totalOptimal += count;
            }
            std::cout << "Number of optimal candidates: " << String::prettyInt(totalOptimal) << std::endl;
        }
    }

    inline DynamicTransferGraph& getShortcutGraph() noexcept {
        return shortcutGraph;
    }

    inline const DynamicTransferGraph& getShortcutGraph() const noexcept {
        return shortcutGraph;
    }

private:
    const Intermediate::Data& data;
    const TimeDependentGraph& tdGraph;
    const size_t numberOfStops;
    DynamicTransferGraph shortcutGraph;
};

}