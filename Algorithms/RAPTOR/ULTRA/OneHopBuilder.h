#pragma once

#include <algorithm>

#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "OneHopShortcutSearch.h"

namespace RAPTOR::ULTRA {

// Parallel driver for OneHopShortcutSearch: computes the one-hop, bounded-radius
// ULTRA shortcut graph. Mirrors ULTRA::Builder, but each per-thread search is
// parameterized by a transfer radius (maxTransferTravelTime, in seconds)
// instead of a witness transfer limit.
template<bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class OneHopBuilder {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool CountOptimalCandidates = COUNT_OPTIMAL_CANDIDATES;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = OneHopBuilder<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

public:
    OneHopBuilder(const Data& data) :
        data(data) {
        shortcutGraph.addVertices(data.numberOfStops());
        for (const Vertex vertex : shortcutGraph.vertices()) {
            shortcutGraph.set(Coordinates, vertex, data.transferGraph.get(Coordinates, vertex));
        }
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int maxTransferTravelTime = 30 * 60, const int minDepartureTime = -never, const int maxDepartureTime = never, const bool verbose = true) noexcept {
        if (verbose) std::cout << "Computing one-hop shortcuts (radius " << maxTransferTravelTime << "s) with " << threadPinning.numberOfThreads << " threads." << std::endl;

        size_t optimalCandidates = 0;
        Progress progress(data.numberOfStops(), verbose);
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();

            DynamicTransferGraph localShortcutGraph = shortcutGraph;
            OneHopShortcutSearch<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates> shortcutSearch(data, localShortcutGraph, maxTransferTravelTime);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.numberOfStops(); i++) {
                shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                progress++;
            }

            #pragma omp critical
            {
                if constexpr (CountOptimalCandidates) {
                    optimalCandidates += shortcutSearch.getNumberOfOptimalCandidates();
                }
                for (const Vertex from : shortcutGraph.vertices()) {
                    for (const Edge edge : localShortcutGraph.edgesFrom(from)) {
                        const Vertex to = localShortcutGraph.get(ToVertex, edge);
                        if (!shortcutGraph.hasEdge(from, to)) {
                            shortcutGraph.addEdge(from, to).set(TravelTime, localShortcutGraph.get(TravelTime, edge));
                        } else {
                            Assert(shortcutGraph.get(TravelTime, shortcutGraph.findEdge(from, to)) == localShortcutGraph.get(TravelTime, edge), "Edge from " << from << " to " << to << " has inconclusive travel time (" << shortcutGraph.get(TravelTime, shortcutGraph.findEdge(from, to)) << ", " << localShortcutGraph.get(TravelTime, edge) << ")");
                        }
                    }
                }
            }
        }
        progress.finished();
        if constexpr (CountOptimalCandidates) {
            std::cout << "#Optimal candidates: " << String::prettyInt(optimalCandidates) << std::endl;
        } else {
            suppressUnusedParameterWarning(optimalCandidates);
        }
    }

    inline const DynamicTransferGraph& getShortcutGraph() const noexcept {
        return shortcutGraph;
    }

    inline DynamicTransferGraph& getShortcutGraph() noexcept {
        return shortcutGraph;
    }

private:
    const Data& data;
    DynamicTransferGraph shortcutGraph;
};

}
