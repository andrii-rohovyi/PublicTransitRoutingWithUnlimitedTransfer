#pragma once

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/Shortcut.h"
#include "../../../DataStructures/TripBased/ShortcutCollection.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "DelayShortcutSearch.h"
#include "ULTRABuilder.h"

namespace TripBased {

template<bool DEBUG = false>
class DelayULTRABuilder {

public:
    inline static constexpr bool Debug = DEBUG;
    using Type = DelayULTRABuilder<Debug>;

public:
    DelayULTRABuilder(const Data& data) :
        data(data),
        shortcuts(data.numberOfStopEvents()) {
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int arrivalDelayBuffer, const int departureDelayBuffer, const size_t memoryLimit = 2048, const int minDepartureTime = -never, const int maxDepartureTime = never, const bool verbose = true) noexcept {
        // ── Δ=0 fast path: dispatch to classical ULTRABuilder ───────────────────────
        // When both delay buffers are zero, no shortcut can ever be "needed under
        // a future delay scenario" because no future delays are admissible. The
        // delay-aware witness criterion then degenerates to classical ULTRA's
        // strict criterion, and we can use the lighter (and tighter) ULTRABuilder
        // directly. Edges are annotated with [δ_min, δ_max] = [0, 0].
        if (arrivalDelayBuffer == 0 && departureDelayBuffer == 0) {
            if (verbose) std::cout << "Δ=0 fast path: dispatching to classical ULTRABuilder." << std::endl;
            ULTRABuilder<Debug> classic(data);
            classic.computeShortcuts(threadPinning, 0, minDepartureTime, maxDepartureTime, verbose);
            const DynamicTransferGraph& classicalGraph = classic.getStopEventGraph();

            stopEventGraph.clear();
            stopEventGraph.addVertices(data.numberOfStopEvents());
            for (const Vertex u : classicalGraph.vertices()) {
                for (const Edge e : classicalGraph.edgesFrom(u)) {
                    const Vertex v = classicalGraph.get(ToVertex, e);
                    const int travelTime = classicalGraph.get(TravelTime, e);
                    const Edge newEdge = stopEventGraph.addEdge(u, v);
                    stopEventGraph.set(TravelTime, newEdge, travelTime);
                    stopEventGraph.set(MinOriginDelay, newEdge, 0);
                    stopEventGraph.set(MaxOriginDelay, newEdge, 0);
                }
            }
            stopEventGraph.sortEdges(ToVertex);
            if (verbose) std::cout << "Δ=0 fast path done: " << stopEventGraph.numEdges() << " shortcuts." << std::endl;
            return;
        }

        if (verbose) std::cout << "Computing shortcuts with " << threadPinning.numberOfThreads << " threads." << std::endl;

        Progress progress(data.numberOfStops(), verbose);
        omp_set_num_threads(threadPinning.numberOfThreads);
        std::vector<ShortcutCollection> threadShortcuts(threadPinning.numberOfThreads, ShortcutCollection(data.numberOfStopEvents()));
        #pragma omp parallel
        {
            threadPinning.pinThread();
            const size_t threadNum = omp_get_thread_num();

            DelayShortcutSearch<Debug> shortcutSearch(data, arrivalDelayBuffer, departureDelayBuffer, threadShortcuts[threadNum]);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.numberOfStops(); i++) {
                shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                if (threadShortcuts[threadNum].memoryUsageInBytes() > static_cast<long long>(memoryLimit * 1024 * 1024)) {
                    #pragma omp critical
                    {
                        shortcuts.merge(threadShortcuts[threadNum]);
                    }
                    threadShortcuts[threadNum].clear();
                }
                progress++;
            }
        }

        for (size_t i = 1; i < threadPinning.numberOfThreads; i *= 2) {
            #pragma omp parallel
            {
                threadPinning.pinThread();
                const size_t threadNum = omp_get_thread_num();
                if ((threadNum % (2*i)) == 0 && threadNum + i < threadPinning.numberOfThreads) {
                    threadShortcuts[threadNum].merge(threadShortcuts[threadNum + i]);
                }
            }
        }

        shortcuts.merge(threadShortcuts[0]);
        stopEventGraph = shortcuts.getGraph();
        progress.finished();
    }


    inline const DynamicDelayGraph& getStopEventGraph() const noexcept {
        return stopEventGraph;
    }

    inline DynamicDelayGraph& getStopEventGraph() noexcept {
        return stopEventGraph;
    }

private:
    const Data& data;
    ShortcutCollection shortcuts;
    DynamicDelayGraph stopEventGraph;
};

}
