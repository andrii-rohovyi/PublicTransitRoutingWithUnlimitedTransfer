#pragma once

#include <algorithm>

#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "DelayShortcutSearch.h"

namespace RAPTOR::ULTRA {

template<bool DEBUG = false>
class DelayBuilder {

public:
    inline static constexpr bool Debug = DEBUG;
    using Type = DelayBuilder<Debug>;

public:
    DelayBuilder(const TripBased::Data& tripData) :
        tripData(tripData),
        data(tripData.raptorData) {
        shortcutGraph.addVertices(data.numberOfStops());
        for (const Vertex vertex : shortcutGraph.vertices()) {
            shortcutGraph.set(Coordinates, vertex, data.transferGraph.get(Coordinates, vertex));
        }
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int arrivalDelayBuffer, const int departureDelayBuffer, [[maybe_unused]] const int witnessTransferLimit = 15 * 60, const int minDepartureTime = -never, const int maxDepartureTime = never, const size_t maxStops = 0, const bool verbose = true) noexcept {
        const size_t numStops = (maxStops > 0 && maxStops < data.numberOfStops()) ? maxStops : data.numberOfStops();
        if (verbose) {
            std::cout << "Computing delay-tolerant stop-to-stop shortcuts with " << threadPinning.numberOfThreads << " threads." << std::endl;
            std::cout << "   Arrival delay buffer:   " << arrivalDelayBuffer << "s" << std::endl;
            std::cout << "   Departure delay buffer: " << departureDelayBuffer << "s" << std::endl;
            if (maxStops > 0) std::cout << "   Processing first " << numStops << " of " << data.numberOfStops() << " stops (test mode)" << std::endl;
            std::cout << std::flush;
        }

        Progress progress(numStops, verbose);
        std::cout << std::flush;

        Timer constructorTimer;
        std::cout << "Entering parallel region, constructing search instances..." << std::endl << std::flush;

        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();

            DynamicTransferGraph localShortcutGraph;
            localShortcutGraph.addVertices(data.numberOfStops());

            DelayShortcutSearch<Debug> shortcutSearch(tripData, localShortcutGraph, arrivalDelayBuffer, departureDelayBuffer);

            #pragma omp single
            {
                std::cout << "All constructors done in " << String::msToString(constructorTimer.elapsedMilliseconds()) << ". Starting stop processing..." << std::endl << std::flush;
            }

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < numStops; i++) {
                shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                progress++;
            }

            #pragma omp critical
            {
                for (const Vertex from : shortcutGraph.vertices()) {
                    for (const Edge edge : localShortcutGraph.edgesFrom(from)) {
                        const Vertex to = localShortcutGraph.get(ToVertex, edge);
                        const int travelTime = localShortcutGraph.get(TravelTime, edge);
                        if (!shortcutGraph.hasEdge(from, to)) {
                            shortcutGraph.addEdge(from, to).set(TravelTime, travelTime);
                        } else {
                            const Edge existing = shortcutGraph.findEdge(from, to);
                            if (shortcutGraph.get(TravelTime, existing) > travelTime) {
                                shortcutGraph.set(TravelTime, existing, travelTime);
                            }
                        }
                    }
                }
            }
        }
        progress.finished();
    }

    inline const DynamicTransferGraph& getShortcutGraph() const noexcept {
        return shortcutGraph;
    }

    inline DynamicTransferGraph& getShortcutGraph() noexcept {
        return shortcutGraph;
    }

private:
    const TripBased::Data& tripData;
    const RAPTOR::Data& data;
    DynamicTransferGraph shortcutGraph;
};

}