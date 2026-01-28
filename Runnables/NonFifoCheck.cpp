#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../Helpers/Types.h"
#include "../DataStructures/Intermediate/Data.h"
#include "../DataStructures/Graph/TimeDependentGraph.h"

struct EdgeKeyHash {
    std::size_t operator()(const std::pair<Vertex, Vertex>& p) const noexcept {
        return std::hash<size_t>()(size_t(p.first)) ^ (std::hash<size_t>()(size_t(p.second)) << 1);
    }
};

struct DepArr {
    int dep;
    int arr;
};

struct ViolationSample {
    Vertex u;
    Vertex v;
    int prevDep;
    int prevArr;
    int dep;
    int arr;
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: NonFifoCheck <intermediate.binary> [maxSamples]\n";
        return 1;
    }

    const std::string inputFile = argv[1];
    const int maxSamples = (argc >= 3) ? std::stoi(argv[2]) : 10;

    std::cout << "Loading Intermediate data from: " << inputFile << std::endl;
    Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);

    std::unordered_map<std::pair<Vertex, Vertex>, std::vector<DepArr>, EdgeKeyHash> tripSegments;
    tripSegments.reserve(inter.trips.size() * 4);

    size_t totalSegments = 0;
    size_t totalStopEvents = 0;
    for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
        const Intermediate::Trip& trip = inter.trips[tripId];
        totalStopEvents += trip.stopEvents.size();
        for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
            const Intermediate::StopEvent& stopEventU = trip.stopEvents[i];
            const Intermediate::StopEvent& stopEventV = trip.stopEvents[i + 1];
            const Vertex u = Vertex(stopEventU.stopId);
            const Vertex v = Vertex(stopEventV.stopId);

            const int buffer = (u < inter.stops.size()) ? inter.stops[u].minTransferTime : 0;
            const int dep = stopEventU.departureTime - buffer;
            const int arr = stopEventV.arrivalTime;

            tripSegments[{u, v}].push_back({dep, arr});
            totalSegments++;
        }
    }

    size_t edgesWithViolations = 0;
    size_t totalViolations = 0;
    int maxDecrease = 0;
    ViolationSample maxSample{};
    std::vector<ViolationSample> samples;
    samples.reserve(maxSamples);

    for (auto& [edge, trips] : tripSegments) {
        std::sort(trips.begin(), trips.end(), [](const DepArr& a, const DepArr& b) {
            if (a.dep != b.dep) return a.dep < b.dep;
            return a.arr < b.arr;
        });

        bool edgeHasViolation = false;
        int prevArr = trips.front().arr;
        int prevDep = trips.front().dep;

        for (size_t i = 1; i < trips.size(); ++i) {
            const int dep = trips[i].dep;
            const int arr = trips[i].arr;

            if (arr < prevArr) {
                edgeHasViolation = true;
                totalViolations++;
                const int decrease = prevArr - arr;
                if (decrease > maxDecrease) {
                    maxDecrease = decrease;
                    maxSample = {edge.first, edge.second, prevDep, prevArr, dep, arr};
                }
                if (samples.size() < static_cast<size_t>(maxSamples)) {
                    samples.push_back({edge.first, edge.second, prevDep, prevArr, dep, arr});
                }
            }

            prevArr = arr;
            prevDep = dep;
        }

        if (edgeHasViolation) edgesWithViolations++;
    }

    std::cout << "\n=== Non-FIFO Analysis ===\n";
    std::cout << "Trips: " << inter.trips.size() << "\n";
    std::cout << "Stop events: " << totalStopEvents << "\n";
    std::cout << "Segments (u->v pairs): " << totalSegments << "\n";
    std::cout << "Edges (unique u->v): " << tripSegments.size() << "\n";
    std::cout << "Edges with non-FIFO: " << edgesWithViolations << "\n";
    std::cout << "Total non-FIFO violations: " << totalViolations << "\n";

    if (edgesWithViolations > 0) {
        std::cout << "Max arrival decrease: " << maxDecrease << " (edge " << maxSample.u << "->" << maxSample.v
                  << ", prev dep/arr=" << maxSample.prevDep << "/" << maxSample.prevArr
                  << ", dep/arr=" << maxSample.dep << "/" << maxSample.arr << ")\n";
    }

    if (!samples.empty()) {
        std::cout << "\nSample violations (up to " << maxSamples << "):\n";
        for (const auto& s : samples) {
            std::cout << "  Edge " << s.u << "->" << s.v
                      << " | prev dep/arr=" << s.prevDep << "/" << s.prevArr
                      << " | dep/arr=" << s.dep << "/" << s.arr << "\n";
        }
    }

    return 0;
}
