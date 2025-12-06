#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <random>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/Dijkstra/TD-DijkstraStateful.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/Queries/Queries.h"
#include "../../Algorithms/CH/CH.h"

class DebugSingleQuery : public ParameterizedCommand {
public:
    DebugSingleQuery(BasicShell& shell) :
        ParameterizedCommand(shell, "debugSingleQuery", "Debug a single query with detailed traces from both MR and TD-Dijkstra.") {
        addParameter("RAPTOR input file");
        addParameter("Intermediate input file");
        addParameter("Core CH input file");
        addParameter("Query index");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        
        CH::CH ch(getParameter("Core CH input file"));

        const size_t queryIndex = getParameter<size_t>("Query index");
        
        // Generate stop-to-stop queries (matching the comparison command)
        const size_t numStops = raptorData.numberOfStops();
        std::vector<VertexQuery> queries;
        std::mt19937 rng(42);
        std::uniform_int_distribution<size_t> stopDist(0, numStops - 1);
        std::uniform_int_distribution<int> timeDist(0, 24 * 60 * 60 - 1);
        for (size_t i = 0; i <= queryIndex; ++i) {
            queries.emplace_back(Vertex(stopDist(rng)), Vertex(stopDist(rng)), timeDist(rng));
        }
        const VertexQuery& query = queries[queryIndex];

        std::cout << "\n========================================" << std::endl;
        std::cout << "Query " << queryIndex << ":" << std::endl;
        std::cout << "  Source: " << query.source << std::endl;
        std::cout << "  Target: " << query.target << std::endl;
        std::cout << "  Departure Time: " << query.departureTime << " (" << secondsToString(query.departureTime) << ")" << std::endl;
        std::cout << "========================================\n" << std::endl;

        // Run MR (with CoreCH, matching the comparison command)
        std::cout << "=== Running MR (DijkstraRAPTOR) with CoreCH ===" << std::endl;
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::NoProfiler, false, false> 
            algorithmMR(raptorData, ch);
        
        algorithmMR.run(query.source, query.departureTime, query.target);
        const int mrArrival = algorithmMR.getEarliestArrivalTime(query.target);
        
        std::cout << "MR Arrival Time: " << mrArrival << " (" << secondsToString(mrArrival) << ")" << std::endl;
        std::cout << "MR Reachable: " << (algorithmMR.reachable(query.target) ? "Yes" : "No") << std::endl;
        
        if (mrArrival < never) {
            const auto& arrivalTimes = algorithmMR.getArrivalTimes(query.target);
            std::cout << "MR Arrival times by round: ";
            for (size_t i = 0; i < arrivalTimes.size(); ++i) {
                if (arrivalTimes[i] < never) {
                    std::cout << "R" << i << "=" << arrivalTimes[i] << " ";
                }
            }
            std::cout << std::endl;
            
            const auto& journeys = algorithmMR.getJourneys(query.target);
            std::cout << "MR found " << journeys.size() << " journeys (one per improving round)" << std::endl;
            if (!journeys.empty()) {
                std::cout << "\nMR Best Journey (" << journeys.back().size() << " legs):" << std::endl;
                for (const auto& leg : journeys.back()) {
                    std::cout << "  ";
                    if (leg.usesRoute) {
                        std::cout << "ROUTE " << leg.routeId << ": ";
                    } else {
                        std::cout << "TRANSFER: ";
                    }
                    std::cout << leg.from << " -> " << leg.to 
                              << " @ " << secondsToString(leg.departureTime)
                              << " arr " << secondsToString(leg.arrivalTime)
                              << " (dur: " << (leg.arrivalTime - leg.departureTime) << "s)"
                              << std::endl;
                }
            }
        }

        // Run TD-Dijkstra (stateful)
        std::cout << "\n=== Running TD-Dijkstra (Stateful) with CoreCH ===" << std::endl;
        using TDDijkstraStateful = TimeDependentDijkstraStateful<TimeDependentGraph, true>;
        // Use CoreCH for initial transfers (matching MR semantics)
        TDDijkstraStateful algorithmTD(graph, raptorData.numberOfStops(), &ch);
        
        algorithmTD.run(query.source, query.departureTime, query.target);
        const int tdArrival = algorithmTD.getArrivalTime(query.target);
        
        std::cout << "TD Arrival Time: " << tdArrival << " (" << secondsToString(tdArrival) << ")" << std::endl;
        std::cout << "Vertices settled: " << algorithmTD.getSettleCount() << std::endl;
        std::cout << "Edges relaxed: " << algorithmTD.getRelaxCount() << std::endl;
        
        if (tdArrival < never) {
            const auto path = algorithmTD.getPath(query.target);
            std::cout << "\nTD Path (" << path.size() << " vertices):" << std::endl;
            std::cout << "(Note: RAPTOR has " << raptorData.numberOfStops() << " stops, ";
            std::cout << "TD graph has " << graph.numVertices() << " vertices)" << std::endl;
            
            for (size_t i = 0; i < path.size(); ++i) {
                const auto& entry = path[i];
                const int minTransfer = graph.getMinTransferTimeAt(entry.vertex);
                const bool isStop = (entry.vertex < raptorData.numberOfStops());
                std::cout << "  [" << i << "] Vertex " << entry.vertex 
                          << (isStop ? " (STOP)" : " (non-stop)")
                          << " @ " << secondsToString(entry.arrivalTime)
                          << " (state: " << (entry.state == TDDijkstraStateful::State::AtStop ? "AtStop" : "OnVehicle") << ")"
                          << " [buffer=" << minTransfer << "s]";
                
                // Check if next is a boarding (AtStop -> OnVehicle)
                if (i + 1 < path.size() && entry.state == TDDijkstraStateful::State::AtStop && 
                    path[i + 1].state == TDDijkstraStateful::State::OnVehicle) {
                    const int waitTime = path[i + 1].arrivalTime - entry.arrivalTime;
                    std::cout << " -> BOARDING (wait: " << waitTime << "s)";
                    if (waitTime < minTransfer) {
                        std::cout << " **VIOLATION** (needs " << minTransfer << "s)";
                    }
                }
                std::cout << std::endl;
            }
        }

        // Compare
        std::cout << "\n=== Comparison ===" << std::endl;
        if (mrArrival == tdArrival) {
            std::cout << "✓ Results match!" << std::endl;
        } else {
            std::cout << "✗ Results differ by " << (mrArrival - tdArrival) << " seconds (" 
                      << String::prettyDouble((mrArrival - tdArrival) / 60.0) << " minutes)" << std::endl;
            if (tdArrival < mrArrival) {
                std::cout << "   TD-Dijkstra found an earlier arrival." << std::endl;
            } else {
                std::cout << "   MR found an earlier arrival." << std::endl;
            }
        }
    }

private:
    inline std::string secondsToString(int seconds) const noexcept {
        if (seconds >= never) return "NEVER";
        int hours = seconds / 3600;
        int minutes = (seconds % 3600) / 60;
        int secs = seconds % 60;
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(2) << hours << ":"
           << std::setfill('0') << std::setw(2) << minutes << ":"
           << std::setfill('0') << std::setw(2) << secs;
        return ss.str();
    }
};
