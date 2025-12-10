#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/CSA/CSA.h"
#include "../../Algorithms/CSA/DijkstraCSA.h"
#include "../../Algorithms/CSA/HLCSA.h"
#include "../../Algorithms/CSA/ULTRACSA.h"
#include "../../Algorithms/RAPTOR/HLRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/Dijkstra/TimeDependentDijkstra.h"
#include "../../Algorithms/Dijkstra/TD-DijkstraStateful.h"

#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"
#include "../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../DataStructures/CSA/Entities/Journey.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../DataStructures/Intermediate/Data.h"

class BuildTDGraph : public ParameterizedCommand {

public:
    BuildTDGraph(BasicShell& shell) :
        ParameterizedCommand(shell, "buildTDGraph", "Builds and serializes a time-dependent graph from intermediate data.") {
        addParameter("Intermediate input file");
        addParameter("TD Graph output file");
    }

    virtual void execute() noexcept {
        std::cout << "Loading intermediate data..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        intermediateData.printInfo();
        
        std::cout << "\nBuilding time-dependent graph..." << std::endl;
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        std::cout << "Time-dependent graph created: " << graph.numVertices() << " vertices, " 
                  << graph.numEdges() << " edges" << std::endl;
        
        const std::string outputFile = getParameter("TD Graph output file");
        std::cout << "\nSerializing to: " << outputFile << std::endl;
        graph.serialize(outputFile);
        std::cout << "Time-dependent graph saved successfully!" << std::endl;
    }
};

class TestTDGraphLoad : public ParameterizedCommand {

public:
    TestTDGraphLoad(BasicShell& shell) :
        ParameterizedCommand(shell, "testTDGraphLoad", "Test loading a time-dependent graph.") {
        addParameter("TD Graph input file");
    }

    virtual void execute() noexcept {
        std::cout << "Loading time-dependent graph..." << std::endl;
        TimeDependentGraph graph = TimeDependentGraph::FromBinary(getParameter("TD Graph input file"));
        std::cout << "Loaded: " << graph.numVertices() << " vertices, " << graph.numEdges() << " edges" << std::endl;
        std::cout << "Success!" << std::endl;
    }
};

class RunTransitiveCSAQueries : public ParameterizedCommand {
public:
    RunTransitiveCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveCSAQueries", "Runs the given number of random transitive CSA queries.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
        addParameter("Target pruning?");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.sortTransferGraphEdgesByTravelTime();
        csaData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const bool targetPruning = getParameter<bool>("Target pruning?");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        if (pruningRule == 1) {
            CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);

            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
            }
            algorithm.getProfiler().printStatistics();
        } else {
            CSA::CSA_prune<true, CSA::AggregateProfiler> algorithm(csaData);

            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
            }
            algorithm.getProfiler().printStatistics();
        }
    }
};

class CheckCSAPruning : public ParameterizedCommand {

public:
    CheckCSAPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkCSAPruning", "Checks if pruning rules yield the same results as no pruning for CSA.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;

        // Run with pruning rule 0 (no pruning)
        std::cout << "--- Running queries with No Pruning (Rule 0) ---" << std::endl;
        CSA::CSA<false, CSA::AggregateProfiler> algo_no_pruning(csaData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        std::cout << "\n--- Running queries with Pruning Rule 1 ---" << std::endl;
        csaData.sortTransferGraphEdgesByTravelTime();
        CSA::CSA_prune<false, CSA::AggregateProfiler> algo_pruning_1(csaData);
        for (const StopQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();

        // Compare the results
        bool pruning_is_correct = (results_no_pruning == results_pruning_1);

        if (pruning_is_correct) {
            std::cout << "\nPruning rule 1 yields the same results as no pruning." << std::endl;
        } else {
            std::cout << "\nPruning rule 1 failed comparison." << std::endl;
        }
    }
};

class RunDijkstraCSAQueries : public ParameterizedCommand {

public:
    RunDijkstraCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraCSAQueries", "Runs the given number of random Dijkstra-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));
        CSA::DijkstraCSA<RAPTOR::CoreCHInitialTransfers, true, CSA::AggregateProfiler> algorithm(csaData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunHLCSAQueries : public ParameterizedCommand {

public:
    RunHLCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runHLCSAQueries", "Runs the given number of random HL-CSA queries.") {
        addParameter("CSA input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        CSA::HLCSA<CSA::AggregateProfiler> algorithm(csaData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunULTRACSAQueries : public ParameterizedCommand {

public:
    RunULTRACSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRACSAQueries", "Runs the given number of random ULTRA-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.sortTransferGraphEdgesByTravelTime(); // Call to sort the transfer graph edges
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        if (pruningRule == 1) {
            executeWithPruning<1>(csaData, ch, queries);
        } else {
            executeWithPruning<0>(csaData, ch, queries);
        }
    }

private:
    template<int ENABLE_PRUNING>
    void executeWithPruning(const CSA::Data& csaData, const CH::CH& ch, const std::vector<VertexQuery>& queries) {
        CSA::ULTRACSA<true, ENABLE_PRUNING, CSA::AggregateProfiler> algorithm(csaData, ch);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};


class CheckULTRACSAPruning : public ParameterizedCommand {

public:
    CheckULTRACSAPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkULTRACSAPruning", "Checks if pruning rules yield the same results as no pruning for ULTRACSA.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning;

        // Run with pruning rule 0 (no pruning)
        CSA::ULTRACSA<false, 0, CSA::AggregateProfiler> algo_no_pruning(csaData, ch);
        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        csaData.sortTransferGraphEdgesByTravelTime(); // Call to sort the transfer graph edges
        CSA::ULTRACSA<false, 1, CSA::AggregateProfiler> algo_pruning(csaData, ch);
        for (const VertexQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algo_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

        // Compare the results
        bool pruning_is_correct = (results_no_pruning == results_pruning);

        if (pruning_is_correct) {
            std::cout << "Pruning rule 1 yields the same results as no pruning." << std::endl;
        } else {
            std::cout << "Pruning rule 1 failed comparison." << std::endl;
        }
    }
};

class RunTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveRAPTORQueries", "Runs the given number of random transitive RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.writeCSV("");
        raptorData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        // READ THE NEW INTEGER PARAMETER
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");

        // We use an if-else block to instantiate the correct version of the algorithm,
        // as the `ENABLE_PRUNING` template parameter must be a compile-time constant.
        if (pruningRule == 1) {
            // Instantiate with TARGET_PRUNING=true and ENABLE_PRUNING=1
            RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algorithm(raptorData);

            double numJourneys = 0;
            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
        } else {
            // Instantiate with TARGET_PRUNING=true and ENABLE_PRUNING=0 (default)
            RAPTOR::RAPTOR_prune<true, RAPTOR::AggregateProfiler, true, false, false> algorithm(raptorData);

            double numJourneys = 0;
            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
        }
    }
};

class TestTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    TestTransitiveRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "testTransitiveRAPTORQueries", "Tests a specific transitive RAPTOR query.") {
        addParameter("RAPTOR input file");
        addParameter("sourceStop");
        addParameter("targetStop");
        addParameter("startTime");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        const StopId sourceStop = StopId(getParameter<int>("sourceStop"));
        const StopId targetStop = StopId(getParameter<int>("targetStop"));

        const int startTime = getParameter<int>("startTime");

        std::cout << "Running query from stop " << sourceStop << " to stop " << targetStop << " at time " << startTime << std::endl;

        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algorithm(raptorData);
        algorithm.run(sourceStop, startTime, targetStop);

        // Corrected line: calling the existing getEarliestJourney function
        const RAPTOR::Journey journey = algorithm.getEarliestJourney(targetStop);
        std::cout << "Journey: " << journey << std::endl;

    }
};

class TestTransitiveCSAQueries : public ParameterizedCommand {

public:
    TestTransitiveCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "testTransitiveCSAQueries", "Tests a specific transitive CSA query.") {
        addParameter("CSA input file");
        addParameter("sourceStop");
        addParameter("targetStop");
        addParameter("startTime");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        const StopId sourceStop = StopId(getParameter<int>("sourceStop"));
        const StopId targetStop = StopId(getParameter<int>("targetStop"));
        const int startTime = getParameter<int>("startTime");

        std::cout << "Running query from stop " << sourceStop << " to stop " << targetStop << " at time " << startTime << std::endl;

        CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);
        algorithm.run(sourceStop, startTime, targetStop);

        const int arrivalTime = algorithm.getEarliestArrivalTime(targetStop);
        std::cout << "Earliest Arrival Time: " << arrivalTime << std::endl;

        const CSA::Journey journey = algorithm.getJourney(targetStop);
        std::cout << "Journey: " << journey << std::endl;
    }
};

class CompareCSAandRAPTOR : public ParameterizedCommand {

public:
    CompareCSAandRAPTOR(BasicShell& shell) :
        ParameterizedCommand(shell, "compareCSAandRAPTOR", "Compares journeys from CSA and RAPTOR for random queries.") {
        addParameter("RAPTOR input file");
        addParameter("CSA input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // Load data for both algorithms
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        runComparison(raptorData, csaData, queries);
    }

private:

    // Function to run the queries and compare the results
    void runComparison(const RAPTOR::Data& raptorData, const CSA::Data& csaData, const std::vector<StopQuery>& queries) const noexcept {
        const bool targetPruning = true;

        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> raptorAlgorithm(raptorData);
        CSA::CSA<true, CSA::AggregateProfiler> csaAlgorithm(csaData);

        size_t mismatches = 0;

        for (size_t i = 0; i < queries.size(); i++) {
            const auto& query = queries[i];

            // Run RAPTOR and get the arrival time
            raptorAlgorithm.run(query.source, query.departureTime, query.target);
            const int raptorArrivalTime = raptorAlgorithm.getEarliestArrivalTime(query.target);

            // Run CSA and get the arrival time
            csaAlgorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
            const int csaArrivalTime = csaAlgorithm.getEarliestArrivalTime(query.target);

            // Compare the arrival times
            if (raptorArrivalTime != csaArrivalTime) {
                mismatches++;
                std::cout << "Mismatch found for query #" << i + 1 << ":" << std::endl;
                std::cout << "  Source: " << query.source << ", Target: " << query.target << ", Time: " << query.departureTime << std::endl;
                std::cout << "--- RAPTOR Arrival Time ---" << std::endl;
                std::cout << raptorArrivalTime << std::endl;
                std::cout << "--- CSA Arrival Time ---" << std::endl;
                std::cout << csaArrivalTime << std::endl;
                std::cout << "-----------------------------------" << std::endl;
            }
        }
        std::cout << "\nTotal queries: " << queries.size() << ", Total mismatches: " << mismatches << std::endl;
    }
};

class CheckRAPTORPruning : public ParameterizedCommand {

public:
    CheckRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkRAPTORPruning", "Checks if RAPTOR pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();

        const size_t n = getParameter<size_t>("Number of queries");
        // Generate StopQueries, not VertexQueries
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;
        // Run with pruning rule 0 (no pruning)
        std::cout << "--- Running with No Pruning (Rule 0) ---" << std::endl;
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algo_no_pruning(raptorData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        std::cout << "\n--- Running with Pruning Rule 1 ---" << std::endl;
        // The transfer graph must be sorted for pruning rule 1 to be effective
        // Start the timer
        auto start = std::chrono::high_resolution_clock::now();

        raptorData.sortTransferGraphEdgesByTravelTime();

        // Stop the timer
        auto stop = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        // Print the duration
        std::cout << "Time taken to sort transfer graph edges: " << duration.count() << " microseconds" << std::endl;

        RAPTOR::RAPTOR_prune<true, RAPTOR::AggregateProfiler, true, false, false> algo_pruning_1(raptorData);
        for (const StopQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();


        // Compare the results
        bool pruning_1_correct = (results_no_pruning == results_pruning_1);
        std::cout << "\n--- Comparison Results ---" << std::endl;
        if (pruning_1_correct) {
            std::cout << "Pruning rule 1 results match no-pruning results. The pruning is correct." << std::endl;
        } else {
            std::cout << "ERROR: Pruning rule 1 failed comparison. Results are not identical." << std::endl;
        }
    }
};

class RunDijkstraRAPTORQueries : public ParameterizedCommand {

public:
    RunDijkstraRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraRAPTORQueries", "Runs the given number of random Dijkstra RAPTOR queries (with CH).") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunDijkstraRAPTORQueriesNoCH : public ParameterizedCommand {

public:
    RunDijkstraRAPTORQueriesNoCH(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraRAPTORQueriesNoCH", "Runs the given number of random Dijkstra RAPTOR queries (without CH).") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        
        std::cout << "Creating reverse network..." << std::endl;
        RAPTOR::Data reverseRaptorData = raptorData.reverseNetwork();
        std::cout << "Reverse network created." << std::endl;
        
        RAPTOR::DijkstraRAPTOR<RAPTOR::DijkstraInitialTransfers, RAPTOR::AggregateProfiler, true, false> 
            algorithm(raptorData, raptorData.transferGraph, reverseRaptorData.transferGraph);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(raptorData.transferGraph.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunTDDijkstraQueries : public ParameterizedCommand {

public:
    RunTDDijkstraQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTDDijkstraQueries", "Runs the given number of random TD-Dijkstra queries.") {
        addParameter("Intermediate input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // Load intermediate data and build time-dependent graph
        std::cout << "Loading intermediate data and building time-dependent graph..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        std::cout << "Time-dependent graph created: " << graph.numVertices() << " vertices, " 
                  << graph.numEdges() << " edges" << std::endl;

        // Create the TD-Dijkstra algorithm instance
        using TDDijkstra = TimeDependentDijkstraStateful<TimeDependentGraph, false>;
        TDDijkstra algorithm(graph);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(graph.numVertices(), n);

        // Statistics accumulators
        size_t totalSettles = 0;
        size_t totalRelaxes = 0;
        double totalTime = 0.0;
        size_t reachableCount = 0;
        int totalArrivalTime = 0;

        std::cout << "\nRunning " << n << " TD-Dijkstra queries..." << std::endl;

        // Run all queries
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            
            totalSettles += algorithm.getSettleCount();
            totalRelaxes += algorithm.getRelaxCount();
            totalTime += algorithm.getElapsedMilliseconds();
            
            if (algorithm.reachable(query.target)) {
                reachableCount++;
                totalArrivalTime += algorithm.getArrivalTime(query.target);
            }
        }

        // Print statistics
        std::cout << "\n=== TD-Dijkstra Statistics ===" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Reachable targets: " << reachableCount << " (" 
                  << String::prettyDouble(100.0 * reachableCount / n) << "%)" << std::endl;
        std::cout << "\nAverage per query:" << std::endl;
        std::cout << "  Vertices settled: " << String::prettyDouble((double)totalSettles / n) << std::endl;
        std::cout << "  Edges relaxed: " << String::prettyDouble((double)totalRelaxes / n) << std::endl;
        std::cout << "  Query time: " << String::prettyDouble(totalTime / n) << " ms" << std::endl;
        
        if (reachableCount > 0) {
            std::cout << "  Arrival time (reachable): " 
                      << String::prettyInt(totalArrivalTime / reachableCount) << std::endl;
        }
        
        std::cout << "\nTotal execution time: " << String::prettyDouble(totalTime) << " ms" << std::endl;
    }
};

class RunTDDijkstraFullQueries : public ParameterizedCommand {

public:
    RunTDDijkstraFullQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTDDijkstraQueries", "Runs the given number of random TD-Dijkstra queries.") {
        addParameter("Intermediate input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // Load intermediate data and build time-dependent graph
        std::cout << "Loading intermediate data and building time-dependent graph..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        CH::CH ch(getParameter("CH data"));
        std::cout << "Time-dependent graph created: " << graph.numVertices() << " vertices, "
                  << graph.numEdges() << " edges" << std::endl;

        // Create the TD-Dijkstra algorithm instance
        using TDDijkstra = TimeDependentDijkstraFull<TimeDependentGraph, false>;
        TDDijkstra algorithm(graph, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(graph.numVertices(), n);

        // Statistics accumulators
        size_t totalSettles = 0;
        size_t totalRelaxes = 0;
        double totalTime = 0.0;
        size_t reachableCount = 0;
        int totalArrivalTime = 0;

        std::cout << "\nRunning " << n << " TD-Dijkstra queries..." << std::endl;

        // Run all queries
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);

            totalSettles += algorithm.getSettleCount();
            totalRelaxes += algorithm.getRelaxCount();
            totalTime += algorithm.getElapsedMilliseconds();

            if (algorithm.reachable(query.target)) {
                reachableCount++;
                totalArrivalTime += algorithm.getArrivalTime(query.target);
            }
        }

        // Print statistics
        std::cout << "\n=== TD-Dijkstra Statistics ===" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Reachable targets: " << reachableCount << " ("
                  << String::prettyDouble(100.0 * reachableCount / n) << "%)" << std::endl;
        std::cout << "\nAverage per query:" << std::endl;
        std::cout << "  Vertices settled: " << String::prettyDouble((double)totalSettles / n) << std::endl;
        std::cout << "  Edges relaxed: " << String::prettyDouble((double)totalRelaxes / n) << std::endl;
        std::cout << "  Query time: " << String::prettyDouble(totalTime / n) << " ms" << std::endl;

        if (reachableCount > 0) {
            std::cout << "  Arrival time (reachable): "
                      << String::prettyInt(totalArrivalTime / reachableCount) << std::endl;
        }

        std::cout << "\nTotal execution time: " << String::prettyDouble(totalTime) << " ms" << std::endl;
    }
};

class RunTDDijkstraQueriesFromBinary : public ParameterizedCommand {

public:
    RunTDDijkstraQueriesFromBinary(BasicShell& shell) :
        ParameterizedCommand(shell, "runTDDijkstraQueriesFromBinary", "Runs the given number of random TD-Dijkstra queries (precomputed TD graph).") {
        addParameter("TD Graph input file");
        addParameter("Intermediate input file"); // <--- Added Parameter
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // Load precomputed time-dependent graph
        std::cout << "Loading pre-computed time-dependent graph..." << std::endl;
        TimeDependentGraph graph = TimeDependentGraph::FromBinary(getParameter("TD Graph input file"));
        std::cout << "Time-dependent graph loaded: " << graph.numVertices() << " vertices, "
                  << graph.numEdges() << " edges" << std::endl;

        // Load intermediate data (Required for O(1) optimization)
        // std::cout << "Loading intermediate data..." << std::endl;
        // Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));

        using TDDijkstra = TimeDependentDijkstraStateful<TimeDependentGraph, false>;
        
        // Pass intermediateData to constructor
        TDDijkstra algorithm(graph);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(graph.numVertices(), n);

        size_t totalSettles = 0;
        size_t totalRelaxes = 0;
        double totalTime = 0.0;
        size_t reachableCount = 0;
        int totalArrivalTime = 0;

        std::cout << "\nRunning " << n << " TD-Dijkstra queries..." << std::endl;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            totalSettles += algorithm.getSettleCount();
            totalRelaxes += algorithm.getRelaxCount();
            totalTime += algorithm.getElapsedMilliseconds();
            if (algorithm.reachable(query.target)) {
                reachableCount++;
                totalArrivalTime += algorithm.getArrivalTime(query.target);
            }
        }

        std::cout << "\n=== TD-Dijkstra Statistics ===" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Reachable targets: " << reachableCount << " ("
                  << String::prettyDouble(100.0 * reachableCount / n) << "%)" << std::endl;
        std::cout << "\nAverage per query:" << std::endl;
        std::cout << "  Vertices settled: " << String::prettyDouble((double)totalSettles / n) << std::endl;
        std::cout << "  Edges relaxed: " << String::prettyDouble((double)totalRelaxes / n) << std::endl;
        std::cout << "  Query time: " << String::prettyDouble(totalTime / n) << " ms" << std::endl;
        if (reachableCount > 0) {
            std::cout << "  Arrival time (reachable): "
                      << String::prettyInt(totalArrivalTime / (int)reachableCount) << std::endl;
        }
        std::cout << "\nTotal execution time: " << String::prettyDouble(totalTime) << " ms" << std::endl;
    }
};

class CompareMRwithTDDijkstra : public ParameterizedCommand {

public:
    CompareMRwithTDDijkstra(BasicShell& shell) :
        ParameterizedCommand(shell, "compareMRwithTDDijkstra", "Compares MR (with CH) with TD-Dijkstra.") {
        addParameter("RAPTOR input file");
        addParameter("Intermediate input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        
        std::cout << "Building time-dependent graph..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        std::cout << "Time-dependent graph created: " << graph.numVertices() << " vertices, " 
                  << graph.numEdges() << " edges" << std::endl;
        
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning;

        // --- Run with Target Pruning disabled (baseline) ---
        std::cout << "\n--- Running MR ---" << std::endl;
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm_no_pruning(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algorithm_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algorithm_no_pruning.getEarliestArrivalTime(query.target));
        }

        std::cout << "--- Statistics MR ---" << std::endl;
        algorithm_no_pruning.getProfiler().printStatistics();

        // --- Run TD-Dijkstra ---
        std::cout << "\n--- Running TD-Dijkstra ---" << std::endl;

        using TDDijkstra = TimeDependentDijkstraFull<TimeDependentGraph, false>;
        TDDijkstra algorithm_td(graph, ch);

        size_t totalSettles = 0;
        size_t totalRelaxes = 0;

        for (const VertexQuery& query : queries) {
            algorithm_td.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algorithm_td.getArrivalTime(query.target));
            totalSettles += algorithm_td.getSettleCount();
            totalRelaxes += algorithm_td.getRelaxCount();
        }
        
        std::cout << "--- Statistics TD-Dijkstra ---" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Avg. vertices settled: " << String::prettyDouble((double)totalSettles / n) << std::endl;
        std::cout << "Avg. edges relaxed: " << String::prettyDouble((double)totalRelaxes / n) << std::endl;

        // --- Compare results ---
        std::cout << "\n--- Comparison Results ---" << std::endl;
        bool pruning_correct = true;
        for (size_t i = 0; i < n; ++i) {
            if (results_no_pruning[i] != results_pruning[i]) {
                std::cout << "ERROR: Mismatch found for query " << i << "." << std::endl;
                std::cout << "  No Pruning Result: " << results_no_pruning[i] << std::endl;
                std::cout << "  Pruning Result: " << results_pruning[i] << std::endl;
                pruning_correct = false;
                break;
            }
        }

        if (pruning_correct) {
            std::cout << "Target pruning results match non-pruning results. The pruning is correct." << std::endl;
        } else {
            std::cout << "ERROR: Target pruning failed comparison. Results are not identical." << std::endl;
        }
    }
};

class CompareMRwithTDStatefulNoCH : public ParameterizedCommand {

public:
    CompareMRwithTDStatefulNoCH(BasicShell& shell) :
        ParameterizedCommand(shell, "compareMRwithTDStatefulNoCH", "Compares MR (without CH) with TD-Dijkstra (stateful buffers on transfers only).") {
        addParameter("RAPTOR input file");
        addParameter("Intermediate input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        
        std::cout << "Building time-dependent graph..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        std::cout << "Time-dependent graph created: " << graph.numVertices() << " vertices, " 
                  << graph.numEdges() << " edges" << std::endl;
        
        std::cout << "Creating reverse network..." << std::endl;
        RAPTOR::Data reverseRaptorData = raptorData.reverseNetwork();
        std::cout << "Reverse network created." << std::endl;

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(raptorData.transferGraph.numVertices(), n);

        std::vector<int> results_mr;
        std::vector<int> results_td;

        // --- Run MR without CH ---
        std::cout << "\n--- Running MR (without CH) ---" << std::endl;
        RAPTOR::DijkstraRAPTOR<RAPTOR::DijkstraInitialTransfers, RAPTOR::AggregateProfiler, true, false> 
            algorithm_mr(raptorData, raptorData.transferGraph, reverseRaptorData.transferGraph);
        
        Timer mrTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_mr.run(query.source, query.departureTime, query.target);
            results_mr.push_back(algorithm_mr.getEarliestArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  MR: " << (i + 1) << "/" << n << " queries (" 
                          << String::msToString(mrTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        std::cout << std::endl;

        std::cout << "--- Statistics MR (without CH) ---" << std::endl;
        algorithm_mr.getProfiler().printStatistics();

        // --- Run TD-Dijkstra (stateful) ---
        std::cout << "\n--- Running TD-Dijkstra (stateful) ---" << std::endl;

        using TDDijkstraStateful = TimeDependentDijkstraStateful<TimeDependentGraph, false>;
        // Pass intermediateData as the second argument
        TDDijkstraStateful algorithm_td(graph, raptorData.numberOfStops());

        size_t totalSettles = 0;
        size_t totalRelaxes = 0;

        Timer tdTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td.run(query.source, query.departureTime, query.target);
            results_td.push_back(algorithm_td.getArrivalTime(query.target));
            totalSettles += algorithm_td.getSettleCount();
            totalRelaxes += algorithm_td.getRelaxCount();
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD: " << (i + 1) << "/" << n << " queries (" 
                          << String::msToString(tdTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        std::cout << std::endl;
        
        std::cout << "--- Statistics TD-Dijkstra (stateful) ---" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Avg. vertices settled: " << String::prettyDouble((double)totalSettles / n) << std::endl;
        std::cout << "Avg. edges relaxed: " << String::prettyDouble((double)totalRelaxes / n) << std::endl;

        // --- Compare results ---
        std::cout << "\n--- Comparison Results ---" << std::endl;
        bool results_match = true;
        size_t mismatchCount = 0;
        std::vector<size_t> mismatchIndices;
        for (size_t i = 0; i < n; ++i) {
            if (results_mr[i] != results_td[i]) {
                // Check if this is a pure walking query (no transit used)
                // A pure walking query has arrival time = departure time + walk distance
                
                if (mismatchCount < 10) {  // Only print first 10 mismatches
                    std::cout << "Mismatch for query " << i << ": MR=" << results_mr[i] 
                              << ", TD-Dijkstra=" << results_td[i] 
                              << " (diff: " << (results_mr[i] - results_td[i]) << "s)"
                              << std::endl;
                }
                results_match = false;
                mismatchCount++;
                mismatchIndices.push_back(i);
            }
        }

        if (results_match) {
            std::cout << "✓ All results match! MR and TD-Dijkstra (stateful) produce identical arrival times." << std::endl;
        } else {
            std::cout << "✗ Found " << mismatchCount << " mismatches out of " << n << " queries." << std::endl;
            std::cout << "\nAll mismatched query indices: ";
            for (size_t i = 0; i < mismatchIndices.size(); ++i) {
                std::cout << mismatchIndices[i];
                if (i < mismatchIndices.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
    }
};

class CompareMRwithTDStatefulCoreCH : public ParameterizedCommand {

public:
    CompareMRwithTDStatefulCoreCH(BasicShell& shell) :
        ParameterizedCommand(shell, "compareMRwithTDStatefulCoreCH", "Compares MR (with CoreCH) with TD-Dijkstra (stateful buffers on transfers only) with CoreCH.") {
        addParameter("RAPTOR input file");
        addParameter("Intermediate input file");
        addParameter("Core CH input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        
        std::cout << "Building time-dependent graph..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        std::cout << "Time-dependent graph created: " << graph.numVertices() << " vertices, " 
                  << graph.numEdges() << " edges" << std::endl;
        
        std::cout << "Loading CoreCH..." << std::endl;
        CH::CH ch(getParameter("Core CH input file"));
        std::cout << "CoreCH loaded." << std::endl;

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.getGraph(FORWARD).numVertices(), n);

        std::vector<int> results_mr;
        std::vector<int> results_td;

        // --- Run MR with CoreCH ---
        std::cout << "\n--- Running MR (with CoreCH) ---" << std::endl;
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> 
            algorithm_mr(raptorData, ch);
        
        Timer mrTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_mr.run(query.source, query.departureTime, query.target);
            results_mr.push_back(algorithm_mr.getEarliestArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  MR: " << (i + 1) << "/" << n << " queries (" 
                          << String::msToString(mrTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        std::cout << std::endl;

        std::cout << "--- Statistics MR (with CoreCH) ---" << std::endl;
        algorithm_mr.getProfiler().printStatistics();

        // --- Run TD-Dijkstra (stateful) with CoreCH ---
        std::cout << "\n--- Running TD-Dijkstra (stateful) with CoreCH ---" << std::endl;

        using TDDijkstraStateful = TimeDependentDijkstraStateful<TimeDependentGraph, true>;
        TDDijkstraStateful algorithm_td(graph, raptorData.numberOfStops(), &ch);

        size_t totalSettles = 0;
        size_t totalRelaxes = 0;

        Timer tdTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td.run(query.source, query.departureTime, query.target);
            results_td.push_back(algorithm_td.getArrivalTime(query.target));
            totalSettles += algorithm_td.getSettleCount();
            totalRelaxes += algorithm_td.getRelaxCount();
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD: " << (i + 1) << "/" << n << " queries (" 
                          << String::msToString(tdTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        std::cout << std::endl;
        
        std::cout << "--- Statistics TD-Dijkstra (stateful) ---" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Avg. vertices settled: " << String::prettyDouble((double)totalSettles / n) << std::endl;
        std::cout << "Avg. edges relaxed: " << String::prettyDouble((double)totalRelaxes / n) << std::endl;

        // --- Compare results ---
        std::cout << "\n--- Comparison Results ---" << std::endl;
        bool results_match = true;
        size_t mismatchCount = 0;
        int maxDiff = 0;
        double totalDiff = 0;
        
        for (size_t i = 0; i < n; ++i) {
            if (results_mr[i] != results_td[i]) {
                int diff = results_td[i] - results_mr[i];  // TD arrival - MR arrival (positive = TD is worse)
                if (diff > maxDiff) maxDiff = diff;
                totalDiff += diff;
                if (mismatchCount < 5) {
                    std::cout << "Mismatch for query " << i << ": MR=" << results_mr[i] 
                              << ", TD-Dijkstra=" << results_td[i] 
                              << " (TD is " << diff << "s later)" << std::endl;
                }
                results_match = false;
                mismatchCount++;
            }
        }
        
        if (results_match) {
            std::cout << "SUCCESS: All " << n << " results match!" << std::endl;
        } else {
            std::cout << "FAILURE: " << mismatchCount << "/" << n << " mismatches (" 
                      << (100.0 * mismatchCount / n) << "%)" << std::endl;
            std::cout << "Max difference: " << maxDiff << "s" << std::endl;
            std::cout << "Avg difference (mismatches only): " << (totalDiff / mismatchCount) << "s" << std::endl;
        }
    }
};

class CheckDijkstraRAPTORPruning : public ParameterizedCommand {

public:
    CheckDijkstraRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "runCheckDijkstraRAPTORPruning", "Runs the given number of random Dijkstra RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning;

        // --- Run with Target Pruning disabled (baseline) ---
        std::cout << "\n--- Running without Target Pruning ---" << std::endl;
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm_no_pruning(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algorithm_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algorithm_no_pruning.getEarliestArrivalTime(query.target));
        }

        // --- Run with Target Pruning enabled ---
        std::cout << "\n--- Running with Target Pruning ---" << std::endl;
        raptorData.sortTransferGraphEdgesByTravelTime();
        RAPTOR::DijkstraRAPTOR_prune<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm_pruning(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algorithm_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algorithm_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics with Target Pruning ---" << std::endl;
        algorithm_pruning.getProfiler().printStatistics();
        std::cout << "--- Statistics without Target Pruning ---" << std::endl;
        algorithm_no_pruning.getProfiler().printStatistics();

        // --- Compare results ---
        std::cout << "\n--- Comparison Results ---" << std::endl;
        bool pruning_correct = true;
        for (size_t i = 0; i < n; ++i) {
            if (results_no_pruning[i] != results_pruning[i]) {
                std::cout << "ERROR: Mismatch found for query " << i << "." << std::endl;
                std::cout << "  No Pruning Result: " << results_no_pruning[i] << std::endl;
                std::cout << "  Pruning Result: " << results_pruning[i] << std::endl;
                pruning_correct = false;
                break;
            }
        }

        if (pruning_correct) {
            std::cout << "Target pruning results match non-pruning results. The pruning is correct." << std::endl;
        } else {
            std::cout << "ERROR: Target pruning failed comparison. Results are not identical." << std::endl;
        }
    }
};

class RunHLRAPTORQueries : public ParameterizedCommand {

public:
    RunHLRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runHLRAPTORQueries", "Runs the given number of random HL-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        RAPTOR::HLRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRARAPTORQueries : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRARAPTORQueries", "Runs the given number of random ULTRA-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.sortTransferGraphEdgesByTravelTime(); // Call to sort the transfer graph edges
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        // RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
        const size_t n = getParameter<size_t>("Number of queries");
        // Read the pruning rule from the user
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
        // double numJourneys = 0;
        //for (const VertexQuery& query : queries) {
        //    algorithm.run(query.source, query.departureTime, query.target);
        //    numJourneys += algorithm.getJourneys().size();
        //}
        //algorithm.getProfiler().printStatistics();
        auto runAndProfile = [&](auto& algorithm) {
            double numJourneys = 0;
            for (const VertexQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
        };

        switch (pruningRule) {
            case 0: {
                RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
                runAndProfile(algorithm);
                break;
            }
            case 1: {
                RAPTOR::ULTRARAPTOR_prune<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
                runAndProfile(algorithm);
                break;
            }
            default: {
                std::cout << "Invalid pruning rule. Please choose 0 or 1" << std::endl;
                break;
            }
        }
    }
};

class CheckULTRARAPTORPruning : public ParameterizedCommand {

public:
    CheckULTRARAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkULTRARAPTORPruning", "Checks if pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;
        std::vector<int> results_pruning_2;

        // Run with pruning rule 0 (no pruning)
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algo_no_pruning(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime());
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();


        // Run with pruning rule 1
        // Start the timer
        auto start = std::chrono::high_resolution_clock::now();

        raptorData.sortTransferGraphEdgesByTravelTime();

        // Stop the timer
        auto stop = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        // Print the duration
        std::cout << "Time taken to sort transfer graph edges: " << duration.count() << " microseconds" << std::endl;

        RAPTOR::ULTRARAPTOR_prune<RAPTOR::AggregateProfiler, false> algo_pruning_1(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime());
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();

        // Compare the results
        bool pruning_1_correct = (results_no_pruning == results_pruning_1);

        if (!pruning_1_correct) {
            std::cout << "Pruning rule 1 failed comparison." << std::endl;
        }
    }
};

class RunULTRARAPTORQueries_updated : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries_updated(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRARAPTORQueries", "Runs the given number of random ULTRA-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunTransitiveTBQueries : public ParameterizedCommand {

public:
    RunTransitiveTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveTBQueries", "Runs the given number of random transitive TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::TransitiveQuery<TripBased::AggregateProfiler> algorithm(tripBasedData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(tripBasedData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRATBQueries : public ParameterizedCommand {
public:
    RunULTRATBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRATBQueries", "Runs the given number of random ULTRA-TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::Query<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class CheckTDDijkstraPruning : public ParameterizedCommand {

public:
    CheckTDDijkstraPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkTDDijkstraPruning", "Checks if TD-Dijkstra pruning rules yield the same results as no pruning.") {
        addParameter("Intermediate input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // 1. Load Data
        std::cout << "Loading intermediate data..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        
        std::cout << "Building time-dependent graph..." << std::endl;
        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        std::cout << "Graph built: " << graph.numVertices() << " vertices, " << graph.numEdges() << " edges." << std::endl;
        
        std::string chFile = getParameter("CH data");
        CH::CH* chPointer = nullptr;
        CH::CH chData; 
        
        if (chFile != "nullptr" && !chFile.empty()) {
            std::cout << "Loading CH data..." << std::endl;
            chData = CH::CH(chFile);
            chPointer = &chData;
        } else {
            std::cout << "Running without CH optimization." << std::endl;
        }

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(graph.numVertices(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning;

        // 2. Run WITHOUT Pruning (TARGET_PRUNING = false)
        std::cout << "\n--- Running without Target Pruning ---" << std::endl;
        // Template Args: <Graph, Debug=false, TargetPruning=false>
        using TDDijkstraNoPrune = TimeDependentDijkstraStateful<TimeDependentGraph, false, false>;
        
        // Constructor: graph, numStops (0=auto), chPointer
        TDDijkstraNoPrune algo_no_pruning(graph, 0, chPointer);
        
        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getArrivalTime(query.target));
        }
        std::cout << "--- Statistics (No Pruning) ---" << std::endl;
        std::cout << "Settles: " << String::prettyDouble((double)algo_no_pruning.getSettleCount() / n) << std::endl;
        std::cout << "Relaxes: " << String::prettyDouble((double)algo_no_pruning.getRelaxCount() / n) << std::endl;
        std::cout << "Time:    " << String::prettyDouble(algo_no_pruning.getElapsedMilliseconds() / n) << " ms" << std::endl;

        // 3. Run WITH Pruning (TARGET_PRUNING = true)
        std::cout << "\n--- Running with Target Pruning ---" << std::endl;
        // Template Args: <Graph, Debug=false, TargetPruning=true>
        using TDDijkstraPrune = TimeDependentDijkstraStateful<TimeDependentGraph, false, true>;
        
        TDDijkstraPrune algo_pruning(graph, 0, chPointer);

        for (const VertexQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algo_pruning.getArrivalTime(query.target));
        }
        std::cout << "--- Statistics (Pruning) ---" << std::endl;
        std::cout << "Settles: " << String::prettyDouble((double)algo_pruning.getSettleCount() / n) << std::endl;
        std::cout << "Relaxes: " << String::prettyDouble((double)algo_pruning.getRelaxCount() / n) << std::endl;
        std::cout << "Time:    " << String::prettyDouble(algo_pruning.getElapsedMilliseconds() / n) << " ms" << std::endl;

        // 4. Compare Results
        std::cout << "\n--- Comparison Results ---" << std::endl;
        bool match = true;
        for (size_t i = 0; i < n; ++i) {
            if (results_no_pruning[i] != results_pruning[i]) {
                std::cout << "Mismatch at query " << i << ": NoPrune=" << results_no_pruning[i] 
                          << " vs Prune=" << results_pruning[i] << std::endl;
                match = false;
                break;
            }
        }

        if (match) {
            std::cout << "SUCCESS: All results match." << std::endl;
        } else {
            std::cout << "FAILURE: Results differ." << std::endl;
        }
    }
};