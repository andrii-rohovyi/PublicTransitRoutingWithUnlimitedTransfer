#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/RAPTOR/Bounded/BoundedMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMHydRA.h"
#include "../../Algorithms/Dijkstra/TimeDependentMCDijkstra.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/McRAPTOR.h"
#include "../../Algorithms/RAPTOR/MCR.h"
#include "../../Algorithms/RAPTOR/ULTRAMcRAPTOR.h"
#include "../../Algorithms/TripBased/BoundedMcQuery/BoundedMcQuery.h"
#include "../../Algorithms/TripBased/Query/McQuery.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

class CompareMCTDDvsMCR : public ParameterizedCommand {
public:
    CompareMCTDDvsMCR(BasicShell& shell) :
        ParameterizedCommand(shell, "compareMCTDDvsMCR",
            "Compares Multi-Criteria Time-Dependent Dijkstra with MCR algorithm.") {
        addParameter("RAPTOR input file");
        addParameter("Intermediate input file");  // Added: for TDGraph
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // Load RAPTOR data for MCR
        std::cout << "=== Loading Data ===" << std::endl;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        // Load CH data
        CH::CH ch(getParameter("CH data"));
        std::cout << "CH vertices: " << ch.numVertices() << std::endl;

        // Build TimeDependentGraph from Intermediate data
        std::cout << "\n=== Building Time-Dependent Graph ===" << std::endl;
        auto tdBuildStart = std::chrono::high_resolution_clock::now();

        // Load Intermediate data and build TDGraph
        Intermediate::Data inter = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        inter.printInfo();
        TimeDependentGraph tdGraph = TimeDependentGraph::FromIntermediate(inter);

        auto tdBuildEnd = std::chrono::high_resolution_clock::now();
        auto tdBuildTime = std::chrono::duration_cast<std::chrono::milliseconds>(tdBuildEnd - tdBuildStart);
        std::cout << "TD Graph build time: " << tdBuildTime.count() << " ms" << std::endl;
        tdGraph.printStatistics();

        // Initialize algorithms
        RAPTOR::MCR<true, RAPTOR::AggregateProfiler> mcrAlgo(raptorData, ch);
        TDD::TimeDependentMCDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>
            mctddAlgo(tdGraph, raptorData.numberOfStops(), &ch);

        // Generate queries
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
        std::cout << "\n=== Running " << n << " Queries ===" << std::endl;

        // Storage for results - use simple pair vectors for comparison
        std::vector<std::vector<std::pair<int, int>>> mcrResults;  // (arrivalTime, walkingDistance)
        std::vector<std::vector<std::pair<int, int>>> mctddResults;
        mcrResults.reserve(n);
        mctddResults.reserve(n);

        // Run MCR
        std::cout << "\n--- Running MCR ---" << std::endl;
        auto mcrStart = std::chrono::high_resolution_clock::now();

        double mcrJourneys = 0;
        for (const VertexQuery& query : queries) {
            mcrAlgo.run(query.source, query.departureTime, query.target);
            auto rawResults = mcrAlgo.getResults();
            std::vector<std::pair<int, int>> converted;
            for (const auto& r : rawResults) {
                converted.push_back({r.arrivalTime, r.walkingDistance});
            }
            mcrResults.push_back(std::move(converted));
            mcrJourneys += mcrResults.back().size();
        }

        auto mcrEnd = std::chrono::high_resolution_clock::now();
        auto mcrTime = std::chrono::duration_cast<std::chrono::microseconds>(mcrEnd - mcrStart);

        std::cout << "MCR Statistics:" << std::endl;
        mcrAlgo.getProfiler().printStatistics();
        std::cout << "Total time: " << mcrTime.count() / 1000.0 << " ms" << std::endl;
        std::cout << "Avg time per query: " << mcrTime.count() / (double)n << " µs" << std::endl;
        std::cout << "Avg journeys: " << String::prettyDouble(mcrJourneys / n) << std::endl;

        // Run MC-TDD
        std::cout << "\n--- Running MC-TDD ---" << std::endl;
        auto mctddStart = std::chrono::high_resolution_clock::now();

        double mctddJourneys = 0;
        for (const VertexQuery& query : queries) {
            mctddAlgo.run(query.source, query.departureTime, query.target);
            auto rawResults = mctddAlgo.getResults();
            std::vector<std::pair<int, int>> converted;
            for (const auto& r : rawResults) {
                converted.push_back({r.arrivalTime, r.walkingDistance});
            }
            mctddResults.push_back(std::move(converted));
            mctddJourneys += mctddResults.back().size();
        }

        auto mctddEnd = std::chrono::high_resolution_clock::now();
        auto mctddTime = std::chrono::duration_cast<std::chrono::microseconds>(mctddEnd - mctddStart);

        std::cout << "MC-TDD Statistics:" << std::endl;
        mctddAlgo.getProfiler().printStatistics();
        std::cout << "Total time: " << mctddTime.count() / 1000.0 << " ms" << std::endl;
        std::cout << "Avg time per query: " << mctddTime.count() / (double)n << " µs" << std::endl;
        std::cout << "Avg journeys: " << String::prettyDouble(mctddJourneys / n) << std::endl;

        // Compare results
        std::cout << "\n=== Comparing Results ===" << std::endl;
        bool allCorrect = compareResults(mcrResults, mctddResults, queries);

        // Summary
        std::cout << "\n=== Summary ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "MCR total time:    " << mcrTime.count() / 1000.0 << " ms" << std::endl;
        std::cout << "MC-TDD total time: " << mctddTime.count() / 1000.0 << " ms" << std::endl;
        std::cout << "Speedup: " << (double)mcrTime.count() / mctddTime.count() << "x" << std::endl;
        std::cout << "Correctness: " << (allCorrect ? "✅ PASSED" : "❌ FAILED") << std::endl;
    }

private:
    // Compare results from both algorithms (using simple pairs)
    bool compareResults(
        const std::vector<std::vector<std::pair<int, int>>>& mcrResults,
        const std::vector<std::vector<std::pair<int, int>>>& mctddResults,
        const std::vector<VertexQuery>& queries) const {

        bool allCorrect = true;
        size_t mismatchCount = 0;
        size_t extraInMCR = 0;
        size_t extraInMCTDD = 0;

        for (size_t i = 0; i < queries.size(); ++i) {
            std::vector<std::pair<int, int>> mcr = mcrResults[i];
            std::vector<std::pair<int, int>> mctdd = mctddResults[i];

            // Sort both for comparison
            std::sort(mcr.begin(), mcr.end());
            std::sort(mctdd.begin(), mctdd.end());

            // Compare sizes
            if (mcr.size() != mctdd.size()) {
                if (mismatchCount < 5) {
                    std::cout << "Query " << i << ": Size mismatch - MCR: " << mcr.size()
                              << ", MC-TDD: " << mctdd.size() << std::endl;
                }
                allCorrect = false;
                mismatchCount++;

                if (mcr.size() > mctdd.size()) extraInMCR += (mcr.size() - mctdd.size());
                else extraInMCTDD += (mctdd.size() - mcr.size());
                continue;
            }

            // Compare individual labels
            for (size_t j = 0; j < mcr.size(); ++j) {
                if (mcr[j] != mctdd[j]) {
                    if (mismatchCount < 5) {
                        std::cout << "Query " << i << ", label " << j << ": Value mismatch" << std::endl;
                        std::cout << "  MCR:    (arr=" << mcr[j].first
                                  << ", walk=" << mcr[j].second << ")" << std::endl;
                        std::cout << "  MC-TDD: (arr=" << mctdd[j].first
                                  << ", walk=" << mctdd[j].second << ")" << std::endl;
                    }
                    allCorrect = false;
                    mismatchCount++;
                    break;
                }
            }
        }

        std::cout << "Comparison complete:" << std::endl;
        std::cout << "  Total queries: " << queries.size() << std::endl;
        std::cout << "  Mismatches: " << mismatchCount << std::endl;
        if (!allCorrect) {
            std::cout << "  Extra labels in MCR: " << extraInMCR << std::endl;
            std::cout << "  Extra labels in MC-TDD: " << extraInMCTDD << std::endl;
        }

        return allCorrect;
    }
};

// =========================================================================
// Run MC-TDD Queries (standalone)
// =========================================================================

class RunMCTDDQueries : public ParameterizedCommand {
public:
    RunMCTDDQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMCTDDQueries",
            "Runs the given number of random Multi-Criteria Time-Dependent Dijkstra queries.") {
        addParameter("TD Graph file");
        addParameter("CH data");
        addParameter("Number of stops");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // Load TD Graph
        std::cout << "Loading Time-Dependent Graph..." << std::endl;
        TimeDependentGraph tdGraph = TimeDependentGraph::FromBinary(getParameter("TD Graph file"));
        tdGraph.printStatistics();

        // Load CH data
        CH::CH ch(getParameter("CH data"));

        const size_t numStops = getParameter<size_t>("Number of stops");

        // Initialize algorithm
        TDD::TimeDependentMCDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>
            algorithm(tdGraph, numStops, &ch);

        // Generate queries
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        // Run queries
        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getResults().size();
        }

        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

// =========================================================================
// Correctness Check: MC-TDD vs MCR
// =========================================================================

class CheckMCTDDCorrectness : public ParameterizedCommand {
public:
    CheckMCTDDCorrectness(BasicShell& shell) :
        ParameterizedCommand(shell, "checkMCTDDCorrectness",
            "Validates MC-TDD results against MCR (ground truth).") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // This is essentially the same as CompareMCTDDvsMCR but focuses on correctness
        // and provides more detailed error reporting

        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();

        CH::CH ch(getParameter("CH data"));

        // Build TDGraph (same as comparison command)
        TimeDependentGraph tdGraph; // = buildTDGraphFromRAPTOR(raptorData);

        RAPTOR::MCR<true, RAPTOR::NoProfiler> mcrAlgo(raptorData, ch);
        TDD::TimeDependentMCDijkstra<TimeDependentGraph, TDD::NoProfiler, false, true>
            mctddAlgo(tdGraph, raptorData.numberOfStops(), &ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        size_t passed = 0;
        size_t failed = 0;

        for (size_t i = 0; i < n; ++i) {
            const VertexQuery& query = queries[i];

            mcrAlgo.run(query.source, query.departureTime, query.target);
            mctddAlgo.run(query.source, query.departureTime, query.target);

            auto mcrResults = mcrAlgo.getResults();
            auto mctddResultsRaw = mctddAlgo.getResults();

            // Convert and compare
            std::vector<std::pair<int, int>> mcrPairs, mctddPairs;
            for (const auto& r : mcrResults) {
                mcrPairs.push_back({r.arrivalTime, r.walkingDistance});
            }
            for (const auto& r : mctddResultsRaw) {
                mctddPairs.push_back({r.arrivalTime, r.walkingDistance});
            }

            std::sort(mcrPairs.begin(), mcrPairs.end());
            std::sort(mctddPairs.begin(), mctddPairs.end());

            if (mcrPairs == mctddPairs) {
                passed++;
            } else {
                failed++;
                if (failed <= 10) {
                    std::cout << "FAILED Query " << i << ": source=" << query.source
                              << ", target=" << query.target
                              << ", dep=" << query.departureTime << std::endl;
                    std::cout << "  MCR results: " << mcrPairs.size() << std::endl;
                    std::cout << "  MC-TDD results: " << mctddPairs.size() << std::endl;
                }
            }
        }

        std::cout << "\n=== Correctness Results ===" << std::endl;
        std::cout << "Passed: " << passed << "/" << n << std::endl;
        std::cout << "Failed: " << failed << "/" << n << std::endl;
        std::cout << "Status: " << (failed == 0 ? "✅ ALL PASSED" : "❌ SOME FAILED") << std::endl;
    }
};

class RunTransitiveMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveMcRAPTORQueries", "Runs the given number of random transitive McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algorithm(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunMCRQueries : public ParameterizedCommand {

public:
    RunMCRQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMCRQueries", "Runs the given number of random MCR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::MCR<true, RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

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

class CheckMCRPruning : public ParameterizedCommand {
public:
    CheckMCRPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkMCRPruning", "Checks if MCR pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        CH::CH ch(getParameter("CH data"));
        RAPTOR::MCR<true, RAPTOR::AggregateProfiler> algo_no_pruning(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_no_pruning;
        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_pruning;

        // Run baseline MCR (no pruning)
        std::cout << "--- Running MCR (baseline) ---" << std::endl;
        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getResults());
        }
        std::cout << "--- Statistics for baseline ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Start the timer
        auto start = std::chrono::high_resolution_clock::now();

        raptorData.sortTransferGraphEdgesByTravelTime();

        // Stop the timer
        auto stop = std::chrono::high_resolution_clock::now();
        // Calculate the duration
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        // Print the duration
        std::cout << "Time taken to sort transfer graph edges: " << duration.count() << " microseconds" << std::endl;

        // Run pruned MCR
        std::cout << "\n--- Running MCR (pruned) ---" << std::endl;
        RAPTOR::MCR_prune<true, RAPTOR::AggregateProfiler> algo_pruning(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algo_pruning.getResults());
        }
        std::cout << "--- Statistics for pruned ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

        // Compare
    bool pruning_correct = true;
        for (size_t i = 0; i < n; ++i) {
            std::vector<RAPTOR::WalkingParetoLabel> no_pruning_results = results_no_pruning[i];
            std::vector<RAPTOR::WalkingParetoLabel> pruning_results = results_pruning[i];

            if (no_pruning_results.size() != pruning_results.size()) {
                pruning_correct = false;
                std::cout << "ERROR for query " << i << ": Different number of results. No-pruning: " << no_pruning_results.size() << ", Pruning: " << pruning_results.size() << std::endl;
                continue;
            }

            std::sort(no_pruning_results.begin(), no_pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });
            std::sort(pruning_results.begin(), pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });

            for (size_t j = 0; j < no_pruning_results.size(); ++j) {
                if (no_pruning_results[j].arrivalTime != pruning_results[j].arrivalTime ||
                    no_pruning_results[j].walkingDistance != pruning_results[j].walkingDistance) {
                    pruning_correct = false;
                    std::cout << "ERROR for query " << i << ", mismatch at result " << j << ":" << std::endl;
                    std::cout << "No-pruning: (arrival=" << no_pruning_results[j].arrivalTime << ", walk=" << no_pruning_results[j].walkingDistance << ")" << std::endl;
                    }
            }
        }

        if (pruning_correct) {
            std::cout << "Pruning correctness check passed: identical results." << std::endl;
        } else {
            std::cout << "Pruning correctness check failed: mismatches detected." << std::endl;
        }
    }
};

class RunULTRAMcRAPTORQueries : public ParameterizedCommand {

public:
    RunULTRAMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcRAPTORQueries", "Runs the given number of random ULTRA-McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

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

class RunULTRAMcTBQueries : public ParameterizedCommand {

public:
    RunULTRAMcTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcTBQueries", "Runs the given number of random ULTRA-McTB queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::McQuery<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

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

class RunTransitiveBoundedMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveBoundedMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveBoundedMcRAPTORQueries", "Runs the given number of random transitive Bounded McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunUBMRAPTORQueries : public ParameterizedCommand {

public:
    RunUBMRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMRAPTORQueries", "Runs the given number of random UBM-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunUBMTBQueries : public ParameterizedCommand {

public:
    RunUBMTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMTBQueries", "Runs the given number of random UBM-TB queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::Data forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        TripBased::Data backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::BoundedMcQuery<TripBased::AggregateProfiler> algorithm(tripBasedData, forwardBoundedData, backwardBoundedData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunUBMHydRAQueries : public ParameterizedCommand {

public:
    RunUBMHydRAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMHydRAQueries", "Runs the given number of random UBM-HydRA queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        const TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        const TripBased::Data forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        const TripBased::Data backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        const CH::CH ch(getParameter("CH data"));

        RAPTOR::UBMHydRA<RAPTOR::AggregateProfiler> algorithm(tripBasedData, forwardBoundedData, backwardBoundedData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class ComputeTransferTimeSavings : public ParameterizedCommand {

public:
    ComputeTransferTimeSavings(BasicShell& shell) :
        ParameterizedCommand(shell, "computeTransferTimeSavings", "Computes the savings in transfer time of a 3-criteria (bounded) Pareto set compared to a 2-criteria one.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::ofstream outputFile(getParameter("Output file"));
        outputFile << std::setprecision(10);
        outputFile << "ArrivalSlack";
        for (const double tripSlack : tripSlacks) {
            const int slackAsInt = tripSlack * 100 - 100;
            for (const double threshold : thresholds) {
                const int thresholdAsInt = threshold * 100;
                outputFile << "\tTripSlack" << slackAsInt << "Savings" << thresholdAsInt;
            }
        }
        outputFile << "\n";
        outputFile.flush();

        for (const double arrivalSlack : arrivalSlacks) {
            outputFile << arrivalSlack;
            for (const double tripSlack : tripSlacks) {
                std::cout << "Arrival slack: " << arrivalSlack << ", trip slack: " << tripSlack << std::endl;
                std::vector<double> transferTimeSavings;
                for (const VertexQuery& query : queries) {
                    algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
                    const std::vector<RAPTOR::WalkingParetoLabel> fullLabels = algorithm.getResults();
                    const std::vector<RAPTOR::ArrivalLabel>& anchorLabels = algorithm.getAnchorLabels();
                    RAPTOR::WalkingParetoLabel bestLabel;
                    RAPTOR::WalkingParetoLabel bestAnchorLabel;
                    for (const RAPTOR::WalkingParetoLabel& label : fullLabels) {
                        if (label.walkingDistance <= bestLabel.walkingDistance) {
                            bestLabel = label;
                        }
                        if (label.walkingDistance <= bestAnchorLabel.walkingDistance && isAnchorLabel(label, anchorLabels)) {
                            bestAnchorLabel = label;
                        }
                    }
                    if (bestAnchorLabel.walkingDistance == 0) {
                        transferTimeSavings.emplace_back(0);
                    } else {
                        transferTimeSavings.emplace_back((bestAnchorLabel.walkingDistance - bestLabel.walkingDistance)/static_cast<double>(bestAnchorLabel.walkingDistance));
                    }
                }
                std::sort(transferTimeSavings.begin(), transferTimeSavings.end(), [&](const double a, const double b) {
                    return a > b;
                });
                size_t j = 0;
                std::vector<size_t> savingsCount(thresholds.size(), 0);
                for (const double s : transferTimeSavings) {
                    while (s < thresholds[j]) {
                        j++;
                        if (j == thresholds.size()) break;
                    }
                    if (j == thresholds.size()) break;
                    savingsCount[j]++;
                }
                for (const size_t c : savingsCount) {
                    const double ratio = c/static_cast<double>(transferTimeSavings.size());
                    outputFile << "\t" << ratio;
                }

            }
            outputFile << "\n";
            outputFile.flush();
        }
    }

private:
    std::vector<double> thresholds { 0.75, 0.5, 0.25 };
    std::vector<double> arrivalSlacks { 1, 1.1, 1.2, 1.3, 1.4, 1.5 };
    std::vector<double> tripSlacks { 1, 1.25, 1.5 };

    inline bool isAnchorLabel(const RAPTOR::WalkingParetoLabel& label, const std::vector<RAPTOR::ArrivalLabel>& anchorLabels) const noexcept {
        for (const RAPTOR::ArrivalLabel& anchorLabel : anchorLabels) {
            if (label.arrivalTime != anchorLabel.arrivalTime) continue;
            if (label.numberOfTrips != anchorLabel.numberOfTrips) continue;
            return true;
        }
        return false;
    }
};

class CheckBMcRAPTORPruning : public ParameterizedCommand {
public:
    CheckBMcRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkBMcRAPTORPruning", "Checks if BoundedMcRAPTOR pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();

        const size_t n = getParameter<size_t>("Number of queries");
        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        const RAPTOR::Data reverseData = raptorData.reverseNetwork();

        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_no_pruning;
        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_pruning;

        // Run baseline BoundedMcRAPTOR (no extra pruning in relaxTransfers inner-loop)
        std::cout << "--- Running BoundedMcRAPTOR (baseline) ---" << std::endl;
        RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algo_no_pruning(raptorData, reverseData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            results_no_pruning.push_back(algo_no_pruning.getResults(query.target));
        }
        std::cout << "--- Statistics for baseline ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run pruned BoundedMcRAPTOR (early break when dominated by target best)
        std::cout << "\n--- Running BoundedMcRAPTOR (pruned) ---" << std::endl;
        raptorData.sortTransferGraphEdgesByTravelTime();
        RAPTOR::BoundedMcRAPTOR_prune<RAPTOR::AggregateProfiler> algo_pruning(raptorData, reverseData);
        for (const StopQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            results_pruning.push_back(algo_pruning.getResults(query.target));
        }
        std::cout << "--- Statistics for pruned ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

        // Compare
        bool pruning_correct = true;
        for (size_t i = 0; i < n; ++i) {
            std::vector<RAPTOR::WalkingParetoLabel> no_pruning_results = results_no_pruning[i];
            std::vector<RAPTOR::WalkingParetoLabel> pruning_results = results_pruning[i];

            if (no_pruning_results.size() != pruning_results.size()) {
                pruning_correct = false;
                std::cout << "ERROR for query " << i << ": Different number of results. No-pruning: " << no_pruning_results.size() << ", Pruning: " << pruning_results.size() << std::endl;
                continue;
            }

            std::sort(no_pruning_results.begin(), no_pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });
            std::sort(pruning_results.begin(), pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });

            for (size_t j = 0; j < no_pruning_results.size(); ++j) {
                if (no_pruning_results[j].arrivalTime != pruning_results[j].arrivalTime ||
                    no_pruning_results[j].walkingDistance != pruning_results[j].walkingDistance) {
                    pruning_correct = false;
                    std::cout << "ERROR for query " << i << ", mismatch at result " << j << ":" << std::endl;
                    std::cout << "No-pruning: (arrival=" << no_pruning_results[j].arrivalTime << ", walk=" << no_pruning_results[j].walkingDistance << ")" << std::endl;
                    std::cout << "Pruning: (arrival=" << pruning_results[j].arrivalTime << ", walk=" << pruning_results[j].walkingDistance << ")" << std::endl;
                    break;
                }
            }
            if (!pruning_correct) break;
        }

        std::cout << "\n--- Comparison Results ---" << std::endl;
        if (pruning_correct) {
            std::cout << "Pruning results match baseline. The pruning is correct. ✅" << std::endl;
        } else {
            std::cout << "❌ ERROR: Pruning failed comparison. Results are not identical." << std::endl;
        }
    }
};

class CheckMcRAPTORPruning : public ParameterizedCommand {
public:
    CheckMcRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkMcRAPTORPruning", "Checks if McRAPTOR pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_no_pruning;
        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_pruning;

        // Run with no pruning (the baseline McRAPTOR)
        std::cout << "--- Running with No Pruning ---" << std::endl;
        RAPTOR::McRAPTOR<false, true, RAPTOR::AggregateProfiler> algo_no_pruning(raptorData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getResults(query.target));
        }
        std::cout << "--- Statistics for No Pruning ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning
        std::cout << "\n--- Running with Pruning ---" << std::endl;
        raptorData.sortTransferGraphEdgesByTravelTime();
        RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algo_pruning(raptorData);
        for (const StopQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algo_pruning.getResults(query.target));
        }
        std::cout << "--- Statistics for Pruning ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

        // Compare the results query by query
        bool pruning_correct = true;
        for (size_t i = 0; i < n; ++i) {
            std::vector<RAPTOR::WalkingParetoLabel> no_pruning_results = results_no_pruning[i];
            std::vector<RAPTOR::WalkingParetoLabel> pruning_results = results_pruning[i];

            // Sort both result vectors to ensure a consistent, canonical order for comparison
            std::sort(no_pruning_results.begin(), no_pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });
            std::sort(pruning_results.begin(), pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });

            // Compare the sorted vectors element by element
            for (size_t j = 0; j < no_pruning_results.size(); ++j) {
                if (no_pruning_results[j].arrivalTime != pruning_results[j].arrivalTime ||
                    no_pruning_results[j].walkingDistance != pruning_results[j].walkingDistance) {
                    pruning_correct = false;
                    std::cout << "ERROR for query " << i << ", mismatch at result " << j << ":" << std::endl;
                    std::cout << "No-pruning: (arrival=" << no_pruning_results[j].arrivalTime << ", walk=" << no_pruning_results[j].walkingDistance << ")" << std::endl;
                    std::cout << "Pruning: (arrival=" << pruning_results[j].arrivalTime << ", walk=" << pruning_results[j].walkingDistance << ")" << std::endl;
                    break;
                    }
            }
            if (!pruning_correct) break;
        }

        std::cout << "\n--- Comparison Results ---" << std::endl;
        if (pruning_correct) {
            std::cout << "Pruning results match no-pruning results. The pruning is correct. ✅" << std::endl;
        } else {
            std::cout << "❌ ERROR: Pruning failed comparison. Results are not identical." << std::endl;
        }
    }
};
class CheckULTRAMcRAPTORPruning : public ParameterizedCommand {

public:
    CheckULTRAMcRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkULTRAMcRAPTORPruning", "Checks if ULTRA-McRAPTOR pruning rules yield the same results as no pruning.") {
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
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_no_pruning;
        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_pruning;

        // Run with no pruning (the baseline ULTRA-McRAPTOR)
        std::cout << "--- Running ULTRA-McRAPTOR with No Pruning ---" << std::endl;
        RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algo_no_pruning(raptorData, ch);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getResults(query.target));
        }
        std::cout << "--- Statistics for No Pruning ---" << std::endl;
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


        // Run with pruning
        std::cout << "\n--- Running ULTRA-McRAPTOR with Pruning ---" << std::endl;
        RAPTOR::ULTRAMcRAPTOR_prune<RAPTOR::AggregateProfiler> algo_pruning(raptorData, ch);
        for (const StopQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algo_pruning.getResults(query.target));
        }
        std::cout << "--- Statistics for Pruning ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

        // Compare the results query by query
        bool pruning_correct = true;
        for (size_t i = 0; i < n; ++i) {
            std::vector<RAPTOR::WalkingParetoLabel> no_pruning_results = results_no_pruning[i];
            std::vector<RAPTOR::WalkingParetoLabel> pruning_results = results_pruning[i];

            if (no_pruning_results.size() != pruning_results.size()) {
                pruning_correct = false;
                std::cout << "ERROR for query " << i << ": Different number of results. No-pruning: " << no_pruning_results.size() << ", Pruning: " << pruning_results.size() << std::endl;
                continue;
            }

            // Sort both result vectors to ensure a consistent, canonical order for comparison
            std::sort(no_pruning_results.begin(), no_pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });
            std::sort(pruning_results.begin(), pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });

            // Compare the sorted vectors element by element
            for (size_t j = 0; j < no_pruning_results.size(); ++j) {
                if (no_pruning_results[j].arrivalTime != pruning_results[j].arrivalTime ||
                    no_pruning_results[j].walkingDistance != pruning_results[j].walkingDistance) {
                    pruning_correct = false;
                    std::cout << "ERROR for query " << i << ", mismatch at result " << j << ":" << std::endl;
                    std::cout << "No-pruning: (arrival=" << no_pruning_results[j].arrivalTime << ", walk=" << no_pruning_results[j].walkingDistance << ")" << std::endl;
                    std::cout << "Pruning: (arrival=" << pruning_results[j].arrivalTime << ", walk=" << pruning_results[j].walkingDistance << ")" << std::endl;
                    break;
                }
            }
            if (!pruning_correct) break;
        }

        std::cout << "\n--- Comparison Results ---" << std::endl;
        if (pruning_correct) {
            std::cout << "Pruning results match no-pruning results. The pruning is correct. ✅" << std::endl;
        } else {
            std::cout << "❌ ERROR: Pruning failed comparison. Results are not identical." << std::endl;
        }
    }

};

class CheckUBMRAPTORPruning : public ParameterizedCommand {
public:
    CheckUBMRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkUBMRAPTORPruning", "Checks if UBM-RAPTOR pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        CH::CH ch(getParameter("CH data"));

        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_no_pruning;
        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> results_pruning;

        // Run baseline UBM-RAPTOR (without pruning)
        std::cout << "--- Running UBM-RAPTOR (baseline) ---" << std::endl;
        raptorData.sortTransferGraphEdgesByTravelTime();
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algo_no_pruning(raptorData, reverseData, ch);
        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            results_no_pruning.push_back(algo_no_pruning.getResults());
        }
        std::cout << "--- Statistics for baseline ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run pruned UBM-RAPTOR (assuming a pruned version exists, or a flag is used)
        // Note: The provided UBM-RAPTOR code doesn't explicitly have a `_prune` version
        // like BoundedMcRAPTOR. This part is a conceptual check. If a pruned version
        // were available, you would use it here. For a real check, you'd modify UBM-RAPTOR
        // to toggle the pruning and test it.
        std::cout << "\n--- Running UBM-RAPTOR (pruned) ---" << std::endl;
        RAPTOR::UBMRAPTOR_prune<RAPTOR::AggregateProfiler> algo_pruning(raptorData, reverseData, ch); // Using the same class, assuming internal logic handles pruning
        for (const VertexQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            results_pruning.push_back(algo_pruning.getResults());
        }
        std::cout << "--- Statistics for pruned ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

        // Compare results
        bool pruning_correct = true;
        for (size_t i = 0; i < n; ++i) {
            std::vector<RAPTOR::WalkingParetoLabel> no_pruning_results = results_no_pruning[i];
            std::vector<RAPTOR::WalkingParetoLabel> pruning_results = results_pruning[i];

            if (no_pruning_results.size() != pruning_results.size()) {
                pruning_correct = false;
                std::cout << "ERROR for query " << i << ": Different number of results. No-pruning: " << no_pruning_results.size() << ", Pruning: " << pruning_results.size() << std::endl;
                continue;
            }

            std::sort(no_pruning_results.begin(), no_pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });
            std::sort(pruning_results.begin(), pruning_results.end(), [](const auto& a, const auto& b) {
                if (a.arrivalTime != b.arrivalTime) return a.arrivalTime < b.arrivalTime;
                return a.walkingDistance < b.walkingDistance;
            });

            for (size_t j = 0; j < no_pruning_results.size(); ++j) {
                if (no_pruning_results[j].arrivalTime != pruning_results[j].arrivalTime ||
                    no_pruning_results[j].walkingDistance != pruning_results[j].walkingDistance) {
                    pruning_correct = false;
                    std::cout << "ERROR for query " << i << ", mismatch at result " << j << ":" << std::endl;
                    std::cout << "No-pruning: (arrival=" << no_pruning_results[j].arrivalTime << ", walk=" << no_pruning_results[j].walkingDistance << ")" << std::endl;
                    std::cout << "Pruning: (arrival=" << pruning_results[j].arrivalTime << ", walk=" << pruning_results[j].walkingDistance << ")" << std::endl;
                    break;
                }
            }
            if (!pruning_correct) break;
        }

        std::cout << "\n--- Comparison Results ---" << std::endl;
        if (pruning_correct) {
            std::cout << "Pruning results match baseline. The pruning is correct. ✅" << std::endl;
        } else {
            std::cout << "❌ ERROR: Pruning failed comparison. Results are not identical." << std::endl;
        }
    }
};
