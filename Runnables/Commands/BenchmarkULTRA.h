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
#include "../../Algorithms/Dijkstra/TimeDependentDijkstraStatefulBucketCH.h"
#include "../../Algorithms/Dijkstra/TD-DijkstraStateful.h"
#include "../../Algorithms/Dijkstra/TD-DijkstraStatefulClassic.h"
#include "../../Algorithms/Dijkstra/TD-DijkstraStatefulFC.h"
#include "../../Algorithms/Dijkstra/TD-DijkstraStatefulCST.h"
#include "../../Algorithms/Dijkstra/JumpTripSearch.h"
#include "../../Algorithms/Dijkstra/TD-DijkstraStatefulBST.h"

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


class CompareAllAlgorithms : public ParameterizedCommand {

public:
    CompareAllAlgorithms(BasicShell& shell) :
        ParameterizedCommand(shell, "compareAllAlgorithms",
            "Compares MR (Core-CH), TD-Dijkstra (Core-CH), TD-Dijkstra (Bucket-CH), and ULTRA-CSA (Bucket-CH).") {
        addParameter("RAPTOR input file");
        addParameter("CSA input file");
        addParameter("Intermediate input file");
        addParameter("Core CH input file");
        addParameter("Full CH input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // ==================== LOAD DATA ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "         LOADING DATA" << std::endl;
        std::cout << "========================================\n" << std::endl;

        // Load RAPTOR data
        std::cout << "Loading RAPTOR data..." << std::endl;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        // Load CSA data
        std::cout << "\nLoading CSA data..." << std::endl;
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        // Load Intermediate data and build TD graph
        std::cout << "\nLoading Intermediate data..." << std::endl;
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        std::cout << "Intermediate data: " << intermediateData.numberOfStops() << " stops, "
                  << intermediateData.numberOfTrips() << " trips" << std::endl;

        std::cout << "\nBuilding TimeDependentGraph..." << std::endl;
        Timer buildTimer;
        TimeDependentGraph tdGraph = TimeDependentGraph::FromIntermediate(intermediateData);
        double tdGraphBuildTime = buildTimer.elapsedMilliseconds();
        std::cout << "TD graph created: " << tdGraph.numVertices() << " vertices, "
                  << tdGraph.numEdges() << " edges in " << String::msToString(tdGraphBuildTime) << std::endl;

        // Load Core-CH
        std::cout << "\nLoading Core-CH..." << std::endl;
        CH::CH coreCH(getParameter("Core CH input file"));
        std::cout << "Core-CH loaded: " << coreCH.numVertices() << " vertices" << std::endl;

        // Load Full CH (for Bucket-CH)
        std::cout << "\nLoading Full CH (for Bucket-CH)..." << std::endl;
        CH::CH fullCH(getParameter("Full CH input file"));
        std::cout << "Full CH loaded: " << fullCH.numVertices() << " vertices" << std::endl;

        // ==================== GENERATE QUERIES ====================
        const size_t n = getParameter<size_t>("Number of queries");

        // Use the smaller vertex count to ensure valid queries for all algorithms
        const size_t maxVertices = std::min(coreCH.numVertices(), fullCH.numVertices());
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(maxVertices, n);

        std::cout << "\nGenerated " << n << " random queries" << std::endl;

        // Results storage
        std::vector<int> results_mr_corech;
        std::vector<int> results_td_corech;
        std::vector<int> results_td_bucketch;
        std::vector<int> results_ultra_csa;

        results_mr_corech.reserve(n);
        results_td_corech.reserve(n);
        results_td_bucketch.reserve(n);
        results_ultra_csa.reserve(n);

        // ==================== ALGORITHM 1: MR with Core-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  1. MR (DijkstraRAPTOR) with Core-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        using MRCoreCH = RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false>;
        MRCoreCH algorithm_mr(raptorData, coreCH);

        Timer mrTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_mr.run(query.source, query.departureTime, query.target);
            results_mr_corech.push_back(algorithm_mr.getEarliestArrivalTime(query.target));

            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  MR (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double mrTime = mrTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- MR (Core-CH) Statistics ---" << std::endl;
        algorithm_mr.getProfiler().printStatistics();
        std::cout << "Total time: " << String::msToString(mrTime)
                  << " (" << (mrTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 2: TD-Dijkstra with Core-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  2. TD-Dijkstra with Core-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        using TDCoreCH = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        TDCoreCH algorithm_td_corech(tdGraph, raptorData.numberOfStops(), &coreCH);

        Timer tdCoreCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td_corech.run(query.source, query.departureTime, query.target);
            results_td_corech.push_back(algorithm_td_corech.getArrivalTime(query.target));

            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double tdCoreCHTime = tdCoreCHTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- TD-Dijkstra (Core-CH) Statistics ---" << std::endl;
        algorithm_td_corech.getProfiler().printStatistics();
        std::cout << "Total time: " << String::msToString(tdCoreCHTime)
                  << " (" << (tdCoreCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 3: TD-Dijkstra with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  3. TD-Dijkstra with Bucket-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "Building Bucket-CH initial transfers (this may take a while)..." << std::endl;
        Timer bucketBuildTimer;

        using TDBucketCH = TimeDependentDijkstraStatefulBucketCH<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        TDBucketCH algorithm_td_bucketch(tdGraph, raptorData.numberOfStops(), &fullCH);

        double bucketBuildTime = bucketBuildTimer.elapsedMilliseconds();
        std::cout << "Bucket-CH preprocessing time: " << String::msToString(bucketBuildTime) << std::endl;

        Timer tdBucketCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td_bucketch.run(query.source, query.departureTime, query.target);
            results_td_bucketch.push_back(algorithm_td_bucketch.getArrivalTime(query.target));

            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD (Bucket-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double tdBucketCHTime = tdBucketCHTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- TD-Dijkstra (Bucket-CH) Statistics ---" << std::endl;
        algorithm_td_bucketch.getProfiler().printStatistics();
        std::cout << "Total time: " << String::msToString(tdBucketCHTime)
                  << " (" << (tdBucketCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 4: ULTRA-CSA with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  4. ULTRA-CSA with Bucket-CH (Full CH)" << std::endl;
        std::cout << "========================================\n" << std::endl;

        CSA::ULTRACSA<true, 0, CSA::AggregateProfiler> algorithm_ultra_csa(csaData, fullCH);

        Timer ultraCSATimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_ultra_csa.run(query.source, query.departureTime, query.target);
            results_ultra_csa.push_back(algorithm_ultra_csa.getEarliestArrivalTime(query.target));

            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  ULTRA-CSA: " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double ultraCSATime = ultraCSATimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- ULTRA-CSA (Bucket-CH) Statistics ---" << std::endl;
        algorithm_ultra_csa.getProfiler().printStatistics();
        std::cout << "Total time: " << String::msToString(ultraCSATime)
                  << " (" << (ultraCSATime / n) << " ms/query)" << std::endl;

        // ==================== CORRECTNESS COMPARISON ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "         CORRECTNESS COMPARISON" << std::endl;
        std::cout << "========================================\n" << std::endl;

        // Use MR (Core-CH) as ground truth
        auto compareResults = [&](const std::string& name, const std::vector<int>& results) {
            size_t matches = 0;
            size_t mismatches = 0;
            size_t bothUnreachable = 0;
            size_t onlyGroundTruthReachable = 0;
            size_t onlyTestReachable = 0;
            int maxDiff = 0;
            double totalDiff = 0;

            std::vector<size_t> mismatchIndices;

            for (size_t i = 0; i < n; ++i) {
                bool gtReachable = (results_mr_corech[i] != never && results_mr_corech[i] != intMax);
                bool testReachable = (results[i] != never && results[i] != intMax);

                if (!gtReachable && !testReachable) {
                    bothUnreachable++;
                    matches++;
                } else if (gtReachable && !testReachable) {
                    onlyGroundTruthReachable++;
                    mismatches++;
                    mismatchIndices.push_back(i);
                } else if (!gtReachable && testReachable) {
                    onlyTestReachable++;
                    mismatches++;
                    mismatchIndices.push_back(i);
                } else {
                    // Both reachable
                    int diff = results[i] - results_mr_corech[i];
                    if (diff == 0) {
                        matches++;
                    } else {
                        mismatches++;
                        if (std::abs(diff) > maxDiff) maxDiff = std::abs(diff);
                        totalDiff += std::abs(diff);
                        mismatchIndices.push_back(i);
                    }
                }
            }

            std::cout << name << " vs MR (Core-CH):" << std::endl;
            std::cout << "  Matches:              " << matches << "/" << n
                      << " (" << (100.0 * matches / n) << "%)" << std::endl;
            std::cout << "  Mismatches:           " << mismatches << std::endl;

            if (mismatches > 0) {
                std::cout << "  Both unreachable:     " << bothUnreachable << std::endl;
                std::cout << "  Only GT reachable:    " << onlyGroundTruthReachable << std::endl;
                std::cout << "  Only test reachable:  " << onlyTestReachable << std::endl;
                std::cout << "  Max difference:       " << maxDiff << "s" << std::endl;
                if (mismatches - bothUnreachable - onlyGroundTruthReachable - onlyTestReachable > 0) {
                    std::cout << "  Avg difference:       " << (totalDiff / mismatches) << "s" << std::endl;
                }

                // Print first few mismatches
                std::cout << "  First mismatches:" << std::endl;
                for (size_t j = 0; j < std::min(mismatchIndices.size(), size_t(5)); ++j) {
                    size_t i = mismatchIndices[j];
                    std::cout << "    Query " << i << ": GT=" << results_mr_corech[i]
                              << ", Test=" << results[i]
                              << " (diff=" << (results[i] - results_mr_corech[i]) << "s)" << std::endl;
                }
            }

            if (matches == n) {
                std::cout << "  ✅ PERFECT MATCH!" << std::endl;
            } else {
                std::cout << "  ❌ MISMATCHES FOUND" << std::endl;
            }
            std::cout << std::endl;

            return matches == n;
        };

        std::cout << "Ground Truth: MR (Core-CH)\n" << std::endl;

        bool td_corech_correct = compareResults("TD-Dijkstra (Core-CH)", results_td_corech);
        bool td_bucketch_correct = compareResults("TD-Dijkstra (Bucket-CH)", results_td_bucketch);
        bool ultra_csa_correct = compareResults("ULTRA-CSA (Bucket-CH)", results_ultra_csa);

        // ==================== PERFORMANCE SUMMARY ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "         PERFORMANCE SUMMARY" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << std::fixed << std::setprecision(3);

        std::cout << "┌─────────────────────────────┬──────────────┬─────────────┬───────────┐" << std::endl;
        std::cout << "│ Algorithm                   │ Total Time   │ Per Query   │ Correct?  │" << std::endl;
        std::cout << "├─────────────────────────────┼──────────────┼─────────────┼───────────┤" << std::endl;

        std::cout << "│ MR (Core-CH)                │ "
                  << std::setw(10) << String::msToString(mrTime) << " │ "
                  << std::setw(9) << (mrTime / n) << " ms │ "
                  << "Ground T. │" << std::endl;

        std::cout << "│ TD-Dijkstra (Core-CH)       │ "
                  << std::setw(10) << String::msToString(tdCoreCHTime) << " │ "
                  << std::setw(9) << (tdCoreCHTime / n) << " ms │ "
                  << (td_corech_correct ? "    ✅    " : "    ❌    ") << "│" << std::endl;

        std::cout << "│ TD-Dijkstra (Bucket-CH)     │ "
                  << std::setw(10) << String::msToString(tdBucketCHTime) << " │ "
                  << std::setw(9) << (tdBucketCHTime / n) << " ms │ "
                  << (td_bucketch_correct ? "    ✅    " : "    ❌    ") << "│" << std::endl;

        std::cout << "│ ULTRA-CSA (Bucket-CH)       │ "
                  << std::setw(10) << String::msToString(ultraCSATime) << " │ "
                  << std::setw(9) << (ultraCSATime / n) << " ms │ "
                  << (ultra_csa_correct ? "    ✅    " : "    ❌    ") << "│" << std::endl;

        std::cout << "└─────────────────────────────┴──────────────┴─────────────┴───────────┘" << std::endl;

        // Speedup comparison
        std::cout << "\nSpeedup vs MR (Core-CH):" << std::endl;
        std::cout << "  TD-Dijkstra (Core-CH):    " << (mrTime / tdCoreCHTime) << "x" << std::endl;
        std::cout << "  TD-Dijkstra (Bucket-CH):  " << (mrTime / tdBucketCHTime) << "x" << std::endl;
        std::cout << "  ULTRA-CSA (Bucket-CH):    " << (mrTime / ultraCSATime) << "x" << std::endl;

        // Preprocessing overhead
        std::cout << "\nPreprocessing Overhead:" << std::endl;
        std::cout << "  TD Graph build time:      " << String::msToString(tdGraphBuildTime) << std::endl;
        std::cout << "  Bucket-CH build time:     " << String::msToString(bucketBuildTime) << std::endl;

        // Break-even analysis
        if (bucketBuildTime > 0 && tdBucketCHTime < tdCoreCHTime) {
            double querySavings = (tdCoreCHTime - tdBucketCHTime) / n;  // ms saved per query
            double breakEven = bucketBuildTime / querySavings;
            std::cout << "\nBucket-CH Break-even: ~" << (int)breakEven
                      << " queries to amortize preprocessing" << std::endl;
        }

        // ==================== FINAL CONCLUSION ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "            CONCLUSION" << std::endl;
        std::cout << "========================================\n" << std::endl;

        if (td_corech_correct && td_bucketch_correct && ultra_csa_correct) {
            std::cout << "✅ All algorithms produce IDENTICAL results!" << std::endl;
        } else {
            std::cout << "❌ Some algorithms produced DIFFERENT results:" << std::endl;
            if (!td_corech_correct) std::cout << "   - TD-Dijkstra (Core-CH)" << std::endl;
            if (!td_bucketch_correct) std::cout << "   - TD-Dijkstra (Bucket-CH)" << std::endl;
            if (!ultra_csa_correct) std::cout << "   - ULTRA-CSA (Bucket-CH)" << std::endl;
        }

        // Find fastest
        double fastest = std::min({mrTime, tdCoreCHTime, tdBucketCHTime, ultraCSATime});
        std::cout << "\nFastest algorithm: ";
        if (fastest == mrTime) std::cout << "MR (Core-CH)";
        else if (fastest == tdCoreCHTime) std::cout << "TD-Dijkstra (Core-CH)";
        else if (fastest == tdBucketCHTime) std::cout << "TD-Dijkstra (Bucket-CH)";
        else std::cout << "ULTRA-CSA (Bucket-CH)";
        std::cout << " at " << String::msToString(fastest) << std::endl;
    }
};

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
class CompareBSTvsClassicVariants : public ParameterizedCommand {

public:
    CompareBSTvsClassicVariants(BasicShell& shell) :
        ParameterizedCommand(shell, "compareBSTvsClassicVariants",
            "Compares TimeDependentDijkstraStatefulBST (Balanced Search Trees) vs TimeDependentDijkstraStatefulClassic (standard binary search).") {
        addParameter("Intermediate input file");
        addParameter("Core CH input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // --- Load Intermediate data ---
        std::cout << "\n=== Loading Intermediate data ===" << std::endl;

        Intermediate::Data intermediateData;

        try {
            intermediateData.deserialize(getParameter("Intermediate input file"));
        } catch (...) {
            std::cout << "ERROR: Could not load intermediate data." << std::endl;
            std::cout << "Please create intermediate data first using the appropriate command." << std::endl;
            return;
        }

        std::cout << "Intermediate data loaded: " << intermediateData.numberOfStops() << " stops, "
                  << intermediateData.numberOfTrips() << " trips" << std::endl;

        // --- Build both graph variants ---
        std::cout << "\n=== Building TimeDependentGraphClassic (Standard Binary Search) ===" << std::endl;
        Timer buildTimer;
        TimeDependentGraphClassic graphClassic = TimeDependentGraphClassic::FromIntermediate(intermediateData);
        double buildTimeClassic = buildTimer.elapsedMilliseconds();

        std::cout << "Classic graph created: " << graphClassic.numVertices() << " vertices, "
                  << graphClassic.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeClassic) << std::endl;
        graphClassic.printStatistics();

        std::cout << "\n=== Building TimeDependentGraphBST (with Balanced Search Trees) ===" << std::endl;
        buildTimer.restart();
        TimeDependentGraphBST graphBST = TimeDependentGraphBST::FromIntermediate(intermediateData);
        double buildTimeBST = buildTimer.elapsedMilliseconds();

        std::cout << "BST graph created: " << graphBST.numVertices() << " vertices, "
                  << graphBST.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeBST) << std::endl;
        graphBST.printStatistics();

        // --- Load CoreCH ---
        std::cout << "\n=== Loading CoreCH ===" << std::endl;
        CH::CH ch(getParameter("Core CH input file"));
        std::cout << "CoreCH loaded." << std::endl;

        // --- Generate queries ---
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.getGraph(FORWARD).numVertices(), n);

        std::vector<int> resultsClassic;
        std::vector<int> resultsBST;
        resultsClassic.reserve(n);
        resultsBST.reserve(n);

        // --- Run Classic Dijkstra ---
        std::cout << "\n=== Running TD-Dijkstra (Classic - Standard Binary Search) ===" << std::endl;

        using TDDijkstraClassic = TimeDependentDijkstraStatefulClassic<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
        TDDijkstraClassic algorithmClassic(graphClassic, intermediateData.numberOfStops(), &ch);

        Timer classicTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmClassic.run(query.source, query.departureTime, query.target);
            resultsClassic.push_back(algorithmClassic.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  Classic: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(classicTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double classicQueryTime = classicTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics Classic (Standard Binary Search) ---" << std::endl;
        algorithmClassic.getProfiler().printStatistics();

        // --- Run BST Dijkstra ---
        std::cout << "\n=== Running TD-Dijkstra (BST - Balanced Search Trees) ===" << std::endl;

        using TDDijkstraBST = TimeDependentDijkstraStatefulBST<TDD::AggregateProfiler, false, true>;
        TDDijkstraBST algorithmBST(graphBST, intermediateData.numberOfStops(), &ch);

        Timer bstTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmBST.run(query.source, query.departureTime, query.target);
            resultsBST.push_back(algorithmBST.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  BST: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(bstTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double bstQueryTime = bstTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics BST (Balanced Search Trees) ---" << std::endl;
        algorithmBST.getProfiler().printStatistics();
        std::cout << "\nBalanced Search Tree Usage:" << std::endl;
        algorithmBST.printBSTStatistics();

        // --- Compare correctness ---
        std::cout << "\n=== Correctness Comparison ===" << std::endl;
        bool resultsMatch = true;
        size_t mismatchCount = 0;
        int maxDiff = 0;
        double totalDiff = 0;

        for (size_t i = 0; i < n; ++i) {
            if (resultsClassic[i] != resultsBST[i]) {
                int diff = resultsBST[i] - resultsClassic[i];
                if (std::abs(diff) > maxDiff) maxDiff = std::abs(diff);
                totalDiff += std::abs(diff);
                if (mismatchCount < 10) {
                    std::cout << "Mismatch for query " << i
                              << " (src=" << queries[i].source
                              << ", tgt=" << queries[i].target
                              << ", dep=" << queries[i].departureTime << "): "
                              << "Classic=" << resultsClassic[i]
                              << ", BST=" << resultsBST[i]
                              << " (diff=" << diff << "s)" << std::endl;
                }
                resultsMatch = false;
                mismatchCount++;
            }
        }

        if (resultsMatch) {
            std::cout << "✓ SUCCESS: All " << n << " results match perfectly!" << std::endl;
        } else {
            std::cout << "✗ FAILURE: " << mismatchCount << "/" << n << " mismatches ("
                      << (100.0 * mismatchCount / n) << "%)" << std::endl;
            std::cout << "Max difference: " << maxDiff << "s" << std::endl;
            if (mismatchCount > 0) {
                std::cout << "Avg difference (mismatches only): " << (totalDiff / mismatchCount) << "s" << std::endl;
            }
        }

        // --- Performance comparison ---
        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);

        std::cout << "\n[Build Time]" << std::endl;
        std::cout << "  Classic (Binary Search):       " << String::msToString(buildTimeClassic) << std::endl;
        std::cout << "  BST (Balanced Search Trees):   " << String::msToString(buildTimeBST) << std::endl;
        double buildSpeedup = buildTimeClassic / buildTimeBST;
        if (buildSpeedup > 1.0) {
            std::cout << "  → BST is " << buildSpeedup << "x faster to build" << std::endl;
        } else {
            std::cout << "  → BST is " << (1.0 / buildSpeedup) << "x slower to build" << std::endl;
        }

        std::cout << "\n[Query Time]" << std::endl;
        std::cout << "  Classic (Binary Search):       " << String::msToString(classicQueryTime)
                  << " (" << (classicQueryTime / n) << " ms/query)" << std::endl;
        std::cout << "  BST (Balanced Search Trees):   " << String::msToString(bstQueryTime)
                  << " (" << (bstQueryTime / n) << " ms/query)" << std::endl;
        double querySpeedup = classicQueryTime / bstQueryTime;
        if (querySpeedup > 1.0) {
            std::cout << "  → BST is " << querySpeedup << "x faster" << std::endl;
        } else {
            std::cout << "  → BST is " << (1.0 / querySpeedup) << "x slower" << std::endl;
        }

        std::cout << "\n[Graph Size]" << std::endl;
        std::cout << "  Vertices: " << graphClassic.numVertices() << " (both)" << std::endl;
        std::cout << "  Edges: " << graphClassic.numEdges() << " (both)" << std::endl;

        size_t classicTripCount = graphClassic.allDiscreteTrips.size();
        size_t bstTripCount = graphBST.allDiscreteTrips.size();

        std::cout << "\n[Connection Count]" << std::endl;
        std::cout << "  Classic:  " << classicTripCount << " connections" << std::endl;
        std::cout << "  BST:      " << bstTripCount << " connections" << std::endl;
        if (classicTripCount != bstTripCount) {
            double diff = 100.0 * std::abs((double)bstTripCount - classicTripCount) / classicTripCount;
            std::cout << "  → Difference: " << diff << "%" << std::endl;
        } else {
            std::cout << "  → Same number of connections" << std::endl;
        }

        std::cout << "\n[Memory Usage Estimation]" << std::endl;
        size_t classicMemory = classicTripCount * sizeof(DiscreteTrip);
        size_t bstMemory = bstTripCount * sizeof(DiscreteTrip);

        std::cout << "  Classic core data: ~" << (classicMemory / 1024.0 / 1024.0) << " MB" << std::endl;
        std::cout << "  BST core data:     ~" << (bstMemory / 1024.0 / 1024.0) << " MB" << std::endl;
        std::cout << "  BST has additional tree structures (std::map per vertex)" << std::endl;

        std::cout << "\n=== Conclusion ===" << std::endl;
        if (resultsMatch) {
            std::cout << "✓ Balanced Search Trees maintains correctness" << std::endl;
            if (querySpeedup > 1.05) {
                std::cout << "✓ Query performance improved by " << querySpeedup << "x" << std::endl;
                std::cout << "  BST optimization is EFFECTIVE for this network" << std::endl;
            } else if (querySpeedup < 0.95) {
                std::cout << "✗ Query performance degraded by " << (1.0/querySpeedup) << "x" << std::endl;
                std::cout << "  BST overhead may outweigh benefits for this network structure" << std::endl;
            } else {
                std::cout << "≈ Query performance similar (within 5%)" << std::endl;
                std::cout << "  BST provides comparable performance to binary search" << std::endl;
            }

            std::cout << "\nBST vs CST comparison:" << std::endl;
            std::cout << "  • Both store same precomputed data" << std::endl;
            std::cout << "  • BST uses std::map (Red-Black Tree) - O(log n) lookup" << std::endl;
            std::cout << "  • CST uses sorted array - O(log n) binary search" << std::endl;
            std::cout << "  • BST may have better insertion performance (not used here)" << std::endl;
            std::cout << "  • CST typically has better cache locality for queries" << std::endl;
        } else {
            std::cout << "✗ WARNING: Results do not match - BST implementation may have errors" << std::endl;
        }

        std::cout << "\nTrade-offs:" << std::endl;
        std::cout << "  Build Time: BST requires " << (buildTimeBST / buildTimeClassic) << "x preprocessing" << std::endl;
        std::cout << "  Query Time: BST provides " << querySpeedup << "x speedup" << std::endl;
        std::cout << "  Memory: BST stores O(unique_departures * num_edges) per vertex in tree structure" << std::endl;
    }
};

class CompareJTSvsTD : public ParameterizedCommand {

public:
    CompareJTSvsTD(BasicShell& shell) :
        ParameterizedCommand(shell, "compareJTSvsTD",
            "Compares JumpTripSearch on JTSGraph vs TimeDependentDijkstraStateful on TimeDependentGraph.") {
        addParameter("Intermediate input file");
        addParameter("Core CH input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // --- Load Intermediate data ---
        std::cout << "\n=== Loading Intermediate data ===" << std::endl;

        Intermediate::Data intermediateData;

        try {
            intermediateData.deserialize(getParameter("Intermediate input file"));
        } catch (...) {
            std::cout << "ERROR: Could not load intermediate data." << std::endl;
            std::cout << "Please create intermediate data first using the appropriate command." << std::endl;
            return;
        }

        std::cout << "Intermediate data loaded: " << intermediateData.numberOfStops() << " stops, "
                  << intermediateData.numberOfTrips() << " trips" << std::endl;

        // --- Build TimeDependentGraph ---
        std::cout << "\n=== Building TimeDependentGraph ===" << std::endl;
        Timer buildTimerTD;
        TimeDependentGraph graphTD = TimeDependentGraph::FromIntermediate(intermediateData);
        double buildTimeTD = buildTimerTD.elapsedMilliseconds();

        std::cout << "TD graph created: " << graphTD.numVertices() << " vertices, "
                  << graphTD.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeTD) << std::endl;

        // --- Build JTSGraph ---
        std::cout << "\n=== Building JTSGraph ===" << std::endl;
        Timer buildTimerJTS;
        JTSGraph jtsGraph = JTSGraph::FromIntermediate(intermediateData);
        double buildTimeJTS = buildTimerJTS.elapsedMilliseconds();

        std::cout << "JTS graph created: " << jtsGraph.numVertices() << " vertices, "
                  << jtsGraph.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeJTS) << std::endl;
        jtsGraph.printStatistics();

        // --- Load CoreCH ---
        std::cout << "\n=== Loading CoreCH ===" << std::endl;
        CH::CH ch(getParameter("Core CH input file"));
        std::cout << "CoreCH loaded." << std::endl;

        // --- Generate queries ---
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(
            ch.getGraph(FORWARD).numVertices(), n);

        std::vector<int> resultsTD;
        std::vector<int> resultsJTS;
        resultsTD.reserve(n);
        resultsJTS.reserve(n);

        // --- Run TimeDependentDijkstraStateful on TimeDependentGraph ---
        std::cout << "\n=== Running TimeDependentDijkstraStateful on TimeDependentGraph ===" << std::endl;

        using TDDijkstra = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        TDDijkstra algorithmTD(graphTD, intermediateData.numberOfStops(), &ch);

        long long totalTDSettles = 0;
        long long totalTDRelaxes = 0;

        Timer tdTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmTD.run(query.source, query.departureTime, query.target);
            resultsTD.push_back(algorithmTD.getArrivalTime(query.target));
            totalTDSettles += algorithmTD.getSettleCount();
            totalTDRelaxes += algorithmTD.getRelaxCount();

            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD-Dijkstra: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(tdTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double tdQueryTime = tdTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics TimeDependentDijkstraStateful ---" << std::endl;
        algorithmTD.getProfiler().printStatistics();
        std::cout << "Avg settles/query: " << (totalTDSettles / (double)n) << std::endl;
        std::cout << "Avg relaxes/query: " << (totalTDRelaxes / (double)n) << std::endl;

        // --- Run JumpTripSearch on JTSGraph ---
        std::cout << "\n=== Running JumpTripSearch on JTSGraph ===" << std::endl;

        using JTSType = JumpTripSearch<JTSGraph, TDD::AggregateProfiler, false, true>;
        JTSType algorithmJTS(jtsGraph, intermediateData.numberOfStops(), &ch);

        long long totalJTSSettles = 0;
        long long totalJTSRelaxes = 0;

        Timer jtsTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmJTS.run(query.source, query.departureTime, query.target);
            resultsJTS.push_back(algorithmJTS.getArrivalTime(query.target));
            totalJTSSettles += algorithmJTS.getSettleCount();
            totalJTSRelaxes += algorithmJTS.getRelaxCount();

            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  JumpTripSearch: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(jtsTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double jtsQueryTime = jtsTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics JumpTripSearch ---" << std::endl;
        algorithmJTS.getProfiler().printStatistics();
        std::cout << "Avg settles/query: " << (totalJTSSettles / (double)n) << std::endl;
        std::cout << "Avg relaxes/query: " << (totalJTSRelaxes / (double)n) << std::endl;

        // --- Compare correctness ---
        std::cout << "\n=== Correctness Comparison ===" << std::endl;
        bool resultsMatch = true;
        size_t mismatchCount = 0;
        int maxDiff = 0;
        double totalDiff = 0;
        size_t bothReachable = 0;
        size_t tdOnlyReachable = 0;
        size_t jtsOnlyReachable = 0;
        size_t neitherReachable = 0;

        for (size_t i = 0; i < n; ++i) {
            bool tdReach = (resultsTD[i] != never && resultsTD[i] != intMax);
            bool jtsReach = (resultsJTS[i] != never && resultsJTS[i] != intMax);

            if (tdReach && jtsReach) {
                bothReachable++;
                if (resultsTD[i] != resultsJTS[i]) {
                    int diff = resultsJTS[i] - resultsTD[i];
                    if (std::abs(diff) > maxDiff) maxDiff = std::abs(diff);
                    totalDiff += std::abs(diff);
                    if (mismatchCount < 10) {
                        std::cout << "Mismatch for query " << i
                                  << " (src=" << queries[i].source
                                  << ", tgt=" << queries[i].target
                                  << ", dep=" << queries[i].departureTime << "): "
                                  << "TD=" << resultsTD[i]
                                  << ", JTS=" << resultsJTS[i]
                                  << " (diff=" << diff << "s)" << std::endl;
                    }
                    resultsMatch = false;
                    mismatchCount++;
                }
            } else if (tdReach && !jtsReach) {
                tdOnlyReachable++;
                resultsMatch = false;
                mismatchCount++;
            } else if (!tdReach && jtsReach) {
                jtsOnlyReachable++;
                resultsMatch = false;
                mismatchCount++;
            } else {
                neitherReachable++;
            }
        }

        std::cout << "\nReachability summary:" << std::endl;
        std::cout << "  Both reachable:    " << bothReachable << std::endl;
        std::cout << "  TD only:           " << tdOnlyReachable << std::endl;
        std::cout << "  JTS only:          " << jtsOnlyReachable << std::endl;
        std::cout << "  Neither:           " << neitherReachable << std::endl;

        if (resultsMatch) {
            std::cout << "\n✓ SUCCESS: All " << n << " results match perfectly!" << std::endl;
        } else {
            std::cout << "\n✗ FAILURE: " << mismatchCount << "/" << n << " mismatches ("
                      << (100.0 * mismatchCount / n) << "%)" << std::endl;
            if (maxDiff > 0) {
                std::cout << "Max difference: " << maxDiff << "s" << std::endl;
                std::cout << "Avg difference (mismatches only): " << (totalDiff / mismatchCount) << "s" << std::endl;
            }
        }

        // --- Performance comparison ---
        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);

        std::cout << "\n[Graph Info - TimeDependentGraph]" << std::endl;
        std::cout << "  Build time: " << String::msToString(buildTimeTD) << std::endl;
        std::cout << "  Vertices: " << graphTD.numVertices() << std::endl;
        std::cout << "  Edges: " << graphTD.numEdges() << std::endl;
        std::cout << "  Connections: " << graphTD.allDiscreteTrips.size() << std::endl;

        std::cout << "\n[Graph Info - JTSGraph]" << std::endl;
        std::cout << "  Build time: " << String::msToString(buildTimeJTS) << std::endl;
        std::cout << "  Vertices: " << jtsGraph.numVertices() << std::endl;
        std::cout << "  Edges: " << jtsGraph.numEdges() << std::endl;
        std::cout << "  Connections: " << jtsGraph.allDiscreteTrips.size() << std::endl;

        std::cout << "\n[Query Time]" << std::endl;
        std::cout << "  TD-Dijkstra (TimeDependentGraph): " << String::msToString(tdQueryTime)
                  << " (" << (tdQueryTime / n) << " ms/query)" << std::endl;
        std::cout << "  JumpTripSearch (JTSGraph):        " << String::msToString(jtsQueryTime)
                  << " (" << (jtsQueryTime / n) << " ms/query)" << std::endl;
        double querySpeedup = tdQueryTime / jtsQueryTime;
        if (querySpeedup > 1.0) {
            std::cout << "  → JumpTripSearch is " << querySpeedup << "x faster" << std::endl;
        } else {
            std::cout << "  → JumpTripSearch is " << (1.0 / querySpeedup) << "x slower" << std::endl;
        }

        std::cout << "\n=== Conclusion ===" << std::endl;
        if (resultsMatch) {
            std::cout << "✓ Both algorithms produce identical results" << std::endl;
            if (querySpeedup > 1.05) {
                std::cout << "✓ JumpTripSearch is " << querySpeedup << "x faster" << std::endl;
            } else if (querySpeedup < 0.95) {
                std::cout << "→ JumpTripSearch is " << (1.0/querySpeedup) << "x slower" << std::endl;
            } else {
                std::cout << "≈ Performance is similar (within 5%)" << std::endl;
            }
        } else {
            std::cout << "✗ Results do not match - investigation needed" << std::endl;
        }
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
        raptorData.writeCSV("customnetwork");
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData);

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

class CompareCSTvsClassicVariants : public ParameterizedCommand {

public:
    CompareCSTvsClassicVariants(BasicShell& shell) :
        ParameterizedCommand(shell, "compareCSTvsClassicVariants",
            "Compares TimeDependentDijkstraStatefulCST (Combined Search Trees) vs TimeDependentDijkstraStatefulClassic (standard binary search).") {
        addParameter("Intermediate input file");
        addParameter("Core CH input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // --- Load Intermediate data ---
        std::cout << "\n=== Loading Intermediate data ===" << std::endl;

        Intermediate::Data intermediateData;

        try {
            intermediateData.deserialize(getParameter("Intermediate input file"));
        } catch (...) {
            std::cout << "ERROR: Could not load intermediate data." << std::endl;
            std::cout << "Please create intermediate data first using the appropriate command." << std::endl;
            return;
        }

        std::cout << "Intermediate data loaded: " << intermediateData.numberOfStops() << " stops, "
                  << intermediateData.numberOfTrips() << " trips" << std::endl;

        // --- Build both graph variants ---
        std::cout << "\n=== Building TimeDependentGraphClassic (Standard Binary Search) ===" << std::endl;
        Timer buildTimer;
        TimeDependentGraphClassic graphClassic = TimeDependentGraphClassic::FromIntermediate(intermediateData);
        double buildTimeClassic = buildTimer.elapsedMilliseconds();

        std::cout << "Classic graph created: " << graphClassic.numVertices() << " vertices, "
                  << graphClassic.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeClassic) << std::endl;
        graphClassic.printStatistics();

        std::cout << "\n=== Building TimeDependentGraphCST (with Combined Search Trees) ===" << std::endl;
        buildTimer.restart();
        TimeDependentGraphCST graphCST = TimeDependentGraphCST::FromIntermediate(intermediateData);
        double buildTimeCST = buildTimer.elapsedMilliseconds();

        std::cout << "CST graph created: " << graphCST.numVertices() << " vertices, "
                  << graphCST.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeCST) << std::endl;
        graphCST.printStatistics();

        // --- Load CoreCH ---
        std::cout << "\n=== Loading CoreCH ===" << std::endl;
        CH::CH ch(getParameter("Core CH input file"));
        std::cout << "CoreCH loaded." << std::endl;

        // --- Generate queries ---
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.getGraph(FORWARD).numVertices(), n);

        std::vector<int> resultsClassic;
        std::vector<int> resultsCST;
        resultsClassic.reserve(n);
        resultsCST.reserve(n);

        // --- Run Classic Dijkstra ---
        std::cout << "\n=== Running TD-Dijkstra (Classic - Standard Binary Search) ===" << std::endl;

        using TDDijkstraClassic = TimeDependentDijkstraStatefulClassic<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
        TDDijkstraClassic algorithmClassic(graphClassic, intermediateData.numberOfStops(), &ch);

        Timer classicTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmClassic.run(query.source, query.departureTime, query.target);
            resultsClassic.push_back(algorithmClassic.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  Classic: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(classicTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double classicQueryTime = classicTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics Classic (Standard Binary Search) ---" << std::endl;
        algorithmClassic.getProfiler().printStatistics();

        // --- Run CST Dijkstra ---
        std::cout << "\n=== Running TD-Dijkstra (CST - Combined Search Trees) ===" << std::endl;

        using TDDijkstraCST = TimeDependentDijkstraStatefulCST<TDD::AggregateProfiler, false, true>;
        TDDijkstraCST algorithmCST(graphCST, intermediateData.numberOfStops(), &ch);

        Timer cstTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmCST.run(query.source, query.departureTime, query.target);
            resultsCST.push_back(algorithmCST.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  CST: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(cstTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double cstQueryTime = cstTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics CST (Combined Search Trees) ---" << std::endl;
        algorithmCST.getProfiler().printStatistics();
        std::cout << "\nCombined Search Tree Usage:" << std::endl;
        algorithmCST.printCSTStatistics();

        // --- Compare correctness ---
        std::cout << "\n=== Correctness Comparison ===" << std::endl;
        bool resultsMatch = true;
        size_t mismatchCount = 0;
        int maxDiff = 0;
        double totalDiff = 0;

        for (size_t i = 0; i < n; ++i) {
            if (resultsClassic[i] != resultsCST[i]) {
                int diff = resultsCST[i] - resultsClassic[i];
                if (std::abs(diff) > maxDiff) maxDiff = std::abs(diff);
                totalDiff += std::abs(diff);
                if (mismatchCount < 10) {
                    std::cout << "Mismatch for query " << i
                              << " (src=" << queries[i].source
                              << ", tgt=" << queries[i].target
                              << ", dep=" << queries[i].departureTime << "): "
                              << "Classic=" << resultsClassic[i]
                              << ", CST=" << resultsCST[i]
                              << " (diff=" << diff << "s)" << std::endl;
                }
                resultsMatch = false;
                mismatchCount++;
            }
        }

        if (resultsMatch) {
            std::cout << "✓ SUCCESS: All " << n << " results match perfectly!" << std::endl;
        } else {
            std::cout << "✗ FAILURE: " << mismatchCount << "/" << n << " mismatches ("
                      << (100.0 * mismatchCount / n) << "%)" << std::endl;
            std::cout << "Max difference: " << maxDiff << "s" << std::endl;
            if (mismatchCount > 0) {
                std::cout << "Avg difference (mismatches only): " << (totalDiff / mismatchCount) << "s" << std::endl;
            }
        }

        // --- Performance comparison ---
        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);

        std::cout << "\n[Build Time]" << std::endl;
        std::cout << "  Classic (Binary Search):       " << String::msToString(buildTimeClassic) << std::endl;
        std::cout << "  CST (Combined Search Trees):   " << String::msToString(buildTimeCST) << std::endl;
        double buildSpeedup = buildTimeClassic / buildTimeCST;
        if (buildSpeedup > 1.0) {
            std::cout << "  → CST is " << buildSpeedup << "x faster to build" << std::endl;
        } else {
            std::cout << "  → CST is " << (1.0 / buildSpeedup) << "x slower to build" << std::endl;
        }

        std::cout << "\n[Query Time]" << std::endl;
        std::cout << "  Classic (Binary Search):       " << String::msToString(classicQueryTime)
                  << " (" << (classicQueryTime / n) << " ms/query)" << std::endl;
        std::cout << "  CST (Combined Search Trees):   " << String::msToString(cstQueryTime)
                  << " (" << (cstQueryTime / n) << " ms/query)" << std::endl;
        double querySpeedup = classicQueryTime / cstQueryTime;
        if (querySpeedup > 1.0) {
            std::cout << "  → CST is " << querySpeedup << "x faster" << std::endl;
        } else {
            std::cout << "  → CST is " << (1.0 / querySpeedup) << "x slower" << std::endl;
        }

        std::cout << "\n[Graph Size]" << std::endl;
        std::cout << "  Vertices: " << graphClassic.numVertices() << " (both)" << std::endl;
        std::cout << "  Edges: " << graphClassic.numEdges() << " (both)" << std::endl;

        size_t classicTripCount = graphClassic.allDiscreteTrips.size();
        size_t cstTripCount = graphCST.allDiscreteTrips.size();

        std::cout << "\n[Connection Count]" << std::endl;
        std::cout << "  Classic:  " << classicTripCount << " connections" << std::endl;
        std::cout << "  CST:      " << cstTripCount << " connections" << std::endl;
        if (classicTripCount != cstTripCount) {
            double diff = 100.0 * std::abs((double)cstTripCount - classicTripCount) / classicTripCount;
            std::cout << "  → Difference: " << diff << "%" << std::endl;
        } else {
            std::cout << "  → Same number of connections" << std::endl;
        }

        std::cout << "\n[Memory Usage Estimation]" << std::endl;
        size_t classicMemory = classicTripCount * sizeof(DiscreteTrip);
        size_t cstMemory = cstTripCount * sizeof(DiscreteTrip);

        std::cout << "  Classic core data: ~" << (classicMemory / 1024.0 / 1024.0) << " MB" << std::endl;
        std::cout << "  CST core data:     ~" << (cstMemory / 1024.0 / 1024.0) << " MB" << std::endl;
        std::cout << "  CST has additional lookup tables (schedule + positionInEdge)" << std::endl;

        std::cout << "\n=== Conclusion ===" << std::endl;
        if (resultsMatch) {
            std::cout << "✓ Combined Search Trees maintains correctness" << std::endl;
            if (querySpeedup > 1.05) {
                std::cout << "✓ Query performance improved by " << querySpeedup << "x" << std::endl;
                std::cout << "  CST optimization is EFFECTIVE for this network" << std::endl;
            } else if (querySpeedup < 0.95) {
                std::cout << "✗ Query performance degraded by " << (1.0/querySpeedup) << "x" << std::endl;
                std::cout << "  CST overhead may outweigh benefits for this network structure" << std::endl;
            } else {
                std::cout << "≈ Query performance similar (within 5%)" << std::endl;
                std::cout << "  CST provides comparable performance to binary search" << std::endl;
            }

            std::cout << "\nCST is most effective when:" << std::endl;
            std::cout << "  • Vertices have many outgoing edges sharing departure times" << std::endl;
            std::cout << "  • Multiple edges depart at similar times" << std::endl;
            std::cout << "  • Network has synchronized timetables" << std::endl;
        } else {
            std::cout << "✗ WARNING: Results do not match - CST implementation may have errors" << std::endl;
        }

        std::cout << "\nTrade-offs:" << std::endl;
        std::cout << "  Build Time: CST requires " << (buildTimeCST / buildTimeClassic) << "x preprocessing" << std::endl;
        std::cout << "  Query Time: CST provides " << querySpeedup << "x speedup" << std::endl;
        std::cout << "  Memory: CST stores O(schedule_size * num_edges) per vertex" << std::endl;

        std::cout << "\nCST vs FC comparison:" << std::endl;
        std::cout << "  • CST: O(1) lookup per edge after ONE binary search" << std::endl;
        std::cout << "  • FC:  O(1) cascading through sorted edge list" << std::endl;
        std::cout << "  • CST uses more memory but simpler lookup" << std::endl;
        std::cout << "  • FC uses less memory but requires cascading logic" << std::endl;
    }
};

class CompareFCvsClassicVariants : public ParameterizedCommand {

public:
    CompareFCvsClassicVariants(BasicShell& shell) :
        ParameterizedCommand(shell, "compareFCvsClassicVariants",
            "Compares TimeDependentDijkstraStatefulFC (Fractional Cascading) vs TimeDependentDijkstraStatefulClassic (standard binary search).") {
        addParameter("Intermediate input file");
        addParameter("Core CH input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // --- Load Intermediate data ---
        std::cout << "\n=== Loading Intermediate data ===" << std::endl;

        Intermediate::Data intermediateData;

        try {
            intermediateData.deserialize(getParameter("Intermediate input file"));
        } catch (...) {
            std::cout << "ERROR: Could not load intermediate data." << std::endl;
            std::cout << "Please create intermediate data first using the appropriate command." << std::endl;
            return;
        }

        std::cout << "Intermediate data loaded: " << intermediateData.numberOfStops() << " stops, "
                  << intermediateData.numberOfTrips() << " trips" << std::endl;

        // --- Build both graph variants ---
        std::cout << "\n=== Building TimeDependentGraphClassic (Standard Binary Search) ===" << std::endl;
        Timer buildTimer;
        TimeDependentGraphClassic graphClassic = TimeDependentGraphClassic::FromIntermediate(intermediateData);
        double buildTimeClassic = buildTimer.elapsedMilliseconds();

        std::cout << "Classic graph created: " << graphClassic.numVertices() << " vertices, "
                  << graphClassic.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeClassic) << std::endl;
        graphClassic.printStatistics();

        std::cout << "\n=== Building TimeDependentGraphFC (with Fractional Cascading) ===" << std::endl;
        buildTimer.restart();
        TimeDependentGraphFC graphFC = TimeDependentGraphFC::FromIntermediate(intermediateData);
        double buildTimeFC = buildTimer.elapsedMilliseconds();

        std::cout << "FC graph created: " << graphFC.numVertices() << " vertices, "
                  << graphFC.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeFC) << std::endl;
        graphFC.printStatistics();

        // --- Load CoreCH ---
        std::cout << "\n=== Loading CoreCH ===" << std::endl;
        CH::CH ch(getParameter("Core CH input file"));
        std::cout << "CoreCH loaded." << std::endl;

        // --- Generate queries ---
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.getGraph(FORWARD).numVertices(), n);

        std::vector<int> resultsClassic;
        std::vector<int> resultsFC;
        resultsClassic.reserve(n);
        resultsFC.reserve(n);

        // --- Run Classic Dijkstra ---
        std::cout << "\n=== Running TD-Dijkstra (Classic - Standard Binary Search) ===" << std::endl;

        using TDDijkstraClassic = TimeDependentDijkstraStatefulClassic<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
        TDDijkstraClassic algorithmClassic(graphClassic, intermediateData.numberOfStops(), &ch);

        Timer classicTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmClassic.run(query.source, query.departureTime, query.target);
            resultsClassic.push_back(algorithmClassic.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  Classic: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(classicTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double classicQueryTime = classicTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics Classic (Standard Binary Search) ---" << std::endl;
        algorithmClassic.getProfiler().printStatistics();

        // --- Run FC Dijkstra ---
        std::cout << "\n=== Running TD-Dijkstra (FC - Fractional Cascading) ===" << std::endl;

        using TDDijkstraFC = TimeDependentDijkstraStatefulFC<TDD::AggregateProfiler, false, true>;
        TDDijkstraFC algorithmFC(graphFC, intermediateData.numberOfStops(), &ch);

        Timer fcTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmFC.run(query.source, query.departureTime, query.target);
            resultsFC.push_back(algorithmFC.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  FC: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(fcTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double fcQueryTime = fcTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics FC (Fractional Cascading) ---" << std::endl;
        algorithmFC.getProfiler().printStatistics();
        std::cout << "\nFractional Cascading Usage:" << std::endl;
        algorithmFC.printFCStatistics();

        // --- Compare correctness ---
        std::cout << "\n=== Correctness Comparison ===" << std::endl;
        bool resultsMatch = true;
        size_t mismatchCount = 0;
        int maxDiff = 0;
        double totalDiff = 0;

        for (size_t i = 0; i < n; ++i) {
            if (resultsClassic[i] != resultsFC[i]) {
                int diff = resultsFC[i] - resultsClassic[i];  // FC - Classic
                if (std::abs(diff) > maxDiff) maxDiff = std::abs(diff);
                totalDiff += std::abs(diff);
                if (mismatchCount < 10) {
                    std::cout << "Mismatch for query " << i
                              << " (src=" << queries[i].source
                              << ", tgt=" << queries[i].target
                              << ", dep=" << queries[i].departureTime << "): "
                              << "Classic=" << resultsClassic[i]
                              << ", FC=" << resultsFC[i]
                              << " (diff=" << diff << "s)" << std::endl;
                }
                resultsMatch = false;
                mismatchCount++;
            }
        }

        if (resultsMatch) {
            std::cout << "✓ SUCCESS: All " << n << " results match perfectly!" << std::endl;
        } else {
            std::cout << "✗ FAILURE: " << mismatchCount << "/" << n << " mismatches ("
                      << (100.0 * mismatchCount / n) << "%)" << std::endl;
            std::cout << "Max difference: " << maxDiff << "s" << std::endl;
            if (mismatchCount > 0) {
                std::cout << "Avg difference (mismatches only): " << (totalDiff / mismatchCount) << "s" << std::endl;
            }
        }

        // --- Performance comparison ---
        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);

        std::cout << "\n[Build Time]" << std::endl;
        std::cout << "  Classic (Binary Search):       " << String::msToString(buildTimeClassic) << std::endl;
        std::cout << "  FC (Fractional Cascading):     " << String::msToString(buildTimeFC) << std::endl;
        double buildSpeedup = buildTimeClassic / buildTimeFC;
        if (buildSpeedup > 1.0) {
            std::cout << "  → FC is " << (1.0 / buildSpeedup) << "x slower (FC preprocessing overhead)" << std::endl;
        } else {
            std::cout << "  → FC is " << buildSpeedup << "x faster" << std::endl;
        }

        std::cout << "\n[Query Time]" << std::endl;
        std::cout << "  Classic (Binary Search):       " << String::msToString(classicQueryTime)
                  << " (" << (classicQueryTime / n) << " ms/query)" << std::endl;
        std::cout << "  FC (Fractional Cascading):     " << String::msToString(fcQueryTime)
                  << " (" << (fcQueryTime / n) << " ms/query)" << std::endl;
        double querySpeedup = classicQueryTime / fcQueryTime;
        if (querySpeedup > 1.0) {
            std::cout << "  → FC is " << querySpeedup << "x faster" << std::endl;
        } else {
            std::cout << "  → FC is " << (1.0 / querySpeedup) << "x slower" << std::endl;
        }

        std::cout << "\n[Graph Size]" << std::endl;
        std::cout << "  Vertices: " << graphClassic.numVertices() << " (both)" << std::endl;
        std::cout << "  Edges: " << graphClassic.numEdges() << " (both)" << std::endl;

        // Get trip counts
        size_t classicTripCount = graphClassic.allDiscreteTrips.size();
        size_t fcTripCount = graphFC.allDiscreteTrips.size();

        std::cout << "\n[Connection Count]" << std::endl;
        std::cout << "  Classic:  " << classicTripCount << " connections" << std::endl;
        std::cout << "  FC:       " << fcTripCount << " connections" << std::endl;
        if (classicTripCount != fcTripCount) {
            double diff = 100.0 * std::abs((double)fcTripCount - classicTripCount) / classicTripCount;
            std::cout << "  → Difference: " << diff << "%" << std::endl;
        } else {
            std::cout << "  → Same number of connections" << std::endl;
        }

        // Memory comparison
        std::cout << "\n[Memory Usage Estimation]" << std::endl;
        size_t classicMemory = classicTripCount * sizeof(DiscreteTrip);
        size_t fcMemory = fcTripCount * sizeof(DiscreteTrip);

        // Estimate FC structure overhead (simplified)
        size_t fcOverhead = 0;
        // This is a rough estimate - actual FC data structure size would need detailed calculation

        std::cout << "  Classic core data: ~" << (classicMemory / 1024.0 / 1024.0) << " MB" << std::endl;
        std::cout << "  FC core data:      ~" << (fcMemory / 1024.0 / 1024.0) << " MB" << std::endl;
        std::cout << "  FC has additional cascading structures (pointers, merged arrays)" << std::endl;

        std::cout << "\n=== Conclusion ===" << std::endl;
        if (resultsMatch) {
            std::cout << "✓ Fractional Cascading maintains correctness" << std::endl;
            if (querySpeedup > 1.05) {
                std::cout << "✓ Query performance improved by " << querySpeedup << "x" << std::endl;
                std::cout << "  FC optimization is EFFECTIVE for this network" << std::endl;
            } else if (querySpeedup < 0.95) {
                std::cout << "✗ Query performance degraded by " << (1.0/querySpeedup) << "x" << std::endl;
                std::cout << "  FC overhead may outweigh benefits for this network structure" << std::endl;
            } else {
                std::cout << "≈ Query performance similar (within 5%)" << std::endl;
                std::cout << "  FC provides comparable performance to binary search" << std::endl;
            }

            std::cout << "\nFC is most effective when:" << std::endl;
            std::cout << "  • Vertices have many outgoing edges" << std::endl;
            std::cout << "  • Edges have many departure times" << std::endl;
            std::cout << "  • Network has high branching factor" << std::endl;
        } else {
            std::cout << "✗ WARNING: Results do not match - FC implementation may have errors" << std::endl;
        }

        std::cout << "\nTrade-offs:" << std::endl;
        std::cout << "  Build Time: FC requires " << (buildTimeFC / buildTimeClassic) << "x preprocessing" << std::endl;
        std::cout << "  Query Time: FC provides " << querySpeedup << "x speedup" << std::endl;
        if (buildTimeFC > buildTimeClassic && querySpeedup > 1.0) {
            double breakEvenQueries = (buildTimeFC - buildTimeClassic) / (classicQueryTime / n - fcQueryTime / n);
            std::cout << "  Break-even: ~" << (int)breakEvenQueries << " queries needed to amortize FC preprocessing" << std::endl;
        }
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
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        auto runBenchmark = [&](auto& algorithm) {
            double numJourneys = 0;
            for (const VertexQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
        };

        if (pruningRule == 1) {
            raptorData.sortTransferGraphEdgesByTravelTime();
            RAPTOR::DijkstraRAPTOR_prune<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData, ch);
            runBenchmark(algorithm);
        } else {
            RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData, ch);
            runBenchmark(algorithm);
        }
    }
};

class RunDijkstraRAPTORQueriesNoCH : public ParameterizedCommand {

public:
    RunDijkstraRAPTORQueriesNoCH(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraRAPTORQueriesNoCH", "Runs the given number of random Dijkstra RAPTOR queries (without CH).") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        std::cout << "Creating reverse network..." << std::endl;
        RAPTOR::Data reverseRaptorData = raptorData.reverseNetwork();
        std::cout << "Reverse network created." << std::endl;

        const size_t n = getParameter<size_t>("Number of queries");
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(raptorData.transferGraph.numVertices(), n);

        auto runBenchmark = [&](auto& algorithm) {
            double numJourneys = 0;
            for (const VertexQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
        };

        if (pruningRule == 1) {
            raptorData.sortTransferGraphEdgesByTravelTime();
            RAPTOR::DijkstraRAPTOR_prune<RAPTOR::DijkstraInitialTransfers, RAPTOR::AggregateProfiler, true, false>
                algorithm(raptorData, raptorData.transferGraph, reverseRaptorData.transferGraph);
            runBenchmark(algorithm);
        } else {
            RAPTOR::DijkstraRAPTOR<RAPTOR::DijkstraInitialTransfers, RAPTOR::AggregateProfiler, true, false>
                algorithm(raptorData, raptorData.transferGraph, reverseRaptorData.transferGraph);
            runBenchmark(algorithm);
        }
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

        // --- DIAGNOSTIC START ---
        int stopsWithBuffer = 0;
        int maxBuffer = 0;
        for (const auto& stop : intermediateData.stops) {
            if (stop.minTransferTime > 0) {
                stopsWithBuffer++;
                maxBuffer = std::max(maxBuffer, stop.minTransferTime);
            }
        }
        std::cout << " DIAGNOSTIC: Stops with Buffer > 0: " << stopsWithBuffer << std::endl;
        std::cout << " DIAGNOSTIC: Max Buffer: " << maxBuffer << " seconds" << std::endl;
        // --- DIAGNOSTIC END ---

        TimeDependentGraph graph = TimeDependentGraph::FromIntermediate(intermediateData);
        std::cout << "Time-dependent graph created: " << graph.numVertices() << " vertices, "
                  << graph.numEdges() << " edges" << std::endl;

        // Create the TD-Dijkstra algorithm instance
        using TDDijkstra = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        TDDijkstra algorithm(graph);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(graph.numVertices(), n);

        // Statistics accumulators
        size_t reachableCount = 0;
        int totalArrivalTime = 0;

        std::cout << "\nRunning " << n << " TD-Dijkstra queries..." << std::endl;

        // Run all queries
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);

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

        algorithm.getProfiler().printStatistics();

        if (reachableCount > 0) {
            std::cout << "  Arrival time (reachable): "
                      << String::prettyInt(totalArrivalTime / reachableCount) << std::endl;
        }
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

        using TDDijkstra = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, true>;

        // Pass intermediateData to constructor
        TDDijkstra algorithm(graph);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(graph.numVertices(), n);

        size_t reachableCount = 0;
        int totalArrivalTime = 0;

        std::cout << "\nRunning " << n << " TD-Dijkstra queries..." << std::endl;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            if (algorithm.reachable(query.target)) {
                reachableCount++;
                totalArrivalTime += algorithm.getArrivalTime(query.target);
            }
        }

        std::cout << "\n=== TD-Dijkstra Statistics ===" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Reachable targets: " << reachableCount << " ("
                  << String::prettyDouble(100.0 * reachableCount / n) << "%)" << std::endl;

        algorithm.getProfiler().printStatistics();

        if (reachableCount > 0) {
            std::cout << "  Arrival time (reachable): "
                      << String::prettyInt(totalArrivalTime / (int)reachableCount) << std::endl;
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

        using TDDijkstraStateful = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false>;
        // Pass intermediateData as the second argument
        TDDijkstraStateful algorithm_td(graph, raptorData.numberOfStops());

        Timer tdTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td.run(query.source, query.departureTime, query.target);
            results_td.push_back(algorithm_td.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(tdTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        std::cout << std::endl;

        std::cout << "--- Statistics TD-Dijkstra (stateful) ---" << std::endl;
        algorithm_td.getProfiler().printStatistics();

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

        using TDDijkstraStateful = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        TDDijkstraStateful algorithm_td(graph, raptorData.numberOfStops(), &ch);

        Timer tdTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td.run(query.source, query.departureTime, query.target);
            results_td.push_back(algorithm_td.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(tdTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        std::cout << std::endl;

        std::cout << "--- Statistics TD-Dijkstra (stateful) ---" << std::endl;
        algorithm_td.getProfiler().printStatistics();

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
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, false, false> algorithm_no_pruning(raptorData, ch);
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

class CompareTDGraphVariants : public ParameterizedCommand {

public:
    CompareTDGraphVariants(BasicShell& shell) :
        ParameterizedCommand(shell, "compareTDGraphVariants",
            "Compares TimeDependentDijkstraStateful performance on TimeDependentGraph vs TimeDependentGraphClassic (with dominated edge filtering).") {
        addParameter("Intermediate binary file");
        addParameter("Core CH input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // --- Load Intermediate data ---
        std::cout << "\n=== Loading Intermediate data ===" << std::endl;

        // Try to load intermediate data
        Intermediate::Data intermediateData;

        intermediateData.deserialize(getParameter("Intermediate binary file"));

        std::cout << "Intermediate data loaded: " << intermediateData.numberOfStops() << " stops, "
                  << intermediateData.numberOfTrips() << " trips" << std::endl;

        // --- Build both graph variants ---
        std::cout << "\n=== Building TimeDependentGraph (Standard) ===" << std::endl;
        Timer buildTimer;
        TimeDependentGraph graphStandard = TimeDependentGraph::FromIntermediate(intermediateData);
        double buildTimeStandard = buildTimer.elapsedMilliseconds();

        std::cout << "Standard graph created: " << graphStandard.numVertices() << " vertices, "
                  << graphStandard.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeStandard) << std::endl;

        std::cout << "\n=== Building TimeDependentGraphClassic (with Domination Filtering) ===" << std::endl;
        buildTimer.restart();
        TimeDependentGraphClassic graphClassic = TimeDependentGraphClassic::FromIntermediate(intermediateData);
        double buildTimeClassic = buildTimer.elapsedMilliseconds();

        std::cout << "Classic graph created: " << graphClassic.numVertices() << " vertices, "
                  << graphClassic.numEdges() << " edges" << std::endl;
        std::cout << "Build time: " << String::msToString(buildTimeClassic) << std::endl;
        graphClassic.printStatistics();

        // --- Load CoreCH ---
        std::cout << "\n=== Loading CoreCH ===" << std::endl;
        CH::CH ch(getParameter("Core CH input file"));
        std::cout << "CoreCH loaded." << std::endl;

        // --- Generate queries ---
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.getGraph(FORWARD).numVertices(), n);

        std::vector<int> resultsStandard;
        std::vector<int> resultsClassic;
        resultsStandard.reserve(n);
        resultsClassic.reserve(n);

        // --- Run TD-Dijkstra on Standard Graph ---
        std::cout << "\n=== Running TD-Dijkstra on TimeDependentGraph (Standard) ===" << std::endl;

        using TDDijkstraStandard = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        TDDijkstraStandard algorithmStandard(graphStandard, intermediateData.numberOfStops(), &ch);

        Timer standardTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmStandard.run(query.source, query.departureTime, query.target);
            resultsStandard.push_back(algorithmStandard.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  Standard: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(standardTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double standardQueryTime = standardTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics TimeDependentGraph (Standard) ---" << std::endl;
        algorithmStandard.getProfiler().printStatistics();

        // --- Run TD-Dijkstra on Classic Graph with Filtering ---
        std::cout << "\n=== Running TD-Dijkstra on TimeDependentGraphClassic (Filtered) ===" << std::endl;

        using TDDijkstraClassic = TimeDependentDijkstraStateful<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
        TDDijkstraClassic algorithmClassic(graphClassic, intermediateData.numberOfStops(), &ch);

        Timer classicTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithmClassic.run(query.source, query.departureTime, query.target);
            resultsClassic.push_back(algorithmClassic.getArrivalTime(query.target));
            if ((i + 1) % 10 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  Classic: " << (i + 1) << "/" << n << " queries ("
                          << String::msToString(classicTimer.elapsedMilliseconds()) << ")" << std::flush;
            }
        }
        double classicQueryTime = classicTimer.elapsedMilliseconds();
        std::cout << std::endl;

        std::cout << "--- Statistics TimeDependentGraphClassic (Filtered) ---" << std::endl;
        algorithmClassic.getProfiler().printStatistics();

        // --- Compare correctness ---
        std::cout << "\n=== Correctness Comparison ===" << std::endl;
        bool resultsMatch = true;
        size_t mismatchCount = 0;
        int maxDiff = 0;
        double totalDiff = 0;

        for (size_t i = 0; i < n; ++i) {
            if (resultsStandard[i] != resultsClassic[i]) {
                int diff = resultsClassic[i] - resultsStandard[i];  // Classic - Standard
                if (std::abs(diff) > maxDiff) maxDiff = std::abs(diff);
                totalDiff += std::abs(diff);
                if (mismatchCount < 10) {
                    std::cout << "Mismatch for query " << i
                              << " (src=" << queries[i].source
                              << ", tgt=" << queries[i].target
                              << ", dep=" << queries[i].departureTime << "): "
                              << "Standard=" << resultsStandard[i]
                              << ", Classic=" << resultsClassic[i]
                              << " (diff=" << diff << "s)" << std::endl;
                }
                resultsMatch = false;
                mismatchCount++;
            }
        }

        if (resultsMatch) {
            std::cout << "✓ SUCCESS: All " << n << " results match perfectly!" << std::endl;
        } else {
            std::cout << "✗ FAILURE: " << mismatchCount << "/" << n << " mismatches ("
                      << (100.0 * mismatchCount / n) << "%)" << std::endl;
            std::cout << "Max difference: " << maxDiff << "s" << std::endl;
            if (mismatchCount > 0) {
                std::cout << "Avg difference (mismatches only): " << (totalDiff / mismatchCount) << "s" << std::endl;
            }
        }

        // --- Performance comparison ---
        std::cout << "\n=== Performance Summary ===" << std::endl;
        std::cout << std::fixed << std::setprecision(2);

        std::cout << "\n[Build Time]" << std::endl;
        std::cout << "  Standard:  " << String::msToString(buildTimeStandard) << std::endl;
        std::cout << "  Classic:   " << String::msToString(buildTimeClassic) << std::endl;
        double buildSpeedup = buildTimeStandard / buildTimeClassic;
        if (buildSpeedup > 1.0) {
            std::cout << "  → Classic is " << buildSpeedup << "x slower (extra filtering overhead)" << std::endl;
        } else {
            std::cout << "  → Classic is " << (1.0 / buildSpeedup) << "x faster" << std::endl;
        }

        std::cout << "\n[Query Time]" << std::endl;
        std::cout << "  Standard:  " << String::msToString(standardQueryTime)
                  << " (" << (standardQueryTime / n) << " ms/query)" << std::endl;
        std::cout << "  Classic:   " << String::msToString(classicQueryTime)
                  << " (" << (classicQueryTime / n) << " ms/query)" << std::endl;
        double querySpeedup = standardQueryTime / classicQueryTime;
        if (querySpeedup > 1.0) {
            std::cout << "  → Classic is " << querySpeedup << "x faster" << std::endl;
        } else {
            std::cout << "  → Classic is " << (1.0 / querySpeedup) << "x slower" << std::endl;
        }

        std::cout << "\n[Graph Size]" << std::endl;
        std::cout << "  Vertices: " << graphStandard.numVertices() << " (both)" << std::endl;
        std::cout << "  Edges: " << graphStandard.numEdges() << " (both)" << std::endl;

        // Get trip counts from the allDiscreteTrips vector sizes
        size_t standardTripCount = graphStandard.allDiscreteTrips.size();
        size_t classicTripCount = graphClassic.allDiscreteTrips.size();

        std::cout << "\n[Connection Count]" << std::endl;
        std::cout << "  Standard:  " << standardTripCount << " connections" << std::endl;
        std::cout << "  Classic:   " << classicTripCount << " connections" << std::endl;
        if (standardTripCount > 0) {
            double reduction = 100.0 * (1.0 - (double)classicTripCount / standardTripCount);
            std::cout << "  → Reduction: " << reduction << "%" << std::endl;

            size_t memorySaved = (standardTripCount - classicTripCount) * sizeof(DiscreteTrip);
            std::cout << "  → Memory saved: ~" << (memorySaved / 1024.0 / 1024.0) << " MB" << std::endl;
        }

        std::cout << "\n=== Conclusion ===" << std::endl;
        if (resultsMatch) {
            std::cout << "✓ Dominated edge filtering maintains correctness" << std::endl;
            if (querySpeedup > 1.0) {
                std::cout << "✓ Query performance improved by " << querySpeedup << "x" << std::endl;
            } else if (querySpeedup < 0.95) {
                std::cout << "✗ Query performance degraded by " << (1.0/querySpeedup) << "x" << std::endl;
            } else {
                std::cout << "≈ Query performance similar (within 5%)" << std::endl;
            }
        } else {
            std::cout << "✗ WARNING: Results do not match - filtering may have introduced errors" << std::endl;
        }
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
        // Template Args: <Graph, Profiler, Debug=false, TargetPruning=false>
        using TDDijkstraNoPrune = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, false>;

        // Constructor: graph, numStops (0=auto), chPointer
        TDDijkstraNoPrune algo_no_pruning(graph, 0, chPointer);

        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getArrivalTime(query.target));
        }
        std::cout << "--- Statistics (No Pruning) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // 3. Run WITH Pruning (TARGET_PRUNING = true)
        std::cout << "\n--- Running with Target Pruning ---" << std::endl;
        // Template Args: <Graph, Profiler, Debug=false, TargetPruning=true>
        using TDDijkstraPrune = TimeDependentDijkstraStateful<TimeDependentGraph, TDD::AggregateProfiler, false, true>;

        TDDijkstraPrune algo_pruning(graph, 0, chPointer);

        for (const VertexQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algo_pruning.getArrivalTime(query.target));
        }
        std::cout << "--- Statistics (Pruning) ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

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