#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/CSA/CSA.h"
#include "../../Algorithms/CSA/DijkstraCSA.h"
#include "../../Algorithms/CSA/HLCSA.h"
#include "../../Algorithms/CSA/HLCSA_BucketCH.h"
#include "../../Algorithms/CSA/ULTRACSA.h"
#include "../../Algorithms/RAPTOR/HLRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/MCR.h"
#include "../../Algorithms/RAPTOR/McRAPTOR.h"
#include "../../Algorithms/RAPTOR/Bounded/BoundedMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRAMcRAPTOR.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstraBucketCH.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstraBSTBucketCH.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstraCSTBucketCH.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstraFCBucketCH.h"
#include "../../Algorithms/Dijkstra/TimeDependentDijkstraBucketCH.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstra.h"
#include "../../Algorithms/Dijkstra/TimeDependentDijkstra.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstraFC.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstraCST.h"
#include "../../Algorithms/Dijkstra/JumpTripSearch.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstraBST.h"

#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/RAPTOR/OneHopRAPTOR.h"
#include "../../Algorithms/RAPTOR/OneHopULTRARAPTOR.h"
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
            "Compares MR, TD-Dijkstra variants, JTS, TTN (FC/CST/BST), and ULTRA-CSA.") {
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

        Timer loadTimer;
        double raptorLoadTime = 0.0;
        double csaLoadTime = 0.0;
        double intermediateLoadTime = 0.0;
        double coreCHLoadTime = 0.0;
        double fullCHLoadTime = 0.0;

        double tdGraphBuildTime = 0.0;
        double tdGraphClassicBuildTime = 0.0;
        double tdGraphFCBuildTime = 0.0;
        double tdGraphCSTBuildTime = 0.0;
        double tdGraphBSTBuildTime = 0.0;

        double bucketClassicBuildTime = 0.0;
        double bucketJTSBuildTime = 0.0;
        double bucketFCBuildTime = 0.0;
        double bucketCSTBuildTime = 0.0;
        double bucketBSTBuildTime = 0.0;

        // Load RAPTOR data
        std::cout << "Loading RAPTOR data..." << std::endl;
        loadTimer.restart();
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        raptorLoadTime = loadTimer.elapsedMilliseconds();

        // Load CSA data
        std::cout << "\nLoading CSA data..." << std::endl;
        loadTimer.restart();
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        csaLoadTime = loadTimer.elapsedMilliseconds();

        // Load Intermediate data and build TD graphs
        std::cout << "\nLoading Intermediate data..." << std::endl;
        loadTimer.restart();
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate input file"));
        std::cout << "Intermediate data: " << intermediateData.numberOfStops() << " stops, "
                  << intermediateData.numberOfTrips() << " trips" << std::endl;
        intermediateLoadTime = loadTimer.elapsedMilliseconds();

        // Build all graph variants
        std::cout << "\nBuilding TimeDependentGraph (JTS)..." << std::endl;
        Timer buildTimer;
        TimeDependentGraph tdGraph = TimeDependentGraph::FromIntermediate(intermediateData);
        tdGraphBuildTime = buildTimer.elapsedMilliseconds();
        std::cout << "TD graph created: " << tdGraph.numVertices() << " vertices, "
                  << tdGraph.numEdges() << " edges in " << String::msToString(tdGraphBuildTime) << std::endl;

        std::cout << "\nBuilding TimeDependentGraphClassic (TD-Dijkstra Classic)..." << std::endl;
        buildTimer.restart();
        TimeDependentGraphClassic tdGraphClassic = TimeDependentGraphClassic::FromIntermediate(intermediateData);
        tdGraphClassicBuildTime = buildTimer.elapsedMilliseconds();
        std::cout << "TD Classic graph created: " << tdGraphClassic.numVertices() << " vertices, "
                  << tdGraphClassic.numEdges() << " edges in " << String::msToString(tdGraphClassicBuildTime) << std::endl;

        std::cout << "\nBuilding TimeDependentGraphFC (TTN-FC)..." << std::endl;
        buildTimer.restart();
        TimeDependentGraphFC tdGraphFC = TimeDependentGraphFC::FromIntermediate(intermediateData);
        tdGraphFCBuildTime = buildTimer.elapsedMilliseconds();
        std::cout << "TD FC graph created: " << tdGraphFC.numVertices() << " vertices, "
                  << tdGraphFC.numEdges() << " edges in " << String::msToString(tdGraphFCBuildTime) << std::endl;

        std::cout << "\nBuilding TimeDependentGraphCST (TTN-CST)..." << std::endl;
        buildTimer.restart();
        TimeDependentGraphCST tdGraphCST = TimeDependentGraphCST::FromIntermediate(intermediateData);
        tdGraphCSTBuildTime = buildTimer.elapsedMilliseconds();
        std::cout << "TD CST graph created: " << tdGraphCST.numVertices() << " vertices, "
                  << tdGraphCST.numEdges() << " edges in " << String::msToString(tdGraphCSTBuildTime) << std::endl;

        std::cout << "\nBuilding TimeDependentGraphBST (TTN-BST)..." << std::endl;
        buildTimer.restart();
        TimeDependentGraphBST tdGraphBST = TimeDependentGraphBST::FromIntermediate(intermediateData);
        tdGraphBSTBuildTime = buildTimer.elapsedMilliseconds();
        std::cout << "TD BST graph created: " << tdGraphBST.numVertices() << " vertices, "
                  << tdGraphBST.numEdges() << " edges in " << String::msToString(tdGraphBSTBuildTime) << std::endl;

        // Load Core-CH
        std::cout << "\nLoading Core-CH..." << std::endl;
        loadTimer.restart();
        CH::CH coreCH(getParameter("Core CH input file"));
        std::cout << "Core-CH loaded: " << coreCH.numVertices() << " vertices" << std::endl;
        coreCHLoadTime = loadTimer.elapsedMilliseconds();

        // Load Full CH (for Bucket-CH)
        std::cout << "\nLoading Full CH (for Bucket-CH)..." << std::endl;
        loadTimer.restart();
        CH::CH fullCH(getParameter("Full CH input file"));
        std::cout << "Full CH loaded: " << fullCH.numVertices() << " vertices" << std::endl;
        fullCHLoadTime = loadTimer.elapsedMilliseconds();

        // ==================== GENERATE QUERIES ====================
        const size_t n = getParameter<size_t>("Number of queries");
        const size_t maxVertices = std::min(coreCH.numVertices(), fullCH.numVertices());
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(maxVertices, n);
        std::cout << "\nGenerated " << n << " random queries" << std::endl;

        // Results storage - 12 algorithms total
        std::vector<int> results_mr_corech;
        std::vector<int> results_td_classic_corech;
        std::vector<int> results_td_classic_bucketch;
        std::vector<int> results_jts_corech;
        std::vector<int> results_jts_bucketch;
        std::vector<int> results_fc_corech;
        std::vector<int> results_fc_bucketch;
        std::vector<int> results_cst_corech;
        std::vector<int> results_cst_bucketch;
        std::vector<int> results_bst_corech;
        std::vector<int> results_bst_bucketch;
        std::vector<int> results_ultra_csa;

        results_mr_corech.reserve(n);
        results_td_classic_corech.reserve(n);
        results_td_classic_bucketch.reserve(n);
        results_jts_corech.reserve(n);
        results_jts_bucketch.reserve(n);
        results_fc_corech.reserve(n);
        results_fc_bucketch.reserve(n);
        results_cst_corech.reserve(n);
        results_cst_bucketch.reserve(n);
        results_bst_corech.reserve(n);
        results_bst_bucketch.reserve(n);
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
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  MR (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double mrTime = mrTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(mrTime) << " (" << (mrTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 2: TD-Dijkstra Classic with Core-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  2. TD-Dijkstra Classic with Core-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        using TDClassicCoreCH = TimeDependentDijkstra<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
        TDClassicCoreCH algorithm_td_classic_corech(tdGraphClassic, raptorData.numberOfStops(), &coreCH);

        Timer tdClassicCoreCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td_classic_corech.run(query.source, query.departureTime, query.target);
            results_td_classic_corech.push_back(algorithm_td_classic_corech.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD Classic (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double tdClassicCoreCHTime = tdClassicCoreCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(tdClassicCoreCHTime) << " (" << (tdClassicCoreCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 3: TD-Dijkstra Classic with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  3. TD-Dijkstra Classic with Bucket-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "Building Bucket-CH for TD Classic..." << std::endl;
        Timer bucketClassicBuildTimer;
        using TDClassicBucketCH = TimeDependentDijkstraBucketCH<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
        TDClassicBucketCH algorithm_td_classic_bucketch(tdGraphClassic, raptorData.numberOfStops(), &fullCH);
        bucketClassicBuildTime = bucketClassicBuildTimer.elapsedMilliseconds();
        std::cout << "Bucket-CH preprocessing time: " << String::msToString(bucketClassicBuildTime) << std::endl;

        Timer tdClassicBucketCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_td_classic_bucketch.run(query.source, query.departureTime, query.target);
            results_td_classic_bucketch.push_back(algorithm_td_classic_bucketch.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TD Classic (Bucket-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double tdClassicBucketCHTime = tdClassicBucketCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(tdClassicBucketCHTime) << " (" << (tdClassicBucketCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 4: JTS with Core-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  4. JTS with Core-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        using JTSCoreCH = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        JTSCoreCH algorithm_jts_corech(tdGraph, raptorData.numberOfStops(), &coreCH);

        Timer jtsCoreCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_jts_corech.run(query.source, query.departureTime, query.target);
            results_jts_corech.push_back(algorithm_jts_corech.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  JTS (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double jtsCoreCHTime = jtsCoreCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(jtsCoreCHTime) << " (" << (jtsCoreCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 5: JTS with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  5. JTS with Bucket-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "Building Bucket-CH for JTS..." << std::endl;
        Timer bucketJTSBuildTimer;
        using JTSBucketCH = TransferAwareDijkstraBucketCH<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        JTSBucketCH algorithm_jts_bucketch(tdGraph, raptorData.numberOfStops(), &fullCH);
        bucketJTSBuildTime = bucketJTSBuildTimer.elapsedMilliseconds();
        std::cout << "Bucket-CH preprocessing time: " << String::msToString(bucketJTSBuildTime) << std::endl;

        Timer jtsBucketCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_jts_bucketch.run(query.source, query.departureTime, query.target);
            results_jts_bucketch.push_back(algorithm_jts_bucketch.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  JTS (Bucket-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double jtsBucketCHTime = jtsBucketCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(jtsBucketCHTime) << " (" << (jtsBucketCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 6: TTN-FC with Core-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  6. TTN-FC with Core-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        using FCCoreCH = TransferAwareDijkstraFC<TDD::AggregateProfiler, false, true>;
        FCCoreCH algorithm_fc_corech(tdGraphFC, raptorData.numberOfStops(), &coreCH);

        Timer fcCoreCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_fc_corech.run(query.source, query.departureTime, query.target);
            results_fc_corech.push_back(algorithm_fc_corech.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TTN-FC (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double fcCoreCHTime = fcCoreCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(fcCoreCHTime) << " (" << (fcCoreCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 7: TTN-FC with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  7. TTN-FC with Bucket-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "Building Bucket-CH for TTN-FC..." << std::endl;
        Timer bucketFCBuildTimer;
        using FCBucketCH = TransferAwareDijkstraFCBucketCH<TimeDependentGraphFC, TDD::AggregateProfiler, false, true>;
        FCBucketCH algorithm_fc_bucketch(tdGraphFC, raptorData.numberOfStops(), &fullCH);
        bucketFCBuildTime = bucketFCBuildTimer.elapsedMilliseconds();
        std::cout << "Bucket-CH preprocessing time: " << String::msToString(bucketFCBuildTime) << std::endl;

        Timer fcBucketCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_fc_bucketch.run(query.source, query.departureTime, query.target);
            results_fc_bucketch.push_back(algorithm_fc_bucketch.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TTN-FC (Bucket-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double fcBucketCHTime = fcBucketCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(fcBucketCHTime) << " (" << (fcBucketCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 8: TTN-CST with Core-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  8. TTN-CST with Core-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        using CSTCoreCH = TransferAwareDijkstraCST<TDD::AggregateProfiler, false, true>;
        CSTCoreCH algorithm_cst_corech(tdGraphCST, raptorData.numberOfStops(), &coreCH);

        Timer cstCoreCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_cst_corech.run(query.source, query.departureTime, query.target);
            results_cst_corech.push_back(algorithm_cst_corech.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TTN-CST (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double cstCoreCHTime = cstCoreCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(cstCoreCHTime) << " (" << (cstCoreCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 9: TTN-CST with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  9. TTN-CST with Bucket-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "Building Bucket-CH for TTN-CST..." << std::endl;
        Timer bucketCSTBuildTimer;
        using CSTBucketCH = TransferAwareDijkstraCSTBucketCH<TimeDependentGraphCST, TDD::AggregateProfiler, false, true>;
        CSTBucketCH algorithm_cst_bucketch(tdGraphCST, raptorData.numberOfStops(), &fullCH);
        bucketCSTBuildTime = bucketCSTBuildTimer.elapsedMilliseconds();
        std::cout << "Bucket-CH preprocessing time: " << String::msToString(bucketCSTBuildTime) << std::endl;

        Timer cstBucketCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_cst_bucketch.run(query.source, query.departureTime, query.target);
            results_cst_bucketch.push_back(algorithm_cst_bucketch.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TTN-CST (Bucket-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double cstBucketCHTime = cstBucketCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(cstBucketCHTime) << " (" << (cstBucketCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 10: TTN-BST with Core-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  10. TTN-BST with Core-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        using BSTCoreCH = TransferAwareDijkstraBST<TDD::AggregateProfiler, false, true>;
        BSTCoreCH algorithm_bst_corech(tdGraphBST, raptorData.numberOfStops(), &coreCH);

        Timer bstCoreCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_bst_corech.run(query.source, query.departureTime, query.target);
            results_bst_corech.push_back(algorithm_bst_corech.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TTN-BST (Core-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double bstCoreCHTime = bstCoreCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(bstCoreCHTime) << " (" << (bstCoreCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 11: TTN-BST with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  11. TTN-BST with Bucket-CH" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "Building Bucket-CH for TTN-BST..." << std::endl;
        Timer bucketBSTBuildTimer;
        using BSTBucketCH = TransferAwareDijkstraBSTBucketCH<TimeDependentGraphBST, TDD::AggregateProfiler, false, true>;
        BSTBucketCH algorithm_bst_bucketch(tdGraphBST, raptorData.numberOfStops(), &fullCH);
        bucketBSTBuildTime = bucketBSTBuildTimer.elapsedMilliseconds();
        std::cout << "Bucket-CH preprocessing time: " << String::msToString(bucketBSTBuildTime) << std::endl;

        Timer bstBucketCHTimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_bst_bucketch.run(query.source, query.departureTime, query.target);
            results_bst_bucketch.push_back(algorithm_bst_bucketch.getArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  TTN-BST (Bucket-CH): " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double bstBucketCHTime = bstBucketCHTimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(bstBucketCHTime) << " (" << (bstBucketCHTime / n) << " ms/query)" << std::endl;

        // ==================== ALGORITHM 12: ULTRA-CSA with Bucket-CH ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "  12. ULTRA-CSA with Bucket-CH (Full CH)" << std::endl;
        std::cout << "========================================\n" << std::endl;

        CSA::ULTRACSA<true, 0, CSA::AggregateProfiler> algorithm_ultra_csa(csaData, fullCH);

        Timer ultraCSATimer;
        for (size_t i = 0; i < queries.size(); ++i) {
            const VertexQuery& query = queries[i];
            algorithm_ultra_csa.run(query.source, query.departureTime, query.target);
            results_ultra_csa.push_back(algorithm_ultra_csa.getEarliestArrivalTime(query.target));
            if ((i + 1) % 100 == 0 || i + 1 == queries.size()) {
                std::cout << "\r  ULTRA-CSA: " << (i + 1) << "/" << n << " queries" << std::flush;
            }
        }
        double ultraCSATime = ultraCSATimer.elapsedMilliseconds();
        std::cout << std::endl;
        std::cout << "Total time: " << String::msToString(ultraCSATime) << " (" << (ultraCSATime / n) << " ms/query)" << std::endl;

        // ==================== PREPROCESSING SUMMARY ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "       PREPROCESSING TIMES" << std::endl;
        std::cout << "========================================\n" << std::endl;

        const double totalPreprocessingTime =
            raptorLoadTime + csaLoadTime + intermediateLoadTime +
            tdGraphBuildTime + tdGraphClassicBuildTime + tdGraphFCBuildTime + tdGraphCSTBuildTime + tdGraphBSTBuildTime +
            coreCHLoadTime + fullCHLoadTime +
            bucketClassicBuildTime + bucketJTSBuildTime + bucketFCBuildTime + bucketCSTBuildTime + bucketBSTBuildTime;

        std::cout << "RAPTOR load:                 " << String::msToString(raptorLoadTime) << std::endl;
        std::cout << "CSA load:                    " << String::msToString(csaLoadTime) << std::endl;
        std::cout << "Intermediate load:           " << String::msToString(intermediateLoadTime) << std::endl;
        std::cout << "TD Graph (JTS):              " << String::msToString(tdGraphBuildTime) << std::endl;
        std::cout << "TD Graph (Classic):          " << String::msToString(tdGraphClassicBuildTime) << std::endl;
        std::cout << "TD Graph (FC):               " << String::msToString(tdGraphFCBuildTime) << std::endl;
        std::cout << "TD Graph (CST):              " << String::msToString(tdGraphCSTBuildTime) << std::endl;
        std::cout << "TD Graph (BST):              " << String::msToString(tdGraphBSTBuildTime) << std::endl;
        std::cout << "Core-CH load:                " << String::msToString(coreCHLoadTime) << std::endl;
        std::cout << "Full CH load:                " << String::msToString(fullCHLoadTime) << std::endl;
        std::cout << "Bucket-CH (Classic):         " << String::msToString(bucketClassicBuildTime) << std::endl;
        std::cout << "Bucket-CH (JTS):             " << String::msToString(bucketJTSBuildTime) << std::endl;
        std::cout << "Bucket-CH (FC):              " << String::msToString(bucketFCBuildTime) << std::endl;
        std::cout << "Bucket-CH (CST):             " << String::msToString(bucketCSTBuildTime) << std::endl;
        std::cout << "Bucket-CH (BST):             " << String::msToString(bucketBSTBuildTime) << std::endl;
        std::cout << "Total preprocessing time:    " << String::msToString(totalPreprocessingTime) << std::endl;

        // ==================== CORRECTNESS COMPARISON ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "         CORRECTNESS COMPARISON" << std::endl;
        std::cout << "========================================\n" << std::endl;

        auto compareResults = [&](const std::string& name, const std::vector<int>& results, bool detailed = false) {
            size_t matches = 0;
            size_t mismatches = 0;
            size_t bothUnreachable = 0;
            size_t onlyGroundTruthReachable = 0;
            size_t onlyTestReachable = 0;
            int maxDiff = 0;
            double totalDiff = 0;
            size_t diffCount = 0;
            std::vector<int> differences;

            for (size_t i = 0; i < n; ++i) {
                bool gtReachable = (results_mr_corech[i] != never && results_mr_corech[i] != intMax);
                bool testReachable = (results[i] != never && results[i] != intMax);

                if (!gtReachable && !testReachable) {
                    bothUnreachable++;
                    matches++;
                } else if (gtReachable && !testReachable) {
                    onlyGroundTruthReachable++;
                    mismatches++;
                } else if (!gtReachable && testReachable) {
                    onlyTestReachable++;
                    mismatches++;
                } else {
                    int diff = results[i] - results_mr_corech[i];
                    if (diff == 0) {
                        matches++;
                    } else {
                        mismatches++;
                        diffCount++;
                        differences.push_back(diff);
                        if (std::abs(diff) > std::abs(maxDiff)) maxDiff = diff;
                        totalDiff += std::abs(diff);
                    }
                }
            }

            std::cout << name << " vs MR (Core-CH): ";
            std::cout << matches << "/" << n << " (" << std::fixed << std::setprecision(1)
                      << (100.0 * matches / n) << "%)";

            if (mismatches > 0 && detailed && diffCount > 0) {
                std::sort(differences.begin(), differences.end());
                std::cout << " | Avg diff: " << (totalDiff / diffCount / 60.0) << " min"
                          << ", Median: " << (differences[differences.size() / 2] / 60.0) << " min"
                          << ", Max: " << (maxDiff / 60.0) << " min";
            }

            if (matches == n) {
                std::cout << " ✅";
            } else {
                std::cout << " ❌";
            }
            std::cout << std::endl;

            return matches == n;
        };

        std::cout << "Ground Truth: MR (Core-CH)\n" << std::endl;

        bool td_classic_corech_correct = compareResults("TD-Dijkstra Classic (Core-CH)", results_td_classic_corech, true);
        bool td_classic_bucketch_correct = compareResults("TD-Dijkstra Classic (Bucket-CH)", results_td_classic_bucketch, true);
        bool jts_corech_correct = compareResults("JTS (Core-CH)", results_jts_corech, false);
        bool jts_bucketch_correct = compareResults("JTS (Bucket-CH)", results_jts_bucketch, false);
        bool fc_corech_correct = compareResults("TTN-FC (Core-CH)", results_fc_corech, true);
        bool fc_bucketch_correct = compareResults("TTN-FC (Bucket-CH)", results_fc_bucketch, true);
        bool cst_corech_correct = compareResults("TTN-CST (Core-CH)", results_cst_corech, true);
        bool cst_bucketch_correct = compareResults("TTN-CST (Bucket-CH)", results_cst_bucketch, true);
        bool bst_corech_correct = compareResults("TTN-BST (Core-CH)", results_bst_corech, true);
        bool bst_bucketch_correct = compareResults("TTN-BST (Bucket-CH)", results_bst_bucketch, true);
        bool ultra_csa_correct = compareResults("ULTRA-CSA (Bucket-CH)", results_ultra_csa, false);

        // ==================== PERFORMANCE SUMMARY ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "         PERFORMANCE SUMMARY" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << std::fixed << std::setprecision(2);

        std::cout << "┌─────────────────────────────────┬────────────┬───────────┬─────────┐" << std::endl;
        std::cout << "│ Algorithm                       │ Time [ms]  │ Speedup   │ Correct │" << std::endl;
        std::cout << "├─────────────────────────────────┼────────────┼───────────┼─────────┤" << std::endl;

        auto printRow = [&](const std::string& name, double time, bool correct, bool isBaseline = false) {
            std::cout << "│ " << std::left << std::setw(31) << name << " │ "
                      << std::right << std::setw(10) << (time / n) << " │ ";
            if (isBaseline) {
                std::cout << std::setw(9) << "baseline";
            } else {
                std::cout << std::setw(8) << (mrTime / time) << "x";
            }
            std::cout << " │ " << (correct ? "  ✅   " : "  ❌   ") << " │" << std::endl;
        };

        printRow("MR (Core-CH)", mrTime, true, true);
        printRow("TD-Dijkstra Classic (Core-CH)", tdClassicCoreCHTime, td_classic_corech_correct);
        printRow("TD-Dijkstra Classic (Bucket-CH)", tdClassicBucketCHTime, td_classic_bucketch_correct);
        printRow("JTS (Core-CH)", jtsCoreCHTime, jts_corech_correct);
        printRow("JTS (Bucket-CH)", jtsBucketCHTime, jts_bucketch_correct);
        printRow("TTN-FC (Core-CH)", fcCoreCHTime, fc_corech_correct);
        printRow("TTN-FC (Bucket-CH)", fcBucketCHTime, fc_bucketch_correct);
        printRow("TTN-CST (Core-CH)", cstCoreCHTime, cst_corech_correct);
        printRow("TTN-CST (Bucket-CH)", cstBucketCHTime, cst_bucketch_correct);
        printRow("TTN-BST (Core-CH)", bstCoreCHTime, bst_corech_correct);
        printRow("TTN-BST (Bucket-CH)", bstBucketCHTime, bst_bucketch_correct);
        printRow("ULTRA-CSA (Bucket-CH)", ultraCSATime, ultra_csa_correct);

        std::cout << "└─────────────────────────────────┴────────────┴───────────┴─────────┘" << std::endl;

        // ==================== PREPROCESSING TIMES ====================
        std::cout << "\nPreprocessing Times:" << std::endl;
        std::cout << "  TD Graph (JTS):       " << String::msToString(tdGraphBuildTime) << std::endl;
        std::cout << "  TD Graph (Classic):   " << String::msToString(tdGraphClassicBuildTime) << std::endl;
        std::cout << "  TD Graph (FC):        " << String::msToString(tdGraphFCBuildTime) << std::endl;
        std::cout << "  TD Graph (CST):       " << String::msToString(tdGraphCSTBuildTime) << std::endl;
        std::cout << "  TD Graph (BST):       " << String::msToString(tdGraphBSTBuildTime) << std::endl;
        std::cout << "  Bucket-CH (Classic):  " << String::msToString(bucketClassicBuildTime) << std::endl;
        std::cout << "  Bucket-CH (JTS):      " << String::msToString(bucketJTSBuildTime) << std::endl;
        std::cout << "  Bucket-CH (FC):       " << String::msToString(bucketFCBuildTime) << std::endl;
        std::cout << "  Bucket-CH (CST):      " << String::msToString(bucketCSTBuildTime) << std::endl;
        std::cout << "  Bucket-CH (BST):      " << String::msToString(bucketBSTBuildTime) << std::endl;

        // ==================== TTN COMPARISON ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "         TTN COMPARISON" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "TTN vs TD-Dijkstra Classic (Core-CH):" << std::endl;
        std::cout << "  TTN-FC:  " << (tdClassicCoreCHTime / fcCoreCHTime) << "x" << std::endl;
        std::cout << "  TTN-CST: " << (tdClassicCoreCHTime / cstCoreCHTime) << "x" << std::endl;
        std::cout << "  TTN-BST: " << (tdClassicCoreCHTime / bstCoreCHTime) << "x" << std::endl;

        std::cout << "\nTTN vs TD-Dijkstra Classic (Bucket-CH):" << std::endl;
        std::cout << "  TTN-FC:  " << (tdClassicBucketCHTime / fcBucketCHTime) << "x" << std::endl;
        std::cout << "  TTN-CST: " << (tdClassicBucketCHTime / cstBucketCHTime) << "x" << std::endl;
        std::cout << "  TTN-BST: " << (tdClassicBucketCHTime / bstBucketCHTime) << "x" << std::endl;

        std::cout << "\nTTN variants comparison (Bucket-CH):" << std::endl;
        double minTTN = std::min({fcBucketCHTime, cstBucketCHTime, bstBucketCHTime});
        std::cout << "  Fastest TTN: ";
        if (minTTN == fcBucketCHTime) std::cout << "TTN-FC";
        else if (minTTN == cstBucketCHTime) std::cout << "TTN-CST";
        else std::cout << "TTN-BST";
        std::cout << " at " << (minTTN / n) << " ms/query" << std::endl;

        // ==================== FINAL CONCLUSION ====================
        std::cout << "\n========================================" << std::endl;
        std::cout << "            CONCLUSION" << std::endl;
        std::cout << "========================================\n" << std::endl;

        // Find fastest correct algorithm
        std::vector<std::pair<std::string, double>> correctAlgos;
        correctAlgos.push_back({"MR (Core-CH)", mrTime});
        if (td_classic_corech_correct) correctAlgos.push_back({"TD-Dijkstra Classic (Core-CH)", tdClassicCoreCHTime});
        if (td_classic_bucketch_correct) correctAlgos.push_back({"TD-Dijkstra Classic (Bucket-CH)", tdClassicBucketCHTime});
        if (jts_corech_correct) correctAlgos.push_back({"JTS (Core-CH)", jtsCoreCHTime});
        if (jts_bucketch_correct) correctAlgos.push_back({"JTS (Bucket-CH)", jtsBucketCHTime});
        if (fc_corech_correct) correctAlgos.push_back({"TTN-FC (Core-CH)", fcCoreCHTime});
        if (fc_bucketch_correct) correctAlgos.push_back({"TTN-FC (Bucket-CH)", fcBucketCHTime});
        if (cst_corech_correct) correctAlgos.push_back({"TTN-CST (Core-CH)", cstCoreCHTime});
        if (cst_bucketch_correct) correctAlgos.push_back({"TTN-CST (Bucket-CH)", cstBucketCHTime});
        if (bst_corech_correct) correctAlgos.push_back({"TTN-BST (Core-CH)", bstCoreCHTime});
        if (bst_bucketch_correct) correctAlgos.push_back({"TTN-BST (Bucket-CH)", bstBucketCHTime});
        if (ultra_csa_correct) correctAlgos.push_back({"ULTRA-CSA (Bucket-CH)", ultraCSATime});

        auto fastest = std::min_element(correctAlgos.begin(), correctAlgos.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        std::cout << "Fastest CORRECT algorithm: " << fastest->first
                  << " at " << (fastest->second / n) << " ms/query" << std::endl;

        // Check if TTN provides speedup
        bool ttn_helps = (fc_corech_correct && fcCoreCHTime < tdClassicCoreCHTime) ||
                         (cst_corech_correct && cstCoreCHTime < tdClassicCoreCHTime) ||
                         (bst_corech_correct && bstCoreCHTime < tdClassicCoreCHTime);

        if (ttn_helps) {
            std::cout << "\n✅ TTN provides speedup over TD-Dijkstra Classic in this C++ implementation." << std::endl;
        } else {
            std::cout << "\n⚠️  TTN does NOT provide significant speedup over TD-Dijkstra Classic in C++." << std::endl;
            std::cout << "   This is expected: std::lower_bound on contiguous vectors already has excellent" << std::endl;
            std::cout << "   cache performance, leaving little room for TTN's optimization." << std::endl;
        }
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
            "Compares TransferAwareDijkstraBST (Balanced Search Trees) vs TimeDependentDijkstra (standard binary search).") {
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

        using TDDijkstraClassic = TimeDependentDijkstra<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
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

        using TDDijkstraBST = TransferAwareDijkstraBST<TDD::AggregateProfiler, false, true>;
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
            "Compares JumpTripSearch on JTSGraph vs TransferAwareDijkstra on TimeDependentGraph.") {
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

        // --- Run TransferAwareDijkstra on TimeDependentGraph ---
        std::cout << "\n=== Running TransferAwareDijkstra on TimeDependentGraph ===" << std::endl;

        using TDDijkstra = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
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

        std::cout << "--- Statistics TransferAwareDijkstra ---" << std::endl;
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

class RunOneHopRAPTORQueries : public ParameterizedCommand {

public:
    RunOneHopRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneHopRAPTORQueries", "Runs random one-hop RAPTOR queries (non-transitive transfers) with a query-time transfer radius.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Max transfer travel time (s)", "1800");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const int maxTransferTravelTime = getParameter<int>("Max transfer travel time (s)");
        std::cout << "Transfer radius: " << maxTransferTravelTime << " s ("
                  << String::prettyDouble(maxTransferTravelTime / 60.0) << " min)" << std::endl;

        RAPTOR::OneHopRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData);
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, maxTransferTravelTime);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

class RunOneHopULTRARAPTORQueries : public ParameterizedCommand {

public:
    RunOneHopULTRARAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneHopULTRARAPTORQueries", "Runs one-hop ULTRA-RAPTOR queries (shortcuts for intermediate, full graph for initial/final, no CH) and compares to one-hop RAPTOR on the full graph.") {
        addParameter("Shortcut RAPTOR file");
        addParameter("Full RAPTOR file");
        addParameter("Number of queries");
        addParameter("Max transfer travel time (s)", "1800");
    }

    virtual void execute() noexcept {
        RAPTOR::Data shortcutData = RAPTOR::Data::FromBinary(getParameter("Shortcut RAPTOR file"));
        shortcutData.useImplicitDepartureBufferTimes();
        shortcutData.sortTransferGraphEdgesByTravelTime();
        std::cout << "Shortcut network:" << std::endl;
        shortcutData.printInfo();

        RAPTOR::Data fullData = RAPTOR::Data::FromBinary(getParameter("Full RAPTOR file"));
        fullData.useImplicitDepartureBufferTimes();
        fullData.sortTransferGraphEdgesByTravelTime();
        std::cout << "Full (initial/final) transfer graph:" << std::endl;
        Graph::printInfo(fullData.transferGraph);

        const size_t n = getParameter<size_t>("Number of queries");
        const int maxTransferTravelTime = getParameter<int>("Max transfer travel time (s)");
        std::cout << "Transfer radius: " << maxTransferTravelTime << " s ("
                  << String::prettyDouble(maxTransferTravelTime / 60.0) << " min)" << std::endl;
        const std::vector<StopQuery> queries = generateRandomStopQueries(shortcutData.numberOfStops(), n);

        // Baseline: one-hop RAPTOR on the full transfer graph.
        RAPTOR::OneHopRAPTOR<RAPTOR::AggregateProfiler> baseline(fullData);
        // ULTRA: shortcuts for intermediate transfers, full graph for initial/final.
        RAPTOR::OneHopULTRARAPTOR<RAPTOR::AggregateProfiler> ultra(shortcutData, fullData.transferGraph);

        double baseJourneys = 0;
        double ultraJourneys = 0;
        size_t identical = 0;
        size_t mismatches = 0;
        for (const StopQuery& query : queries) {
            baseline.run(query.source, query.departureTime, query.target, maxTransferTravelTime);
            ultra.run(query.source, query.departureTime, query.target, maxTransferTravelTime);
            const std::vector<RAPTOR::ArrivalLabel> baseArrivals = baseline.getArrivals();
            const std::vector<RAPTOR::ArrivalLabel> ultraArrivals = ultra.getArrivals();
            baseJourneys += baseArrivals.size();
            ultraJourneys += ultraArrivals.size();
            if (baseArrivals == ultraArrivals) {
                identical++;
            } else {
                mismatches++;
                if (mismatches <= 10) {
                    std::cout << "MISMATCH q(" << query.source << "->" << query.target << " @" << query.departureTime << "): "
                              << "full=" << baseArrivals.size() << " ultra=" << ultraArrivals.size() << std::endl;
                }
            }
        }
        std::cout << "=== Correctness over " << n << " queries ===" << std::endl;
        std::cout << "Full one-hop RAPTOR   avg journeys: " << String::prettyDouble(baseJourneys / n) << std::endl;
        std::cout << "One-hop ULTRA-RAPTOR  avg journeys: " << String::prettyDouble(ultraJourneys / n) << std::endl;
        std::cout << "Identical Pareto sets: " << identical << " / " << n << std::endl;
        std::cout << "Mismatches:            " << mismatches << " / " << n << std::endl;

        std::cout << "=== Performance: full one-hop RAPTOR on transitive closure ===" << std::endl;
        baseline.getProfiler().printStatistics();
        std::cout << "=== Performance: one-hop ULTRA-RAPTOR (shortcuts + transitive initial/final) ===" << std::endl;
        ultra.getProfiler().printStatistics();
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
            "Compares TransferAwareDijkstraCST (Combined Search Trees) vs TimeDependentDijkstra (standard binary search).") {
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

        using TDDijkstraClassic = TimeDependentDijkstra<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
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

        using TDDijkstraCST = TransferAwareDijkstraCST<TDD::AggregateProfiler, false, true>;
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
            "Compares TransferAwareDijkstraFC (Fractional Cascading) vs TimeDependentDijkstra (standard binary search).") {
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

        using TDDijkstraClassic = TimeDependentDijkstra<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
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

        using TDDijkstraFC = TransferAwareDijkstraFC<TDD::AggregateProfiler, false, true>;
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
        [[maybe_unused]] size_t fcOverhead = 0;
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

class CheckMRBucketCH : public ParameterizedCommand {
public:
    CheckMRBucketCH(BasicShell& shell) :
        ParameterizedCommand(shell, "checkMRBucketCH",
            "Checks if MR with Bucket-CH initial transfers yields the same results as standard MR (Core-CH).") {
        addParameter("RAPTOR input file");
        addParameter("CoreCH data (for standard MR)");
        addParameter("Regular CH data (for Bucket-CH MR; same one ULTRA uses)");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH coreCH(getParameter("CoreCH data (for standard MR)"));
        CH::CH regularCH(getParameter("Regular CH data (for Bucket-CH MR; same one ULTRA uses)"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(coreCH.numVertices(), n);

        std::vector<int> arrivals_corech;
        std::vector<int> arrivals_bucketch;
        std::vector<size_t> journey_counts_corech;
        std::vector<size_t> journey_counts_bucketch;

        std::cout << "--- Running MR (CoreCHInitialTransfers, the shipped variant) ---" << std::endl;
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false>
            algo_corech(raptorData, coreCH);
        for (const VertexQuery& query : queries) {
            algo_corech.run(query.source, query.departureTime, query.target);
            arrivals_corech.push_back(algo_corech.getEarliestArrivalTime(query.target));
            journey_counts_corech.push_back(algo_corech.getJourneys().size());
        }
        std::cout << "--- Statistics for MR (Core-CH) ---" << std::endl;
        algo_corech.getProfiler().printStatistics();

        std::cout << "\n--- Running MR (BucketCHInitialTransfers, using regular CH like ULTRA does) ---" << std::endl;
        RAPTOR::DijkstraRAPTOR<RAPTOR::BucketCHInitialTransfers, RAPTOR::AggregateProfiler, true, false>
            algo_bucketch(raptorData, regularCH);
        for (const VertexQuery& query : queries) {
            algo_bucketch.run(query.source, query.departureTime, query.target);
            arrivals_bucketch.push_back(algo_bucketch.getEarliestArrivalTime(query.target));
            journey_counts_bucketch.push_back(algo_bucketch.getJourneys().size());
        }
        std::cout << "--- Statistics for MR (Bucket-CH) ---" << std::endl;
        algo_bucketch.getProfiler().printStatistics();

        size_t mismatch_count = 0;
        size_t journey_count_mismatch = 0;
        int max_arrival_diff = 0;
        for (size_t i = 0; i < n; ++i) {
            if (arrivals_corech[i] != arrivals_bucketch[i]) {
                ++mismatch_count;
                const int diff = std::abs(arrivals_corech[i] - arrivals_bucketch[i]);
                if (diff > max_arrival_diff) max_arrival_diff = diff;
                if (mismatch_count <= 5) {
                    std::cout << "Mismatch on query " << i
                              << ": Core-CH arrival = " << arrivals_corech[i]
                              << ", Bucket-CH arrival = " << arrivals_bucketch[i] << std::endl;
                }
            }
            if (journey_counts_corech[i] != journey_counts_bucketch[i]) {
                ++journey_count_mismatch;
            }
        }

        std::cout << "\n--- Comparison Results ---" << std::endl;
        std::cout << "Total queries: " << n << std::endl;
        std::cout << "Arrival-time mismatches: " << mismatch_count << std::endl;
        std::cout << "Journey-count mismatches: " << journey_count_mismatch << std::endl;
        if (mismatch_count == 0 && journey_count_mismatch == 0) {
            std::cout << "MR with Bucket-CH produces identical results to MR with Core-CH." << std::endl;
        } else {
            std::cout << "ERROR: MR with Bucket-CH diverges from MR with Core-CH. Max arrival diff: "
                      << max_arrival_diff << " seconds." << std::endl;
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
        using TDDijkstra = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
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

        using TDDijkstra = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>;

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

        using TDDijkstraStateful = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false>;
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

        using TDDijkstraStateful = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
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

        // --- Run with vertex-level Target Pruning only (baseline) ---
        std::cout << "\n--- Running with vertex-level Target Pruning (DijkstraRAPTOR) ---" << std::endl;
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

class ComparePaperAlgorithms : public ParameterizedCommand {
public:
    ComparePaperAlgorithms(BasicShell& shell) :
        ParameterizedCommand(shell, "comparePaperAlgorithms",
            "Runs MR, MR(Bucket-CH), HL-CSA, HL-RAPTOR, TD-Dijkstra (CoreCH/BucketCH), TAD (CoreCH/BucketCH), ULTRA-CSA, ULTRA-CSA(EP) on the same query set.") {
        addParameter("RAPTOR (Contracted) input file");
        addParameter("CSA (Full graph) input file");
        addParameter("CSA (ULTRA Shortcuts) input file");
        addParameter("Intermediate (Full) input file");
        addParameter("Core-CH data (Contracted/ch)");
        addParameter("Regular CH data (CH/ch)");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("RAPTOR (ULTRA Shortcuts) input file");
        addParameter("Trip-Based (ULTRA E2E) input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        std::cout << "=== Loading data ===" << std::endl;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Contracted) input file"));
        raptorData.useImplicitDepartureBufferTimes();
        CSA::Data csaDataFull = CSA::Data::FromBinary(getParameter("CSA (Full graph) input file"));
        csaDataFull.sortConnectionsAscending();
        CSA::Data csaDataShortcuts = CSA::Data::FromBinary(getParameter("CSA (ULTRA Shortcuts) input file"));
        csaDataShortcuts.sortConnectionsAscending();
        csaDataShortcuts.sortTransferGraphEdgesByTravelTime();
        RAPTOR::Data raptorDataShortcuts = RAPTOR::Data::FromBinary(getParameter("RAPTOR (ULTRA Shortcuts) input file"));
        raptorDataShortcuts.useImplicitDepartureBufferTimes();
        raptorDataShortcuts.sortTransferGraphEdgesByTravelTime();
        TripBased::Data tripBasedData(getParameter("Trip-Based (ULTRA E2E) input file"));
        Intermediate::Data intermediateData = Intermediate::Data::FromBinary(getParameter("Intermediate (Full) input file"));
        std::cout << "Building TimeDependentGraph variants..." << std::endl;
        TimeDependentGraph tdGraph = TimeDependentGraph::FromIntermediate(intermediateData);
        TimeDependentGraphClassic tdGraphClassic = TimeDependentGraphClassic::FromIntermediate(intermediateData);
        CH::CH coreCH(getParameter("Core-CH data (Contracted/ch)"));
        CH::CH regularCH(getParameter("Regular CH data (CH/ch)"));
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(coreCH.numVertices(), n);

        struct Result {
            std::string name;
            double totalMs = 0.0;
            std::vector<int> arrivals;
        };
        std::vector<Result> all;

        auto bench = [&](const std::string& name, auto runQuery) {
            Result r{name, 0.0, std::vector<int>()};
            r.arrivals.reserve(n);
            Timer t;
            for (const VertexQuery& q : queries) {
                r.arrivals.push_back(runQuery(q));
            }
            r.totalMs = t.elapsedMilliseconds();
            std::cout << "[" << name << "] total " << r.totalMs << " ms, avg "
                      << (r.totalMs / n) << " ms/query" << std::endl;
            all.push_back(std::move(r));
        };

        std::cout << "\n=== Running queries (" << n << ") ===" << std::endl;

        {
            RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::NoProfiler, true, false> algo(raptorData, coreCH);
            bench("MR (Core-CH)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            RAPTOR::DijkstraRAPTOR<RAPTOR::BucketCHInitialTransfers, RAPTOR::NoProfiler, true, false> algo(raptorData, regularCH);
            bench("MR (Bucket-CH)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            CSA::HLCSA<CSA::NoProfiler> algo(csaDataFull, outHubs, inHubs);
            bench("HL-CSA", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            RAPTOR::HLRAPTOR<RAPTOR::NoProfiler> algo(raptorData, outHubs, inHubs);
            bench("HL-RAPTOR", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            using TDClassicCoreCH = TimeDependentDijkstra<TimeDependentGraphClassic, TDD::NoProfiler, false, true>;
            TDClassicCoreCH algo(tdGraphClassic, raptorData.numberOfStops(), &coreCH);
            bench("TD-Dijkstra (Core-CH)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getArrivalTime(q.target);
            });
        }
        {
            using TDClassicBucketCH = TimeDependentDijkstraBucketCH<TimeDependentGraphClassic, TDD::NoProfiler, false, true>;
            TDClassicBucketCH algo(tdGraphClassic, raptorData.numberOfStops(), &regularCH);
            bench("TD-Dijkstra (Bucket-CH)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getArrivalTime(q.target);
            });
        }
        {
            using TADCoreCH = TransferAwareDijkstra<TimeDependentGraph, TDD::NoProfiler, false, true>;
            TADCoreCH algo(tdGraph, raptorData.numberOfStops(), &coreCH);
            bench("TAD (Core-CH)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getArrivalTime(q.target);
            });
        }
        {
            using TADBucketCH = TransferAwareDijkstraBucketCH<TimeDependentGraph, TDD::NoProfiler, false, true>;
            TADBucketCH algo(tdGraph, raptorData.numberOfStops(), &regularCH);
            bench("TAD (Bucket-CH)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getArrivalTime(q.target);
            });
        }
        {
            CSA::ULTRACSA<true, 0, CSA::NoProfiler> algo(csaDataShortcuts, regularCH);
            bench("ULTRA-CSA (no EP)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            CSA::ULTRACSA<true, 1, CSA::NoProfiler> algo(csaDataShortcuts, regularCH);
            bench("ULTRA-CSA (EP)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            RAPTOR::ULTRARAPTOR<RAPTOR::NoProfiler, false> algo(raptorDataShortcuts, regularCH);
            bench("ULTRA-RAPTOR", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            RAPTOR::ULTRARAPTOR_prune<RAPTOR::NoProfiler, false> algo(raptorDataShortcuts, regularCH);
            bench("ULTRA-RAPTOR (EP)", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime(q.target);
            });
        }
        {
            TripBased::Query<TripBased::NoProfiler> algo(tripBasedData, regularCH);
            bench("ULTRA-TB", [&](const VertexQuery& q) {
                algo.run(q.source, q.departureTime, q.target);
                return algo.getEarliestArrivalTime();
            });
        }

        std::cout << "\n=== Correctness (MR Core-CH as reference) ===" << std::endl;
        const auto& ref = all[0].arrivals;
        for (size_t k = 1; k < all.size(); ++k) {
            const auto& cur = all[k].arrivals;
            size_t mismatches = 0;
            int maxDiff = 0;
            size_t firstMismatchIdx = 0;
            for (size_t i = 0; i < n; ++i) {
                if (ref[i] != cur[i]) {
                    if (mismatches == 0) firstMismatchIdx = i;
                    ++mismatches;
                    int d = std::abs(ref[i] - cur[i]);
                    if (d > maxDiff) maxDiff = d;
                }
            }
            std::cout << "  " << all[k].name << ": ";
            if (mismatches == 0) {
                std::cout << "OK (0 mismatches)" << std::endl;
            } else {
                std::cout << mismatches << " mismatches, max diff " << maxDiff
                          << "s, first at query " << firstMismatchIdx
                          << " (ref=" << ref[firstMismatchIdx]
                          << ", cur=" << cur[firstMismatchIdx] << ")" << std::endl;
            }
        }

        std::cout << "\n=== Summary (ms per query, " << n << " queries) ===" << std::endl;
        for (const auto& r : all) {
            std::cout << "  " << r.name << ": " << (r.totalMs / n) << " ms" << std::endl;
        }
    }
};

class CompareOneAlgEP : public ParameterizedCommand {
public:
    CompareOneAlgEP(BasicShell& shell) :
        ParameterizedCommand(shell, "compareOneAlgEP",
            "Run a single algorithm with 3 variants (unsorted, sorted, EP) on same query set, AggregateProfiler.") {
        addParameter("Algorithm", "RAPTOR", {"RAPTOR","McRAPTOR","BM-M","BM-FBM","CSA","ULTRA-RAPTOR","ULTRA-McRAPTOR","UBM-M","UBM-FBM","ULTRA-CSA"});
        addParameter("City label");
        addParameter("Transitive RAPTOR input file");
        addParameter("ULTRA Shortcuts RAPTOR input file");
        addParameter("Mc Shortcuts RAPTOR input file");
        addParameter("Transitive CSA input file");
        addParameter("ULTRA Shortcuts CSA input file");
        addParameter("Regular CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        const std::string algo = getParameter("Algorithm");
        const std::string city = getParameter("City label");
        const size_t n = getParameter<size_t>("Number of queries");
        const double aS = getParameter<double>("Arrival slack");
        const double tS = getParameter<double>("Trip slack");

        auto stats = [](const std::vector<double>& times) {
            double sum = 0; for (auto t : times) sum += t;
            const double mean = sum / times.size();
            double sumSq = 0; for (auto t : times) sumSq += (t - mean) * (t - mean);
            const double stddev = std::sqrt(sumSq / (times.size() - 1));
            return std::make_pair(mean / 1000.0, stddev / 1000.0); // us -> ms
        };
        const std::string saveDir = "/tmp/per_query_times/" + city;
        std::system(("mkdir -p '" + saveDir + "'").c_str());

        auto saveTimes = [&saveDir](const std::string& label, const std::vector<double>& times) {
            std::string safeLabel = label;
            std::replace(safeLabel.begin(), safeLabel.end(), '/', '_');
            std::replace(safeLabel.begin(), safeLabel.end(), ' ', '_');
            std::replace(safeLabel.begin(), safeLabel.end(), '(', '_');
            std::replace(safeLabel.begin(), safeLabel.end(), ')', '_');
            const std::string fname = saveDir + "/" + safeLabel + ".txt";
            std::ofstream f(fname);
            for (double t : times) f << t << "\n";
        };

        auto report3 = [&stats, &saveTimes](const std::string& name, const std::vector<double>& tU, const std::vector<double>& tSort, const std::vector<double>& tE) {
            const auto [meanU, sdU] = stats(tU);
            const auto [meanS, sdS] = stats(tSort);
            const auto [meanE, sdE] = stats(tE);
            std::cout << "[" << name << "]"
                      << "  unsorted_mean=" << meanU << " sd=" << sdU
                      << "  sorted_mean="   << meanS << " sd=" << sdS
                      << "  EP_mean="       << meanE << " sd=" << sdE
                      << " ms/q" << std::endl;
            std::cout.flush();
            saveTimes(name + "__unsorted", tU);
            saveTimes(name + "__sorted",   tSort);
            saveTimes(name + "__EP",       tE);
        };

        auto run3Generic = [n](auto& algo, const auto& queries, auto runFn, std::vector<double>& times) {
            times.clear();
            times.reserve(n);
            for (const auto& q : queries) {
                Timer t;
                runFn(algo, q);
                times.push_back(t.elapsedMicroseconds());
            }
        };

        // --- CSA (transitive, stop queries) ---
        if (algo == "CSA") {
            CSA::Data data = CSA::Data::FromBinary(getParameter("Transitive CSA input file"));
            data.sortConnectionsAscending();
            const std::vector<StopQuery> queries = generateRandomStopQueries(data.numberOfStops(), n);
            using P = CSA::AggregateProfiler;
            std::vector<double> tU, tS_, tE;
            auto runFn = [](auto& a, const StopQuery& q) { a.run(q.source, q.departureTime, q.target); };
            { CSA::CSA<false, P> a(data); run3Generic(a, queries, runFn, tU); }
            data.sortTransferGraphEdgesByTravelTime();
            { CSA::CSA<false, P> a(data); run3Generic(a, queries, runFn, tS_); }
            { CSA::CSA_prune<false, P> a(data); run3Generic(a, queries, runFn, tE); }
            report3("CSA (Transitive)", tU, tS_, tE);
            return;
        }

        // --- Transitive graph algorithms (use stop queries) ---
        if (algo == "RAPTOR" || algo == "McRAPTOR" || algo == "BM-M" || algo == "BM-FBM") {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("Transitive RAPTOR input file"));
            data.useImplicitDepartureBufferTimes();
            const size_t numStops = data.numberOfStops();
            const std::vector<StopQuery> queries = generateRandomStopQueries(numStops, n);
            using P = RAPTOR::AggregateProfiler;

            if (algo == "RAPTOR") {
                std::vector<double> tU, tS_, tE;
                auto runFn = [](auto& a, const StopQuery& q) { a.run(q.source, q.departureTime, q.target); };
                { RAPTOR::RAPTOR<true, P, true, false, false> a(data); run3Generic(a, queries, runFn, tU); }
                data.sortTransferGraphEdgesByTravelTime();
                { RAPTOR::RAPTOR<true, P, true, false, false> a(data); run3Generic(a, queries, runFn, tS_); }
                { RAPTOR::RAPTOR_prune<true, P, true, false, false> a(data); run3Generic(a, queries, runFn, tE); }
                report3("RAPTOR (Transitive)", tU, tS_, tE);
            } else if (algo == "McRAPTOR") {
                std::vector<double> tU, tS_, tE;
                auto runFn = [](auto& a, const StopQuery& q) { a.run(q.source, q.departureTime, q.target); };
                { RAPTOR::McRAPTOR<false, true, P> a(data); run3Generic(a, queries, runFn, tU); }
                data.sortTransferGraphEdgesByTravelTime();
                { RAPTOR::McRAPTOR<false, true, P> a(data); run3Generic(a, queries, runFn, tS_); }
                { RAPTOR::McRAPTOR<true, true, P> a(data); run3Generic(a, queries, runFn, tE); }
                report3("McRAPTOR (Transitive)", tU, tS_, tE);
            } else if (algo == "BM-M" || algo == "BM-FBM") {
                std::vector<double> tU, tS_, tE;
                auto runFn = [aS, tS](auto& a, const StopQuery& q) { a.run(q.source, q.departureTime, q.target, aS, tS); };
                // unsorted baseline: rev built from unsorted data
                { RAPTOR::Data revU = data.reverseNetwork();
                  RAPTOR::BoundedMcRAPTOR<P> a(data, revU); run3Generic(a, queries, runFn, tU); }
                // sort data; build rev from sorted data and also sort rev so EP on backward graph breaks at the right place
                data.sortTransferGraphEdgesByTravelTime();
                RAPTOR::Data rev = data.reverseNetwork();
                rev.sortTransferGraphEdgesByTravelTime();
                { RAPTOR::BoundedMcRAPTOR<P> a(data, rev); run3Generic(a, queries, runFn, tS_); }
                using Fwd  = RAPTOR::ForwardPruningRAPTOR<P>;
                using FwdP = RAPTOR::ForwardPruningRAPTOR_prune<P>;
                if (algo == "BM-M") {
                    RAPTOR::BoundedMcRAPTOR_prune<P, Fwd, RAPTOR::BackwardPruningRAPTOR<P, Fwd>> a(data, rev);
                    run3Generic(a, queries, runFn, tE);
                    report3("BM-RAPTOR (M)", tU, tS_, tE);
                } else {
                    RAPTOR::BoundedMcRAPTOR_prune<P, FwdP, RAPTOR::BackwardPruningRAPTOR_prune<P, FwdP>> a(data, rev);
                    run3Generic(a, queries, runFn, tE);
                    report3("BM-RAPTOR (F+B+M)", tU, tS_, tE);
                }
            }
            return;
        }

        // --- ULTRA family (use vertex queries with CH for some) ---
        CH::CH ch(getParameter("Regular CH data"));
        using P = RAPTOR::AggregateProfiler;

        if (algo == "ULTRA-CSA") {
            CSA::Data data = CSA::Data::FromBinary(getParameter("ULTRA Shortcuts CSA input file"));
            data.sortConnectionsAscending();
            const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
            using CP = CSA::AggregateProfiler;
            std::vector<double> tU, tS_, tE;
            auto runFn = [](auto& a, const VertexQuery& q) { a.run(q.source, q.departureTime, q.target); };
            { CSA::ULTRACSA<true, 0, CP> a(data, ch); run3Generic(a, queries, runFn, tU); }
            data.sortTransferGraphEdgesByTravelTime();
            { CSA::ULTRACSA<true, 0, CP> a(data, ch); run3Generic(a, queries, runFn, tS_); }
            { CSA::ULTRACSA<true, 1, CP> a(data, ch); run3Generic(a, queries, runFn, tE); }
            report3("ULTRA-CSA", tU, tS_, tE);
            return;
        }

        if (algo == "ULTRA-RAPTOR") {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("ULTRA Shortcuts RAPTOR input file"));
            data.useImplicitDepartureBufferTimes();
            const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
            std::vector<double> tU, tS_, tE;
            auto runFn = [](auto& a, const VertexQuery& q) { a.run(q.source, q.departureTime, q.target); };
            { RAPTOR::ULTRARAPTOR<P, false> a(data, ch); run3Generic(a, queries, runFn, tU); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::ULTRARAPTOR<P, false> a(data, ch); run3Generic(a, queries, runFn, tS_); }
            { RAPTOR::ULTRARAPTOR_prune<P, false> a(data, ch); run3Generic(a, queries, runFn, tE); }
            report3("ULTRA-RAPTOR", tU, tS_, tE);
        } else if (algo == "ULTRA-McRAPTOR") {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("Mc Shortcuts RAPTOR input file"));
            data.useImplicitDepartureBufferTimes();
            const size_t numStops = data.numberOfStops();
            const std::vector<StopQuery> queries = generateRandomStopQueries(numStops, n);
            std::vector<double> tU, tS_, tE;
            auto runFn = [](auto& a, const StopQuery& q) { a.run(q.source, q.departureTime, q.target); };
            { RAPTOR::ULTRAMcRAPTOR<P> a(data, ch); run3Generic(a, queries, runFn, tU); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::ULTRAMcRAPTOR<P> a(data, ch); run3Generic(a, queries, runFn, tS_); }
            { RAPTOR::ULTRAMcRAPTOR_prune<P> a(data, ch); run3Generic(a, queries, runFn, tE); }
            report3("ULTRA-McRAPTOR", tU, tS_, tE);
        } else if (algo == "UBM-M" || algo == "UBM-FBM") {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("Mc Shortcuts RAPTOR input file"));
            data.useImplicitDepartureBufferTimes();
            const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
            std::vector<double> tU, tS_, tE;
            auto runFn = [aS, tS](auto& a, const VertexQuery& q) { a.run(q.source, q.departureTime, q.target, aS, tS); };
            // unsorted baseline
            { RAPTOR::Data revU = data.reverseNetwork();
              RAPTOR::UBMRAPTOR<P> a(data, revU, ch); run3Generic(a, queries, runFn, tU); }
            // sort data; build rev from sorted data and also sort rev
            data.sortTransferGraphEdgesByTravelTime();
            RAPTOR::Data rev = data.reverseNetwork();
            rev.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::UBMRAPTOR<P> a(data, rev, ch); run3Generic(a, queries, runFn, tS_); }
            using IT   = RAPTOR::BucketCHInitialTransfers;
            using Fwd  = RAPTOR::ForwardPruningULTRARAPTOR<P, IT>;
            using FwdP = RAPTOR::ForwardPruningULTRARAPTOR_prune<P, IT>;
            if (algo == "UBM-M") {
                RAPTOR::UBMRAPTOR_prune<P, Fwd, RAPTOR::BackwardPruningULTRARAPTOR<P, IT, Fwd>> a(data, rev, ch);
                run3Generic(a, queries, runFn, tE);
                report3("UBM-RAPTOR (M)", tU, tS_, tE);
            } else {
                RAPTOR::UBMRAPTOR_prune<P, FwdP, RAPTOR::BackwardPruningULTRARAPTOR_prune<P, IT, FwdP>> a(data, rev, ch);
                run3Generic(a, queries, runFn, tE);
                report3("UBM-RAPTOR (F+B+M)", tU, tS_, tE);
            }
        }
    }
};

// BM-RAPTOR / UBM-RAPTOR phase-level EP comparison: 5 EP placements + 2 baselines, same query set.
class CompareBMPhasesEP : public ParameterizedCommand {
public:
    CompareBMPhasesEP(BasicShell& shell) :
        ParameterizedCommand(shell, "compareBMPhasesEP",
            "BM-RAPTOR/UBM-RAPTOR per-phase EP: unsorted, sorted, EP-F, EP-B, EP-FB, EP-FMc, EP-BMc.") {
        addParameter("Algorithm", "BM-RAPTOR", {"BM-RAPTOR","UBM-RAPTOR"});
        addParameter("City label");
        addParameter("Transitive RAPTOR input file");
        addParameter("Mc Shortcuts RAPTOR input file");
        addParameter("Regular CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        const std::string algo = getParameter("Algorithm");
        const std::string city = getParameter("City label");
        const size_t n = getParameter<size_t>("Number of queries");
        const double aS = getParameter<double>("Arrival slack");
        const double tS = getParameter<double>("Trip slack");

        auto stats = [](const std::vector<double>& times) {
            double sum = 0; for (auto t : times) sum += t;
            const double mean = sum / times.size();
            double sumSq = 0; for (auto t : times) sumSq += (t - mean) * (t - mean);
            const double stddev = std::sqrt(sumSq / (times.size() - 1));
            return std::make_pair(mean / 1000.0, stddev / 1000.0); // us -> ms
        };

        const std::string saveDir = "/tmp/per_query_times/" + city;
        std::system(("mkdir -p '" + saveDir + "'").c_str());

        auto saveTimes = [&saveDir](const std::string& label, const std::vector<double>& times) {
            std::string fname = saveDir + "/" + label + ".txt";
            std::ofstream f(fname);
            for (double t : times) f << t << "\n";
        };

        auto report = [&stats, &saveTimes, &algo](const std::string& variant, const std::vector<double>& times) {
            const auto [mean, sd] = stats(times);
            std::cout << "[" << algo << "][" << variant << "]"
                      << "  mean=" << mean << " ms/q"
                      << "  sd=" << sd << " ms/q" << std::endl;
            std::cout.flush();
            saveTimes(algo + "__" + variant, times);
        };

        auto runQueries = [n](auto& a, const auto& queries, auto runFn, std::vector<double>& times) {
            times.clear();
            times.reserve(n);
            for (const auto& q : queries) {
                Timer t;
                runFn(a, q);
                times.push_back(t.elapsedMicroseconds());
            }
        };

        using P = RAPTOR::AggregateProfiler;

        if (algo == "BM-RAPTOR") {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("Transitive RAPTOR input file"));
            data.useImplicitDepartureBufferTimes();
            const std::vector<StopQuery> queries = generateRandomStopQueries(data.numberOfStops(), n);
            auto runFn = [aS, tS](auto& a, const StopQuery& q) { a.run(q.source, q.departureTime, q.target, aS, tS); };

            using Fwd  = RAPTOR::ForwardPruningRAPTOR<P>;
            using FwdP = RAPTOR::ForwardPruningRAPTOR_prune<P>;
            using BwdNoEP_Fwd  = RAPTOR::BackwardPruningRAPTOR<P, Fwd>;
            using BwdNoEP_FwdP = RAPTOR::BackwardPruningRAPTOR<P, FwdP>;
            using BwdEP_Fwd  = RAPTOR::BackwardPruningRAPTOR_prune<P, Fwd>;
            using BwdEP_FwdP = RAPTOR::BackwardPruningRAPTOR_prune<P, FwdP>;

            // unsorted baseline
            std::vector<double> tU;
            { RAPTOR::Data d = data; RAPTOR::Data rev = d.reverseNetwork();
              RAPTOR::BoundedMcRAPTOR<P> a(d, rev); runQueries(a, queries, runFn, tU); }
            report("unsorted", tU);

            // sort transfer graph once; reverse and sort rev so EP can break on sorted edges
            data.sortTransferGraphEdgesByTravelTime();
            RAPTOR::Data rev = data.reverseNetwork();
            rev.sortTransferGraphEdgesByTravelTime();

            // sorted baseline
            std::vector<double> tS_;
            { RAPTOR::BoundedMcRAPTOR<P> a(data, rev); runQueries(a, queries, runFn, tS_); }
            report("sorted", tS_);

            // EP-F: FwdP + Bwd + Mc-no-EP
            std::vector<double> tF;
            { RAPTOR::BoundedMcRAPTOR<P, FwdP, BwdNoEP_FwdP> a(data, rev); runQueries(a, queries, runFn, tF); }
            report("ep_F", tF);

            // EP-B: Fwd + BwdP + Mc-no-EP
            std::vector<double> tB;
            { RAPTOR::BoundedMcRAPTOR<P, Fwd, BwdEP_Fwd> a(data, rev); runQueries(a, queries, runFn, tB); }
            report("ep_B", tB);

            // EP-FB: FwdP + BwdP + Mc-no-EP
            std::vector<double> tFB;
            { RAPTOR::BoundedMcRAPTOR<P, FwdP, BwdEP_FwdP> a(data, rev); runQueries(a, queries, runFn, tFB); }
            report("ep_FB", tFB);

            // EP-FMc: FwdP + Bwd + Mc-EP
            std::vector<double> tFMc;
            { RAPTOR::BoundedMcRAPTOR_prune<P, FwdP, BwdNoEP_FwdP> a(data, rev); runQueries(a, queries, runFn, tFMc); }
            report("ep_FMc", tFMc);

            // EP-BMc: Fwd + BwdP + Mc-EP
            std::vector<double> tBMc;
            { RAPTOR::BoundedMcRAPTOR_prune<P, Fwd, BwdEP_Fwd> a(data, rev); runQueries(a, queries, runFn, tBMc); }
            report("ep_BMc", tBMc);
            return;
        }

        // UBM-RAPTOR
        CH::CH ch(getParameter("Regular CH data"));
        RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("Mc Shortcuts RAPTOR input file"));
        data.useImplicitDepartureBufferTimes();
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
        auto runFn = [aS, tS](auto& a, const VertexQuery& q) { a.run(q.source, q.departureTime, q.target, aS, tS); };

        using IT = RAPTOR::BucketCHInitialTransfers;
        using Fwd  = RAPTOR::ForwardPruningULTRARAPTOR<P, IT>;
        using FwdP = RAPTOR::ForwardPruningULTRARAPTOR_prune<P, IT>;
        using BwdNoEP_Fwd  = RAPTOR::BackwardPruningULTRARAPTOR<P, IT, Fwd>;
        using BwdNoEP_FwdP = RAPTOR::BackwardPruningULTRARAPTOR<P, IT, FwdP>;
        using BwdEP_Fwd  = RAPTOR::BackwardPruningULTRARAPTOR_prune<P, IT, Fwd>;
        using BwdEP_FwdP = RAPTOR::BackwardPruningULTRARAPTOR_prune<P, IT, FwdP>;

        std::vector<double> tU;
        { RAPTOR::Data d = data; RAPTOR::Data rev = d.reverseNetwork();
          RAPTOR::UBMRAPTOR<P> a(d, rev, ch); runQueries(a, queries, runFn, tU); }
        report("unsorted", tU);

        data.sortTransferGraphEdgesByTravelTime();
        RAPTOR::Data rev = data.reverseNetwork();
        rev.sortTransferGraphEdgesByTravelTime();

        std::vector<double> tS_;
        { RAPTOR::UBMRAPTOR<P> a(data, rev, ch); runQueries(a, queries, runFn, tS_); }
        report("sorted", tS_);

        std::vector<double> tF;
        { RAPTOR::UBMRAPTOR<P, FwdP, BwdNoEP_FwdP> a(data, rev, ch); runQueries(a, queries, runFn, tF); }
        report("ep_F", tF);

        std::vector<double> tB;
        { RAPTOR::UBMRAPTOR<P, Fwd, BwdEP_Fwd> a(data, rev, ch); runQueries(a, queries, runFn, tB); }
        report("ep_B", tB);

        std::vector<double> tFB;
        { RAPTOR::UBMRAPTOR<P, FwdP, BwdEP_FwdP> a(data, rev, ch); runQueries(a, queries, runFn, tFB); }
        report("ep_FB", tFB);

        std::vector<double> tFMc;
        { RAPTOR::UBMRAPTOR_prune<P, FwdP, BwdNoEP_FwdP> a(data, rev, ch); runQueries(a, queries, runFn, tFMc); }
        report("ep_FMc", tFMc);

        std::vector<double> tBMc;
        { RAPTOR::UBMRAPTOR_prune<P, Fwd, BwdEP_Fwd> a(data, rev, ch); runQueries(a, queries, runFn, tBMc); }
        report("ep_BMc", tBMc);
    }
};

class CompareFullEPTable : public ParameterizedCommand {
public:
    CompareFullEPTable(BasicShell& shell) :
        ParameterizedCommand(shell, "compareFullEPTable",
            "Per algorithm: unsorted baseline, sorted baseline, sorted+EP, same query set, AggregateProfiler throughout.") {
        addParameter("RAPTOR (Transitive) input file");
        addParameter("CSA (Transitive) input file");
        addParameter("CSA (ULTRA Shortcuts) input file");
        addParameter("RAPTOR (ULTRA Shortcuts) input file");
        addParameter("RAPTOR (Mc Shortcuts) input file");
        addParameter("Regular CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        const size_t n = getParameter<size_t>("Number of queries");
        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");
        CH::CH ch(getParameter("Regular CH data"));
        const std::vector<VertexQuery> vertexQueries = generateRandomVertexQueries(ch.numVertices(), n);
        size_t numStops = [&]() {
            RAPTOR::Data tmp = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            return tmp.numberOfStops();
        }();
        const std::vector<StopQuery> stopQueries = generateRandomStopQueries(numStops, n);

        auto report3 = [n](const std::string& name, double tU, double tS, double tE) {
            std::cout << "[" << name << "]"
                      << "  unsorted=" << (tU / n) << " ms/q"
                      << "  sorted=" << (tS / n) << " ms/q"
                      << "  EP=" << (tE / n) << " ms/q" << std::endl;
        };

        // 1) RAPTOR (Transitive)
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU, tS, tE;
            { RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tS = t.elapsedMilliseconds(); }
            { RAPTOR::RAPTOR_prune<true, RAPTOR::AggregateProfiler, true, false, false> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tE = t.elapsedMilliseconds(); }
            report3("RAPTOR (Transitive)", tU, tS, tE);
        }

        // 2) McRAPTOR (Transitive): McRAPTOR<TARGET_PRUNING, TRANSITIVE, PROFILER>
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU, tS, tE;
            { RAPTOR::McRAPTOR<false, true, RAPTOR::AggregateProfiler> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::McRAPTOR<false, true, RAPTOR::AggregateProfiler> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tS = t.elapsedMilliseconds(); }
            { RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tE = t.elapsedMilliseconds(); }
            report3("McRAPTOR (Transitive)", tU, tS, tE);
        }

        // 3) BM-RAPTOR (Transitive): M and F+B+M
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            data.useImplicitDepartureBufferTimes();
            const RAPTOR::Data reverseData = data.reverseNetwork();
            double tU, tS, tE_M, tE_FBM;
            { RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tS = t.elapsedMilliseconds(); }
            using Prof = RAPTOR::AggregateProfiler;
            using Fwd  = RAPTOR::ForwardPruningRAPTOR<Prof>;
            using FwdP = RAPTOR::ForwardPruningRAPTOR_prune<Prof>;
            { RAPTOR::BoundedMcRAPTOR_prune<Prof, Fwd, RAPTOR::BackwardPruningRAPTOR<Prof, Fwd>> algo(data, reverseData); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tE_M = t.elapsedMilliseconds(); }
            { RAPTOR::BoundedMcRAPTOR_prune<Prof, FwdP, RAPTOR::BackwardPruningRAPTOR_prune<Prof, FwdP>> algo(data, reverseData); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tE_FBM = t.elapsedMilliseconds(); }
            report3("BM-RAPTOR (M)",     tU, tS, tE_M);
            report3("BM-RAPTOR (F+B+M)", tU, tS, tE_FBM);
        }

        // 4) CSA (Transitive)
        {
            CSA::Data data = CSA::Data::FromBinary(getParameter("CSA (Transitive) input file"));
            data.sortConnectionsAscending();
            double tU, tS, tE;
            { CSA::CSA<false, CSA::AggregateProfiler> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { CSA::CSA<false, CSA::AggregateProfiler> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tS = t.elapsedMilliseconds(); }
            { CSA::CSA_prune<false, CSA::AggregateProfiler> algo(data); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tE = t.elapsedMilliseconds(); }
            report3("CSA (Transitive)", tU, tS, tE);
        }

        // 5) ULTRA-CSA
        {
            CSA::Data data = CSA::Data::FromBinary(getParameter("CSA (ULTRA Shortcuts) input file"));
            data.sortConnectionsAscending();
            double tU, tS, tE;
            { CSA::ULTRACSA<true, 0, CSA::AggregateProfiler> algo(data, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { CSA::ULTRACSA<true, 0, CSA::AggregateProfiler> algo(data, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
              tS = t.elapsedMilliseconds(); }
            { CSA::ULTRACSA<true, 1, CSA::AggregateProfiler> algo(data, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
              tE = t.elapsedMilliseconds(); }
            report3("ULTRA-CSA", tU, tS, tE);
        }

        // 6) ULTRA-RAPTOR
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (ULTRA Shortcuts) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU, tS, tE;
            { RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algo(data, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algo(data, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
              tS = t.elapsedMilliseconds(); }
            { RAPTOR::ULTRARAPTOR_prune<RAPTOR::AggregateProfiler, false> algo(data, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
              tE = t.elapsedMilliseconds(); }
            report3("ULTRA-RAPTOR", tU, tS, tE);
        }

        // 7) ULTRA-McRAPTOR
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Mc Shortcuts) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU, tS, tE;
            { RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, ch); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, ch); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tS = t.elapsedMilliseconds(); }
            { RAPTOR::ULTRAMcRAPTOR_prune<RAPTOR::AggregateProfiler> algo(data, ch); Timer t;
              for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
              tE = t.elapsedMilliseconds(); }
            report3("ULTRA-McRAPTOR", tU, tS, tE);
        }

        // 8) UBM-RAPTOR: M and F+B+M
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Mc Shortcuts) input file"));
            data.useImplicitDepartureBufferTimes();
            const RAPTOR::Data reverseData = data.reverseNetwork();
            double tU, tS, tE_M, tE_FBM;
            { RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tU = t.elapsedMilliseconds(); }
            data.sortTransferGraphEdgesByTravelTime();
            { RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tS = t.elapsedMilliseconds(); }
            using Prof = RAPTOR::AggregateProfiler;
            using IT   = RAPTOR::BucketCHInitialTransfers;
            using Fwd  = RAPTOR::ForwardPruningULTRARAPTOR<Prof, IT>;
            using FwdP = RAPTOR::ForwardPruningULTRARAPTOR_prune<Prof, IT>;
            { RAPTOR::UBMRAPTOR_prune<Prof, Fwd, RAPTOR::BackwardPruningULTRARAPTOR<Prof, IT, Fwd>> algo(data, reverseData, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tE_M = t.elapsedMilliseconds(); }
            { RAPTOR::UBMRAPTOR_prune<Prof, FwdP, RAPTOR::BackwardPruningULTRARAPTOR_prune<Prof, IT, FwdP>> algo(data, reverseData, ch); Timer t;
              for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, arrivalSlack, tripSlack);
              tE_FBM = t.elapsedMilliseconds(); }
            report3("UBM-RAPTOR (M)",     tU, tS, tE_M);
            report3("UBM-RAPTOR (F+B+M)", tU, tS, tE_FBM);
        }
    }
};

class CompareSortingEffectAll : public ParameterizedCommand {
public:
    CompareSortingEffectAll(BasicShell& shell) :
        ParameterizedCommand(shell, "compareSortingEffectAll",
            "For each baseline algorithm, runs once on UNSORTED transfer edges and once on SORTED transfer edges, on the same query set.") {
        addParameter("RAPTOR (Transitive) input file");
        addParameter("CSA (Transitive) input file");
        addParameter("CSA (ULTRA Shortcuts) input file");
        addParameter("RAPTOR (ULTRA Shortcuts) input file");
        addParameter("RAPTOR (Mc Shortcuts) input file");
        addParameter("Regular CH data");
        addParameter("Number of queries");
        addParameter("Save queries prefix (\"\" to skip)", "");
        addParameter("Run mode", "both", {"both", "missing"});
    }

    virtual void execute() noexcept {
        const size_t n = getParameter<size_t>("Number of queries");
        const std::string prefix = getParameter("Save queries prefix (\"\" to skip)");
        const std::string mode = getParameter("Run mode");
        // For "missing" mode: RAPTOR-family already have UNSORTED from check* commands,
        // so we only need to run SORTED for them. BM-RAPTOR/UBM-RAPTOR already have
        // SORTED from check*Stages commands, so we only need UNSORTED for them.
        const bool runBothForRaptorFamily = (mode == "both");
        const bool runBothForBMFamily = (mode == "both");

        CH::CH ch(getParameter("Regular CH data"));

        const std::vector<VertexQuery> vertexQueries = generateRandomVertexQueries(ch.numVertices(), n);

        // Pre-load one RAPTOR file just to compute the stop count for stop queries
        const size_t numStops = [&]() {
            RAPTOR::Data tmp = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            return tmp.numberOfStops();
        }();
        const std::vector<StopQuery> stopQueries = generateRandomStopQueries(numStops, n);

        if (!prefix.empty()) {
            saveVertexQueries(prefix + "_vertex.tsv", vertexQueries);
            saveStopQueries(prefix + "_stop.tsv", stopQueries);
            std::cout << "Saved queries to " << prefix << "_vertex.tsv and " << prefix << "_stop.tsv" << std::endl;
        }

        auto report = [n](const std::string& name, double tUnsorted, double tSorted) {
            const double pct = 100.0 * (tUnsorted - tSorted) / tUnsorted;
            std::cout << "[" << name << "]"
                      << "  unsorted=" << (tUnsorted / n) << " ms/q"
                      << "  sorted=" << (tSorted / n) << " ms/q"
                      << "  delta=" << pct << "%" << std::endl;
        };

        std::cout << "\n=== Transitive graph algorithms ===" << std::endl;

        // 1. RAPTOR (Transitive) — UNSORTED already from checkRAPTORPruning
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU = -1, tS = -1;
            if (runBothForRaptorFamily) {
                RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algo(data);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tU = t.elapsedMilliseconds();
            }
            data.sortTransferGraphEdgesByTravelTime();
            {
                RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algo(data);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tS = t.elapsedMilliseconds();
            }
            if (tU < 0) std::cout << "[RAPTOR (Transitive)]  sorted=" << (tS / n) << " ms/q  (UNSORTED from prior check command)" << std::endl;
            else report("RAPTOR (Transitive)", tU, tS);
        }

        // 2. McRAPTOR (Transitive) — UNSORTED already from checkMcRAPTORPruning
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU = -1, tS = -1;
            if (runBothForRaptorFamily) {
                RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algo(data);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tU = t.elapsedMilliseconds();
            }
            data.sortTransferGraphEdgesByTravelTime();
            {
                RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algo(data);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tS = t.elapsedMilliseconds();
            }
            if (tU < 0) std::cout << "[McRAPTOR (Transitive)]  sorted=" << (tS / n) << " ms/q  (UNSORTED from prior check command)" << std::endl;
            else report("McRAPTOR (Transitive)", tU, tS);
        }

        // 3. BM-RAPTOR (Transitive), sigma=1.25 — SORTED already from checkBMcRAPTORPruningStages
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            data.useImplicitDepartureBufferTimes();
            const RAPTOR::Data reverseData = data.reverseNetwork();
            double tU = -1, tS = -1;
            {
                RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, 1.25, 1.25);
                tU = t.elapsedMilliseconds();
            }
            if (runBothForBMFamily) {
                data.sortTransferGraphEdgesByTravelTime();
                RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, 1.25, 1.25);
                tS = t.elapsedMilliseconds();
            }
            if (tS < 0) std::cout << "[BM-RAPTOR (sigma=1.25)]  unsorted=" << (tU / n) << " ms/q  (SORTED from prior check*Stages command)" << std::endl;
            else report("BM-RAPTOR (Transitive, sigma=1.25)", tU, tS);
        }

        // 4. CSA (Transitive) — UNSORTED already from checkCSAPruning
        {
            CSA::Data data = CSA::Data::FromBinary(getParameter("CSA (Transitive) input file"));
            data.sortConnectionsAscending();
            double tU = -1, tS = -1;
            if (runBothForRaptorFamily) {
                CSA::CSA<false, CSA::AggregateProfiler> algo(data);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tU = t.elapsedMilliseconds();
            }
            data.sortTransferGraphEdgesByTravelTime();
            {
                CSA::CSA<false, CSA::AggregateProfiler> algo(data);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tS = t.elapsedMilliseconds();
            }
            if (tU < 0) std::cout << "[CSA (Transitive)]  sorted=" << (tS / n) << " ms/q  (UNSORTED from prior check command)" << std::endl;
            else report("CSA (Transitive)", tU, tS);
        }

        std::cout << "\n=== Stop-level ULTRA shortcuts ===" << std::endl;

        // 5. ULTRA-CSA — UNSORTED already from checkULTRACSAPruning
        {
            CSA::Data data = CSA::Data::FromBinary(getParameter("CSA (ULTRA Shortcuts) input file"));
            data.sortConnectionsAscending();
            double tU = -1, tS = -1;
            if (runBothForRaptorFamily) {
                CSA::ULTRACSA<true, 0, CSA::AggregateProfiler> algo(data, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
                tU = t.elapsedMilliseconds();
            }
            data.sortTransferGraphEdgesByTravelTime();
            {
                CSA::ULTRACSA<true, 0, CSA::AggregateProfiler> algo(data, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
                tS = t.elapsedMilliseconds();
            }
            if (tU < 0) std::cout << "[ULTRA-CSA]  sorted=" << (tS / n) << " ms/q  (UNSORTED from prior check command)" << std::endl;
            else report("ULTRA-CSA", tU, tS);
        }

        // 6. ULTRA-RAPTOR — UNSORTED already from checkULTRARAPTORPruning
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (ULTRA Shortcuts) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU = -1, tS = -1;
            if (runBothForRaptorFamily) {
                RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algo(data, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
                tU = t.elapsedMilliseconds();
            }
            data.sortTransferGraphEdgesByTravelTime();
            {
                RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algo(data, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target);
                tS = t.elapsedMilliseconds();
            }
            if (tU < 0) std::cout << "[ULTRA-RAPTOR]  sorted=" << (tS / n) << " ms/q  (UNSORTED from prior check command)" << std::endl;
            else report("ULTRA-RAPTOR", tU, tS);
        }

        std::cout << "\n=== Multi-criteria ULTRA shortcuts ===" << std::endl;

        // 7. UBM-RAPTOR, sigma=1.25 — SORTED already from checkUBMRAPTORPruningStages
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Mc Shortcuts) input file"));
            data.useImplicitDepartureBufferTimes();
            const RAPTOR::Data reverseData = data.reverseNetwork();
            double tU = -1, tS = -1;
            {
                RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, 1.25, 1.25);
                tU = t.elapsedMilliseconds();
            }
            if (runBothForBMFamily) {
                data.sortTransferGraphEdgesByTravelTime();
                RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, 1.25, 1.25);
                tS = t.elapsedMilliseconds();
            }
            if (tS < 0) std::cout << "[UBM-RAPTOR (sigma=1.25)]  unsorted=" << (tU / n) << " ms/q  (SORTED from prior check*Stages command)" << std::endl;
            else report("UBM-RAPTOR (sigma=1.25)", tU, tS);
        }

        // 8. BM-RAPTOR, sigma=1.0 (for F+B+M row baseline) — SORTED already from checkBMcRAPTORPruningStages
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Transitive) input file"));
            data.useImplicitDepartureBufferTimes();
            const RAPTOR::Data reverseData = data.reverseNetwork();
            double tU = -1, tS = -1;
            {
                RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, 1.0, 1.0);
                tU = t.elapsedMilliseconds();
            }
            if (runBothForBMFamily) {
                data.sortTransferGraphEdgesByTravelTime();
                RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target, 1.0, 1.0);
                tS = t.elapsedMilliseconds();
            }
            if (tS < 0) std::cout << "[BM-RAPTOR (sigma=1.0)]  unsorted=" << (tU / n) << " ms/q  (SORTED from prior check*Stages command)" << std::endl;
            else report("BM-RAPTOR (Transitive, sigma=1.0)", tU, tS);
        }

        // 9. UBM-RAPTOR, sigma=1.0 (for F+B+M row baseline) — SORTED already from checkUBMRAPTORPruningStages
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Mc Shortcuts) input file"));
            data.useImplicitDepartureBufferTimes();
            const RAPTOR::Data reverseData = data.reverseNetwork();
            double tU = -1, tS = -1;
            {
                RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, 1.0, 1.0);
                tU = t.elapsedMilliseconds();
            }
            if (runBothForBMFamily) {
                data.sortTransferGraphEdgesByTravelTime();
                RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algo(data, reverseData, ch);
                Timer t;
                for (const auto& q : vertexQueries) algo.run(q.source, q.departureTime, q.target, 1.0, 1.0);
                tS = t.elapsedMilliseconds();
            }
            if (tS < 0) std::cout << "[UBM-RAPTOR (sigma=1.0)]  unsorted=" << (tU / n) << " ms/q  (SORTED from prior check*Stages command)" << std::endl;
            else report("UBM-RAPTOR (sigma=1.0)", tU, tS);
        }

        // 10. ULTRA-McRAPTOR — UNSORTED already from checkULTRAMcRAPTORPruning
        {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Mc Shortcuts) input file"));
            data.useImplicitDepartureBufferTimes();
            double tU = -1, tS = -1;
            if (runBothForRaptorFamily) {
                RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, ch);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tU = t.elapsedMilliseconds();
            }
            data.sortTransferGraphEdgesByTravelTime();
            {
                RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algo(data, ch);
                Timer t;
                for (const auto& q : stopQueries) algo.run(q.source, q.departureTime, q.target);
                tS = t.elapsedMilliseconds();
            }
            if (tU < 0) std::cout << "[ULTRA-McRAPTOR]  sorted=" << (tS / n) << " ms/q  (UNSORTED from prior check command)" << std::endl;
            else report("ULTRA-McRAPTOR", tU, tS);
        }
    }
};

class CompareCSAEP : public ParameterizedCommand {
public:
    CompareCSAEP(BasicShell& shell) :
        ParameterizedCommand(shell, "compareCSAEP",
            "Compares transitive CSA and ULTRA-CSA with and without Early Pruning. Multiple repetitions for noise reduction.") {
        addParameter("Transitive CSA input file");
        addParameter("ULTRA-CSA input file");
        addParameter("CH data (for ULTRA-CSA)");
        addParameter("Number of queries");
        addParameter("Repetitions", "5");
    }

    virtual void execute() noexcept {
        std::cout << "=== Loading data ===" << std::endl;
        CSA::Data csaTrans = CSA::Data::FromBinary(getParameter("Transitive CSA input file"));
        csaTrans.sortConnectionsAscending();
        CSA::Data csaTransSorted = CSA::Data::FromBinary(getParameter("Transitive CSA input file"));
        csaTransSorted.sortConnectionsAscending();
        csaTransSorted.sortTransferGraphEdgesByTravelTime();

        CSA::Data csaULTRA = CSA::Data::FromBinary(getParameter("ULTRA-CSA input file"));
        csaULTRA.sortConnectionsAscending();
        CSA::Data csaULTRASorted = CSA::Data::FromBinary(getParameter("ULTRA-CSA input file"));
        csaULTRASorted.sortConnectionsAscending();
        csaULTRASorted.sortTransferGraphEdgesByTravelTime();
        CH::CH ch(getParameter("CH data (for ULTRA-CSA)"));

        const size_t n = getParameter<size_t>("Number of queries");
        const size_t reps = getParameter<size_t>("Repetitions");
        const std::vector<StopQuery> stopQueries = generateRandomStopQueries(csaTrans.numberOfStops(), n);
        const std::vector<VertexQuery> vertexQueries = generateRandomVertexQueries(ch.numVertices(), n);

        auto repeatBench = [&](const std::string& name, auto runOne) {
            std::vector<double> times;
            times.reserve(reps);
            std::vector<int> firstArrivals;
            for (size_t r = 0; r < reps; ++r) {
                Timer t;
                std::vector<int> arrivals = runOne();
                const double ms = t.elapsedMilliseconds();
                times.push_back(ms);
                if (r == 0) firstArrivals = std::move(arrivals);
                std::cout << "[" << name << "] rep " << (r + 1) << "/" << reps
                          << ": " << ms << " ms total, " << (ms / n) << " ms/query" << std::endl;
            }
            double minT = times[0], maxT = times[0], sumT = 0;
            for (double t : times) { minT = std::min(minT, t); maxT = std::max(maxT, t); sumT += t; }
            const double avgT = sumT / reps;
            std::cout << "[" << name << "] SUMMARY"
                      << " min=" << (minT / n) << " ms/query"
                      << " avg=" << (avgT / n) << " ms/query"
                      << " max=" << (maxT / n) << " ms/query"
                      << std::endl;
            return std::make_pair(minT / n, std::move(firstArrivals));
        };

        std::cout << "\n=== Transitive CSA ===" << std::endl;
        auto [transNoEP, transNoEParr] = repeatBench("Transitive CSA (no EP)", [&]() {
            CSA::CSA<false, CSA::NoProfiler> algo(csaTrans);
            std::vector<int> arrivals;
            arrivals.reserve(n);
            for (const StopQuery& q : stopQueries) {
                algo.run(q.source, q.departureTime, q.target);
                arrivals.push_back(algo.getEarliestArrivalTime(q.target));
            }
            return arrivals;
        });
        auto [transEP, transEParr] = repeatBench("Transitive CSA (EP)", [&]() {
            CSA::CSA_prune<false, CSA::NoProfiler> algo(csaTransSorted);
            std::vector<int> arrivals;
            arrivals.reserve(n);
            for (const StopQuery& q : stopQueries) {
                algo.run(q.source, q.departureTime, q.target);
                arrivals.push_back(algo.getEarliestArrivalTime(q.target));
            }
            return arrivals;
        });

        std::cout << "\n=== ULTRA-CSA ===" << std::endl;
        auto [ultraNoEP, ultraNoEParr] = repeatBench("ULTRA-CSA (no EP)", [&]() {
            CSA::ULTRACSA<true, 0, CSA::NoProfiler> algo(csaULTRA, ch);
            std::vector<int> arrivals;
            arrivals.reserve(n);
            for (const VertexQuery& q : vertexQueries) {
                algo.run(q.source, q.departureTime, q.target);
                arrivals.push_back(algo.getEarliestArrivalTime(q.target));
            }
            return arrivals;
        });
        auto [ultraEP, ultraEParr] = repeatBench("ULTRA-CSA (EP)", [&]() {
            CSA::ULTRACSA<true, 1, CSA::NoProfiler> algo(csaULTRASorted, ch);
            std::vector<int> arrivals;
            arrivals.reserve(n);
            for (const VertexQuery& q : vertexQueries) {
                algo.run(q.source, q.departureTime, q.target);
                arrivals.push_back(algo.getEarliestArrivalTime(q.target));
            }
            return arrivals;
        });

        std::cout << "\n=== Correctness ===" << std::endl;
        size_t mTrans = 0;
        for (size_t i = 0; i < n; ++i) if (transNoEParr[i] != transEParr[i]) ++mTrans;
        std::cout << "Transitive CSA (no EP) vs (EP): " << (mTrans == 0 ? "OK (0 mismatches)" : std::to_string(mTrans) + " mismatches") << std::endl;
        size_t mUltra = 0;
        for (size_t i = 0; i < n; ++i) if (ultraNoEParr[i] != ultraEParr[i]) ++mUltra;
        std::cout << "ULTRA-CSA (no EP) vs (EP): " << (mUltra == 0 ? "OK (0 mismatches)" : std::to_string(mUltra) + " mismatches") << std::endl;

        std::cout << "\n=== Min-of-" << reps << " summary (ms per query) ===" << std::endl;
        std::cout << "  Transitive CSA  (no EP): " << transNoEP << std::endl;
        std::cout << "  Transitive CSA  (EP)   : " << transEP << std::endl;
        std::cout << "  ULTRA-CSA       (no EP): " << ultraNoEP << std::endl;
        std::cout << "  ULTRA-CSA       (EP)   : " << ultraEP << std::endl;
        const double sTrans = 100.0 * (transNoEP - transEP) / transNoEP;
        const double sUltra = 100.0 * (ultraNoEP - ultraEP) / ultraNoEP;
        std::cout << "  Transitive CSA  speedup: " << sTrans << "%" << std::endl;
        std::cout << "  ULTRA-CSA       speedup: " << sUltra << "%" << std::endl;
    }
};

class CheckMCRParetoEquivalence : public ParameterizedCommand {
public:
    CheckMCRParetoEquivalence(BasicShell& shell) :
        ParameterizedCommand(shell, "checkMCRParetoEquivalence",
            "Runs MCR(Core-CH) and MCR(Bucket-CH) on the same query set and checks Pareto-set equality.") {
        addParameter("RAPTOR (Contracted) input file");
        addParameter("Core-CH data (Contracted/ch)");
        addParameter("Regular CH data (CH/ch)");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        std::cout << "=== Loading data ===" << std::endl;
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR (Contracted) input file"));
        raptorData.useImplicitDepartureBufferTimes();
        CH::CH coreCH(getParameter("Core-CH data (Contracted/ch)"));
        CH::CH regularCH(getParameter("Regular CH data (CH/ch)"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(coreCH.numVertices(), n);

        std::cout << "\n=== Running MCR (Core-CH) on " << n << " queries ===" << std::endl;
        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> coreResults(n);
        {
            RAPTOR::MCR<true, RAPTOR::NoProfiler, RAPTOR::CoreCHInitialTransfers> algo(raptorData, coreCH);
            for (size_t i = 0; i < n; ++i) {
                const VertexQuery& q = queries[i];
                algo.run(q.source, q.departureTime, q.target);
                coreResults[i] = algo.getResults();
            }
        }

        std::cout << "=== Running MCR (Bucket-CH) on " << n << " queries ===" << std::endl;
        std::vector<std::vector<RAPTOR::WalkingParetoLabel>> bucketResults(n);
        {
            RAPTOR::BucketCHInitialTransfers bucketIT(regularCH.forward, regularCH.backward, raptorData.numberOfStops(), Weight);
            RAPTOR::MCR<true, RAPTOR::NoProfiler, RAPTOR::BucketCHInitialTransfers> algo(raptorData, std::move(bucketIT));
            for (size_t i = 0; i < n; ++i) {
                const VertexQuery& q = queries[i];
                algo.run(q.source, q.departureTime, q.target);
                bucketResults[i] = algo.getResults();
            }
        }

        std::cout << "\n=== Comparing Pareto sets ===" << std::endl;
        size_t totalQueries = 0;
        size_t sizeMismatches = 0;
        size_t labelMismatches = 0;
        size_t firstMismatchIdx = n;
        size_t totalLabels = 0;
        size_t emptyBoth = 0;
        for (size_t i = 0; i < n; ++i) {
            auto a = coreResults[i];
            auto b = bucketResults[i];
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            totalLabels += a.size();
            if (a.empty() && b.empty()) {
                ++emptyBoth;
                continue;
            }
            ++totalQueries;
            if (a.size() != b.size()) {
                ++sizeMismatches;
                if (firstMismatchIdx == n) firstMismatchIdx = i;
                continue;
            }
            bool diff = false;
            for (size_t k = 0; k < a.size(); ++k) {
                if (!(a[k] == b[k])) {
                    diff = true;
                    break;
                }
            }
            if (diff) {
                ++labelMismatches;
                if (firstMismatchIdx == n) firstMismatchIdx = i;
            }
        }

        std::cout << "Queries with non-empty Pareto set: " << totalQueries << " / " << n << std::endl;
        std::cout << "Queries with both empty: " << emptyBoth << std::endl;
        std::cout << "Total labels (Core-CH): " << totalLabels << std::endl;
        std::cout << "Pareto-set size mismatches: " << sizeMismatches << std::endl;
        std::cout << "Pareto-set label mismatches (same size, different content): " << labelMismatches << std::endl;
        if (sizeMismatches == 0 && labelMismatches == 0) {
            std::cout << "RESULT: OK - MCR(Core-CH) and MCR(Bucket-CH) produce identical Pareto sets on all " << n << " queries." << std::endl;
        } else {
            std::cout << "RESULT: FAIL - first divergence at query " << firstMismatchIdx << std::endl;
            const VertexQuery& q = queries[firstMismatchIdx];
            std::cout << "  Query: src=" << q.source << " tgt=" << q.target << " dep=" << q.departureTime << std::endl;
            std::cout << "  Core-CH Pareto set (" << coreResults[firstMismatchIdx].size() << " labels):" << std::endl;
            for (const auto& l : coreResults[firstMismatchIdx]) {
                std::cout << "    " << l << std::endl;
            }
            std::cout << "  Bucket-CH Pareto set (" << bucketResults[firstMismatchIdx].size() << " labels):" << std::endl;
            for (const auto& l : bucketResults[firstMismatchIdx]) {
                std::cout << "    " << l << std::endl;
            }
        }
    }
};

class ExportTransferGraphText : public ParameterizedCommand {
public:
    ExportTransferGraphText(BasicShell& shell) :
        ParameterizedCommand(shell, "exportTransferGraphText",
            "Exports a TransferGraph to text format (src dst travelTime per line, for hl_trans).") {
        addParameter("Graph input file (binary, e.g. ../Networks/.../csa.binary.graph)");
        addParameter("Output text file");
    }

    virtual void execute() noexcept {
        const std::string in = getParameter("Graph input file (binary, e.g. ../Networks/.../csa.binary.graph)");
        const std::string out = getParameter("Output text file");
        TransferGraph g(in);
        std::ofstream f(out);
        Assert(f.is_open(), "cannot open " << out);
        const size_t numV = g.numVertices();
        size_t numE = 0;
        for (const Vertex u : g.vertices()) {
            for (const Edge e : g.edgesFrom(u)) {
                f << u << " " << g.get(ToVertex, e) << " " << g.get(TravelTime, e) << "\n";
                ++numE;
            }
        }
        std::cout << "Exported " << numV << " vertices, " << numE << " edges to " << out << std::endl;
    }
};

class ImportHubsFromText : public ParameterizedCommand {
public:
    ImportHubsFromText(BasicShell& shell) :
        ParameterizedCommand(shell, "importHubsFromText",
            "Reads hl_trans 'hubs' output and writes two TransferGraph binaries (out-hubs, in-hubs).") {
        addParameter("hl_trans hubs output file");
        addParameter("Number of vertices");
        addParameter("Out-hubs output binary basename");
        addParameter("In-hubs output binary basename");
    }

    virtual void execute() noexcept {
        const std::string in = getParameter("hl_trans hubs output file");
        const size_t numV = getParameter<size_t>("Number of vertices");
        const std::string outOut = getParameter("Out-hubs output binary basename");
        const std::string outIn = getParameter("In-hubs output binary basename");

        EdgeList<WithCoordinates, WithTravelTime> outEdges;
        EdgeList<WithCoordinates, WithTravelTime> inEdges;
        outEdges.addVertices(numV);
        inEdges.addVertices(numV);

        std::ifstream f(in);
        Assert(f.is_open(), "cannot open " << in);
        std::string line;
        size_t numOut = 0;
        size_t numIn = 0;
        size_t skipped = 0;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            char type = line[0];
            std::istringstream ss(line.substr(2));
            long long a, b, len;
            ss >> a >> b >> len;
            if (!ss) { ++skipped; continue; }
            if (type == 'o') {
                // node a → out-hub b with distance len
                outEdges.addEdge(Vertex(a), Vertex(b)).set(TravelTime, int(len));
                ++numOut;
            } else if (type == 'i') {
                // in-hub a → node b with distance len  ⇒  store edge "stop b ← hub a" indexed as edges from b in inHubs (reverse direction)
                inEdges.addEdge(Vertex(b), Vertex(a)).set(TravelTime, int(len));
                ++numIn;
            } else {
                ++skipped;
            }
        }

        TransferGraph outHubGraph;
        TransferGraph inHubGraph;
        Graph::move(std::move(outEdges), outHubGraph);
        Graph::move(std::move(inEdges), inHubGraph);
        outHubGraph.writeBinary(outOut);
        inHubGraph.writeBinary(outIn);

        std::cout << "Out-hubs: " << numOut << " arcs across " << numV << " vertices -> " << outOut << std::endl;
        std::cout << "In-hubs:  " << numIn  << " arcs across " << numV << " vertices -> " << outIn << std::endl;
        if (skipped) std::cout << "Skipped " << skipped << " non-hub lines (likely 'c' transitive closure)." << std::endl;
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
            "Compares TransferAwareDijkstra performance on TimeDependentGraph vs TimeDependentGraphClassic (with dominated edge filtering).") {
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

        using TDDijkstraStandard = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>;
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

        using TDDijkstraClassic = TransferAwareDijkstra<TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
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
        using TDDijkstraNoPrune = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, false>;

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
        using TDDijkstraPrune = TransferAwareDijkstra<TimeDependentGraph, TDD::AggregateProfiler, false, true>;

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