#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/RAPTOR/Bounded/BoundedMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMHydRA.h"
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

        // Run with pruning
        std::cout << "\n--- Running ULTRA-McRAPTOR with Pruning ---" << std::endl;
        raptorData.sortTransferGraphEdgesByTravelTime();
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
