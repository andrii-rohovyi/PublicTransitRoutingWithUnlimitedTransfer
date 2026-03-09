[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# ULTRA: UnLimited TRAnsfers for Efficient Multimodal Journey Planning

ULTRA is a C++ framework for efficient journey planning in multimodal networks that combine public transit and non-schedule-based transfer modes (e.g., walking, cycling, e-scooter). It was developed at [KIT](https://www.kit.edu) in the [group of Prof. Dorothea Wagner](https://i11www.iti.kit.edu/).

This fork extends the original framework with Dijkstra-based algorithms for the unlimited transfer problem (TD-Dijkstra and TAD) and with stop-level delay-tolerant shortcuts (Delay-ULTRA-CSA, Delay-ULTRA-RAPTOR). The publications describing the extensions are currently under review and will be listed below at a later date. 

The original KIT codebase contains code for the following publications:

- _UnLimited TRAnsfers for Multi-Modal Route Planning: An Efficient Solution._
  Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf.
  In: Proceedings of the 27th Annual European Symposium on Algorithms (ESA'19), Leibniz International Proceedings in Informatics, pages 14:1–14:16, 2019.
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2019/11135/pdf/LIPIcs-ESA-2019-14.pdf) [arXiv](https://arxiv.org/abs/1906.04832)

- _Integrating ULTRA and Trip-Based Routing._
  Jonas Sauer, Dorothea Wagner, Tobias Zündorf.
  In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 4:1–4:15, 2020.
  [pdf](http://i11www.iti.kit.edu/extra/publications/swz-iultr-20.pdf)

- _An Efficient Solution for One-to-Many Multi-Modal Journey Planning._
  Jonas Sauer, Dorothea Wagner, Tobias Zündorf.
  In: Proceedings of the 20th Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'20), OpenAccess Series in Informatics, pages 1:1–1:15, 2020.
  [pdf](https://i11www.iti.kit.edu/extra/publications/swz-aesom-20.pdf)

- _Fast Multimodal Journey Planning for Three Criteria._
  Moritz Potthoff, Jonas Sauer.
  In: Proceedings of the 24th Workshop on Algorithm Engineering and Experiments (ALENEX'22), SIAM, pages 145–157, 2022.
  [pdf](https://epubs.siam.org/doi/epdf/10.1137/1.9781611977042.12) [arXiv](https://arxiv.org/abs/2110.12954)

- _Efficient Algorithms for Fully Multimodal Journey Planning._
  Moritz Potthoff, Jonas Sauer.
  In: Proceedings of the 22nd Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS'22), OpenAccess Series in Informatics, pages 14:1–14:15, 2022.
  [pdf](https://drops.dagstuhl.de/opus/volltexte/2022/17118/pdf/OASIcs-ATMOS-2022-14.pdf)

- _ULTRA: Unlimited Transfers for Efficient Multimodal Journey Planning._
  Moritz Baum, Valentin Buchhold, Jonas Sauer, Dorothea Wagner, Tobias Zündorf.
  In: Transportation Science, volume 57(6), pages 1536-1559, 2023.
  [pdf](https://pubsonline.informs.org/doi/epdf/10.1287/trsc.2022.0198)

- _Fast and Delay-Robust Multimodal Journey Planning._
  Dominik Bez, Jonas Sauer.
  In: Proceedings of the 26th Workshop on Algorithm Engineering and Experiments (ALENEX'24), SIAM, pages 105–117, 2024.
  [pdf](https://epubs.siam.org/doi/epdf/10.1137/1.9781611977929.8) [arXiv](https://arxiv.org/abs/2310.20554)

- _Closing the Performance Gap Between Public Transit and Multimodal Journey Planning._
  Jonas Sauer.
  PhD thesis, 2024.
  [pdf](https://publikationen.bibliothek.kit.edu/1000173225/154031997)

## Compiling

To compile all executables in release mode, run

```bash
mkdir -p cmake-build-release
cd cmake-build-release
cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . --target All --config Release
```

Make sure you have OpenMP installed.

This produces the following executables:

- `ULTRA` — preprocessing and query algorithms (CH, ULTRA shortcuts, all query commands)
- `Network` — network data conversion and manipulation
- `ULTRAPHAST` — one-to-all and one-to-many query algorithms
- `DelayExperiments` — delay-robust evaluation (scenarios, updates, coverage, performance)
- `NonFifoCheck` — standalone tool to detect non-FIFO violations in a timetable

## Usage

All executables (except `NonFifoCheck`) provide an interactive shell. Commands are invoked by piping them to the binary:

```bash
echo "commandName arg1 arg2 ..." | ./cmake-build-release/ULTRA
```

### Contraction Hierarchies

- `buildCH` performs a regular CH precomputation. The output is used by the (Mc)ULTRA query algorithms for the Bucket-CH searches.
- `buildCoreCH` performs a Core-CH precomputation. The output is used by the (Mc)ULTRA shortcut computation and by the MCSA and M(C)R query algorithms.

### ULTRA Shortcut Computation

- `computeStopToStopShortcuts` computes stop-to-stop ULTRA shortcuts for use with ULTRA-CSA and ULTRA-RAPTOR.
- `computeEventToEventShortcuts` computes event-to-event ULTRA shortcuts for use with ULTRA-TB.
- `computeDelayEventToEventShortcuts` computes delay-tolerant event-to-event ULTRA shortcuts.
- `computeDelayStopToStopShortcuts` computes delay-tolerant stop-to-stop ULTRA shortcuts for use with Delay-ULTRA-CSA and Delay-ULTRA-RAPTOR. _(new in this fork)_
- `computeMcStopToStopShortcuts` computes stop-to-stop McULTRA shortcuts for use with ULTRA-McRAPTOR and UBM-RAPTOR.
- `computeMcEventToEventShortcuts` computes event-to-event McULTRA shortcuts for use with ULTRA-McTB and UBM-TB.
- `computeMultimodalMcStopToStopShortcuts` computes multimodal stop-to-stop McULTRA shortcuts for use with ULTRA-McRAPTOR and UBM-RAPTOR.
- `computeMultimodalMcEventToEventShortcuts` computes multimodal event-to-event McULTRA shortcuts for use with UBM-HydRA.
- `augmentTripBasedData` performs the shortcut augmentation step that is required for UBM-TB.
- `validateStopToStopShortcuts` and `validateEventToEventShortcuts` test the validity of the computed shortcuts by comparing them to paths in the original transfer graph.

### TB Transfer Generation

- `raptorToTripBased` takes a network in RAPTOR format as input and runs the TB transfer generation.
  - With a transitively closed transfer graph as input, this performs the original TB preprocessing.
  - With stop-to-stop ULTRA shortcuts as input, this performs the sequential ULTRA-TB preprocessing.
  - The parameter "Route-based pruning?" enables the optimized preprocessing proposed by Lehoux and Loiodice.

### Time-Dependent Graph Construction _(new in this fork)_

- `buildTDGraph` builds and serializes a time-dependent graph from intermediate data. Required for TD-Dijkstra and TAD queries.
- `intermediateToTDGraph` converts a network in Intermediate format to a time-dependent graph (available in `Network`).

### Query Algorithms

The following table lists all query algorithms available in `ULTRA`:

| Command                               | Algorithm      | Transfers  | Query type       | Criteria                                                  |
| ------------------------------------- | -------------- | ---------- | ---------------- | --------------------------------------------------------- |
| `runTransitiveCSAQueries`             | CSA            | Transitive | Stop-to-stop     | Arrival time                                              |
| `runDijkstraCSAQueries`               | MCSA           | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| `runHLCSAQueries`                     | HL-CSA         | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| `runULTRACSAQueries`                  | ULTRA-CSA      | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| `runTransitiveRAPTORQueries`          | RAPTOR         | Transitive | Stop-to-stop     | Arrival time, number of trips                             |
| `runDijkstraRAPTORQueries`            | MR             | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| `runDijkstraRAPTORQueriesNoCH`        | MR (no CH)     | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| `runHLRAPTORQueries`                  | HL-RAPTOR      | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| `runULTRARAPTORQueries`               | ULTRA-RAPTOR   | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| `runTransitiveTBQueries`              | TB             | Transitive | Stop-to-stop     | Arrival time, number of trips                             |
| `runULTRATBQueries`                   | ULTRA-TB       | Unlimited  | Vertex-to-vertex | Arrival time, number of trips                             |
| `runTDDijkstraQueries`                | TAD            | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| `runTDDijkstraQueriesFromBinary`      | TAD            | Unlimited  | Vertex-to-vertex | Arrival time                                              |
| `runTransitiveMcRAPTORQueries`        | McRAPTOR       | Transitive | Stop-to-stop     | Arrival time, number of trips, transfer time (full)       |
| `runMCRQueries`                       | MCR            | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| `runMCTDDQueries`                     | MC-TDD         | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| `runULTRAMcRAPTORQueries`             | ULTRA-McRAPTOR | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| `runULTRAMcTBQueries`                 | ULTRA-McTB     | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (full)       |
| `runTransitiveBoundedMcRAPTORQueries` | BM-RAPTOR      | Transitive | Stop-to-stop     | Arrival time, number of trips, transfer time (restricted) |
| `runUBMRAPTORQueries`                 | UBM-RAPTOR     | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |
| `runUBMTBQueries`                     | UBM-TB         | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |
| `runUBMHydRAQueries`                  | UBM-HydRA      | Unlimited  | Vertex-to-vertex | Arrival time, number of trips, transfer time (restricted) |

Rows marked as new algorithms in this fork:

- **TAD** (Transfer Aware Dijkstra): Dijkstra-based algorithm that correctly handles buffer times by scanning entire trip sequences. Uses `TransferAwareDijkstra` class with `TimeDependentGraph`. Does not require dominated connection filtering or ULTRA preprocessing.
- **TAD from binary**: Same algorithm but loads a precomputed time-dependent graph instead of building it from intermediate data.
- **MR (no CH)**: MR without Contraction Hierarchy acceleration, for comparison purposes.
- **MC-TDD** (Multi-Criteria Time-Dependent Dijkstra): Multi-criteria extension of TD-Dijkstra using `TimeDependentMCDijkstra`.

TD-Dijkstra (the classic variant with dominated connection filtering) is available through the `compareAllAlgorithms` and `compareMRwithTDStatefulCoreCH` comparison commands. It uses `TimeDependentDijkstra` with `TimeDependentGraphClassic` and is only valid on networks without buffer times.

### Comparison and Validation Commands _(new in this fork)_

- `compareAllAlgorithms` runs MR, TD-Dijkstra, TAD (all TTN variants: FC/CST/BST), and ULTRA-CSA on the same queries and compares results.
- `compareMRwithTDStatefulCoreCH` compares MR (with Core-CH) against TAD (with Core-CH).
- `compareMRwithTDStatefulNoCH` compares MR (without CH) against TAD (without CH).
- `compareMCTDDvsMCR` compares Multi-Criteria TD-Dijkstra against MCR.
- `compareBSTvsClassicVariants` compares TransferAwareDijkstra BST variant vs standard binary search.
- `compareCSTvsClassicVariants` compares TransferAwareDijkstra CST variant vs standard binary search.
- `compareFCvsClassicVariants` compares TransferAwareDijkstra FC variant vs standard binary search.
- `compareTDGraphVariants` compares different time-dependent graph construction variants.
- `compareCSAandRAPTOR` compares journeys from CSA and RAPTOR for random queries.
- `checkCSAPruning`, `checkRAPTORPruning`, `checkULTRACSAPruning`, `checkULTRARAPTORPruning`, `checkTDDijkstraPruning`, `checkMcRAPTORPruning`, `checkBMcRAPTORPruning`, `checkULTRAMcRAPTORPruning`, `checkUBMRAPTORPruning`, `checkMCRPruning`, `checkMCTDDCorrectness`, `runCheckDijkstraRAPTORPruning` verify that pruning rules produce the same results as the unpruned algorithm.

### Debugging _(new in this fork)_

- `debugSingleQuery` runs a single random query with detailed traces from both MR and TAD, comparing results.
- `debugExplicitQuery` runs a query with explicit source, target, and departure time, printing detailed algorithm traces.

## Networks

We use custom data formats for loading the public transit network and the transfer graph: The Intermediate format allows for easy network manipulation, while the RAPTOR format is required by the preprocessing and all query algorithms except for CSA, which uses its own format. The Switzerland and London networks used in our experiments are available at [https://i11www.iti.kit.edu/PublicTransitData/ULTRA/](https://i11www.iti.kit.edu/PublicTransitData/ULTRA/) in the required formats. Unfortunately, we cannot provide the Germany and Stuttgart networks because they are proprietary.

The `Network` application provides commands for manipulating the network data and for converting public transit data to our custom format. It includes the following commands:

- `parseGTFS` converts GTFS data in CSV format to a binary format.
- `gtfsToIntermediate` converts GTFS binary data to the Intermediate network format.
- `intermediateToCSA` converts a network in Intermediate format to CSA format.
- `intermediateToRAPTOR` converts a network in Intermediate format to RAPTOR format.
- `intermediateToTDGraph` converts a network in Intermediate format to a time-dependent graph. _(new in this fork)_
- `raptorToIntermediate` converts a network in RAPTOR format back to Intermediate format. _(new in this fork)_
- `geoJsonToDimacs` converts GeoJSON to DIMACS format. _(new in this fork)_
- `loadDimacsGraph` converts a graph in the format used by the [9th DIMACS Implementation Challenge](http://diag.uniroma1.it/challenge9/download.shtml) to our custom binary graph format.
- `duplicateTrips` duplicates all trips in the network and shifts them by a specified time offset. This is used to extend networks that only comprise a single day to two days, in order to allow for overnight journeys.
- `reverseRAPTORNetwork` generates a reversed copy of a RAPTOR network. _(new in this fork)_
- `addGraph` adds a transfer graph to a network in Intermediate format. Existing transfer edges in the network are preserved.
- `replaceGraph` replaces the transfer graph of a network with a specified transfer graph.
- `reduceGraph` contracts all vertices with degree less than 3 in the transfer graph.
- `reduceToMaximumConnectedComponent` reduces a network to its largest connected component.
- `reduceToMaximumConnectedComponentWithTransitive` reduces a network to its largest connected component and adjusts the transitive network accordingly. _(new in this fork)_
- `applyBoundingBox` removes all parts of a network that lie outside a predefined bounding box.
- `applyCustomBoundingBox` removes all parts of a network that lie outside a specified bounding box.
- `makeOneHopTransfers` computes one-hop transfers for all stops whose distance is below a specified threshold. This is used to create a transitively closed network for comparison with non-multi-modal algorithms.
- `makeOneHopTransfersByGeoDistance` computes one-hop transfers for all stops within a specified geographic distance. _(new in this fork)_
- `applyMaxTransferSpeed` applies a maximum transfer speed to all edges in the transfer graph.
- `applyConstantTransferSpeed` applies a constant transfer speed to all edges in the transfer graph and computes the travel times accordingly.
- `applyMinTransferTravelTime` applies a minimum travel time to all transfer edges. _(new in this fork)_

An example script that combines all steps necessary to load a public transit network is provided at `Runnables/BuildNetworkExample.script`. It can be run from the `Network` application using `runScript BuildNetworkExample.script`. It takes as input GTFS data in CSV format located at `Networks/Switzerland/GTFS/` and a road graph in DIMACS format located at `Networks/Switzerland/OSM/dimacs`. A more detailed preprocessing guide for Switzerland is available in `PreprocessingStepForMR.md`.

## Multiple Transfer Modes

The algorithms listed above support bimodal networks with public transit and a single transfer mode. Additionally, this framework provides algorithms for multimodal networks with multiple transfer modes. The required multimodal data structures can be built with the following commands in `Network`:

- `buildMultimodalRAPTORData` converts unimodal RAPTOR data into multimodal RAPTOR data. The transfer graph contained in the RAPTOR data is used for the "free" transfers whose transfer time is not penalized. The transfer graphs for the non-"free" modes must be added separately with the `addModeToMultimodalRAPTORData`.
- `addModeToMultimodalRAPTORData` adds a transfer graph for a specified transfer mode to the given multimodal RAPTOR data.
- `buildMultimodalTripBasedData` converts unimodal TB data into multimodal TB data. The transfer graph contained in the TB data is used for the "free" transfers whose transfer time is not penalized. The transfer graphs for the non-"free" modes must be added separately with the `addModeToMultimodalTripBasedData`.
- `addModeToMultimodalTripBasedData` adds a shortcut graph for a specified transfer mode to the given multimodal TB data.

Additionally, the command `buildFreeTransferGraph` in `ULTRA` builds a "free" transfer graph by connecting all pairs of stops within a specified geographical distance and then computing the transitive closure.

The `ULTRA` application offers the following multimodal query algorithms. All algorithms optimize arrival time, number of trips and one transfer time criterion per transfer mode.

- `runMultimodalMCRQueries`: MCR for full Pareto sets
- `runMultimodalULTRAMcRAPTORQueries`: ULTRA-McRAPTOR with stop-to-stop shortcuts for full Pareto sets
- `runMultimodalUBMRAPTORQueries`: UBM-RAPTOR with stop-to-stop shortcuts for restricted Pareto sets
- `runMultimodalUBMHydRAQueries`: UBM-HydRA with event-to-event shortcuts for restricted Pareto sets

## One-to-Many Journey Planning

The query algorithms in the `ULTRA` application only support one-to-one queries. The `ULTRAPHAST` application provides algorithms for one-to-all and one-to-many queries:

| Command                                      | Algorithm | Target set     | Criteria                      |
| -------------------------------------------- | --------- | -------------- | ----------------------------- |
| `runOneToAllDijkstraCSAQueriesToVertices`    | MCSA      | Vertices       | Arrival time                  |
| `runOneToManyDijkstraCSAQueriesToStops`      | MCSA      | Stops          | Arrival time                  |
| `runOneToManyDijkstraCSAQueriesToBall`       | MCSA      | Ball           | Arrival time                  |
| `runUPCSAQueries`                            | UP-CSA    | Vertices/Stops | Arrival time                  |
| `runUPCSAQueriesToBall`                      | UP-CSA    | Ball           | Arrival time                  |
| `runOneToAllDijkstraRAPTORQueriesToVertices` | MR        | Vertices       | Arrival time, number of trips |
| `runOneToManyDijkstraRAPTORQueriesToStops`   | MR        | Stops          | Arrival time, number of trips |
| `runOneToManyDijkstraRAPTORQueriesToBall`    | MR        | Ball           | Arrival time, number of trips |
| `runUPRAPTORQueries`                         | UP-RAPTOR | Vertices/Stops | Arrival time, number of trips |
| `runUPRAPTORQueriesToBall`                   | UP-RAPTOR | Ball           | Arrival time, number of trips |
| `runUPTBQueries`                             | UP-TB     | Vertices/Stops | Arrival time, number of trips |

Random ball target sets can be generated with the command `createBallTargetSets`. CH and Core-CH precomputations for these target sets can be run with `buildUPCHForTargetSets` and `buildCoreCHForTargetSets`, respectively.

## Delay-Robustness

The application `DelayExperiments` provides commands for evaluating Delay-ULTRA, the variant of ULTRA that anticipates possible vehicle delays. The delay-robust shortcut computation itself is run with the command `computeDelayEventToEventShortcuts` (event-to-event) or `computeDelayStopToStopShortcuts` (stop-to-stop) in `ULTRA`. All delays up to the specified limit (measured in seconds) are accounted for.

### Scenario and Query Generation

- `generateDelayScenario` generates a delay scenario for the given network, using a synthetic delay model.
- `generateDelayQueries` generates queries for the specified delay scenario that are answered incorrectly by an algorithm without delay information. These "affected queries" are used for coverage evaluation.
- `buildFakeDelayData` takes as input a network with regular ULTRA shortcuts and converts it to the format used by Delay-ULTRA. This is useful for comparing Delay-ULTRA to regular ULTRA.

### Update Phase Simulation

- `runDelayUpdatesWithoutReplacement` simulates basic delay updates for the given delay scenario.
- `runDelayUpdatesWithReplacement` simulates advanced delay updates for the given delay scenario. A heuristic replacement search is performed to find missing shortcuts.

### Coverage Evaluation

- `measureDelayULTRAQueryCoverage` measures the result quality of TB using Delay-ULTRA shortcuts (real-time mode: updates applied with simulated processing lag).
- `measureHypotheticalDelayULTRAQueryCoverage` measures the result quality of TB using Delay-ULTRA shortcuts (hypothetical mode: updates assumed to be applied instantly).
- `measureDelayQueryCoverage` evaluates per-query coverage of TB, RAPTOR, RAPTOR (EP), and CSA using hypothetical mode. Tests all four algorithms on affected queries with per-query delays and ground truth comparison. _(new in this fork)_

### Performance Evaluation

- `measureDelayULTRAQueryPerformance` measures query performance of ULTRA-TB with delay shortcuts and replacement search.
- `measureDelayULTRACSAQueryPerformance` measures query performance of Delay-ULTRA-CSA, Delay-ULTRA-RAPTOR, Delay-ULTRA-RAPTOR (EP), Delay-ULTRA-TB, MR, TAD, and TD-Dijkstra on random queries with delay state applied at a snapshot time. _(new in this fork)_

### Analysis _(new in this fork)_

- `analyzeHeadwayDistribution` analyzes the headway distribution of routes in the given network.
- `analyzeTransferSlacks` analyzes the transfer slacks of optimal journeys for random queries.
- `validateDelayULTRATripBased` validates journeys computed by ULTRA-TB with delay-tolerant shortcuts by comparing to MR on random queries.

## Non-FIFO Check

The standalone `NonFifoCheck` binary detects non-FIFO violations in a timetable — cases where a later-departing connection on the same edge arrives earlier than an earlier-departing one. This is relevant because TAD handles non-FIFO schedules by scanning multiple trips with trip pruning, while TD-Dijkstra relies on dominated connection filtering that restores FIFO order.

Usage:

```bash
./cmake-build-release/NonFifoCheck <intermediate.binary> [maxSamples]
```
