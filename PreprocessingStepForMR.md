# ULTRA Transit Routing System Setup Documentation

This documentation describes the complete process for setting up and running the ULTRA transit routing system for Switzerland, including data preparation, preprocessing, and algorithm execution.

## Overview

The ULTRA system requires several preprocessing steps to convert raw GTFS and OSM data into optimized data structures for efficient routing. The process involves:

1. Parsing raw GTFS data
2. Processing OSM walking data
3. Creating intermediate network representations
4. Applying network optimizations
5. Building specialized data structures (CH, RAPTOR, CSA)
6. Running routing algorithms

## Prerequisites

- Raw GTFS data for Switzerland from [GTFS.geops.ch](https://gtfs.geops.ch/#completefeed)
- OSM data for Switzerland from [Geofabrik](https://download.geofabrik.de/europe/switzerland.html)
- ULTRA codebase properly compiled
- Osmium tool for OSM data conversion
- RapidJSON library for processing GeoJSON data

RapidJSON is a header-only C++ library for parsing and generating JSON data.
The ULTRA codebase uses it particularly for processing GeoJSON files in the network import process.

There are different installation guides, stick with the one you prefer.

Network structure after preprocessing:

```
MainDirectory/
└──Networks/
	└── Switzerland/
		├── GTFS/
		│   ├── gtfs_complete/    # Raw GTFS data
		│   └── GTFS.binary       # Binary GTFS representation
		├── OSM/
		│   ├── switzerland-*.osm.pbf    # Raw OSM data
		│   ├── switzerland-*.geojson    # Converted GeoJSON
		│   ├── dimacs/                  # DIMACS format files
		│   └── graphs/                  # Graph data structures
		├── intermediate/
		│   └── intermediate.binary      # Intermediate network format
		├── RAPTOR/
		│   └── raptor.binary            # RAPTOR data structure
		├── CSA/
		│   └── csa.binary               # CSA data structure
		└── CH/
			└── ch.*                     # Contraction Hierarchies
```

## Preprocessing Steps

### 1. Parse GTFS Data

Download and parse the GTFS data into a binary format:

```
parseGTFS ../Networks/Switzerland/GTFS/gtfs_complete/ ../Networks/Switzerland/GTFS/GTFS.binary
```

### 2. Process OSM Walking Data

#### 2.1. Convert OSM PBF to GeoJSON
```
osmium export ../Networks/Switzerland/OSM/switzerland-250908.osm.pbf -o ../Networks/Switzerland/OSM/switzerland-250908.geojson
```

#### 2.2. Split GeoJSON into DIMACS Files
```
geoJsonToDimacs ../Networks/Switzerland/OSM/switzerland-250908.geojson ../Networks/Switzerland/OSM/dimacs/dimacs
```

#### 2.3. Load DIMACS Files and Create Graph
```
loadDimacsGraph ../Networks/Switzerland/OSM/dimacs/dimacs ../Networks/Switzerland/OSM/graphs/graph dynamic 1
```


### 3. Create Intermediate Network Representation

Convert the binary GTFS data to the intermediate format:

```
gtfsToIntermediate ../Networks/Switzerland/GTFS/GTFS.binary 20241215 20241216 true true ../Networks/Switzerland/intermediate/intermediate.binary
```


### 4. Apply Geographic Bounding Box

Restrict the network to the Switzerland area:

```
applyBoundingBox ../Networks/Switzerland/intermediate/intermediate.binary switzerland ../Networks/Switzerland/intermediate/intermediate.binary
```

### 5. Reduce Graph

Simplify the graph by contracting vertices with degree ≤ 2:

```
reduceGraph ../Networks/Switzerland/intermediate/intermediate.binary ../Networks/Switzerland/intermediate/intermediate.binary
```

### 6. Integrate Walking Network

Add the walking graph to the intermediate network:

```
addGraph ../Networks/Switzerland/intermediate/intermediate.binary ../Networks/Switzerland/OSM/graphs/graph ../Networks/Switzerland/intermediate/intermediate.binary
```

**Note**: This step may take significant time to complete.

### 7. Extract Maximum Connected Component

Keep only the largest connected component in the network:

```
reduceToMaximumConnectedComponent ../Networks/Switzerland/intermediate/intermediate.binary ../Networks/Switzerland/intermediate/intermediate.binary
```

**Note**: If this step causes segmentation faults, verify your memory resources and consider splitting the process or using a machine with more RAM.

### 8. Apply Transfer Speed Limit

Cap transfer speeds to a realistic walking speed:

```
applyMaxTransferSpeed ../Networks/Switzerland/intermediate/intermediate.binary 4.5 ../Networks/Switzerland/intermediate/intermediate.binary
```

### 9. Final Graph Reduction

Perform a final simplification of the graph:

```
reduceGraph ../Networks/Switzerland/intermediate/intermediate.binary ../Networks/Switzerland/intermediate/intermediate.binary
```

### 10. Convert to Specialized Data Structures

#### 10.1. Create RAPTOR Data Structure
```
intermediateToRAPTOR ../Networks/Switzerland/intermediate/intermediate.binary ../Networks/Switzerland/RAPTOR/raptor.binary
```

#### 10.2. Create CSA Data Structure
```
intermediateToCSA ../Networks/Switzerland/intermediate/intermediate.binary ../Networks/Switzerland/CSA/csa.binary
```

### 11. Build Contraction Hierarchies

Build contraction hierarchies for efficient transfers:

```
buildCH ../Networks/Switzerland/CSA/csa.binary.graph ../Networks/Switzerland/CH/ch ../Networks/Switzerland/CH/ch
```

## Optional Steps

### Create One-Hop Transfers

This step creates direct transfers between stops within a specified travel time limit. It's typically used for comparison with non-multimodal algorithms:

```
makeOneHopTransfers ../Networks/Switzerland/intermediate/intermediate.binary 86400 ../Networks/Switzerland/intermediate/intermediate.binary true
```

## Running Routing Algorithms

### Dijkstra-RAPTOR Example

Run 100 random Dijkstra-RAPTOR queries:

```
runDijkstraRAPTORQueries ../Networks/Switzerland/RAPTOR/raptor.binary ../Networks/Switzerland/CH/ch 100
```

Example output:
```
runDijkstraRAPTORQueries ../Networks/Switzerland/RAPTOR/raptor.binary ../Networks/Switzerland/CH/ch 100
Loading static graph from ../Networks/Switzerland/RAPTOR/raptor.binary.graph
RAPTOR public transit data:
   Number of Stops:                32,730
   Number of Routes:               18,940
   Number of Trips:               479,116
   Number of Stop Events:       5,079,231
   Number of Connections:       4,600,115
   Number of Vertices:          1,764,290
   Number of Edges:             5,538,794
   First Day:                           0
   Last Day:                            2
   Bounding Box:             [(5.89326, 45.4871) | (10.7489, 48.1195)]
Loading static graph from ../Networks/Switzerland/CH/ch.forward
Loading static graph from ../Networks/Switzerland/CH/ch.backward

Statistics:
   Round       Routes     Segments     Vertices        Edges Stops (trip) Stops (transfer)           Init        Collect           Scan      Transfers          Total
   clear            0            0            0            0            0                0            0µs            0µs            0µs            0µs      1ms 935µs
    init            0            0            0            0            0                2           67µs            0µs            0µs          707µs          775µs
       0           12          133      706,403    2,230,452           93            8,722           55µs            2µs            7µs    108ms 741µs    108ms 809µs
       1        4,042       50,557    1,018,190    3,208,683       16,191           14,941           51µs      1ms 139µs          917µs    198ms 146µs    200ms 255µs
       2        7,753      100,420    1,003,506    3,157,768       25,267           15,855           40µs      1ms 948µs      1ms 722µs    219ms 566µs    223ms 280µs
       3        8,791      114,818      835,085    2,624,613       20,260           13,864           41µs      2ms  98µs      1ms 937µs    198ms 316µs    202ms 396µs
       4        8,300      107,608      587,165    1,843,777       13,493           10,109           40µs      1ms 883µs      1ms 837µs    136ms 659µs    140ms 423µs
       5        6,492       83,205      354,181    1,111,407        8,088            6,282           41µs      1ms 393µs      1ms 410µs     78ms 764µs     81ms 611µs
       6        4,309       54,702      163,649      511,936        4,091            3,187           40µs          931µs          943µs     33ms 145µs     35ms  62µs
       7        2,400       29,985       54,107      168,557        1,840            1,232           40µs          491µs          550µs      9ms 907µs     10ms 991µs
       8        1,089       13,353       19,185       59,550          646              469           40µs          205µs          238µs      3ms 403µs      3ms 888µs
       9          431        5,311        7,670       23,701          274              204           33µs           78µs           97µs      1ms 221µs      1ms 432µs
      10          175        2,150        3,135        9,641          138               94           28µs           33µs           35µs          459µs          557µs
      11           70          895        1,168        3,569           53               39           23µs           14µs           16µs          150µs          206µs
      12           28          360          314          962           12                9           16µs            4µs            5µs           33µs           60µs
      13            8           98           39          119            1                1           11µs            1µs            1µs            3µs           16µs
      14            1           12            0            0            0                0           10µs            0µs            0µs            0µs           10µs
      15            0            0            0            0            0                0            4µs            0µs            0µs            0µs            4µs
   total       43,901      563,607    4,753,797   14,954,735       90,447           75,010          588µs     10ms 225µs      9ms 722µs    989ms 226µs 1s  11ms 719µs
Total time: 1s  11ms 726µs
Avg. rounds: 9.02
Avg. journeys: 3.14
```

## Troubleshooting

- **Segmentation Faults**: If you encounter segmentation faults during the `reduceToMaximumConnectedComponent` step, consider:
  - Running on a machine with more RAM
  - Processing a smaller geographic area
  - Ommit step if not neccessary

## Additional Information

For more details about the specific algorithms and data structures, refer to the ULTRA codebase documentation and related research papers.