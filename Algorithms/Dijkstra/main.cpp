#include <iostream>
#include "TD-DijkstraFromBase.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"

int main() {
    // --- 1. Construct a Sample Graph ---
    TimeDependentGraph sampleGraph;
    sampleGraph.addVertex(); // Vertex 0 (A)
    sampleGraph.addVertex(); // Vertex 1 (B)
    sampleGraph.addVertex(); // Vertex 2 (C)

    ArrivalTimeFunction atf_A_to_B;
    atf_A_to_B.discreteTrips.push_back({.departureTime = 800, .arrivalTime = 815});
    sampleGraph.addTimeDependentEdge(Vertex(0), Vertex(1), atf_A_to_B);

    ArrivalTimeFunction atf_B_to_C;
    atf_B_to_C.discreteTrips.push_back({.departureTime = 820, .arrivalTime = 830});
    sampleGraph.addTimeDependentEdge(Vertex(1), Vertex(2), atf_B_to_C);

    ArrivalTimeFunction atf_A_to_C;
    atf_A_to_C.walkTime = 45;
    sampleGraph.addTimeDependentEdge(Vertex(0), Vertex(2), atf_A_to_C);


    // --- 2. Instantiate and Run the Algorithm ---
    // Now correctly referencing the class inside the CH namespace
    TimeDependentDijkstra<TimeDependentGraph> tdDijkstra(sampleGraph);

    const Vertex source = Vertex(0);
    const Vertex target = Vertex(2);
    const int departureTime = 750;

    std::cout << "Running TD-Dijkstra from " << source << " to " << target << " at time " << departureTime << std::endl;
    tdDijkstra.run(source, departureTime, target);


    // --- 3. Print the Results ---
    int earliestArrival = tdDijkstra.getEarliestArrivalTime(target);

    if (earliestArrival != -1) {
        std::cout << "Earliest arrival time at target " << target << " is: " << earliestArrival << std::endl;
    } else {
        std::cout << "Target " << target << " is not reachable." << std::endl;
    }

    return 0;
}