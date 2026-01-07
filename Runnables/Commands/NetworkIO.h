#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#pragma once

#include <string>

#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/GTFS/Data.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/MultimodalData.h"
#include "../../DataStructures/TripBased/MultimodalData.h"

// --- Project Includes ---
// Assuming these are the necessary includes for your environment:
#include "../../DataStructures/Graph/TimeDependentGraph.h"      // For TimeDependentGraph
#include "../../Helpers/Timer.h"          // For time profiling

#include "../../Shell/Shell.h"

using namespace Shell;

class ParseGTFS : public ParameterizedCommand {

public:
    ParseGTFS(BasicShell& shell) :
        ParameterizedCommand(shell, "parseGTFS", "Parses raw GTFS data from the given directory and converts it to a binary representation.") {
        addParameter("Input directory");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string gtfsDirectory = getParameter("Input directory");
        const std::string outputFile = getParameter("Output file");

        GTFS::Data data = GTFS::Data::FromGTFS(gtfsDirectory);
        data.printInfo();
        data.serialize(outputFile);
    }

};

class RAPTORToIntermediate : public ParameterizedCommand {

public:
    RAPTORToIntermediate(BasicShell& shell) :
        ParameterizedCommand(shell, "raptorToIntermediate", "Converts RAPTOR network format to binary intermediate data.") {
        addParameter("Input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");

        RAPTOR::Data raptor = RAPTOR::Data::FromBinary(inputFile);
        raptor.printInfo();
        Graph::printInfo(raptor.transferGraph);
        raptor.transferGraph.printAnalysis();

        Intermediate::Data inter = Intermediate::Data::FromRAPTOR(raptor);
        inter.printInfo();
        inter.serialize(outputFile);
    }

};

class GTFSToIntermediate : public ParameterizedCommand {

public:
    GTFSToIntermediate(BasicShell& shell) :
        ParameterizedCommand(shell, "gtfsToIntermediate", "Converts binary GTFS data to the intermediate network format.") {
        addParameter("Input directory");
        addParameter("First day");
        addParameter("Last day");
        addParameter("Use days of operation?");
        addParameter("Use frequencies?");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string gtfsDirectory = getParameter("Input directory");
        const std::string outputFile = getParameter("Output file");
        const int firstDay = stringToDay(getParameter("First day"));
        const int lastDay = stringToDay(getParameter("Last day"));
        const bool useDaysOfOperation = getParameter<bool>("Use days of operation?");
        const bool useFrequencies = getParameter<bool>("Use frequencies?");

        GTFS::Data gtfs = GTFS::Data::FromBinary(gtfsDirectory);
        gtfs.printInfo();
        Intermediate::Data inter = Intermediate::Data::FromGTFS(gtfs, firstDay, lastDay, !useDaysOfOperation, !useFrequencies);
        inter.printInfo();
        inter.serialize(outputFile);
    }

};

class IntermediateToCSA : public ParameterizedCommand {

public:
    IntermediateToCSA(BasicShell& shell) :
        ParameterizedCommand(shell, "intermediateToCSA", "Converts binary intermediate data to CSA network format.") {
        addParameter("Input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
        inter.printInfo();
        CSA::Data data = CSA::Data::FromIntermediate(inter);
        data.printInfo();
        data.serialize(outputFile);
    }

};

class IntermediateToRAPTOR : public ParameterizedCommand {

public:
    IntermediateToRAPTOR(BasicShell& shell) :
        ParameterizedCommand(shell, "intermediateToRAPTOR", "Converts binary intermediate data to RAPTOR network format.") {
        addParameter("Input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
        inter.printInfo();
        RAPTOR::Data data = RAPTOR::Data::FromIntermediate(inter);
        data.printInfo();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }

};

class IntermediateToTDGraph : public ParameterizedCommand {
public:
    // The command name is updated to reflect the target format (TDGraph)
    IntermediateToTDGraph(BasicShell& shell) :
        ParameterizedCommand(shell, "intermediateToTDGraph", "Converts binary intermediate data to a Time-Dependent Graph (TDGraph) network.") {
        addParameter("Input file (Intermediate::Data binary)");
        addParameter("Output file (TDGraph binary)");
    }

    virtual void execute() noexcept {
        Timer timer;

        const std::string inputFile = getParameter("Input file (Intermediate::Data binary)");
        const std::string outputFile = getParameter("Output file (TDGraph binary)");

        // 1. Deserialize Intermediate Data
        std::cout << "Loading Intermediate Data from " << inputFile << "... ";
        Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
        inter.printInfo();

        // 2. Convert to TimeDependentGraph using the new ATF model
        timer.restart();
        std::cout << "Converting to TimeDependentGraph (Discrete ATF)... ";
        TimeDependentGraph tdGraph = TimeDependentGraph::FromIntermediate(inter);

        // Output basic info (You may need to add a printInfo method to TimeDependentGraph)
        std::cout << "TDGraph Info: V=" << tdGraph.numVertices()
                  << ", E=" << tdGraph.numEdges() << std::endl;

        // 3. Serialize the TimeDependentGraph
        timer.restart();
        std::cout << "Serializing TDGraph to " << outputFile << "... ";
        tdGraph.serialize(outputFile); // Assuming TimeDependentGraph has a serialize method
    }
};


class BuildMultimodalRAPTORData : public ParameterizedCommand {

public:
    BuildMultimodalRAPTORData(BasicShell& shell) :
        ParameterizedCommand(shell, "buildMultimodalRAPTORData", "Builds multimodal RAPTOR data based on RAPTOR data.") {
        addParameter("RAPTOR data");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.printInfo();
        const RAPTOR::MultimodalData multimodalData(raptorData);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class AddModeToMultimodalRAPTORData : public ParameterizedCommand {

public:
    AddModeToMultimodalRAPTORData(BasicShell& shell) :
        ParameterizedCommand(shell, "addModeToMultimodalRAPTORData", "Adds a transfer graph for the specified mode to multimodal RAPTOR data.") {
        addParameter("Multimodal RAPTOR data");
        addParameter("Transfer graph");
        addParameter("Mode");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::MultimodalData multimodalData(getParameter("Multimodal RAPTOR data"));
        multimodalData.printInfo();
        RAPTOR::TransferGraph graph;
        graph.readBinary(getParameter("Transfer graph"));
        const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
        multimodalData.addTransferGraph(mode, graph);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class BuildMultimodalTripBasedData : public ParameterizedCommand {

public:
    BuildMultimodalTripBasedData(BasicShell& shell) :
        ParameterizedCommand(shell, "buildMultimodalTripBasedData", "Builds multimodal Trip-Based data based on Trip-Based data.") {
        addParameter("Trip-Based data");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const TripBased::Data tripBasedData(getParameter("Trip-Based data"));
        tripBasedData.printInfo();
        const TripBased::MultimodalData multimodalData(tripBasedData);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class AddModeToMultimodalTripBasedData : public ParameterizedCommand {

public:
    AddModeToMultimodalTripBasedData(BasicShell& shell) :
        ParameterizedCommand(shell, "addModeToMultimodalTripBasedData", "Adds a transfer graph for the specified mode to multimodal Trip-Based data.") {
        addParameter("Multimodal Trip-Based data");
        addParameter("Transfer graph");
        addParameter("Mode");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        TripBased::MultimodalData multimodalData(getParameter("Multimodal Trip-Based data"));
        multimodalData.printInfo();
        TransferGraph graph;
        graph.readBinary(getParameter("Transfer graph"));
        const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
        multimodalData.addTransferGraph(mode, graph);
        multimodalData.printInfo();
        multimodalData.serialize(getParameter("Output file"));
    }
};

class LoadDimacsGraph : public ParameterizedCommand {

public:
    LoadDimacsGraph(BasicShell& shell) :
        ParameterizedCommand(shell, "loadDimacsGraph", "Converts DIMACS graph data to our transfer graph format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Graph type", "dynamic", { "static", "dynamic" });
        addParameter("Coordinate factor", "0.000001");
    }

    virtual void execute() noexcept {
        std::string graphType = getParameter("Graph type");
        if (graphType == "static") {
            load<TransferGraph>();
        } else {
            load<DynamicTransferGraph>();
        }
    }

private:
    template<typename GRAPH_TYPE>
    inline void load() const noexcept {
        DimacsGraphWithCoordinates dimacs;
        dimacs.fromDimacs<true>(getParameter("Input file"), getParameter<double>("Coordinate factor"));
        Graph::printInfo(dimacs);
        dimacs.printAnalysis();
        GRAPH_TYPE graph;
        Graph::move(std::move(dimacs), graph);
        Graph::printInfo(graph);
        graph.printAnalysis();
        graph.writeBinary(getParameter("Output file"));
    }
};

class GeoJSONToDimacs : public ParameterizedCommand {
public:
    GeoJSONToDimacs(BasicShell& shell) :
        ParameterizedCommand(shell, "geoJsonToDimacs", "Converts GeoJSON to DIMACS format.") {
        addParameter("Input file");
        addParameter("Output file base name");
        addParameter("Default speed (km/h)", "5.0");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFileBase = getParameter("Output file base name");
        const double defaultSpeed = getParameter<double>("Default speed (km/h)");
        
        auto start = std::chrono::high_resolution_clock::now();
        std::cout << "Starting GeoJSON to DIMACS conversion..." << std::endl;
        
        // Use EdgeListImplementation directly with correct template parameters
        DimacsGraphWithCoordinates graph;
        
        // Parse GeoJSON and populate graph
        parseGeoJSON(inputFile, graph);
        
        auto parseEnd = std::chrono::high_resolution_clock::now();
        auto parseDuration = std::chrono::duration_cast<std::chrono::seconds>(parseEnd - start).count();
        std::cout << "Parsing completed in " << parseDuration << " seconds." << std::endl;
        
        // Compute travel times based on coordinates
        std::cout << "Computing travel times..." << std::endl;
        Graph::computeTravelTimes(graph, defaultSpeed);
        
        // Output to DIMACS format
        std::cout << "Writing DIMACS files..." << std::endl;
        Graph::toDimacs(outputFileBase, graph, graph[TravelTime]);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto totalDuration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
        std::cout << "Conversion completed in " << totalDuration << " seconds." << std::endl;
        std::cout << "Files written: " << outputFileBase << ".gr and " 
                  << outputFileBase << ".co" << std::endl;
    }
    
private:
    void parseGeoJSON(const std::string& filename, DimacsGraphWithCoordinates& graph) {
        auto start = std::chrono::high_resolution_clock::now();
        
        FILE* fp = fopen(filename.c_str(), "r");
        if (!fp) {
            std::cerr << "Cannot open GeoJSON file: " << filename << std::endl;
            return;
        }

        std::unordered_map<std::pair<int64_t, int64_t>, Vertex, PairHash> pointToVertex;
        
        size_t featuresProcessed = 0;
        size_t lineStringsProcessed = 0;
        size_t pointsProcessed = 0;
        size_t edgesCreated = 0;
        size_t lastReportTime = 0;

        char readBuffer[65536];
        rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
        rapidjson::Document document;
        document.ParseStream(is);
        fclose(fp);

        auto parseEnd = std::chrono::high_resolution_clock::now();
        auto parseTime = std::chrono::duration_cast<std::chrono::seconds>(parseEnd - start).count();
        std::cout << "JSON parsing completed in " << parseTime << " seconds." << std::endl;
        
        if (document.HasParseError()) {
            std::cerr << "JSON parse error: " << document.GetParseError() << std::endl;
            return;
        }
        
        if (!document.HasMember("features") || !document["features"].IsArray()) {
            std::cerr << "No features array found in GeoJSON" << std::endl;
            return;
        }
        
        const auto& features = document["features"];
        const size_t featureCount = features.Size();
        std::cout << "Processing " << featureCount << " features..." << std::endl;
        
        for (rapidjson::SizeType i = 0; i < featureCount; i++) {
            featuresProcessed++;
            
            auto now = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - start).count();
            if (featuresProcessed % 1000 == 0 || now - lastReportTime >= 5) {
                std::cout << "Processed " << featuresProcessed << " of " << featureCount 
                          << " features (" << (featuresProcessed * 100 / featureCount) << "%)" << std::endl;
                lastReportTime = now;
            }

            const auto& feature = features[i];
            if (!feature.HasMember("geometry") || !feature["geometry"].IsObject()) continue;
            
            const auto& geometry = feature["geometry"];
            if (!geometry.HasMember("type") || !geometry["type"].IsString()) continue;
            
            const char* geoType = geometry["type"].GetString();
            if (strcmp(geoType, "LineString") != 0) continue;
            
            lineStringsProcessed++;
            
            if (!geometry.HasMember("coordinates") || !geometry["coordinates"].IsArray()) continue;
            
            const auto& coordinates = geometry["coordinates"];
            const size_t coordCount = coordinates.Size();
            
            std::vector<Vertex> pathVertices;
            pathVertices.reserve(coordCount);
            
            for (rapidjson::SizeType j = 0; j < coordCount; j++) {
                if (!coordinates[j].IsArray() || coordinates[j].Size() < 2) continue;
                
                pointsProcessed++;
                
                const int64_t lon = static_cast<int64_t>(coordinates[j][0].GetDouble() * 1000000);
                const int64_t lat = static_cast<int64_t>(coordinates[j][1].GetDouble() * 1000000);
                
                auto key = std::make_pair(lon, lat);
                auto it = pointToVertex.find(key);
                Vertex v;
                
                if (it == pointToVertex.end()) {
                    v = graph.addVertex();
                    graph.set(Coordinates, v, Geometry::Point(Construct::LatLong, 
                              lat / 1000000.0, lon / 1000000.0));
                    pointToVertex[key] = v;
                } else {
                    v = it->second;
                }
                
                pathVertices.push_back(v);
            }
            
            for (size_t k = 1; k < pathVertices.size(); k++) {
                graph.addEdge(pathVertices[k-1], pathVertices[k]);
                graph.addEdge(pathVertices[k], pathVertices[k-1]);
                edgesCreated += 2;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
        
        std::cout << "GeoJSON parsing stats:" << std::endl
                  << "  Features processed: " << featuresProcessed << std::endl
                  << "  LineStrings found: " << lineStringsProcessed << std::endl
                  << "  Points processed: " << pointsProcessed << std::endl
                  << "  Vertices created: " << graph.numVertices() << std::endl
                  << "  Edges created: " << graph.numEdges() << std::endl
                  << "  Time taken: " << duration << " seconds" << std::endl;
    }
    
    struct PairHash {
        size_t operator()(const std::pair<int64_t, int64_t>& p) const {
            return std::hash<int64_t>()(p.first) ^ std::hash<int64_t>()(p.second);
        }
    };
};