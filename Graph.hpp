// File: Graph.hpp
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <list>
#include "Vertex.hpp"

// Directed adjacency‐list graph built from OSM XML.
class Graph {
public:
    // Load and build the graph from the given OSM XML file.
    explicit Graph(const std::string& filename);
    ~Graph() = default;

    // Add a vertex (id, latitude, longitude).
    void addVertex(unsigned long id, double latitude, double longitude);
    // Remove a vertex (and all edges to/from it).
    void removeVertex(unsigned long id);

    // Compact the graph by splicing out intermediate
    // degree-2 nodes on uniform-way segments.
    void compact();

    // Shortest path from startID → endID using Dijkstra.
    // Returns list of node IDs.
    std::list<unsigned long> dijkstra(unsigned long startID,
                                      unsigned long endID) const;

    // Breadth-first search traversal from startID.
    std::list<unsigned long> bfs(unsigned long startID) const;

    // Depth-first search traversal from startID.
    std::list<unsigned long> dfs(unsigned long startID) const;

    // Lookup a Vertex by ID (throws std::out_of_range if missing).
    const Vertex& getVertex(unsigned long id) const;

private:
    std::vector<Vertex> vertices;
    std::unordered_map<unsigned long, unsigned int> id_to_index;

    // Load <node> and <way> elements from an OSM XML file.
    void loadFromOSM(const std::string& filename);
    // Remove any vertices with no edges.
    void removeUnconnectedNodes();
};

#endif // GRAPH_HPP
