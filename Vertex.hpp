#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <list>
#include "Edge.hpp"

// Vertex represents a graph node, storing its unique ID, lat/lon, and outgoing edges.
class Vertex {
private:
    unsigned long id;
    double latitude;
    double longitude;
    std::list<Edge> edges;  // adjacency list of outgoing edges

public:
    // Construct a vertex with ID, latitude, and longitude
    Vertex(unsigned long id, double lat, double lon);

    // Copy constructor
    Vertex(const Vertex& other);

    // Add an outgoing edge from this vertex
    void addEdge(const Edge& edge);

    // Remove any outgoing edge whose destinationID == destID
    void removeEdgeTo(unsigned long destID);

    // Getters
    unsigned long getID() const;
    double getLatitude() const;
    double getLongitude() const;

    // Return const reference to the adjacency list
    const std::list<Edge>& getEdges() const;
};

#endif // VERTEX_HPP
