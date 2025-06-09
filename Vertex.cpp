#include "Vertex.hpp"

Vertex::Vertex(unsigned long id, double lat, double lon)
    : id(id), latitude(lat), longitude(lon) {}

Vertex::Vertex(const Vertex& other)
    : id(other.id),
      latitude(other.latitude),
      longitude(other.longitude),
      edges(other.edges) {}

void Vertex::addEdge(const Edge& edge) {
    edges.push_back(edge);
}

void Vertex::removeEdgeTo(unsigned long destID) {
    edges.remove_if([destID](const Edge& e) {
        return e.getDestinationID() == destID;
    });
}

unsigned long Vertex::getID() const {
    return id;
}

double Vertex::getLatitude() const {
    return latitude;
}

double Vertex::getLongitude() const {
    return longitude;
}

const std::list<Edge>& Vertex::getEdges() const {
    return edges;
}
