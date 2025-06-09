// File: Edge.cpp
#include "Edge.hpp"

Edge::Edge(unsigned long dest, double dist, unsigned long way)
    : destinationID(dest), distance(dist), wayID(way) {}

Edge::Edge(const Edge& other)
    : destinationID(other.destinationID),
      distance(other.distance),
      wayID(other.wayID) {}

unsigned long Edge::getDestinationID() const { return destinationID; }
double        Edge::getDistance()      const { return distance; }
unsigned long Edge::getWayID()         const { return wayID; }

void Edge::setDistance(double dist) { distance = dist; }
