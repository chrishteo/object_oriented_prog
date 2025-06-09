// File: Edge.hpp
#ifndef EDGE_HPP
#define EDGE_HPP

class Edge {
private:
    unsigned long destinationID;
    double distance;
    unsigned long wayID;    // the OSM way this edge came from

public:
    // Constructor now takes wayID
    Edge(unsigned long dest, double dist, unsigned long way);

    // Copy constructor
    Edge(const Edge& other);

    // Accessors
    unsigned long getDestinationID() const;
    double        getDistance()      const;
    unsigned long getWayID()         const;

    // Mutator
    void setDistance(double dist);
};

#endif // EDGE_HPP
