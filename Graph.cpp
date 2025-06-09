// File: Graph.cpp

#include "Graph.hpp"
#include "Edge.hpp"
#include "tinyxml2.h"

#include <cmath>
#include <tuple>
#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
#include <set>
#include <algorithm>
#include <limits>

using namespace tinyxml2;

// ——— Constructor & Cleanup ———
Graph::Graph(const std::string& filename) {
    loadFromOSM(filename);
    removeUnconnectedNodes();
}

void Graph::addVertex(unsigned long id, double lat, double lon) {
    if (!id_to_index.count(id)) {
        vertices.emplace_back(id, lat, lon);
        id_to_index[id] = (unsigned int)vertices.size() - 1;
    }
}

void Graph::removeVertex(unsigned long id) {
    auto it = id_to_index.find(id);
    if (it == id_to_index.end()) return;
    unsigned int idx = it->second;
    // remove edges into this node
    for (auto &v : vertices) v.removeEdgeTo(id);
    // erase vertex
    vertices.erase(vertices.begin() + idx);
    // rebuild index map
    id_to_index.clear();
    for (unsigned int i = 0; i < vertices.size(); ++i)
        id_to_index[vertices[i].getID()] = i;
}

void Graph::removeUnconnectedNodes() {
    std::set<unsigned long> connected;
    for (auto &v : vertices) {
        if (!v.getEdges().empty()) {
            connected.insert(v.getID());
            for (auto &e : v.getEdges())
                connected.insert(e.getDestinationID());
        }
    }
    std::vector<Vertex> kept;
    for (auto &v : vertices)
        if (connected.count(v.getID()))
            kept.push_back(v);
    vertices = std::move(kept);
    id_to_index.clear();
    for (unsigned int i = 0; i < vertices.size(); ++i)
        id_to_index[vertices[i].getID()] = i;
}

// ——— Load from OSM XML ———
void Graph::loadFromOSM(const std::string& filename) {
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) return;
    XMLElement* osm = doc.FirstChildElement("osm");
    if (!osm) return;

    // 1) Read nodes
    std::unordered_map<unsigned long,std::pair<double,double>> nodeData;
    for (XMLElement* el = osm->FirstChildElement("node");
         el; el = el->NextSiblingElement("node")) {
        const char *pid = el->Attribute("id"),
                   *plat = el->Attribute("lat"),
                   *plon = el->Attribute("lon");
        if (!pid||!plat||!plon) continue;
        unsigned long id = std::stoul(pid);
        double lat = std::stod(plat), lon = std::stod(plon);
        nodeData[id] = {lat, lon};
        addVertex(id, lat, lon);
    }

    // 2) Read ways
    for (XMLElement* el = osm->FirstChildElement("way");
         el; el = el->NextSiblingElement("way")) {
        unsigned long wayID = el->Unsigned64Attribute("id");
        bool isRoad=false, oneway=false;
        std::string highwayType;
        std::vector<unsigned long> refs;

        // collect nd refs
        for (XMLElement* nd = el->FirstChildElement("nd");
             nd; nd = nd->NextSiblingElement("nd")) {
            if (auto *r = nd->Attribute("ref"))
                refs.push_back(std::stoul(r));
        }
        // collect tags
        for (XMLElement* tag = el->FirstChildElement("tag");
             tag; tag = tag->NextSiblingElement("tag")) {
            auto *k = tag->Attribute("k"), *v = tag->Attribute("v");
            if (!k||!v) continue;
            if (std::string(k) == "highway") {
                isRoad = true;
                highwayType = v;
            }
            if (std::string(k)=="oneway" &&
                (std::string(v)=="yes"||std::string(v)=="1")) {
                oneway = true;
            }
        }
        if (!isRoad) continue;

        double factor = 1.0;
        if (highwayType=="motorway"||highwayType=="trunk")             factor=0.5;
        else if (highwayType=="primary"||highwayType=="secondary")    factor=0.75;
        else if (highwayType=="tertiary"||highwayType=="residential") factor=1.0;
        else if (highwayType=="living_street"||highwayType=="unclassified") factor=1.25;
        else if (highwayType=="service"||highwayType=="track")        factor=1.5;

        // build edges
        for (size_t i = 1; i < refs.size(); ++i) {
            unsigned long src = refs[i-1], dst = refs[i];
            auto iS = id_to_index.find(src), iD = id_to_index.find(dst);
            if (iS==id_to_index.end()||iD==id_to_index.end()) continue;
            auto [lat1, lon1] = nodeData[src];
            auto [lat2, lon2] = nodeData[dst];
            double raw = std::hypot(lat1 - lat2, lon1 - lon2) * 1e6;
            double dist = raw * factor;

            // forward
            vertices[iS->second].addEdge(Edge(dst, dist, wayID));
            // reverse if not one-way
            if (!oneway)
                vertices[iD->second].addEdge(Edge(src, dist, wayID));
        }
    }
}

// ——— Dijkstra ———
std::list<unsigned long> Graph::dijkstra(unsigned long startID,
                                         unsigned long endID) const {
    std::list<unsigned long> path;
    if (!id_to_index.count(startID) || !id_to_index.count(endID))
        return path;

    using P = std::pair<double,unsigned long>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    std::unordered_map<unsigned long,double> dist;
    std::unordered_map<unsigned long,unsigned long> prev;
    for (auto &v : vertices)
        dist[v.getID()] = std::numeric_limits<double>::infinity();
    dist[startID] = 0.0;
    pq.push({0.0, startID});

    while (!pq.empty()) {
        auto [d,u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == endID) break;
        unsigned int ui = id_to_index.at(u);
        for (auto &e : vertices[ui].getEdges()) {
            unsigned long w = e.getDestinationID();
            double nd = d + e.getDistance();
            if (nd < dist[w] || (nd == dist[w] && u < prev[w])) {
                dist[w] = nd;
                prev[w] = u;
                pq.push({nd, w});
            }
        }
    }

    // reconstruct
    for (unsigned long at = endID; ; ) {
        path.push_front(at);
        if (at == startID) break;
        at = prev[at];
    }
    return path;
}

// ——— BFS ———
std::list<unsigned long> Graph::bfs(unsigned long startID) const {
    std::list<unsigned long> out;
    if (!id_to_index.count(startID)) return out;
    std::queue<unsigned long> q;
    std::set<unsigned long> vis;
    q.push(startID); vis.insert(startID);

    while (!q.empty()) {
        unsigned long u = q.front(); q.pop();
        out.push_back(u);
        unsigned int ui = id_to_index.at(u);
        std::vector<unsigned long> nbrs;
        for (auto &e : vertices[ui].getEdges())
            nbrs.push_back(e.getDestinationID());
        std::sort(nbrs.begin(), nbrs.end());
        for (auto v : nbrs)
            if (!vis.count(v)) { vis.insert(v); q.push(v); }
    }
    return out;
}

// ——— DFS ———
std::list<unsigned long> Graph::dfs(unsigned long startID) const {
    std::list<unsigned long> out;
    if (!id_to_index.count(startID)) return out;
    std::stack<unsigned long> st;
    std::set<unsigned long> vis;
    st.push(startID);

    while (!st.empty()) {
        unsigned long u = st.top(); st.pop();
        if (vis.count(u)) continue;
        vis.insert(u); out.push_back(u);
        unsigned int ui = id_to_index.at(u);
        std::vector<unsigned long> nbrs;
        for (auto &e : vertices[ui].getEdges())
            nbrs.push_back(e.getDestinationID());
        // push reverse‐sorted for ascending visit
        std::sort(nbrs.begin(), nbrs.end(), std::greater<unsigned long>());
        for (auto v : nbrs)
            if (!vis.count(v)) st.push(v);
    }
    return out;
}

// ——— Compact ———
void Graph::compact() {
    bool removed = true;
    while (removed) {
        removed = false;

        // build incoming map with wayID
        struct Inc { unsigned long src; double d; unsigned long way; };
        std::unordered_map<unsigned long,std::vector<Inc>> incoming;
        for (auto &u : vertices) {
            for (auto &e : u.getEdges()) {
                incoming[e.getDestinationID()]
                    .push_back({u.getID(), e.getDistance(), e.getWayID()});
            }
        }

        // first collapse any one‐way degree-2
        for (size_t idx = 0; idx < vertices.size(); ++idx) {
            auto &v = vertices[idx];
            auto it = incoming.find(v.getID());
            size_t inC = (it==incoming.end()?0:it->second.size());
            size_t outC = v.getEdges().size();
            if (inC==1 && outC==1) {
                auto [uID,duv,way] = it->second[0];
                auto &vw = v.getEdges().front();
                unsigned long wID = vw.getDestinationID();
                double dvw = vw.getDistance();
                unsigned int iu = id_to_index.at(uID);
                vertices[iu].addEdge(Edge(wID, duv + dvw, way));
                removeVertex(v.getID());
                removed = true;
                break;
            }
        }
        if (removed) continue;

        // then collapse two‐way only if same wayID
        for (size_t idx = 0; idx < vertices.size(); ++idx) {
            auto &v = vertices[idx];
            auto it = incoming.find(v.getID());
            size_t inC = (it==incoming.end()?0:it->second.size());
            size_t outC = v.getEdges().size();
            if (inC==2 && outC==2) {
                auto [n1,d1,w1] = it->second[0];
                auto [n2,d2,w2] = it->second[1];
                if (w1 != w2) continue;
                auto eit = v.getEdges().begin();
                unsigned long o1 = eit->getDestinationID(); double od1 = eit->getDistance(); ++eit;
                unsigned long o2 = eit->getDestinationID(); double od2 = eit->getDistance();
                std::set<unsigned long> incSet={n1,n2}, outSet={o1,o2};
                if (incSet != outSet) continue;
                double dn1n2 = (o1==n2? d1+od1 : d1+od2);
                double dn2n1 = (o1==n1? d2+od1 : d2+od2);
                unsigned int i1 = id_to_index.at(n1), i2 = id_to_index.at(n2);
                vertices[i1].addEdge(Edge(n2, dn1n2, w1));
                vertices[i2].addEdge(Edge(n1, dn2n1, w1));
                removeVertex(v.getID());
                removed = true;
                break;
            }
        }
    }
}
