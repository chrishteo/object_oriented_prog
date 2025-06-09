#include <iostream>
#include <string>
#include <sstream>
#include "Graph.hpp"
#include "tinyxml2.h"

using namespace tinyxml2;

int main() {
    Graph* graph = nullptr;
    std::string line;

    while (true) {
        // Print prompt exactly as expected, with a leading blank line
        std::cout << "\n";
        std::cout << "Enter your choice: \n";

        if (!std::getline(std::cin, line)) break;
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

        if (cmd == "-i") {
            std::string path;
            iss >> path;
            delete graph;
            graph = nullptr;

            XMLDocument doc;
            XMLError e = doc.LoadFile(path.c_str());
            if (e == XML_ERROR_FILE_NOT_FOUND || e == XML_ERROR_EMPTY_DOCUMENT) {
                std::cout << "Unable to open file: " << path << "\n";
                continue;
            }
            if (e != XML_SUCCESS) {
                std::cout << "Invalid format for file: " << path << "\n";
                continue;
            }

            graph = new Graph(path);
            std::cout << "Graph OK\n";
        }
        else if (cmd == "-c") {
            if (graph) {
                graph->compact();
                std::cout << "Compact OK\n";
            }
        }
        else if (cmd == "-p") {
            unsigned long sid, eid;
            iss >> sid >> eid;
            if (!graph) continue;
            auto pathIDs = graph->dijkstra(sid, eid);
            std::cout << "Dijkstra " << sid << " " << eid << "\n";
            for (auto id : pathIDs) {
                std::cout << id << "\n";
            }
        }
        else if (cmd == "-b") {
            unsigned long sid;
            iss >> sid;
            if (!graph) continue;
            for (auto id : graph->bfs(sid)) {
                std::cout << id << "\n";
            }
        }
        else if (cmd == "-d") {
            unsigned long sid;
            iss >> sid;
            if (!graph) continue;
            for (auto id : graph->dfs(sid)) {
                std::cout << id << "\n";
            }
        }
        else if (cmd == "-q") {
            delete graph;
            break;
        }
        // ignore any other input lines
    }

    return 0;
}
