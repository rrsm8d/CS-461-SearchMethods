#ifndef SEARCHFUNCTIONS_H
#define SEARCHFUNCTIONS_H

#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <set>
#include <cmath> // for pow and sqrt
#include <string>
#include <limits>
#include <algorithm> // for std::reverse

void BFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity);

//

// Old, don't like how I implemented it
// void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string start);

void DFSRecursive(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string currentCity, std::set<std::string> &visited);

void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity);

//

bool DLS(const std::map<std::string, std::vector<std::string>>& adjacencyMap, std::string currentCity, std::string goalCity, int limit, std::set<std::string> &visited);

bool IDDFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity, int maxDepth);

//

double calculateDistance(double lat1, double lon1, double lat2, double lon2);

std::vector<std::string> AStarSearch(
    const std::map<std::string, std::vector<std::string>> &adjacencyMap,
    const std::map<std::string, std::pair<double, double>> &coordinateMap,
    const std::string startCity,
    const std::string goalCity);

#endif // SEARCHFUNCTIONS_H
