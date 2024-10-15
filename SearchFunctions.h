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


// PRE: A filled map of adjacent cities, a selected starting point
// POST: Will print the path taken
void BFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity);

//

// PRE: A filled map of adjacent cities, the chosen path from the previous recursive call, the current path taken
// POST: Either recursively call itself or return when queue is empty
void DFSRecursive(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string currentCity, std::set<std::string> &visited, std::string goalCity);

// PRE: A filled map of adjacent cities, a selected start point
// POST: Will print the path taken
void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity);

//

bool DLS(const std::map<std::string, std::vector<std::string>>& adjacencyMap, std::string currentCity, std::string goalCity, int limit, std::set<std::string> &visited);

bool IDDFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity, int maxDepth);

//

// PRE: The latitude and longitude of two points, order does not matter
// POST: Return the euclidean distance
double calculateDistance(double lat1, double lon1, double lat2, double lon2);

// PRE: The filled adjacencyMap, coordinateMap, a city to start from, and a city to reach
// POST: Return the path taken as a vector
std::vector<std::string> AStarSearch(
    const std::map<std::string, std::vector<std::string>> &adjacencyMap,
    const std::map<std::string, std::pair<double, double>> &coordinateMap,
    const std::string startCity,
    const std::string goalCity);

std::vector<std::string> BestFirstSearch(
    const std::map<std::string, std::vector<std::string>> &adjacencyMap,
    const std::map<std::string, std::pair<double, double>> &coordinateMap,
    const std::string startCity,
    const std::string goalCity);

#endif // SEARCHFUNCTIONS_H
