#ifndef SEARCHFUNCTIONS_H
#define SEARCHFUNCTIONS_H

#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <set>

void BFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity);

//

// Old, don't like how I implemented it
// void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string start);

void DFSRecursive(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string currentCity, std::set<std::string> &visited);

void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity);

//

bool DLS(const std::map<std::string, std::vector<std::string>>& adjacencyMap, std::string currentCity, std::string goalCity, int limit, std::set<std::string> &visited);

bool IDDFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity, int maxDepth);

#endif // SEARCHFUNCTIONS_H
