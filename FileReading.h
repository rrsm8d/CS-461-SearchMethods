#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>

void readAdjacencies(std::ifstream &in, std::map<std::string, std::vector<std::string>> &adjacencyMap);
void printAdjacencies(std::map<std::string, std::vector<std::string>> &adjacencyMap);
std::map<std::string, std::vector<std::string>> fillAdjacencies(std::string filename);


void readCoordinates(std::ifstream &in, std::map<std::string, std::pair<double, double>> &coordinateMap);
void printCoordinates(std::map<std::string, std::pair<double, double>> &coordinateMap);
std::map<std::string, std::pair<double, double>> fillCoordinates(std::string filename);
