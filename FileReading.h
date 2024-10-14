#ifndef FILEREADING_H
#define FILEREADING_H

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>

// PRE: A file stream, empty map
// POST: Read from the file and load the map
void readAdjacencies(std::ifstream &in, std::map<std::string, std::vector<std::string>> &adjacencyMap);

// PRE: A filled map of the adjacent cities
// POST: Display each city and their list of adjacent cities, each on a new line
void printAdjacencies(std::map<std::string, std::vector<std::string>> &adjacencyMap);

// PRE: A filename of the input file
// POST: Return the filled map
std::map<std::string, std::vector<std::string>> fillAdjacencies(std::string filename);

//


// PRE: A file stream, empty map
// Post: read from the file and load the map
void readCoordinates(std::ifstream &in, std::map<std::string, std::pair<double, double>> &coordinateMap);

// PRE: A filled map of the city coordinates
// POST: Display each city and their coordinates
void printCoordinates(std::map<std::string, std::pair<double, double>> &coordinateMap);

// PRE: A filename of the input file
// POST: Return the filled map
std::map<std::string, std::pair<double, double>> fillCoordinates(std::string filename);

#endif // FILEREADING_H
