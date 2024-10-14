#include "FileReading.h"

void readAdjacencies(std::ifstream &in, std::map<std::string, std::vector<std::string>> &adjacencyMap) {
    std::string readString;
    std::string city1, city2;

    while(std::getline(in, readString)) {
        std::stringstream ss(readString);

        ss >> city1 >> city2;

        // Should be bidirectional, so add to both
        adjacencyMap[city1].push_back(city2);
        adjacencyMap[city2].push_back(city1);
    }
    return;
}

void printAdjacencies(std::map<std::string, std::vector<std::string>> &adjacencyMap) {
    for(const auto &entry : adjacencyMap) {
        std::cout << entry.first << ": ";
        for(const auto &adjacentCity : entry.second) {
            std::cout << adjacentCity << " ";
        }
        std::cout << std::endl;
    }
}

std::map<std::string, std::vector<std::string>> fillAdjacencies(std::string filename = "Adjacencies.txt") {
    std::map<std::string, std::vector<std::string>> adjacencyMap;
    try {
        std::ifstream in;
        in.open(filename);

        // Failure to open file
        if(!in.good()) {
            throw std::runtime_error("Failure opening file.");
        }

        readAdjacencies(in, adjacencyMap);
        in.close();
    }
    catch (std::runtime_error &e) {
        std::cout << e.what() << std::endl;
    }
    return adjacencyMap;
}

void readCoordinates(std::ifstream &in, std::map<std::string, std::pair<double, double>> &coordinateMap) {
    std::string readString;
    std::string city;
    double xCoord, yCoord;

    while(std::getline(in, readString)) {
        std::stringstream ss(readString);
        std::string tempString;
        // Not pretty, but fuck I forgot how to read from a csv with stringstream and I'm tired
        std::getline(ss, city, ',');
        std::getline(ss, tempString, ',');
        xCoord = std::stod(tempString);
        std::getline(ss, tempString, ',');
        yCoord = std::stod(tempString);

        coordinateMap[city] = std::make_pair(xCoord, yCoord);
    }
    return;
}

void printCoordinates(std::map<std::string, std::pair<double, double>> &coordinateMap) {
    for(const auto &entry : coordinateMap) {
        std::cout << entry.first << ": " << entry.second.first << " " << entry.second.second << std::endl;
    }
}

std::map<std::string, std::pair<double, double>> fillCoordinates(std::string filename = "coordinates.csv") {
    std::map<std::string, std::pair<double, double>> coordinateMap;
    try {
        std::ifstream in;
        in.open(filename);

        // Failure to open file
        if(!in.good()) {
            throw std::runtime_error("Failure opening file.");
        }

        readCoordinates(in, coordinateMap);
        in.close();
    }
    catch (std::runtime_error &e) {
        std::cout << e.what() << std::endl;
    }
    return coordinateMap;
}
