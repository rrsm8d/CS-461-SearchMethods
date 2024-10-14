cmake_minimum_required(VERSION 3.5)

project(CS-461-SearchMethods LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(CS-461-SearchMethods main.cpp
    FileReading.h
    FileReading.cpp
    coordinates.csv
    Adjacencies.txt
    SearchFunctions.h
    SearchFunctions.cpp)

include(GNUInstallDirs)
install(TARGETS CS-461-SearchMethods
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
