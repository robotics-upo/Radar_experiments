#ifndef DBSCAN_LINES_H
#define DBSCAN_LINES_H

#include <vector>
#include <cmath>
#include <random>
#include "dbscan.h"
#include "detected_line.hpp"

#define UNCLASSIFIED_LINES -4

// Adds line detection to DBScan clustering method, 
class DBSCANLines:public DBSCAN {
public:    
    DBSCANLines(unsigned int minPts, float eps, std::vector<Point> &points);

    ~DBSCANLines(){}

    virtual int run();
    int getLine(int candidate, int clusterID);

    int getRandomPoint() const; 
    
protected:
    int m_n_lines = 0;
    int m_available_points = -1;
    std::vector<DetectedLine> m_detected_lines;
    // For random numbers
    static std::default_random_engine m_generator;
};

#endif // DBSCAN_LINES_H
