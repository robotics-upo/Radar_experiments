#ifndef DBSCAN_LINES_H
#define DBSCAN_LINES_H

#include <vector>
#include <cmath>
#include <random>
#include <queue>
#include "dbscan.h"
#include "detected_line.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#define UNCLASSIFIED_LINES -4
#define IN_QUEUE -5

// Adds line detection to DBScan clustering method, 
class DBSCANLines:public DBSCAN {
public:    
    DBSCANLines(unsigned int minPts, float eps, std::vector<Point> &points, double gamma = 0.1, double theta = 0.01);

    ~DBSCANLines(){}

    virtual int run();
    int getLine(int candidate, int clusterID);

    int getRandomPoint() const;

    inline Eigen::Vector2d get2DPoint(int index) const {
            Eigen::Vector2d pt;
            pt[0] = m_points[index].x;
            pt[1] = m_points[index].y;
            return pt;
         }
    

    int m_n_lines = 0;
protected:
    int m_available_points = -1;
    std::vector<DetectedLine> m_detected_lines;
    DetectedLine m_curr_line;
    std::queue<int> m_q;
    double m_gamma, m_theta;
    // For random numbers
    static std::default_random_engine m_generator;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> m_es;

    int getNearestNeighbor(int index) const;

    void addPixelToRegion(int index, int _curr_region_id);

    bool updateMatrices(const Eigen::Vector2d& v);


};

#endif // DBSCAN_LINES_H
