#ifndef DETECTED_LINE_HPP__
#define DETECTED_LINE_HPP__

#include "line.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>
class DetectedLine:public Line 
{
public:
  // Detecting data
  Eigen::Vector2d r_g; // Center of gravity
  // Estimating the Line
  Eigen::Vector2d s_g; // Cumulative sum of points (r_g = s_k/n_points)
  Eigen::Matrix2d m_k; // Matrix to estimate n and v (n = eigen vector related to minimum eigenval)
  Eigen::Matrix2d p_k; // P_k = sum(v_k*v_k')
  Eigen::Matrix2d S; // Scatter matrix (filled by Line detector)
  double mse; // Minimum square error of estimation
  int n_points; // Number of points 
  double weight;
  
  DetectedLine affine(const Eigen::Matrix2d &rot, const Eigen::Vector2d &trans) const;
  
  DetectedLine affine(const Eigen::Affine2d &T) const;
  
  DetectedLine();
  
  inline void init();
  
  virtual std::string toString(bool verbose = false) const;
};

void printLines(const std::vector<DetectedLine> &p);

#endif