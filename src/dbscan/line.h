#ifndef LINE_HPP__
#define LINE_HPP__

#include <eigen3/Eigen/Geometry>
#include <sstream>

// Describes the Line as all the points (p) that fulfills v*p = d
class Line {
public:
  // Basic data: norm vector and distance to origin
  Eigen::Vector2d v;
  double d;
  
  //! @brief Default constructor
  Line();
  //! @brief Recommended constructor
  Line(const Eigen::Vector2d v_, double d_);
  
  //! @brief Returns the distance from the point v_ to the Line
  //! @return The distance from v_ to the Line
  double distance(const Eigen::Vector2d v_) const;
  
  virtual std::string toString() const;
  
  void makeDPositive();
};

#endif