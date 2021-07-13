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
  Line() {
    v(0) = v(1) = d = 0.0;
  }
  
  //! @brief Recommended constructor
  Line(const Eigen::Vector2d v_, double d_):v(v_),d(d_) {
    
  }
  
  //! @brief Returns the distance from the point v_ to the Line
  //! @return The distance from v_ to the Line
  double distance(const Eigen::Vector2d v_) const;
  
  virtual std::string toString() const;
  
  void makeDPositive();
  
};

double Line::distance(const Eigen::Vector2d v_) const
{
  return fabs(v_.dot(v) - d); 
}

std::string Line::toString() const
{
  std::ostringstream os;
  
  os << "n = " << v.transpose() << "\t d = " << d;
  
  return os.str();
}

void Line::makeDPositive()
{
  if (d < 0.0) {
    d *= -1.0;
    v *= -1.0;
  }
}

#endif