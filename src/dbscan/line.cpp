#include "line.h"

Line::Line() {
  v(0) = v(1) = d = 0.0;
}
  
//! @brief Recommended constructor
Line::Line(const Eigen::Vector2d v_, double d_):v(v_),d(d_) {
  
}
  
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
