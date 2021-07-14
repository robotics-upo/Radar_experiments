#include "detected_line.h"
  
DetectedLine::DetectedLine():Line() {
  init();
}
  
inline void DetectedLine::init() {
  n_points = 0;
  mse = 0.0;
  s_g(0) = s_g(1) = 0.0;
  r_g(0) = r_g(1) = 0.0;
  p_k(0,0) = p_k(0,1) = 0.0;
  p_k(1,0) = p_k(1,1) = 0.0;
}
  
std::string DetectedLine::toString(bool verbose) const
{
  std::ostringstream os;
  
  os << Line::toString() << "\t";
  if (verbose) {
    os << "MSE = " << mse << "\t r_g = " << r_g.transpose();
  }
  
  return os.str();
}

void printLines(const std::vector<DetectedLine> &p) {
  std::cout << "Detected Lines: " << p.size() << std::endl;
  for (unsigned int i = 0; i < p.size(); i++) {
    std::cout << "Line " << i << ": " << p[i].toString(true) << std::endl;
  }
}

// NOTE: If the transform translates coordinates r2 into the transform 1 r1 (r1 = rot*r2 + trans) 
// then this function gives n2 and d2 (in coordinate transform 2) from a Line in transform 1
DetectedLine DetectedLine::affine(const Eigen::Matrix2d& rot_, const Eigen::Vector2d& trans_) const
{
  DetectedLine ret;
  
  ret.v = rot_*v;
  ret.d = d + ret.v.transpose()*trans_;
  
  ret.makeDPositive();
  
  return ret;
}

DetectedLine DetectedLine::affine(const Eigen::Affine2d &T) const
{
  return affine(T.matrix().block<2,2>(0,0), T.matrix().block<2,1>(0,2));
}

