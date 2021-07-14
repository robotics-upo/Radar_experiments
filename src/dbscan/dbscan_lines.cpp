#include "dbscan_lines.h"
#include <eigen3/Eigen/Eigenvalues>
#include "detected_line.hpp"
#include <queue>

using namespace std;
default_random_engine DBSCANLines::m_generator;

DBSCANLines::DBSCANLines(unsigned int minPts, float eps, std::vector<Point> &points):DBSCAN(minPts, eps, points)  {
}

int DBSCANLines::run()
{
    int clusterID = 1;
    
    // First try to detect lines
    m_available_points = m_points.size();
    while (m_available_points > 0)
    {
        int p = getRandomPoint();
        if ( m_points[p].clusterID == UNCLASSIFIED )
        {
            if ( getLine(p, clusterID) != FAILURE )
            {
                clusterID++;
                m_n_lines++;
            }
        }
    }

    // Then detect groups of unclassified data as a regular DBScan
    vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter)
    {
        if ( iter->clusterID == UNCLASSIFIED )
        {
            if ( expandCluster(*iter, clusterID) != FAILURE )
            {
                clusterID += 1;
            }
        }
    }

    return clusterID;
}

 int DBSCANLines::getLine(int candidate, int region_id) {
    int nearest = getNearestNeighbor(candidate);

    std::vector<int> curr_region;
    
    if (nearest > 0) 
    {
      m_points[nearest].clusterID = region_id;
      m_points[candidate].clusterID = region_id;
      addPixelToRegion(candidate, region_id);
      addPixelToRegion(nearest, region_id);
      
      // Initialize matrices and vectors
      Eigen::Vector2d r_1(m_points[candidate].x, m_points[candidate].y);
      Eigen::Vector2d r_2(m_points[nearest].x, m_points[nearest].y);
      m_curr_line.s_g = r_1 + r_2;
      m_curr_line.m_k = (r_1 - m_curr_line.s_g * 0.5)*(r_1 - m_curr_line.s_g * 0.5).transpose();
      m_curr_line.m_k += (r_2 - m_curr_line.s_g * 0.5)*(r_2 - m_curr_line.s_g * 0.5).transpose();
      m_curr_line.p_k = r_1 * r_1.transpose() + r_2*r_2.transpose();
      m_curr_line.n_points = 2;
      
      while (!m_q.empty()) 
      {
        int new_point = m_q.front();
        m_q.pop();
        Eigen::Vector2d v(m_points[new_point].x, m_points[new_point].y);
        if (updateMatrices(v)) {
          addPixelToRegion(new_point, region_id);
        } else {
          m_available_points--;
        }
      }
      
      // The queue has been emptied --> clear possible QUEUE status and add the region to the detected planes if condition of step 12 (Algorithm 1)
      if (m_curr_line.n_points > m_minPoints) {
        m_curr_line.makeDPositive();
//         std::cout << "Detected line: " << m_curr_line.toString() << std::endl;
        m_detected_lines.push_back(m_curr_line);
      }
    } else {
      // No nearest neighbor available --> discard (to R_PRIMA)
      m_points[candidate].clusterID = (int)FAILURE;
      m_available_points--;
      return -1;
    }
    return region_id;
 }

void DBSCANLines::addPixelToRegion(int index, int _curr_region_id)
{
  m_points[index].clusterID = _curr_region_id;

  Eigen::Vector2d v = get2DPoint(index);
  m_available_points--;
  
  int neighbor = getNearestNeighbor(index);
  while (neighbor >= 0) {
    Eigen::Vector2d v_2 = get2DPoint(neighbor);
    if ( (v-v_2).norm() < m_epsilon) { // First check --> the neighbor is sufficiently near to the point
      m_points[neighbor].clusterID = (int)IN_QUEUE;
      m_q.push(neighbor);
      neighbor = getNearestNeighbor(index);
    } else {
      neighbor = -1;
    }
  }
}

int DBSCANLines::getRandomPoint() const {
  std::uniform_int_distribution<int> distribution(0, m_points.size());
  int ret_val;
  
  do {
    ret_val = distribution(m_generator);
  } while (m_points[ret_val].clusterID != UNCLASSIFIED);
  
  return ret_val;
}

//! Gets the unprocessed nearest neighbor of a pixel in a window of size 9
int DBSCANLines::getNearestNeighbor(int index) const
{
  int near = -1;
  int aux;
  float min_dist = 1e10;
  
  Eigen::Vector2d v = get2DPoint(index);
  Eigen::Vector2d v_;
  for (int i = -1; i < 2; i+=2) {
    aux = index + i;
  
    if (aux > 0 && aux < m_points.size()) { // Check bounds
      if (m_points[aux].clusterID == (int)UNCLASSIFIED) {
        v_ = get2DPoint(aux);
        double dist = (v - v_).norm();
        if (dist < min_dist) {
          near = aux;
          min_dist = dist;
        }
      }
    }
  }
  return near;
}

bool DBSCANLines::updateMatrices(const Eigen::Vector2d& v)
{
  size_t k = m_curr_line.n_points;
  double div_1 = 1.0 / (double)(k + 1);
  double div = 1.0 / (double)(k);

  DetectedLine _p; // Candidate line
  
  _p.s_g = m_curr_line.s_g + v;
  _p.m_k = m_curr_line.m_k + v*v.transpose() - (_p.s_g * div_1)*_p.s_g.transpose() + (m_curr_line.s_g * div) * m_curr_line.s_g.transpose();
  _p.p_k = m_curr_line.p_k + v*v.transpose();
  
  // Calculate d and n (n is the eigenvector related to the lowest eigenvalue)
  
  m_es.compute(_p.m_k);
  double min_eigenval = 1e10;
  int min_i;
  for (int i = 0; i < 2; i++) {
    double curr_eigen = fabs(m_es.eigenvalues()(i));
    if (curr_eigen < min_eigenval) {
      min_eigenval = curr_eigen;
      min_i = i;
    }
    
  }
  _p.v = m_es.eigenvectors().col(min_i);
  _p.d = div_1 * _p.v.dot(_p.s_g);
  
  // Update the MSE (Eq 3)
  _p.mse = div_1 * _p.v.transpose() * _p.p_k * _p.v - 2 * _p.v.dot(_p.s_g) * _p.d * div_1 + _p.d * _p.d;
  _p.n_points = k + 1;
  
  
  // Check if the new plane meets the constraint of algorithm 1 step 8. If so, update the values of the matrices of the class
  if (_p.mse < m_epsilon && _p.distance(v) < m_epsilon) 
  {
    // Meets the constraints --> Actualize the plane
    _p.r_g = _p.s_g * div_1;
    m_curr_line = _p;
    
    return true;
  }
  return false;
}
