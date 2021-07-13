#include "dbscan_lines.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include "detected_line.hpp"
#include <queue>

using namespace std;
default_random_engine DBSCANLines::m_generator;

DBSCANLines::DBSCANLines(unsigned int minPts, float eps, std::vector<Point> &points)  {
    m_minPoints = minPts;
    m_epsilon = eps;
    m_points = points;
    m_pointSize = points.size();
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

    DetectedLine curr_line;
    std::queue<int> q;
    std::vector<int> curr_region;
    
    if (nearest > 0) 
    {
      m_points[nearest].clusterID = region_id;
      m_points[candidate].clusterID = region_id;
      addPixelToRegion(candidate, region_id);
      addPixelToRegion(nearest, region_id);
      
      // Initialize matrices and vectors
      Eigen::Vector3d r_1(m_points[candidate].x, m_points[candidate].y, m_points[candidate].z);
      Eigen::Vector3d r_2(m_points[nearest].x, m_points[nearest].y, m_points[nearest].z);
      curr_line.s_g = r_1 + r_2;
      curr_line.m_k = (r_1 - curr_line.s_g * 0.5)*(r_1 - curr_line.s_g * 0.5).transpose();
      curr_line.m_k += (r_2 - curr_line.s_g * 0.5)*(r_2 - curr_line.s_g * 0.5).transpose();
      curr_line.p_k = r_1 * r_1.transpose() + r_2*r_2.transpose();
      curr_line.n_points = 2;
      
      while (!q.empty()) 
      {
        int new_point = q.front();
        q.pop();
        Eigen::Vector3d v(m_points[new_point].x, m_points[new_point].y, m_points[candidate].z);
        if (updateMatrices(v)) {
          addPixelToRegion(new_point, region_id);
        } else {
          m_available_points--;
        }
      }
      
      // The queue has been emptied --> clear possible QUEUE status and add the region to the detected planes if condition of step 12 (Algorithm 1)
      if (curr_line.n_points > m_minPoints) {
        curr_line.makeDPositive();
//         std::cout << "Detected plane: " << _curr_plane.toString() << std::endl;
        m_detected_lines.push_back(curr_line);
      }
    } else {
      // No nearest neighbor available --> discard (to R_PRIMA)
      m_points[candidate].clusterID = (int)FAILURE;
      m_available_points--;
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


