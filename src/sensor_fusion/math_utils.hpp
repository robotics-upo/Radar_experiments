#pragma once

#include <pcl/point_types.h>

template <typename T>
double dist2Origin(const T &point)
    {
        return sqrtf(point.x * point.x + point.y * point.y + point.z * point.z);
    }
    template <typename T>
    double pointYaw(const T &point)
    {
        if (point.y == 0.0 && point.x == 0)
            return 0;

        return atan2(point.y, point.x);
    }
    template <typename T>
    void pointToSpherical(const T &_point, double &_rho, double &_theta, double &_phi){
        _theta = atan2(_point.y, _point.x);
        _rho   = dist2Origin(_point);
        _phi   = acos(_point.z/_rho);
    }
    template <typename T=pcl::PointXYZI>
    T sphericalToXY(const double &_rho, const double &_theta, const double &_phi, bool z_zero = true){
        T p;
        p.x = _rho * sin(_phi) * cos(_theta);
        p.y = _rho * sin(_phi) * sin(_theta);
        if(!z_zero)
            p.z = _rho * cos(_theta);

        return p;
    }