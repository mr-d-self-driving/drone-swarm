#ifndef COORDINATE3D_H_
#define COORDINATE3D_H_

#include "Vector3D.h"

class Coordinate3D {
  public:
    Coordinate3D(); // Empty constructor
    Coordinate3D(float xIn, float yIn, float zIn); // Intializing constructor
    Coordinate3D(const Coordinate3D& copy); // Copy constructor

    Vector3D* operator-(const Coordinate3D& lhs, const Coordinate3D& rhs); // Returns a vector pointing towards the rhs coordinate
    bool operator==(const Coordinate3D& lhs, const Coordinate3D& rhs);
    bool operator!=(const Coordinate3D& lhs, const Coordinate3D& rhs);
    Coordinate3D* operator+(Coordinate3D& lhs, const Vector3D& rhs);

    inline void setX(double input) { x = input; }
    inline void setY(double input) { y = input; }
    inline void setZ(double input) { z = input; }

    inline double X() const { return x; } // does not change any member vars
    inline double Y() const { return y; }
    inline double Z() const { return z; }

  private:
    double x, y, z;
};

#endif // COORDINATE3D_H_
