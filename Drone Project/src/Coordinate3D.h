#pragma once
#include "Vector3D.h"

class Coordinate3D {
 private:
  double x, y, z;

 public:
  // Empty constructor
  Coordinate3D();
  // Intializing constructor
  Coordinate3D(float xIn, float yIn, float zIn);
  // Copy constructor
  Coordinate3D(const Coordinate3D& copy);

  // Private variable functions
  double X() const;
  double Y() const;
  double Z() const;

  void setX(double input);
  void setY(double input);
  void setZ(double input);
};

// Returns a vector pointing towards the rhs coordinate
Vector3D* operator-(const Coordinate3D& lhs, const Coordinate3D& rhs);

bool operator==(const Coordinate3D& lhs, const Coordinate3D& rhs);

bool operator!=(const Coordinate3D& lhs, const Coordinate3D& rhs);

Coordinate3D* operator+(Coordinate3D& lhs, const Vector3D& rhs);
