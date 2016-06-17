#include "Vector3D.h"
#include "Coordinate3D.h"

Coordinate3D::Coordinate3D() { x = y = z = 0; }

Coordinate3D::Coordinate3D(float xIn, float yIn, float zIn) {
  x = xIn;
  y = yIn;
  z = zIn;
}

Coordinate3D::Coordinate3D(const Coordinate3D& copy) {
  x = copy.x;
  y = copy.y;
  z = copy.z;
}

Vector3D* Coordinate3D::operator-(const Coordinate3D& lhs, const Coordinate3D& rhs) {
  float x = lhs.X() - rhs.X();
  float y = lhs.Y() - rhs.Y();
  float z = lhs.Z() - rhs.Z();
  Vector3D* vector = new Vector3D(x, y, z);
  return vector;
}

// Too precise, need abs
bool Coordinate3D::operator==(const Coordinate3D& lhs, const Coordinate3D& rhs) {
  if (std::abs((lhs.X() - rhs.X()) < 1) && (std::abs(lhs.Y() - rhs.Y()) < 1) &&
      (std::abs(lhs.Z() - rhs.Z()) < 1))
    return true;
  else
    return false;
}

bool Coordinate3D::operator!=(const Coordinate3D& lhs, const Coordinate3D& rhs) {
  return !(lhs == rhs);
}

Coordinate3D* Coordinate3D::operator+(Coordinate3D& lhs, const Vector3D& rhs) {
  lhs.setX(lhs.X() + rhs.X());
  lhs.setY(lhs.Y() + rhs.Y());
  lhs.setZ(lhs.Z() + rhs.Z());
  return &lhs;
}
