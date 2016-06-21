#include <cmath>
#include <vector>
#include "Vector3D.h"

// Constructor w/ initialization
Vector3D::Vector3D(double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
}


// Copy constructor
Vector3D::Vector3D(const Vector3D& copy) {
  this->x = copy.x;
  this->y = copy.y;
  this->z = copy.z;
}

double Vector3D::Dot(const Vector3D &rhs) const {
  return ((x * rhs.x) + (y * rhs.y) + (z * rhs.z));
}

double Vector3D::Magnitude() const {
  return (std::sqrt((x * x) + (y * y) + (z * z)));
}

std::vector<Vector3D> Vector3D::Repmat(Vector3D* vectorToRepeat, int timesToRepeat) {
  std::vector<Vector3D> result(timesToRepeat);

  for (auto iter = result.begin(); iter != result.end(); iter++) {
    *iter = *vectorToRepeat;
  }

  return result;
}

// Returns a unit vector of the given vector
Vector3D Vector3D::UnitVector() {
  return Vector3D((*this).Scale(1/this->Magnitude()));
}

Vector3D Vector3D::RotateZ(double angle) {
  const double DegreeConversion = 3.14159265 / 180;
  // cos(theta), -sin(theta), 0
  // sin(theta), cos(theta), 0
  // 0, 0, 1
  float x, y, z;
  x = std::cos(angle * DegreeConversion);
  y = -std::sin(angle * DegreeConversion);
  z = 0;
  Vector3D rotationMatrixRowOne = Vector3D(x, y, z);
  x = std::sin(angle * DegreeConversion);
  y = std::cos(angle * DegreeConversion);
  z = 0;
  Vector3D rotationMatrixRowTwo = Vector3D(x, y, z);
  Vector3D rotationMatrixRowThree = Vector3D(0.0, 0.0, 1.0);

  x = rotationMatrixRowOne.Dot(*this);  // dot of row one
  y = rotationMatrixRowTwo.Dot(*this);  // dot of row two
  z = rotationMatrixRowThree.Dot(*this);  // dot of row three

  return Vector3D(x, y, z);
}


Vector3D Vector3D::Project(const Vector3D &onto) {
  double scalar = this->Dot(onto) / std::pow(onto.Magnitude(), 2);
  return Vector3D(onto.getX() * scalar, onto.getY() * scalar, onto.getZ() * scalar);
}

Vector3D Vector3D::Scale(double rhs) {
  return Vector3D(getX() * rhs, getY() * rhs, getZ() * rhs);
}

Vector3D operator-(const Vector3D& lhs, const Vector3D& rhs) {
  return Vector3D(lhs.getX() - rhs.getX, lhs.getY() - rhs.getY(), lhs.getZ() - rhs.getZ());
}

// Too precise, need abs
bool operator==(const Vector3D& lhs, const Vector3D& rhs) {
  return std::abs((lhs.getX() - rhs.getX()) < 1) && (std::abs(lhs.getY() - rhs.getY()) < 1) &&
    (std::abs(lhs.getZ() - rhs.getZ()) < 1);
}

bool operator!=(const Vector3D& lhs, const Vector3D& rhs) {
  return !(lhs == rhs);
}

Vector3D operator+(const Vector3D& lhs, const Vector3D& rhs) {
  return Vector3D(lhs.getX() + rhs.getX, lhs.getX() + rhs.getY(), lhs.getZ() + rhs.getZ());
}