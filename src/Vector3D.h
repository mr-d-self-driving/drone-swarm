#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include <vector>

class Vector3D {
 public:
  Vector3D() {} // Empty constructor
  Vector3D(double x, double y, double z); // Constructor w/ initialization
  Vector3D(const Vector3D& copy); // Copy constructor

  double Dot(const Vector3D &rhs) const;
  double Magnitude() const;
  Vector3D Scale(double scalar);
  Vector3D UnitVector();
  Vector3D RotateZ(double angle);  // Rotate the vector in the z axis 'angle' degrees
  Vector3D Project(const Vector3D &onto);

  static std::vector<Vector3D> Repmat(Vector3D* vectorToRepeat, int timesToRepeat);

  inline void setX(double x) { this->x = x; }
  inline void setY(double y) { this->y = y; }
  inline void setZ(double y) { this->z = z; }
  inline double getX() const { return x; } // does not change any member vars
  inline double getY() const { return y; }
  inline double getZ() const { return z; }

 private:
   double x = 0;
   double y = 0;
   double z = 0;
};

Vector3D operator-(const Vector3D &lhs, const Vector3D &rhs); // Returns a vector pointing towards the rhs coordinate
bool operator==(const Vector3D &lhs, const Vector3D &rhs);
bool operator!=(const Vector3D &lhs, const Vector3D &rhs);
Vector3D operator+(const Vector3D &lhs, const Vector3D &rhs);

#endif
