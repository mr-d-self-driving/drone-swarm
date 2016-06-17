#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include <cmath>
#include <vector>

class Vector3D {
 public:
  Vector3D(); // Empty constructor
  Vector3D(double xIn, double yIn, double zIn); // Constructor w/ initialization
  Vector3D(const Vector3D& copy); // Copy constructor

  double Dot(Vector3D* rhs);
  double Magnitude();
  Vector3D* UnitVector();
  Vector3D* RotateZ(double angle);  // Rotate the vector in the z axis 'angle' degrees

  static Vector3D* Projection(Vector3D* of, Vector3D* onto);
  static std::vector<Vector3D>* Repmat(Vector3D* vectorToRepeat, int timesToRepeat);

  Vector3D* operator*(double rhs);
  Vector3D* operator/(double rhs);

  inline void Vector3D::setX(double input) { x = input; }
  inline void Vector3D::setY(double input) { y = input; }
  inline void Vector3D::setZ(double input) { z = input; }
  inline double Vector3D::X() const { return x; }
  inline double Vector3D::Y() const { return y; }
  inline double Vector3D::Z() const { return z; }

 private:
  double x, y, z;
};

#endif
