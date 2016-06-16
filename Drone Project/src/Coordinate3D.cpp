#include "Vector3D.h"
#include "Coordinate3D.h"

using std::abs;

Coordinate3D::Coordinate3D()
{
	x = y = z = 0;
}

Coordinate3D::Coordinate3D(float xIn, float yIn, float zIn)
{
	x = xIn;
	y = yIn;
	z = zIn;
}

Coordinate3D::Coordinate3D(const Coordinate3D& copy)
{
	x = copy.x;
	y = copy.y;
	z = copy.z;
}

double Coordinate3D::X() const
{
	return x;
}
double Coordinate3D::Y() const
{
	return y;
}
double Coordinate3D::Z() const
{
	return z;
}

void Coordinate3D::setX(double input)
{
	x = input;
}
void Coordinate3D::setY(double input)
{
	y = input;
}
void Coordinate3D::setZ(double input)
{
	z = input;
}

Vector3D* operator-(const Coordinate3D& lhs, const Coordinate3D& rhs)
{
	float x = lhs.X() - rhs.X();
	float y = lhs.Y() - rhs.Y();
	float z = lhs.Z() - rhs.Z();
	Vector3D* vector = new Vector3D(x, y, z);
	return vector;
}

//Too precise, need abs
bool operator==(const Coordinate3D& lhs, const Coordinate3D& rhs)
{
	if (abs((lhs.X() - rhs.X()) < 1) && (abs(lhs.Y() - rhs.Y()) < 1) && (abs(lhs.Z() - rhs.Z()) < 1))
		return true;
	else
		return false;
}

bool operator!=(const Coordinate3D& lhs, const Coordinate3D& rhs)
{
	return !(lhs == rhs);
}

Coordinate3D* operator+(Coordinate3D& lhs, const Vector3D& rhs)
{
	lhs.setX(lhs.X() + rhs.X());
	lhs.setY(lhs.Y() + rhs.Y());
	lhs.setZ(lhs.Z() + rhs.Z());
	return &lhs;
}
