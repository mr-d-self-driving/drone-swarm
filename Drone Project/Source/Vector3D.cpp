#include <cmath>
#include <vector>
#include "Vector3D.h"

using std::vector;
using std::iterator;
using std::sin;
using std::cos;
using std::pow;

#define PI 3.14159265
#define DegreeConversion (PI/180)

	//Constructor w/ initialization
	Vector3D::Vector3D(double xIn, double yIn, double zIn)
	{
		x = xIn;
		y = yIn;
		z = zIn;
	}

	//Empty constructor
	Vector3D::Vector3D()
	{
		x = y = z = 0;
	}

	//Copy constructor
	Vector3D::Vector3D(const Vector3D& copy)
	{
		x = copy.x;
		y = copy.y;
		z = copy.z;
	}

	double Vector3D::Dot(Vector3D *rhs)
	{
		return ((x * rhs->x) + (y * rhs->y) + (z * rhs->z));
	}

	double Vector3D::Magnitude()
	{
		return (std::sqrt((x * x) + (y * y) + (z * z)));
	}

vector<Vector3D>* Repmat(Vector3D *vectorToRepeat, int timesToRepeat)
{
	vector<Vector3D>* result = new vector<Vector3D>(timesToRepeat);

	for (auto iter = result->begin(); iter != result->end(); iter++)
	{
		*iter = *vectorToRepeat;
	}

	return result;
}

//Divides each element of the vector by a scalar value
Vector3D* Vector3D::operator/(double rhs)
{
	float newX = (x / rhs);
	float newY = (y / rhs);
	float newZ = (z / rhs);
	Vector3D *result = new Vector3D(newX, newY, newZ);
	return result;
}

//Returns a unit vector of the given vector
Vector3D* Vector3D::UnitVector()
{
	Vector3D *result = new Vector3D(*(*this / this->Magnitude()));
	return result;
}

Vector3D* Vector3D::RotateZ(double angle)
{
	Vector3D *rotationMatrixRowOne, *rotationMatrixRowTwo, *rotationMatrixRowThree;
	//cos(theta), -sin(theta), 0
	//sin(theta), cos(theta), 0
	//0, 0, 1
	float x, y, z;
	x = cos(angle * DegreeConversion);
	y = -sin(angle * DegreeConversion);
	z = 0;
	rotationMatrixRowOne = new Vector3D(x, y, z);
	x = sin(angle * DegreeConversion);
	y = cos(angle * DegreeConversion);
	z = 0;
	rotationMatrixRowTwo = new Vector3D(x, y, z);
	rotationMatrixRowThree = new Vector3D(0.0, 0.0, 1.0);

	x = rotationMatrixRowOne->Dot(this);//dot of row one
	y = rotationMatrixRowTwo->Dot(this);//dot of row two
	z = rotationMatrixRowThree->Dot(this);//dot of row three

	Vector3D *result = new Vector3D(x, y, z);

	return result;
}

double Vector3D::X() const
{
	return x;
}

double Vector3D::Y() const
{
	return y;
}

double Vector3D::Z() const
{
	return z;
}

void Vector3D::setX(double input)
{
    x = input;
}

void Vector3D::setY(double input)
{
    y = input;
}

void Vector3D::setZ(double input)
{
    z = input;
}

Vector3D* Projection(Vector3D *of, Vector3D *onto)
{
    double aDotb = of->Dot(onto);
    double magASquared = pow(onto->Magnitude(), 2);
    double scalar = aDotb / magASquared;
    return new Vector3D(onto->X() * scalar, onto->Y() * scalar, onto->Z());
}

Vector3D* Vector3D::operator*(double rhs)
{
    return new Vector3D(this->X() * rhs, this->Y() * rhs, this->Z() * rhs);
}
