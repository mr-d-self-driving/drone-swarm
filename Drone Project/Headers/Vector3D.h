#pragma once
#include <cmath>
#include <vector>

using std::vector;
using std::iterator;

class Vector3D
{
private:
	double x, y, z;
public:
	//Constructor w/ initialization
	Vector3D(double xIn, double yIn, double zIn);

	//Empty constructor
	Vector3D();

	//Copy constructor
	Vector3D(const Vector3D& copy);

	double Dot(Vector3D rhs);

	double Magnitude();

	double X() const;

	double Y() const;

	double Z() const;

    void setX(double input);

	void setY(double input);

	void setZ(double input);

	Vector3D* UnitVector();

	Vector3D* operator/(double rhs);
	//Rotate the vector in the z axis 'angle' degrees
	Vector3D* RotateZ(double angle);
};

vector<Vector3D>* Repmat(Vector3D *vectorToRepeat, int timesToRepeat);
