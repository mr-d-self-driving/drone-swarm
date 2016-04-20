#include "Coordinate3D.h"
#include "Vector3D.h"
#include <iostream>

//Enable the demo code
#define DEMO 1
#include "Demo.h"
#ifdef DEMO
#endif // DEMO

using std::cout;
using std::endl;

int main()
{

    #ifdef DEMO
    cout << "Demo detected" << endl;
    Initialize();
    return 0;
    #endif // DEMO

	Coordinate3D *leadDrone, *target;
	leadDrone = new Coordinate3D(49.0317, 50.8399, 100);
	target = new Coordinate3D(550, 560, 100);
	Vector3D *formationVector, *v1, *v2, *unitFormationVector;
	//formationVector = *target - *leadDrone;
	//formationVector = new Vector3D(500.9683, 509.1601, 0);

	//from matlab
	int formationSpeed = 4;

	while (leadDrone != target)
	{
		formationVector = *target - *leadDrone;
		v1 = formationVector->RotateZ((45 / 2) + 180);
		v2 = formationVector->RotateZ(-(45 / 2) + 180);
		unitFormationVector = formationVector->UnitVector();
		leadDrone = *leadDrone + *unitFormationVector;
		//cout << leadDrone->X() << ',' << leadDrone->Y() << ',' << leadDrone->Z() << endl;
		cout << v1->X() << ',' << v1->Y() << ',' << v1->Z() << endl;
	}

	return 0;
}
