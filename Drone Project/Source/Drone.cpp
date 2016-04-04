//Get coordinates
//Send coordinates

//Get leader votes
//Send leader vote

//Calculate movement towards target
//Calculate movement in (or into) formation

//Decide on formation alignment (which side of the V to move on)

#include "Vector3D.h"
#include <string>
#include <stdlib.h>
#include <math.h>

using std::string;

//Parses the string contained in Net[] so the needed information can be extracted and used when needed.
string ParseNet(string tempStringForParsing, int value_to_return)
{
    string delimiter = ":";
    string token;
    string id, x_coordinate, y_coordinate, z_coordinate, is_alive, battery_life;
    size_t pos = 0;
    int counter = 0;

    //Searches the string and breaks it apart based on the ':' delimiter
    //Stores relevant information in appropriate variable
    while ((pos = tempStringForParsing.find(delimiter) != string::npos))
    {
        token = tempStringForParsing.substr(0,pos);
        if(value_to_return == counter)
        {
            return id;
        }
        else if(value_to_return == counter)
        {
            return token;
        }
        else if(value_to_return == counter)
        {
            return token;
        }
        else if(value_to_return ==counter)
        {
            return token;
        }
        else if(value_to_return == counter)
        {
            return token;
        }
            tempStringForParsing.erase(0, pos + delimiter.length());
            counter++;
    }

    if (value_to_return == 5)
    {
            return tempStringForParsing;
    }

    return "-1";

}

//This just computes the distance of each drone from the target
double ComputeDistances(string Net, Vector3D *target)
{
    Vector3D *coordinate = new Vector3D(strtod(ParseNet(Net,1).c_str(),NULL), strtod(ParseNet(Net,2).c_str(), NULL), strtod(ParseNet(Net,3).c_str(), NULL));

    double result = sqrt(pow((coordinate->X()-target->X()), 2.0) + pow((coordinate->Y()-target->Y()), 2.0) + pow((coordinate->Z()-target->Z()), 2.0));

    return result;
}

//Starts calculating the positions of each drone and where they need to go in order to fly in a V formation
void MoveDronesVFormation(string Net[], Vector3D *formationVector, string leadDrone, double formationDistance, Vector3D *target, int num_drones)
{
    int formationSpeed = 4;
    Vector3D *leadDroneChange = new Vector3D(*(*formationVector / formationDistance));

    Vector3D *v1 = new Vector3D(*(formationVector->RotateZ((45/2)+180)));
    Vector3D *v2 = new Vector3D(*(formationVector->RotateZ(-(45/2)+180)));
    Vector3D *uv1 = new Vector3D(*(*v1/v1->Magnitude()));
    Vector3D *uv2 = new Vector3D(*(*v2/v2->Magnitude()));

    vector<Vector3D> *baseChange = Repmat(formationVector,num_drones);

    Vector3D* leadDroneCoordinates = new Vector3D(strtod(ParseNet(leadDrone,1).c_str(),NULL), strtod(ParseNet(leadDrone,2).c_str(),NULL), strtod(ParseNet(leadDrone,3).c_str(),NULL));
    vector<Vector3D> *leadDroneCoords = Repmat(leadDroneCoordinates, 9);
    vector<Vector3D> *repmatFormationVector = Repmat(formationVector, 9);

    vector<Vector3D> *a = new vector<Vector3D>(9);
    int counter = 0;
    for(auto it = a->begin(); it != a->end();it++)
    {
        Vector3D *temp = new Vector3D(((*leadDroneCoords)[counter]).X() - ((*repmatFormationVector)[counter]).X(), ((*leadDroneCoords)[counter]).Y() - ((*repmatFormationVector)[counter]).Y(), ((*leadDroneCoords)[counter]).Z() - ((*repmatFormationVector)[counter]).Z());
        (*a)[counter] = *temp;
        counter++;
    }

    //determines which side of the lead drone each drone is on
    double sides[num_drones] = {0};
    for(int i = 0; i <= num_drones;i++)
    {
        sides[i] = ((*leadDroneCoords)[i].X() - (*a)[i].X()) * (strtod(ParseNet(Net[i], 2).c_str(), NULL) - (*a)[i].Y()) - ((*leadDroneCoords)[i].Y() - (*a)[i].Y()) * (strtod(ParseNet(Net[i], 1).c_str(), NULL) - (*a)[i].X());
    }

    vector<int> v1folllows;
    vector<int> v2folllows;

    //create two arrays and each one contains all the drones that are on one side of the lead
    for(int i = 0;i < num_drones;i++)
    {
        if(sides[i] < 0)
        {
            v1folllows.push_back(i);
        }
        else
        {
            v2folllows.push_back(i);
        }
    }

    vector<Vector3D> *vformChange = new vector<Vector3D>(num_drones);
    int l1=v1folllows.size();
    int l2=v2folllows.size();

    // Stores vector v1 to the rows of those that follows v1, and same for v2
    // This should..should have the same effect.
    // Maybe there is a better way to do this?
    counter = 0;
    for(auto iter = vformChange->begin(); iter != vformChange->end(); iter++)
    {
        if(counter < l1)
        {
            Vector3D *temp = new Vector3D(*v1);
            (*vformChange)[counter] = *temp;
            counter++;
        }
        else
        {
            Vector3D *temp = new Vector3D(*v2);
            (*vformChange)[counter] = *temp;
            counter++;
        }
    }

    // remakes/resets leadDroneCoords for no apparent reason..?
    leadDroneCoords = Repmat(leadDroneCoordinates, num_drones);

    // matrix of distance between vector to the lead drone
    //
    vector<Vector3D> *vectorToLeadDrone = new vector<Vector3D>(num_drones);
    counter = 0;
    for(auto iter = vectorToLeadDrone->begin(); iter != vectorToLeadDrone->end(); iter++)
    {
        Vector3D *temp = new Vector3D(
                                      ((*leadDroneCoords)[counter]).X() - (strtod(ParseNet(Net[counter], 1).c_str(),NULL)),
                                      ((*leadDroneCoords)[counter]).Y() - (strtod(ParseNet(Net[counter], 2).c_str(),NULL)),
                                      ((*leadDroneCoords)[counter]).Z() - (strtod(ParseNet(Net[counter], 3).c_str(),NULL))
                                      );
        (*vectorToLeadDrone)[counter] = *temp;
        counter++;
    }

    // projVtldFV = (dot(vtld, vformChange, 2)/dot(vformChange, vformChange, 2))* vformChange;
    // projection of vectorToLeadDrone onto vformChange
    vector<Vector3D> *projVectorToLeadDroneFV = new vector<Vector3D>(num_drones);
    counter = 0;
    for(auto iter = projVectorToLeadDroneFV->begin(); iter != projVectorToLeadDroneFV->end(); iter++)
    {
        Vector3D *temp = new Vector3D(
                                      ((*vectorToLeadDrone)[counter]).Dot((*vformChange)[counter]) / ((*vformChange)[counter]).Dot((*vformChange)[counter]) * (*vformChange)[counter].X(),
                                      ((*vectorToLeadDrone)[counter]).Dot((*vformChange)[counter]) / ((*vformChange)[counter]).Dot((*vformChange)[counter]) * (*vformChange)[counter].Y(),
                                      ((*vectorToLeadDrone)[counter]).Dot((*vformChange)[counter]) / ((*vformChange)[counter]).Dot((*vformChange)[counter]) * (*vformChange)[counter].Z()
                                      );
        (*projVectorToLeadDroneFV)[counter] = *temp;
        counter++;
    }

    // projVtldFV(v1follows,1:2) = [projVtldFV(v1follows,2) projVtldFV(v1follows,1)];
    // swaps all x and y for all following v1
    counter = 0;
    for(auto iter = projVectorToLeadDroneFV->begin(); iter != projVectorToLeadDroneFV->end(); iter++)
    {
        if(counter < l1)
        {
            Vector3D *temp = new Vector3D(
                                          (*projVectorToLeadDroneFV)[counter].Y(),
                                          (*projVectorToLeadDroneFV)[counter].X(),
                                          (*projVectorToLeadDroneFV)[counter].Z()
                                          );
            (*projVectorToLeadDroneFV)[counter] = *temp;
            counter++;
        }
        else
        {
            // do nothing
        }
    }


    // matrix droneVector = (leadDroneCoords - projVtldFV) - Net(:,1:3);
    vector<Vector3D> *droneVector = new vector<Vector3D>(num_drones);
    counter = 0;
    for(auto iter = droneVector->begin(); iter != droneVector->end(); iter++)
    {
        Vector3D *temp = new Vector3D(
                                      ((*leadDroneCoords)[counter]).X() - ((*droneVector)[counter]).X() - (strtod(ParseNet(Net[counter], 1).c_str(),NULL)),
                                      ((*leadDroneCoords)[counter]).Y() - ((*droneVector)[counter]).Y() - (strtod(ParseNet(Net[counter], 2).c_str(),NULL)),
                                      ((*leadDroneCoords)[counter]).Z() - ((*droneVector)[counter]).Z() - (strtod(ParseNet(Net[counter], 3).c_str(),NULL))
                                      );
        (*droneVector)[counter] = *temp;
        counter++;
    }

    // declares matrix state changes of numDrones x 3
    // coder.varsize('StateChanges', [num_drones 3]);

    // baseChange is now a unit vector
    counter = 0;
    for(auto iter = baseChange->begin(); iter != baseChange->end(); iter++)
    {
        //(*baseChange)[counter] = *(*baseChange)[counter].UnitVector();
        Vector3D *temp = new Vector3D();
        temp = (*baseChange)[counter].UnitVector();
        (*baseChange)[counter] = *temp;
        counter++;
    }

    // droneVector is now a unit vector
    counter = 0;
    for(auto iter = droneVector->begin(); iter != droneVector->end(); iter++)
    {
        //(*droneVector)[counter] = *(*droneVector)[counter].UnitVector();
        Vector3D *temp = new Vector3D();    vector<Vector3D> *droneVector = new vector<Vector3D>(num_drones);
    counter = 0;
    for(auto iter = droneVector->begin(); iter != droneVector->end(); iter++)
    {
        Vector3D *temp = new Vector3D(
                                      ((*leadDroneCoords)[counter]).X() - ((*droneVector)[counter]).X() - (strtod(ParseNet(Net[counter], 1).c_str(),NULL)),
                                      ((*leadDroneCoords)[counter]).Y() - ((*droneVector)[counter]).Y() - (strtod(ParseNet(Net[counter], 2).c_str(),NULL)),
                                      ((*leadDroneCoords)[counter]).Z() - ((*droneVector)[counter]).Z() - (strtod(ParseNet(Net[counter], 3).c_str(),NULL))
                                      );
        (*droneVector)[counter] = *temp;
        counter++;
    }
        temp = (*droneVector)[counter].UnitVector();
        (*droneVector)[counter] = *temp;
        counter++;
    }

    // StateChanges = combines moving in formationVector direction with moving into formation
    //StateChanges = (baseChange(1:num_drones,1:3) * formationSpeed) + droneVector(1:num_drones,1:3);
    vector<Vector3D> *StateChanges = new vector<Vector3D>(num_drones);
    counter = 0;
    for(auto iter = StateChanges->begin(); iter != StateChanges->end(); iter++)
    {
        Vector3D *temp = new Vector3D(
                                      ((*baseChange)[counter]).X() * formationSpeed + ((*droneVector)[counter]).X(),
                                      ((*baseChange)[counter]).Y() * formationSpeed + ((*droneVector)[counter]).Y(),
                                      ((*baseChange)[counter]).Z() * formationSpeed + ((*droneVector)[counter]).Z()
                                      );
        (*StateChanges)[counter] = *temp;
        counter++;
    }

/*******************************************************************************************
// TODO
// CollisionAvoidence:
********************************************************************************************/
    vector<Vector3D> *collisionAvoidenceVector = new vector<Vector3D>(num_drones);
    // %collisionAvoidenceVector = avoidCollisions(Net, StateChanges, num_drones);
    // we pass in StateChanges here, aka what we want the drones to do
    // %collisionAvoidenceVector = rdivide(collisionAvoidenceVector,repmat(magnitudes(collisionAvoidenceVector), [1 3]));
    // make collisionAvoidance a unit vector



    // same as before but include collision avoidance movement
    // StateChanges = combines moving in formationVector direction with moving into formation
    // as well as include the collision avoidance movement.
    // StateChanges =
    //                  (baseChange(1:num_drones,1:3) * formationSpeed)
    //                  + droneVector(1:num_drones,1:3)
    //                  + collisionAvoidenceVector(1:num_drones, 1:3);
    counter = 0;
    for(auto iter = StateChanges->begin(); iter != StateChanges->end(); iter++)
    {
        Vector3D *temp = new Vector3D(
                                      ((*baseChange)[counter]).X() * formationSpeed + ((*droneVector)[counter]).X() + ((*collisionAvoidenceVector)[counter]).X(),
                                      ((*baseChange)[counter]).Y() * formationSpeed + ((*droneVector)[counter]).Y() + ((*collisionAvoidenceVector)[counter]).Y(),
                                      ((*baseChange)[counter]).Z() * formationSpeed + ((*droneVector)[counter]).Z() + ((*collisionAvoidenceVector)[counter]).Z()
                                      );
        (*StateChanges)[counter] = *temp;
        counter++;
    }

    // There are code for dead drones. It just moves dead drones down 1 unit on the z axis.
    // sim only, so skipping.
}

//This is the first function from the Matlab Drones file.
//Basically, it just takes in information and then determines the lead drone based upon who is closest.
//Then it calls the appropriate formation function based upon the formation number passed to it.
//Since there is only the V Formation, that is the function that is called.
//Net[] each row contains the information that is sent for each drone.
//The format assumed in the file is a string of format "DroneID:XCoordinate:YCoordinate:ZCoordinate:IsAlive:BatteryLife"
//I.E. "1:50.42:39.76:85.92:1:75"
//DroneID = 1
//XCoordinate=50.42
//YCoordinate=39.76
//ZCoordinate=85.92
//IsAlive = 1 - means drone is alive
//BatteryLife = 75 or 75%
void NetOut(string Net[], int State, Vector3D *target, int num_drones, int formationNumber)
{
    //Set initial variable to hold point to point distances
    double distances[num_drones]={0};
    for(int n = 0;n <= num_drones;n++)
    {
        distances[n] = ComputeDistances(Net[n], target);
    }

    //Stores the found minimal distance from drone to target
    double minDist = -1.0;
    for(int i = 0;i < num_drones + 1;i++)
    {
        if (minDist != -1.0 && distances[i] < minDist)
        {
            minDist = distances[i];
        }
    }

    //Finds the lead drone based upon distance from target
    string leadDrone = "-1";
    double leadDistance = minDist;

    for(int m = 0;m < num_drones + 1;m++)
    {
        if(minDist == distances[m])
        {
            leadDrone = Net[m];
        }
    }

    //Create lead drone coordinates and formation vector to be passed to formation creation
    Vector3D *leadDroneCoordinates = new Vector3D(strtod(ParseNet(leadDrone,1).c_str(),NULL), strtod(ParseNet(leadDrone,2).c_str(), NULL), strtod(ParseNet(leadDrone,3).c_str(), NULL));;
    Vector3D *formationVector = new Vector3D(target->X()-leadDroneCoordinates->X(), target->Y()-leadDroneCoordinates->Y(), target->Z()-leadDroneCoordinates->Z());

    //Choose which formation to utilize based upon passed in value
    switch(formationNumber)
    {
    case 1:
        MoveDronesVFormation(Net, formationVector, leadDrone, leadDistance, target, num_drones);
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    }

    //I did not include anything after the switch statement from matlab as it didn't seem to be relevant or useful at this time.
}

