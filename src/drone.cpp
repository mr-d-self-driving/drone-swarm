// Get coordinates
// Send coordinates

// Get leader votes
// Send leader vote

// Calculate movement towards target
// Calculate movement in (or into) formation

// Decide on formation alignment (which side of the V to move on)
#include <string>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include "drone.h"
#include "vector3d.h"
#include "droneinfo.h"

Drone::Drone(const Vector3D &target, const DroneInfo &position) {
  this->target = target;
  this->info = position;
}

void Drone::Move(double speed) {
  Vector3D current_location = info.getLocation();
  Vector3D movement_vector = (waypoint - current_location).UnitVector().Scale(speed);
  info.setLocation(current_location + movement_vector);
}

size_t split(const std::string &txt, std::vector<std::string>* strs)
{
  char delimiter = ':';
  size_t pos = txt.find(delimiter);
  size_t initialPos = 0;
  strs->clear();

  // Decompose statement
  while (pos != std::string::npos) {
    strs->push_back(txt.substr(initialPos, pos - initialPos + 1));
    initialPos = pos + 1;

    pos = txt.find(delimiter, initialPos);
  }

  // Add the last one
  strs->push_back(txt.substr(initialPos, std::min(pos, txt.size()) - initialPos + 1));

  return strs->size();
}

// Parses the string contained in Net[] so the needed information can be
// extracted and used when needed.
std::string ParseNet(std::string input_str, int value_to_return) {
  std::string delimiter = ":";
  std::string token;
  size_t pos = 0;
  int counter = 0;

  // Searches the string and breaks it apart based on the ':' delimiter
  // Stores relevant information in appropriate variable
  while ((pos = input_str.find(delimiter)) != std::string::npos) { //while there exists the delimiter in the string
    token = input_str.substr(0, pos); //portion of the string before the delimiter
    
    /* ??? */
    if (value_to_return == counter) {
      return token;
    }

    input_str.erase(0, pos + delimiter.length()); //erase the string up to the delimiter
    counter++;
  }

  return "-1";
}

// This just computes the distance of each drone from the target
double Drone::ComputeDistanceToTarget(const std::string &Net, const Vector3D &target) {
  //std::vector<std::string> str_coordinates;
  //split(Net, &str_coordinates);
  Vector3D coordinate(strtod(ParseNet(Net, 1).c_str(), NULL), //must convert to c_str for strtod
                      strtod(ParseNet(Net, 2).c_str(), NULL),
                      strtod(ParseNet(Net, 3).c_str(), NULL));

  return sqrt(pow((coordinate.getX() - target.getX()), 2) +
              pow((coordinate.getY() - target.getY()), 2) +
              pow((coordinate.getZ() - target.getZ()), 2));
}

// Returns 1 if point is left of formation vector, -1 if to the right, and 0 if
// aligned
int Orientation(const Vector3D &target, const Vector3D &leadDrone,
                const Vector3D &point) {
  double result =
      ((target.getX() - leadDrone.getX()) * (point.getY() - leadDrone.getY())) -
      ((target.getY() - leadDrone.getY()) * (point.getX() - leadDrone.getX()));
  if (result == 0) {
    return 0;
  }
  else if (result > 0) {
    return 1;
  }
  else {
    return -1;
  }
}

void Drone::CalculateWaypoint(const DroneInfo &leadDrone) {
  // if the drone is the leader, go straight for the target
  if (info.isLead()) { 
    this->waypoint = this->target;
    return;
  }

  // Formation vector is just a vector from the lead drone to the target.
  // It is the overall vector the drones follow.
  Vector3D formationVector = this->target - leadDrone.getLocation(); 

  // calculate which side of the V this drone is on and will follow
  Vector3D VectorToUse;
  if (Orientation(target, leadDrone.getLocation(), this->info.getLocation()) > 0) {
    VectorToUse = formationVector.RotateZ((45 / 2) + 180).UnitVector();
  }
  else {
    VectorToUse = formationVector.RotateZ((-45 / 2) + 180).UnitVector();
  }

  //the vector from this to the lead drone
  Vector3D VectorToLead = leadDrone.getLocation() - (this->info.getLocation());
  //project the vector the the lead dron
  Vector3D prjVecToLeadOnVForm = VectorToLead.Project(VectorToUse);

  this->waypoint = Vector3D(leadDrone.getLocation() - prjVecToLeadOnVForm);
}

/*
// Starts calculating the positions of each drone and where they need to go in
// order to fly in a V formation
void MoveDronesVFormation(std::string Net[], Vector3D *formationVector,
                          std::string leadDrone, double formationDistance,
                          Vector3D *target, int num_drones) {
  /*
  int formationSpeed = 4;
  Vector3D *leadDroneChange = new Vector3D(*(*formationVector /
formationDistance));

  Vector3D *v1 = new Vector3D(*(formationVector->RotateZ((45/2)+180)));
  Vector3D *v2 = new Vector3D(*(formationVector->RotateZ(-(45/2)+180)));
  Vector3D *uv1 = new Vector3D(*(*v1/v1->Magnitude()));
  Vector3D *uv2 = new Vector3D(*(*v2/v2->Magnitude()));

  vector<Vector3D> *baseChange = Repmat(formationVector,num_drones);

  Vector3D* leadDroneCoordinates = new
Vector3D(strtod(ParseNet(leadDrone,1).c_str(),NULL),
strtod(ParseNet(leadDrone,2).c_str(),NULL),
strtod(ParseNet(leadDrone,3).c_str(),NULL));
  vector<Vector3D> *leadDroneCoords = Repmat(leadDroneCoordinates, 9);
  vector<Vector3D> *repmatFormationVector = Repmat(formationVector, 9);

  vector<Vector3D> *a = new vector<Vector3D>(9);
  int counter = 0;
  for(auto it = a->begin(); it != a->end();it++)
  {
      Vector3D *temp = new Vector3D(((*leadDroneCoords)[counter]).getX() -
((*repmatFormationVector)[counter]).getX(), ((*leadDroneCoords)[counter]).getY() -
((*repmatFormationVector)[counter]).getY(), ((*leadDroneCoords)[counter]).getZ() -
((*repmatFormationVector)[counter]).getZ());
      (*a)[counter] = *temp;
      counter++;
  }

  //determines which side of the lead drone each drone is on
  double sides[num_drones] = {0};
  for(int i = 0; i <= num_drones;i++)
  {
      sides[i] = ((*leadDroneCoords)[i].getX() - (*a)[i].getX()) *
(strtod(ParseNet(Net[i], 2).c_str(), NULL) - (*a)[i].getY()) -
((*leadDroneCoords)[i].getY() - (*a)[i].getY()) * (strtod(ParseNet(Net[i], 1).c_str(),
NULL) - (*a)[i].getX());
  }

  vector<int> v1folllows;
  vector<int> v2folllows;

  //create two arrays and each one contains all the drones that are on one side
of the lead
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
  for(auto iter = vectorToLeadDrone->begin(); iter != vectorToLeadDrone->end();
iter++)
  {
      Vector3D *temp = new Vector3D(
                                    ((*leadDroneCoords)[counter]).getX() -
(strtod(ParseNet(Net[counter], 1).c_str(),NULL)),
                                    ((*leadDroneCoords)[counter]).getY() -
(strtod(ParseNet(Net[counter], 2).c_str(),NULL)),
                                    ((*leadDroneCoords)[counter]).getZ() -
(strtod(ParseNet(Net[counter], 3).c_str(),NULL))
                                    );
      (*vectorToLeadDrone)[counter] = *temp;
      counter++;
  }

  // projVtldFV = (dot(vtld, vformChange, 2)/dot(vformChange, vformChange, 2))*
vformChange;
  // projection of vectorToLeadDrone onto vformChange
  vector<Vector3D> *projVectorToLeadDroneFV = new vector<Vector3D>(num_drones);
  counter = 0;
  for(auto iter = projVectorToLeadDroneFV->begin(); iter !=
projVectorToLeadDroneFV->end(); iter++)
  {
      Vector3D *temp = new Vector3D(
                                    ((*vectorToLeadDrone)[counter]).Dot((*vformChange)[counter])
/ ((*vformChange)[counter]).Dot((*vformChange)[counter]) *
(*vformChange)[counter].getX(),
                                    ((*vectorToLeadDrone)[counter]).Dot((*vformChange)[counter])
/ ((*vformChange)[counter]).Dot((*vformChange)[counter]) *
(*vformChange)[counter].getY(),
                                    ((*vectorToLeadDrone)[counter]).Dot((*vformChange)[counter])
/ ((*vformChange)[counter]).Dot((*vformChange)[counter]) *
(*vformChange)[counter].getZ()
                                    );
      (*projVectorToLeadDroneFV)[counter] = *temp;
      counter++;
  }

  // projVtldFV(v1follows,1:2) = [projVtldFV(v1follows,2)
projVtldFV(v1follows,1)];
  // swaps all x and y for all following v1
  counter = 0;
  for(auto iter = projVectorToLeadDroneFV->begin(); iter !=
projVectorToLeadDroneFV->end(); iter++)
  {
      if(counter < l1)
      {
          Vector3D *temp = new Vector3D(
                                        (*projVectorToLeadDroneFV)[counter].getY(),
                                        (*projVectorToLeadDroneFV)[counter].getX(),
                                        (*projVectorToLeadDroneFV)[counter].getZ()
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
                                    ((*leadDroneCoords)[counter]).getX() -
((*droneVector)[counter]).getX() - (strtod(ParseNet(Net[counter],
1).c_str(),NULL)),
                                    ((*leadDroneCoords)[counter]).getY() -
((*droneVector)[counter]).getY() - (strtod(ParseNet(Net[counter],
2).c_str(),NULL)),
                                    ((*leadDroneCoords)[counter]).getZ() -
((*droneVector)[counter]).getZ() - (strtod(ParseNet(Net[counter], 3).c_str(),NULL))
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
      Vector3D *temp = new Vector3D();
      temp = (*droneVector)[counter].UnitVector();
      (*droneVector)[counter] = *temp;
      counter++;
  }

  // StateChanges = combines moving in formationVector direction with moving
into formation
  //StateChanges = (baseChange(1:num_drones,1:3) * formationSpeed) +
droneVector(1:num_drones,1:3);
  vector<Vector3D> *StateChanges = new vector<Vector3D>(num_drones);
  counter = 0;
  for(auto iter = StateChanges->begin(); iter != StateChanges->end(); iter++)
  {
      Vector3D *temp = new Vector3D(
                                    ((*baseChange)[counter]).getX() *
formationSpeed + ((*droneVector)[counter]).getX(),
                                    ((*baseChange)[counter]).getY() *
formationSpeed + ((*droneVector)[counter]).getY(),
                                    ((*baseChange)[counter]).getZ() *
formationSpeed + ((*droneVector)[counter]).getZ()
                                    );
      (*StateChanges)[counter] = *temp;
      counter++;
  }

//******************************************************************************************
// TODO
// CollisionAvoidence:
//******************************************************************************************
  //Ignoring collision avoidance for now
  vector<Vector3D> *collisionAvoidenceVector = new vector<Vector3D>(num_drones);
  // collisionAvoidenceVector = avoidCollisions(Net, StateChanges, num_drones);
  // we pass in StateChanges here, aka what we want the drones to do
  // make collisionAvoidance a unit vector
  counter = 0;
  for(auto iter = collisionAvoidenceVector->begin(); iter !=
collisionAvoidenceVector->end(); iter++)
  {
      //(*collisionAvoidenceVector)[counter] =
*(*collisionAvoidenceVector)[counter].UnitVector();
      Vector3D *temp = new Vector3D();
      temp = (*collisionAvoidenceVector)[counter].UnitVector();
      (*collisionAvoidenceVector)[counter] = *temp;
      counter++;
  }

  // same as before but include collision avoidance movement
  // StateChanges = combines moving in formationVector direction with moving
into formation
  // as well as include the collision avoidance movement.
  // StateChanges =
  //                  (baseChange(1:num_drones,1:3) * formationSpeed)
  //                  + droneVector(1:num_drones,1:3)
  //                  + collisionAvoidenceVector(1:num_drones, 1:3);
  counter = 0;
  for(auto iter = StateChanges->begin(); iter != StateChanges->end(); iter++)
  {
      Vector3D *temp = new Vector3D(
                                    ((*baseChange)[counter]).getX() *
formationSpeed + ((*droneVector)[counter]).getX() +
((*collisionAvoidenceVector)[counter]).getX(),
                                    ((*baseChange)[counter]).getY() *
formationSpeed + ((*droneVector)[counter]).getY() +
((*collisionAvoidenceVector)[counter]).getY(),
                                    ((*baseChange)[counter]).getZ() *
formationSpeed + ((*droneVector)[counter]).getZ() +
((*collisionAvoidenceVector)[counter]).getZ()
                                    );
      (*StateChanges)[counter] = *temp;
      counter++;
  }

  // There are code for dead drones. It just moves dead drones down 1 unit on
the z axis.
  // sim only, so skipping.
  */
//}
//void avoidCollision(std::string Net[], std::vector<Vector3D> &changeVector,
//                    int num_drones) {
//  int minCollisionDistance = 12;
//
//  // creating a matrix of only living drones
//  std::vector<Vector3D> livingDrones (num_drones);
//  int counter = 0;
//  int droneCounter = 0;  // counter for living drones.
//  for (auto iter = livingDrones.begin(); iter != livingDrones.end(); iter++) {
//    if (strtod(ParseNet(Net[counter], 5).c_str(), NULL) == 1) {
//      Vector3D temp = Vector3D(
//                       (strtod(ParseNet(Net[counter], 1).c_str(), NULL)),
//                       (strtod(ParseNet(Net[counter], 2).c_str(), NULL)),
//                       (strtod(ParseNet(Net[counter], 3).c_str(), NULL)));
//      livingDrones[droneCounter] = temp;
//      droneCounter++;
//    }
//    counter++;
//  }
//
//  // x is a copy matrix of livingDrones. Used to calculate the distance
//  std::vector<Vector3D> x (livingDrones.size());
//  counter = 0;
//  for (auto iter = x.begin(); iter != x.end(); iter++) {
//    Vector3D temp = Vector3D(livingDrones[counter]);
//    x[counter] = temp;
//    counter++;
//  }
//  int numDronesLiving = x.size();
//
//  // distance vector with size m(m-1)/2
//  std::vector<double> distance (numDronesLiving * (numDronesLiving - 1) / 2);
//  // Distance in Euclidean 3 space
//  counter = 0;
//  for (auto iter = distance.begin(); iter != distance.end(); iter++) {
//    distance[counter] = pow(x[counter].Magnitude(), 2);
//    counter++;
//  }
//
//  // Matrix of (number of rows in x) getX (number of rows in x)
//  // where everything below the diagonal is 1
//  // 0 0 0
//  // 1 0 0
//  // 1 1 0
//  int tempMatrix[numDronesLiving][numDronesLiving] = {0};
//
//  for (int row = 1; row < (*x).size(); row++) {
//    for (int column = 0; column < row; column++) {
//      tempMatrix[row][column] = 1;
//    }
//  }
//
//  /*
//  %# get the indices of the 1's
//  [rowIdx,colIdx] = find(tmp);
//
//  %# create the output
//  out = [D',livingDrones(rowIdx,:),livingDrones(colIdx,:)];
//  */
//  // This is most definitely not right
//  int unknowsize = 4;  // magic number, need to comeback and fix
//  double outputMatrix[distance->size()][unknowsize];
//  for (int i = 0; i < distance->size(); i++) {
//    outputMatrix[i][0] = (*distance)[counter];
//    outputMatrix[i][1] = (*livingDrones)[counter].getX();
//    outputMatrix[i][2] = (*livingDrones)[counter].getY();
//    outputMatrix[i][3] = (*livingDrones)[counter].getZ();
//  }
//
//  std::vector<double> selectCollisions;
//
//  counter = 0;
//  for (auto iter = distance->begin(); iter != distance->end(); iter++) {
//    if ((*distance)[counter] < minCollisionDistance) {
//      selectCollisions.push_back(1);
//    } else {
//      selectCollisions.push_back(0);
//    }
//    counter++;
//  }
//
//  double collisions[distance->size()][unknowsize];
//  counter = 0;
//  int tempCounter = 0;
//  while (counter < distance->size()) {
//    if (selectCollisions[counter] == 1) {
//      collisions[tempCounter][0] = (*distance)[counter];
//      collisions[tempCounter][1] = (*livingDrones)[counter].getX();
//      collisions[tempCounter][2] = (*livingDrones)[counter].getY();
//      collisions[tempCounter][3] = (*livingDrones)[counter].getZ();
//      tempCounter++;
//    }
//    counter++;
//  }
//
//  double collisionDroneMatrix[distance->size()][2];
//  for (int i = 0; i < distance->size(); i++) {
//    collisionDroneMatrix[i][0] = collisions[i][7];
//    collisionDroneMatrix[i][1] = collisions[i][14];
//  }
//
//  std::vector<Vector3D> *vbpt = new std::vector<Vector3D>(numDronesLiving);
//  counter = 0;
//  for (auto iter = vbpt->begin(); iter != vbpt->end(); iter++) {
//    Vector3D *temp = new Vector3D(
//        strtod(ParseNet(Net[int(collisionDroneMatrix[counter][0])], 1).c_str(),
//               NULL) -
//            strtod(
//                ParseNet(Net[int(collisionDroneMatrix[counter][1])], 1).c_str(),
//                NULL),
//        strtod(ParseNet(Net[int(collisionDroneMatrix[counter][0])], 2).c_str(),
//               NULL) -
//            strtod(
//                ParseNet(Net[int(collisionDroneMatrix[counter][1])], 2).c_str(),
//                NULL),
//        strtod(ParseNet(Net[int(collisionDroneMatrix[counter][0])], 3).c_str(),
//               NULL) -
//            strtod(
//                ParseNet(Net[int(collisionDroneMatrix[counter][1])], 3).c_str(),
//                NULL));
//    (*vbpt)[counter] = *temp;
//
//    counter++;
//  }
//
//  std::vector<double> magnitudeVbpt;
//  for (auto iter = vbpt->begin(); iter != vbpt->end(); iter++) {
//    double temp = (*vbpt)[counter].Magnitude();
//    magnitudeVbpt.push_back(temp);
//
//    counter++;
//  }
//
//  std::vector<Vector3D> *uvbpt = new std::vector<Vector3D>(numDronesLiving);
//  counter = 0;
//  for (auto iter = uvbpt->begin(); iter != uvbpt->end(); iter++) {
//    Vector3D *temp =
//        new Vector3D(((*vbpt)[counter]).getX() / magnitudeVbpt[counter],
//                     ((*vbpt)[counter]).getY() / magnitudeVbpt[counter],
//                     ((*vbpt)[counter]).getZ() / magnitudeVbpt[counter]);
//    (*uvbpt)[droneCounter] = *temp;
//    counter++;
//  }
//
//  std::vector<Vector3D> *chvt = new std::vector<Vector3D>(numDronesLiving);
//  counter = 0;
//  for (auto iter = chvt->begin(); iter != chvt->end(); chvt++) {
//    Vector3D *temp =
//        new Vector3D(((*uvbpt)[counter]).getX() *
//                         (minCollisionDistance - magnitudeVbpt[counter]),
//                     ((*uvbpt)[counter]).getY() *
//                         (minCollisionDistance - magnitudeVbpt[counter]),
//                     ((*uvbpt)[counter]).getZ() *
//                         (minCollisionDistance - magnitudeVbpt[counter]));
//    (*chvt)[droneCounter] = *temp;
//    counter++;
//  }
//
//  std::vector<Vector3D> *collisionAvoidVector =
//      new std::vector<Vector3D>(numDronesLiving);
//  counter = 0;
//  for (auto iter = collisionAvoidVector->begin();
//       iter != collisionAvoidVector->end(); iter++) {
//    Vector3D *temp = new Vector3D(
//        0, 0,
//        0  //(*collisionAvoidVector)[int(collisionDroneMatrix[counter][0])].getX()
//           //+ (*chvt)[counter].getX()
//        );
//    (*collisionAvoidVector)[counter] = *temp;
//    counter++;
//  }
//
//  counter = 0;
//  for (int i = 0; i < distance->size(); i++) {
//    (*collisionAvoidVector)[int(collisionDroneMatrix[i][0])].setX(
//        (*collisionAvoidVector)[int(collisionDroneMatrix[i][0])].getX() +
//        (*chvt)[counter].getX());
//    (*collisionAvoidVector)[int(collisionDroneMatrix[i][0])].setY(
//        (*collisionAvoidVector)[int(collisionDroneMatrix[i][0])].getX() +
//        (*chvt)[counter].getY());
//    (*collisionAvoidVector)[int(collisionDroneMatrix[i][0])].setZ(
//        (*collisionAvoidVector)[int(collisionDroneMatrix[i][0])].getX() +
//        (*chvt)[counter].getZ());
//    counter++;
//  }
//
//  counter = 0;
//  for (int i = 0; i < distance->size(); i++) {
//    (*collisionAvoidVector)[int(collisionDroneMatrix[i][1])].setX(
//        (*collisionAvoidVector)[int(collisionDroneMatrix[i][1])].getX() -
//        (*chvt)[counter].getX());
//    (*collisionAvoidVector)[int(collisionDroneMatrix[i][1])].setY(
//        (*collisionAvoidVector)[int(collisionDroneMatrix[i][1])].getX() -
//        (*chvt)[counter].getY());
//    (*collisionAvoidVector)[int(collisionDroneMatrix[i][1])].setZ(
//        (*collisionAvoidVector)[int(collisionDroneMatrix[i][1])].getX() -
//        (*chvt)[counter].getZ());
//    counter++;
//  }
//}
//
//// This is the first function from the Matlab Drones file.
//// Basically, it just takes in information and then determines the lead drone
//// based upon who is closest.
//// Then it calls the appropriate formation function based upon the formation
//// number passed to it.
//// Since there is only the V Formation, that is the function that is called.
//// Net[] each row contains the information that is sent for each drone.
//// The format assumed in the file is a string of format
//// "DroneID:XCoordinate:YCoordinate:ZCoordinate:IsAlive:BatteryLife"
//// I.E. "1:50.42:39.76:85.92:1:75"
//// DroneID = 1
//// XCoordinate=50.42
//// YCoordinate=39.76
//// ZCoordinate=85.92
//// IsAlive = 1 - means drone is alive
//// BatteryLife = 75 or 75%
//void NetOut(std::string Net[], int State, const Vector3D &target, int num_drones,
//            int formationNumber) {
//  // Set initial variable to hold point to point distances
//  double distances[num_drones] = {0};
//  for (int n = 0; n <= num_drones; n++) {
//    distances[n] = ComputeDistanceToTarget(Net[n], target);
//  }
//
//  // Stores the found minimal distance from drone to target
//  double minDist = -1.0;
//  for (int i = 0; i < num_drones + 1; i++) {
//    if (minDist != -1.0 && distances[i] < minDist) {
//      minDist = distances[i];
//    }
//  }
//
//  // Finds the lead drone based upon distance from target
//  std::string leadDrone = "-1";
//  double leadDistance = minDist;
//
//  for (int m = 0; m < num_drones + 1; m++) {
//    if (minDist == distances[m]) {
//      leadDrone = Net[m];
//    }
//  }
//
//  // Create lead drone coordinates and formation vector to be passed to
//  // formation creation
//  Vector3D leadDroneCoordinates = Vector3D(
//                   strtod(ParseNet(leadDrone, 1).c_str(), NULL),
//                   strtod(ParseNet(leadDrone, 2).c_str(), NULL),
//                   strtod(ParseNet(leadDrone, 3).c_str(), NULL));
//
//  Vector3D formationVector = Vector3D(target - leadDroneCoordinates);
//
//  // Choose which formation to utilize based upon passed in value
//  switch (formationNumber) {
//    case 1:
//      MoveDronesVFormation(Net, formationVector, leadDrone, leadDistance,
//                           target, num_drones);
//      break;
//    case 2:
//      break;
//    case 3:
//      break;
//    case 4:
//      break;
//  }
//}