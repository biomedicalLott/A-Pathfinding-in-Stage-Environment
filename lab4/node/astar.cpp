//#pragma region includes 
#include "ros/ros.h"
#include <ros/package.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <iterator>
#include <algorithm>
#include <vector>
#include <math.h>
#include <time.h> /* time_t, struct tm, difftime, time, mktime */
#include <chrono>
#include <thread>
#include <ctime>
#include <fstream> /*for reading in map */

/* FOR QUATERNION HANDLING FOR ROTATION */
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
/* FOR QUATERNION HANDLING FOR ROTATION */
//#pragma endregion
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;
using namespace tf2;

struct node
{
    int id;
    int yIndex;
    int xIndex;
    int xPos;
    int yPos;
    node* Parent;
    float gCost;
    float hCost; 
    float fCost;
    int walkability;
    bool goal;
    bool occupied ;
    bool obstacle ;
    bool searched;
};
node currentNode;
node goalNode;
  bool foundDestination = false;

vector<vector<float>> mapArray;
vector<vector<float>> costMap;
vector<vector<float>> occupancyMap;
vector<vector<vector<float>>> coordinateArray;
vector<vector<node>> nodeMap;
int maxDistance = 20;
int stepSize = 0.3;
//#pragma region scanning variables 
//Scanning section
double max_range = 3;
int scan_beams = 361;
double line_growth_limit = 5;
double line_extension = 0;
/*--------------------GOAL SET----------------------*/
double xGoal = 4.5;
double yGoal = 9.0;
/*--------------------GOAL SET----------------------*/
double altXGoal = -7;
double altYGoal = -6;
vector<int> goalIndex;
const double pi = 3.14159265359;
vector<float> scanAngles(361, 0);
//#pragma endregion
//End scanning Section
string importedMap;
//#pragma region pose variables
vector<double> startingPose(4, 0);
vector<double> robotPose(4, 0);
geometry_msgs::Twist robotVelocity;
geometry_msgs::Vector3 euler_orientation, startingPosition, robotPosition;
//#pragma endregion 
//#pragma region subscriber publisher variables
//Listen for laser messages
ros::Subscriber laser_sub;

// Listen for odom messages
ros::Subscriber odom_sub;
// Publish drive data
ros::Publisher drive_pub;
ros::Publisher ransac_pub;
//#pragma endregion
std::string fpath;

//#pragma region Get Index For Cost Maps Relative To Some Position
vector<int> PositionToIndex()
{
  int x = round(robotPose[0]) + 9;
  int y = abs(round(robotPose[1]) - 10);
  vector<int> xy = {x,y};
return xy;
}
vector<int> PositionToIndex(float xCoord, float yCoord)
{
  int x = round(xCoord) + 9;
  int y = abs(round(yCoord) - 10);
  vector<int> xy = {x,y};
return xy;
}
vector<int> PositionToIndex(vector<float> xyCoord)
{
  int x = round(xyCoord[0]) + 9;
  int y = abs(round(xyCoord[1]) - 10);
  vector<int> xy = {x,y};
return xy;
}
vector<float> IndexToPosition(float xCoord, float yCoord)
{
  float x = xCoord - 9;
  float y = -1*(yCoord - 10);
  vector<float> xy = {x,y};
return xy;
}
vector<float> IndexToPosition(vector<float> xyCoord)
{
  float x = (xyCoord[0]) + 9;
  float y = -1*(xyCoord[1] - 10);
  vector<float> xy = {x,y};
return xy;
}
//#pragma endregion



//#pragma region odometer callback
vector<int> oldPositionIndex;
bool firstOdomCalled = true;
int countDownToPathPlan = 190;
int thresholdForPathPlan = 200;
void odom_callback(const nav_msgs::Odometry &msg)
{
  // ROS_ERROR_STREAM("14");
  if (firstOdomCalled)
  {
    //store initial pose, just in case. 
    startingPose[0] = msg.pose.pose.position.x;
    startingPose[1] = msg.pose.pose.position.y;
    startingPose[2] = msg.pose.pose.orientation.z;
    startingPose[3] = msg.pose.pose.orientation.w;

    firstOdomCalled = false;
    //now grab the index of the map 
    oldPositionIndex = (PositionToIndex(startingPose[0],startingPose[1]));
    //make the nodemap know where you are
    nodeMap[oldPositionIndex[1]][oldPositionIndex[0]].occupied = true;
    currentNode = nodeMap[oldPositionIndex[1]][oldPositionIndex[0]];
    //set f cost to 1000 so you have a good starting amount to compare against
    nodeMap[oldPositionIndex[1]][oldPositionIndex[0]].fCost = 1000;
    //log where the index of the goal is and set the position on node map up 
    goalIndex = PositionToIndex(xGoal, yGoal);
    nodeMap[goalIndex[1]][goalIndex[0]].goal = true;
    goalNode = nodeMap[goalIndex[1]][goalIndex[0]];
  }
  //get current location 
  robotPose[0] = msg.pose.pose.position.x;
  robotPose[1] = msg.pose.pose.position.y;
  robotPose[2] = msg.pose.pose.orientation.z;
  robotPose[3] = msg.pose.pose.orientation.w;
  //remove wiggle noise
  for (int i = 0; i < 4; i++)
  {
    if (abs(robotPose[i]) < 0.0001)
      robotPose[i] = 0;
  }
  //get rotation information 
  tf2::Quaternion quat;
  fromMsg(msg.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  // the found angles are written in a geometry_msgs::Vector3
  euler_orientation.x = roll;
  euler_orientation.y = pitch;
  euler_orientation.z = yaw;
  //update occupied node 
  vector<int> newCostIndex = (PositionToIndex());
  currentNode.occupied = false;
  node newNode = nodeMap[newCostIndex[1]][newCostIndex[0]];
  newNode.occupied = true;
  currentNode = newNode;
  oldPositionIndex = newCostIndex;
  countDownToPathPlan++;
}
//#pragma endregion

//#pragma region Map Preparation
/*****************MAP PREPARATION*****************/
void mapToArray(int widthOfMap, int heightOfMap)
{
  //read the from from wherever the map is located 
  vector<float> rowArray;
  // std::ifstream fileRead("../stage_ws/src/lab4/node/map.txt");
 std::string fpath = ros::package::getPath("lab4") + "/node/map.txt";
 std::ifstream fileRead(fpath);


  char input;
  bool matrixFound = false;
  int maxy = heightOfMap;
  int maxx = widthOfMap;
  vector<node> rowOfNodes; 

  
  cout << importedMap << endl;

  char letter = 'p';

  int idCounter = 0;
  //so long as the file is open this loop will run. 
  while (fileRead.is_open())
  {
    //until the matrix is started, characters will be read in and thrown away 
    fileRead.get(letter);
    if (!matrixFound)
    {
      if (letter == '[')
      {
        matrixFound = true;
      }
    }
    //once they're found, it'll enter 2 for loops of the size of the map to import 
    if (matrixFound)
    {
      for (int i = 0; i < maxy; i++)
      {
        for (int j = 0; j < maxx; j++)
        {
          fileRead.get(letter);
          bool checking = true;
          while (checking)//the loop will purposefully skip anything that's not a numeral
          {
            if (letter == '0' || letter == '1')
            {
              rowArray.push_back(letter - '0');
              node N;
              N.id = idCounter;
              N.yIndex = i;
              N.xIndex = j;
              N.xPos = j-9;
              N.yPos = -1*(i-10);
              N.gCost = 0;
              N.hCost = 0; 
              N.fCost = 100;
              N.walkability = 1;
              N.goal = false;
              N.occupied = false;
              N.obstacle =rowArray.back() == 1;
              N.searched = false;
              rowOfNodes.push_back(N);

              checking = false;
              idCounter++;
            }
            else
            {
              fileRead.get(letter);
            }
          }
          if (letter == ']')//if it gets this far, my for loops were too big.
          {
            ROS_ERROR_STREAM("we're done here.");
            return;
          }
        }
        fileRead.get(letter);
        mapArray.push_back(rowArray);
        nodeMap.push_back(rowOfNodes);
        rowArray.clear();
      }
      matrixFound = false;
      return;
    }
  }
}

/*****************END MAP PREPARATION*****************/
//#pragma endregion

//#pragma region AStarPlanning
void calculateHCost()
{
  node Goal = nodeMap[goalIndex[1]][goalIndex[0]];
  for(int i = 0; i < nodeMap.size();i ++)
  {  
    for(int j = 0; j < nodeMap[0].size(); j++)
    {
      float distX = Goal.xPos - nodeMap[i][j].xPos;
      float distY = Goal.yPos - nodeMap[i][j].yPos;
      //I'm assuming that I won't change anything so 
      //I'm just going to calculate once and use euclidean
      nodeMap[i][j].hCost = sqrt(double(distX*distX + distY+distY));
      
    // Euclidian Distance
    // d = static_cast<int>(sqrt((double)(distX*distX + distY*distY)));

    // Manhattan distance
    // d = abs(distX) + abs(distY);
    
    // Chebyshev distance
    //d = max(abs(distX), abs(distY));
    }
  }
}
  vector<node> nodePath;
  vector<vector<float>> pathCoordinates;
void pathMap()
{
  for(int i = 0; i < pathCoordinates.size(); i++)
  {
    vector<float> xy = pathCoordinates[i];
    nodeMap[xy[1]][xy[0]].walkability = 9;
  }
  cout << "Best Path Map" << endl;
  cout << "[" ;
  for(int i = 0; i < nodeMap.size();i ++)
  {  
    for(int j = 0; j < nodeMap[0].size(); j++)
    {
      if(nodeMap[i][j].walkability < 9)
        nodeMap[i][j].walkability = mapArray[i][j];
      cout << nodeMap[i][j].walkability << "," ;
      nodeMap[i][j].walkability = mapArray[i][j];
    }
    cout << endl;
  }
  cout << "]"<< endl;
}


void tracePath(node& destinationNode)
{
  //function goes up the tree parent nodes within parent nodes, passing their 
  //indicies/positions to pathCoordinates.  
  node parentNode = *destinationNode.Parent;
  vector<float> xy{0,0,0,0};
  nodePath.insert(nodePath.begin(), destinationNode);
  xy[0] = destinationNode.xIndex;//normally you want pos
  xy[1] = destinationNode.yIndex;
  xy[2] = xGoal;
  xy[3] = yGoal;
  //insert at the beginning since it's moving backwards through the chain
  pathCoordinates.insert(pathCoordinates.begin(), xy);
    while (!parentNode.occupied) 
    {
      nodePath.insert(nodePath.begin(), parentNode);
      xy[0] = parentNode.xIndex;
      xy[1] = parentNode.yIndex;
      xy[2] = destinationNode.xPos;
      xy[3] = destinationNode.yPos;
      pathCoordinates.insert(pathCoordinates.begin(), xy);
      parentNode = *parentNode.Parent;
    }
    return;
}
bool goalCheck(node& testNode, node& parentNode)
{
  //check to see if we're at the goal yet, if we are trace the path along the nodes
if(testNode.goal)
  {
    testNode.Parent = &parentNode;
    tracePath(testNode);
    foundDestination = true;
    return true;
  }
  return false;
}

void astarProcess(node& testNode, node& parentNode, vector<node>& searchList,  float gCost)
{
  // cout << "aStar process 1" << endl;
    // cout << "node ID: " << testNode->id << endl;
        if(testNode.searched == true)
        {
          // cout << testNode.searched << " what does the test node say?" << endl;
          // cout << "Already saw node " << testNode.id << " boss" << endl;
          return;
        }
          if(testNode.obstacle  == true)
          {
            // cout << "aStar process early fail" << endl;
            return;
          }
          float newG = testNode.gCost + gCost;
          float distX = testNode.xPos - goalNode.xPos;
          float distY = testNode.yPos - goalNode.yPos;
          testNode.hCost = sqrt(double(distX*distX + distY+distY));
          float newF  = testNode.hCost + newG;
          // cout << "astar process 1.5, fcost of parent without ref . " << nodeMap[1][12].fCost << endl;
          // cout << "aStar process 2, f cost " << newF << " vs. " << parentNode.fCost<< endl;
          if(parentNode.fCost > newF)//If  the f cost is higher here than the parent, just skip this entirely.
          {
            // cout << "aStar process newF is good" << endl;
              // cout << "newF node ID: " << parentNode.id << endl;
              
            testNode.gCost = newG;
            testNode.fCost = newF;
            testNode.Parent = &parentNode;
            testNode.Parent->searched = true; 
            // testNode.searched = true;
            searchList.push_back(testNode);
            // cout << "aStar process good return" << endl;
            return;
          }
          // parentNode->searched = true;
      // cout << "aStar ahh shit not greater. Let's keep going." << endl;

}

void planAStar()
{
  pathCoordinates.clear();
  foundDestination = false;
  costMap = mapArray;
  int maxBottom = costMap.size();
  int maxRight = costMap[0].size();
  ///Check to make sure the node won't check ones around it that don't matter.
  vector<node> nodesBeingSearchedFrom;
  vector<vector<node>>tempNodeMap = nodeMap;
  vector<int> xy = PositionToIndex(robotPose[0], robotPose[1]);
  vector<int> goalxy = PositionToIndex(xGoal, yGoal);
  tempNodeMap[xy[1]][xy[0]].occupied = true;
  tempNodeMap[xy[1]][xy[0]].fCost = 1000;

  int counter = 0;
  //I couldn't figure out why, but my code was forgetting the changes i made to the structs.
  //therefor i had to do it here, this is dumb. 
  for(int i = 0; i < tempNodeMap.size(); i++)
  {
    for(int j = 0; j < tempNodeMap[0].size(); j++)
    {
      if(mapArray[i][j] == 1)
        {
          nodeMap[i][j].obstacle = true;
          tempNodeMap[i][j].obstacle = true;
        }
        else
        {
          tempNodeMap[i][j].obstacle = false;
          nodeMap[i][j].obstacle = false;
        }
        nodeMap[i][j].goal = false;
        tempNodeMap[i][j].goal = false;
        nodeMap[i][j].searched = false;
        tempNodeMap[i][j].searched = false;
        nodeMap[i][j].xIndex = j;
        tempNodeMap[i][j].xIndex = j;
        nodeMap[i][j].yIndex = i;
        tempNodeMap[i][j].yIndex = i;
        nodeMap[i][j].id = counter;
        tempNodeMap[i][j].id = counter;
        counter++;
    }
  }
  tempNodeMap[goalxy[1]][goalxy[0]].goal = true;
  nodeMap[goalxy[1]][goalxy[0]].goal = true;


  //search through nodes one at a time, 
  nodesBeingSearchedFrom.push_back(tempNodeMap[xy[1]][xy[0]]);
     int astarCounter = 0;
  while(!foundDestination)
  {
    if(nodesBeingSearchedFrom.size() == 0)
    {
      nodeMap = tempNodeMap;
      return;
    }
    astarCounter ++;
    //on each run all the nodes that the previous nodes searched that are valid 
    //are added to the "nodes being searched from" list which stores in old nodes being searched from 
    //and the nodes searched from are cleared.  
    vector<node> oldNodesBeingSearchedFrom = nodesBeingSearchedFrom;
    nodesBeingSearchedFrom.clear();
    for(int n = 0; n < oldNodesBeingSearchedFrom.size(); n++)
    {
      int y = oldNodesBeingSearchedFrom[n].yIndex;
      int x = oldNodesBeingSearchedFrom[n].xIndex;

      if(y == 0)//if it's against the top edge
      {
        if(x == 0)//if it's against the left edge, only bottom right corner
        {
          if(goalCheck(tempNodeMap[y][x+1],tempNodeMap[y][x]))//east
          return;
          if(goalCheck(tempNodeMap[y+1][x],tempNodeMap[y][x]))//south
          return;
          if(goalCheck(tempNodeMap[y+1][x+1],tempNodeMap[y][x]))//southeast
          return;

          astarProcess(tempNodeMap[y][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//east
          astarProcess(tempNodeMap[y+1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//south
          astarProcess(tempNodeMap[y+1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//southeast
        }
        else if(x == maxRight -1)//if it's against right edge, only bottom left corner
        {
          if(goalCheck(tempNodeMap[y][x-1],tempNodeMap[y][x]))//west
          return;
          if(goalCheck(tempNodeMap[y+1][x-1],tempNodeMap[y][x]))//southwest
          return;
          if(goalCheck(tempNodeMap[y+1][x],tempNodeMap[y][x]))//south
          return;
        }
        else//Check all but top row
        {
          if(goalCheck(tempNodeMap[y][x-1],tempNodeMap[y][x]))//west
          return;
          if(goalCheck(tempNodeMap[y][x+1],tempNodeMap[y][x]))//east
          return;
          if(goalCheck(tempNodeMap[y+1][x-1],tempNodeMap[y][x]))//southwest
          return;
          if(goalCheck(tempNodeMap[y+1][x],tempNodeMap[y][x]))//south
          return;
          if(goalCheck(tempNodeMap[y+1][x+1],tempNodeMap[y][x]))//southeast
          return;
          astarProcess(tempNodeMap[y][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//west
          astarProcess(tempNodeMap[y][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//east
          astarProcess(tempNodeMap[y+1][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//southwest
          astarProcess(tempNodeMap[y+1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//south
          astarProcess(tempNodeMap[y+1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//southeast
        }
      }
      else if(y == maxBottom -1)//if it's against bottom edge
      {
        if(x == 0)//if it's against the left edge, only topright
        {
          if(goalCheck(tempNodeMap[y-1][x],tempNodeMap[y][x]))//north
          return;
          if(goalCheck(tempNodeMap[y-1][x+1],tempNodeMap[y][x]))//northeast
          return;
          if(goalCheck(tempNodeMap[y][x+1],tempNodeMap[y][x]))//east
          return;
          astarProcess(tempNodeMap[y-1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//north
          astarProcess(tempNodeMap[y-1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northeast
          astarProcess(tempNodeMap[y][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//east
        }
        else if(x == maxRight -1)//if it's against right edge, only top left
        {
          if(goalCheck(tempNodeMap[y-1][x-1],tempNodeMap[y][x]))//northwest
          return;
          if(goalCheck(tempNodeMap[y-1][x],tempNodeMap[y][x]))//north
          return;
          if(goalCheck(tempNodeMap[y][x-1],tempNodeMap[y][x]))//west
          return;
          astarProcess(tempNodeMap[y-1][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northwest
          astarProcess(tempNodeMap[y-1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//north
          astarProcess(tempNodeMap[y][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//west
        }
        else//check all but bottom row 
        {
          if(goalCheck(tempNodeMap[y-1][x-1],tempNodeMap[y][x]))//northwest
          return;
          if(goalCheck(tempNodeMap[y-1][x],tempNodeMap[y][x]))//north
          return;
          if(goalCheck(tempNodeMap[y-1][x+1],tempNodeMap[y][x]))//northeast
          return;
          if(goalCheck(tempNodeMap[y][x-1],tempNodeMap[y][x]))//west
          return;
          if(goalCheck(tempNodeMap[y][x+1],tempNodeMap[y][x]))//east
          return;
          astarProcess(tempNodeMap[y-1][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northwest
          astarProcess(tempNodeMap[y-1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//north
          astarProcess(tempNodeMap[y-1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northeast
          astarProcess(tempNodeMap[y][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//west
          astarProcess(tempNodeMap[y][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//east
        }
      }
      else 
      {
        if(x == 0)//if it's against left edge, only check right two columns
        {
          if(goalCheck(tempNodeMap[y-1][x],tempNodeMap[y][x]))//north
          return;
          if(goalCheck(tempNodeMap[y-1][x+1],tempNodeMap[y][x]))//northeast
          return;
          if(goalCheck(tempNodeMap[y][x+1],tempNodeMap[y][x]))//east
          return;
          if(goalCheck(tempNodeMap[y+1][x],tempNodeMap[y][x]))//south
          return;
          if(goalCheck(tempNodeMap[y+1][x+1],tempNodeMap[y][x]))//southeast
          return;
          astarProcess(tempNodeMap[y-1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//north
          astarProcess(tempNodeMap[y-1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northeast
          astarProcess(tempNodeMap[y][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//east
          astarProcess(tempNodeMap[y+1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//south
          astarProcess(tempNodeMap[y+1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//southeast
        }
        else if(x == maxRight -1) //if it's against right edge, only check left two columns
        {
          if(goalCheck(tempNodeMap[y-1][x-1],tempNodeMap[y][x]))//northwest
          return;
          if(goalCheck(tempNodeMap[y-1][x],tempNodeMap[y][x]))//north
          return;
          if(goalCheck(tempNodeMap[y][x-1],tempNodeMap[y][x]))//west
          return;
          if(goalCheck(tempNodeMap[y+1][x-1],tempNodeMap[y][x]))//southwest
          return;
          if(goalCheck(tempNodeMap[y+1][x],tempNodeMap[y][x]))//south
          return;
          astarProcess(tempNodeMap[y-1][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northwest
          astarProcess(tempNodeMap[y-1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//north
          astarProcess(tempNodeMap[y][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//west
          astarProcess(tempNodeMap[y+1][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//southwest
          astarProcess(tempNodeMap[y+1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//south
        }
        else//check all sides
        {
          if(goalCheck(tempNodeMap[y-1][x-1],tempNodeMap[y][x]))//northwest
          return;
          if(goalCheck(tempNodeMap[y-1][x],tempNodeMap[y][x]))//north
          return;
          if(goalCheck(tempNodeMap[y-1][x+1],tempNodeMap[y][x]))//northeast
          return;
          if(goalCheck(tempNodeMap[y][x-1],tempNodeMap[y][x]))//west
          return;
          if(goalCheck(tempNodeMap[y][x+1],tempNodeMap[y][x]))//east
          return;
          if(goalCheck(tempNodeMap[y+1][x-1],tempNodeMap[y][x]))//southwest
          return;
          if(goalCheck(tempNodeMap[y+1][x],tempNodeMap[y][x]))//south
          return;
          if(goalCheck(tempNodeMap[y+1][x+1],tempNodeMap[y][x]))//southeast
          return;
          astarProcess(tempNodeMap[y-1][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northwest
          astarProcess(tempNodeMap[y-1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//north
          astarProcess(tempNodeMap[y-1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//northeast
          astarProcess(tempNodeMap[y][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//west
          astarProcess(tempNodeMap[y][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//east
          astarProcess(tempNodeMap[y+1][x-1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//southwest
          astarProcess(tempNodeMap[y+1][x], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.00);//south
          astarProcess(tempNodeMap[y+1][x+1], tempNodeMap[y][x], nodesBeingSearchedFrom,  1.41);//southeast

        }

      }
    
    }
  }
  nodeMap = tempNodeMap;
  return;
}

// //#pragma endregion

//#pragma region Movement
int aStarStep = 0;
float K_linear = 4;
float K_angular = 4;
float distThresh = 0.2;
float angleThresh = 0.2;
float goalThresh = 0.5;
void MoveAlongAstar()
{
  vector<float>  nextGoal = IndexToPosition(pathCoordinates[aStarStep][0],pathCoordinates[aStarStep][1]);
  if(aStarStep == pathCoordinates.size())
  {
    nextGoal = {xGoal, yGoal};
  }
    
    //get error
  float goalDx = xGoal - robotPose[0];
  float goalDy = yGoal - robotPose[1];
  float goalDist = sqrt(goalDx*goalDx + goalDy*goalDy);
  float dx = nextGoal[0] - robotPose[0];
  float dy = nextGoal[1] - robotPose[1];
  float distance = sqrt(dx*dx + dy*dy);
  float direction = atan2(dy,dx);
  float error = direction - euler_orientation.z;
  if(goalDist < goalThresh)
  {
    cout << "!!!!CONGRATULATIONS YOU ARRIVED AT THE GOAL!!!!" << endl;
    foundDestination = false;
    robotVelocity.linear.x = 0.0;
    robotVelocity.angular.z = 0.0;
    drive_pub.publish(robotVelocity);
  return;
  }
  if(error > pi)
  {
    error = error - 2*pi;
  }
  if(error < -pi)
  {
    error = error + 2*pi;
  }
  //correct heading 
  if(abs(distance) >  distThresh)
  {
    if(abs(error) > angleThresh)
    {
      robotVelocity.linear.x = 0;
      robotVelocity.angular.z = K_angular*error;
    }
    else
    {
      robotVelocity.linear.x = K_linear*distance;
      robotVelocity.angular.z = 0.0;
    }
  }
  else
  {
    aStarStep++;
    robotVelocity.linear.x = 0.0;
    robotVelocity.angular.z = 0.0;
  }
    drive_pub.publish(robotVelocity);

}

//#pragma endregion



int main(int argc, char **argv)
{
  /*--------------------GOAL SET----------------------*/
  xGoal = 4.5;
  yGoal = 9.0;
  /*--------------------GOAL SET----------------------*/
  ////calculate scan stuff
  oldPositionIndex.push_back(0);
  oldPositionIndex.push_back(0);
  //ros initialization stuff
  ros::init(argc, argv, "astar");
  ros::NodeHandle n;
  std::string scan_topic, drive_topic, odom_topic, move_topic; 
  // n.getParam("lab4/odom_topic", odom_topic);
    // n.getParam("lab4/map", importedMap);
      // n.getParam("map", fpath);

  ros::Rate loop_rate(100);
  mapToArray(18, 20);
  occupancyMap = costMap;
    odom_sub = n.subscribe("/odom", 1, odom_callback);

  // ROS_ERROR_STREAM("3");
  drive_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  while (ros::ok()) 
  {
    ros::spinOnce();
    loop_rate.sleep();
    if(countDownToPathPlan > thresholdForPathPlan)
    {
      aStarStep = 1;
      countDownToPathPlan -= thresholdForPathPlan;
      robotVelocity.linear.x = 0;
      robotVelocity.angular.z = 0;
      drive_pub.publish(robotVelocity);
        calculateHCost();
        planAStar();  
        pathMap();     
        if(!foundDestination)
        countDownToPathPlan += thresholdForPathPlan-3;
    }
    if(foundDestination)
    {
      
      MoveAlongAstar();

    }
  }
  return 0;
}
