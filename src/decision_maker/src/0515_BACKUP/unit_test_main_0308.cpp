#include <iostream>
//#include "../inc/ParkingPlannerThread.h"
#include <ros/ros.h>    //ROS기본 헤더 파일
#include <cmath>
#include "../inc/VectorPursuit.h"
#include "../inc/quadtree.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeometricUtils.h>
#include <MathParam.h>
#include <AngleUtils.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ClothoidStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/DORRTstar.h>
#include <boost/program_options.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ITER = 1;
bool isOriginal = false;
double START[3]={85.0,-8.0,RADIANS(179)};
double GOAL[3]={20.0,10.0,RADIANS(-90)};
vector<Vector2d> OBSTACLE;
ob::RealVectorBounds BOUNDS(2);
vector<vector<VectorXd> > VectorMap;
vector<vector<VectorXd> > VectorObstacleMap;
vector<vector<double> > VectorMapSize;
vector<vector<VectorXd> > VectorCollisionMap;

ros::Publisher potential_publisher_;
ros::Publisher potential2_publisher_;
ros::Publisher potential3_publisher_;
ros::Publisher msgpub3;
ros::Publisher map_pub;
geometry_msgs::PoseArray poseArray;
geometry_msgs::PoseArray poseArray2;
geometry_msgs::PoseArray poseArray3;

//Desired Orientation
int RAND_ANGLE_RANGE = 30; // UNIT: DEGREE

// GRID
double grid_offset_x=20.0;
double grid_offset_y=22.0;

double m_per_cell_ = 0.5;

int grid_dim_x_=0.0;
int grid_dim_y_=0.0;

//Potential Field
double SafeRegion = 0.3;  
  
double Range_obs = 1.0;  
double K_rep_obs = 0.05f;

double Range_rep=180.0;
double Range_att=180.0;

double K_rep = 100.0;    
double K_att = 0.002;

//NAG
//double vfgain_ = 1.0;
//double vfMgain_ = 0.5;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetEnvironment(int type)
{
    switch(type)
    {
        case 0:
            START[0]=30.0;
            START[1]=50.0;
            START[2]=RADIANS(0.1);
            GOAL[0]=80.0;
            GOAL[1]=0.0;
            GOAL[2]=RADIANS(90);

            BOUNDS.low[0] = 20.0;
            BOUNDS.high[0] = 90.0;

            BOUNDS.low[1] = -10;
            BOUNDS.high[1] = 60.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            for(double x=50; x<60; x+=0.2 )
                for(double y=-10; y<20; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=50; x<60; x+=0.2 )
                for(double y=25; y<60; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));

            for(double x=-20; x<90; x+=0.2 )
                for(double y=-10; y<-8; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-20; x<90; x+=0.2 )
                for(double y=58; y<60; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            break;
        case 1:
            START[0]=85.0;
            START[1]=0.0;
            START[2]=RADIANS(0.1);
            GOAL[0]=00.0;
            GOAL[1]=10.0;
            GOAL[2]=RADIANS(-90);

            BOUNDS.low[0] = -20.0;
            BOUNDS.high[0] = 90.0;

            BOUNDS.low[1] = -50;
            BOUNDS.high[1] = 60.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            for(double x=8; x<10; x+=0.4 )
                for(double y=2; y<15; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));
            
            for(double x=25; x<27; x+=0.4 )
                for(double y=-15; y<5; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=65; x<67; x+=0.4 )
                for(double y=-13; y<5; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=50; x<65; x+=0.4 )
                for(double y=0; y<2; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-20; x<90; x+=0.4 )
                for(double y=-24; y<-22; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-20; x<90; x+=0.4 )
                for(double y=16; y<18; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));
            break;
        case 2: // Backward Driving
            START[0]=28.0;
            START[1]=10.0;
            START[2]=RADIANS(-90);
            GOAL[0]=5.0;
            GOAL[1]=5.0;
            GOAL[2]=RADIANS(-90);

            BOUNDS.low[0] = -20.0;
            BOUNDS.high[0] = 40.0;

            BOUNDS.low[1] = -20;
            BOUNDS.high[1] = 20.0;
        
            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            for(double x=-12; x<25; x+=0.2 )
            {
                if( x > 3 && x < 7 ) continue;

                for(double y=-10; y<8; y+=0.2)
                {
                    if( y < 1 && y > -8 ) continue;
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            
            for(double x=3; x<7; x+=0.2 )
            {
                for(double y=-10; y<-8; y+=0.2)
                {
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            for(double x=32; x<40; x+=0.2 )
                for(double y=-12; y<12; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-12; x<25; x+=0.2 )
            {
                for(double y=-12; y<-10; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
                for(double y=8; y<12; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            }
            for(double x=-4; x<-2; x+=0.2 )
                for(double y=-10; y<-6; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=10; x<12; x+=0.2 )
                for(double y=-10; y<-6; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
          break;
        case 3: // Forward Driving
            START[0]=28.0;
            START[1]=10.0;
            START[2]=RADIANS(-90);
            GOAL[0]=5.0;
            GOAL[1]=4.0;
            GOAL[2]=RADIANS(90);

            BOUNDS.low[0] = -20.0;
            BOUNDS.high[0] = 40.0;

            BOUNDS.low[1] = -20;
            BOUNDS.high[1] = 20.0;
        
            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            for(double x=-12; x<25; x+=0.2 )
            {
                if( x > 3 && x < 7 ) continue;

                for(double y=-10; y<8; y+=0.2)
                {
                    if( y < 1 && y > -8 ) continue;
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            for(double x=3; x<7; x+=0.2 )
            {
                for(double y=-10; y<-8; y+=0.2)
                {
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            for(double x=32; x<40; x+=0.2 )
                for(double y=-12; y<12; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-12; x<25; x+=0.2 )
            {
                for(double y=-12; y<-10; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
                for(double y=8; y<12; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            }
            for(double x=-4; x<-2; x+=0.2 )
                for(double y=-10; y<-6; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=10; x<12; x+=0.2 )
                for(double y=-10; y<-6; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
          break;
         case 4:
            START[0]=2.5;
            START[1]=3.0;
            START[2]=RADIANS(90);
            GOAL[0]=32.5;
            GOAL[1]=6.0;
            GOAL[2]=RADIANS(-90);

            BOUNDS.low[0] = -10.0;
            BOUNDS.high[0] = 50.0;

            BOUNDS.low[1] = -10;
            BOUNDS.high[1] = 50.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            for(double x=5; x<10; x+=0.2 )
                for(double y=0; y<40; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=15; x<20; x+=0.2 )
                for(double y=10; y<50; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=25; x<30; x+=0.2)
                for(double y=0; y<40; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=35; x<40; x+=0.2)
                for(double y=0; y<50; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-5; x<0; x+=0.2)
                for(double y=0; y<50; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-5; x<50; x+=0.2)
                for(double y=-5; y<0; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-5; x<50; x+=0.2)
                for(double y=50; y<55; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
          break;
         case 5:
            START[0]=28.0;
            START[1]=10.0;
            START[2]=RADIANS(-90);
            GOAL[0]=3.0;
            GOAL[1]=-7.0;
            GOAL[2]=RADIANS(0);

            BOUNDS.low[0] = -20.0;
            BOUNDS.high[0] = 40.0;

            BOUNDS.low[1] = -20;
            BOUNDS.high[1] = 20.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            for(double x=-12; x<25; x+=0.2 )
            {
                if( x > 3 && x < 7 ) continue;

                for(double y=-10; y<8; y+=0.2)
                {
                    if( y < 1 && y > -8 ) continue;
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            for(double x=3; x<7; x+=0.2 )
            {
                for(double y=-10; y<-8; y+=0.2)
                {
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            for(double x=32; x<40; x+=0.2 )
                for(double y=-12; y<12; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-12; x<25; x+=0.2 )
            {
                for(double y=-12; y<-10; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
                for(double y=10; y<12; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            }
            for(double x=-4; x<-2; x+=0.2 )
                for(double y=-10; y<-6; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=10; x<12; x+=0.2 )
                for(double y=-10; y<-6; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));
          break;
       default:
            break;
    }
   
    grid_dim_x_ = ceil((BOUNDS.high[0] - BOUNDS.low[0])/m_per_cell_);
    grid_dim_y_ = ceil((BOUNDS.high[1] - BOUNDS.low[1])/m_per_cell_);
    
    nav_msgs::MapMetaData info;
    info.width = grid_dim_x_;
    info.height = grid_dim_y_;
    info.resolution = m_per_cell_;

    info.origin.position.x = -grid_offset_x;
    info.origin.position.y = -grid_offset_y;

    QuadTree::Map map(info.width, info.height);
    
    for (int x = 0; x < info.width; x++)
    {
        for (int y = 0; y < info.height; y++)
        {
            int obsval = 0;
            for(int k=0; k<OBSTACLE.size(); k++)
            {
                int x_obs= int((OBSTACLE[k](0) +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
                int y_obs= int((OBSTACLE[k](1) +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);

                if( x == x_obs && y == y_obs )
                {
                    obsval =1;
                    break;
                }
            }
            map.Insert(QuadTree::Cell(x,y,info.width,obsval));
        }
    }

    nav_msgs::OccupancyGrid* newGrid = map.Grid();
    
    newGrid->header.frame_id = "/camera_init";
    newGrid->header.stamp = ros::Time::now();
    newGrid->info = info;
    map_pub.publish(*newGrid);
}



void magneticVectorForce(const double* init, const double* target, double *B)
{
	double ln = 60.0; 	// Length of conductors
	double d = 1.61/2.0;		// Distance between the conductors m & n
	int m = 1; 		// Direction of current through the conductor, m(Right) 1(INTO), -1(OUT)
	int n = -1; 		// Direction of current through the conductor, n(Left) 1(INTO), -1(OUT)
	int N = 12; 		// Number sections/elements in the conductors
	int dl = ln/N; 		// Length of each element

    double xxP = target[0];
    double zzP = target[1];

    double q[3]={init[0], init[1], init[2]-RADIANS(90.0)};

	MatrixXd Rot(2,2);
	Rot << cos(q[2]), -sin(q[2]),
	       sin(q[2]),  cos(q[2]);

    MatrixXd Cm = Vector2d(q[0], q[1])+Rot*Vector2d(d/2.0,0); // Cm_ --> (2,1)
    MatrixXd Cn = Vector2d(q[0], q[1])+Rot*Vector2d(-d/2.0,0); // Cn_ --> (2,1)
    //cout << Cm << endl;
    //cout << Cn << endl;
 
	MatrixXd xCm = Cm(0)*MatrixXd::Ones(1,N); 
	MatrixXd xCn = Cn(0)*MatrixXd::Ones(1,N);
    
    // cout <<xCm <<endl;
    // cout <<xCn <<endl;

	// Y Coordinate of each element from origin, half on +Y & other half on -Y and also sam for both conductors
    double low = -ln/2.0+dl/2.0;
    double high = ln/2.0-dl/2.0;

	VectorXd yC = VectorXd::LinSpaced( ((high-low)/dl+1),low, low+dl*(N-1) );
    //cout << yC <<endl;

    // zC remains 0 throughout the length, as conductors are lying on XY plane
	MatrixXd zCm = Cm(1)*MatrixXd::Ones(1,N);
	MatrixXd zCn = Cn(1)*MatrixXd::Ones(1,N);
	
    //cout << zCm << endl;
    //cout << zCn << endl;

	// Length(Projection) 7 Direction of each current element in Vector form
	MatrixXd Lx = MatrixXd::Zero(1,N);	// Length of each element is zero on X axis
	MatrixXd Ly = dl*MatrixXd::Ones(1,N);	// Length of each element is dl on Y axis
	MatrixXd Lz = MatrixXd::Zero(1,N);	// Length of each element is zero on Z axis
    
    double Bx = 0;
    double By = 0;
    double Bz = 0;

	for( int i=0; i<N; i++ )
	{
		double rxm = xxP - xCm(i); // Displacement Vector along X direction, from cond m..
		double rxn = xxP - xCn(i); // Displacement Vector along X direction, from cond n..
		
		double ry = yC(i);	// Same for m & n, no detector points on Y direction..
		
		double rzm = zzP - zCm(i); // Same for m & n..
		double rzn = zzP - zCn(i); // Same for m & n..
		
        double rm = sqrt(rxm*rxm + ry*ry + rzm*rzm);// Displacement Magnitude for an element on cond m..
        double rn = sqrt(rxn*rxn + ry*ry + rzn*rzn);// Displacement Magnitude for an element on cond n..

		double r3m = rm*rm*rm;
		double r3n = rn*rn*rn;
		
		Bx = Bx + m*Ly(i)*rzm/r3m + n*Ly(i)*rzn/r3n;	// m & n, direction of current element..
		Bz = Bz - m*Ly(i)*rxm/r3m - n*Ly(i)*rxn/r3n;	// m & n, direction of current element..
		// By = 0;
    //    cout << Bx <<" "<< Bz <<" " <<endl;
	}
    B[0] += Bx;
    B[1] += Bz;
}

double computeRepulsiveForce(double K, double dist, double range, double x, double tar_x)
{
    // cout <<"A "<< dist << " " << range <<" " << x << " " << tar_x << " " << K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist*dist))*(x-tar_x) << endl;

    if( dist <= range )
        return K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist*dist))*(x-tar_x);
    else
        return 0;

}

double computeAttractiveForce(double K, double x, double tar_x)
{
    return -1.0f * K * (x - tar_x);
}

void VectorFieldGenerator(double* start, double* goal, ob::RealVectorBounds bounds)
{
    ///////////////////////////////////////////////////////////////////
    // 1. Potential Field

    int grid_dim_x = ceil((bounds.high[0] - bounds.low[0])/m_per_cell_);
    int grid_dim_y = ceil((bounds.high[1] - bounds.low[1])/m_per_cell_);


    double num_obs[grid_dim_x][grid_dim_y];
    
	// Initialize Potential Map
    for (int x = 0; x < grid_dim_x; x++)
    {
		vector<VectorXd> v;
		VectorMap.push_back(v);
		
        vector<double> vs;
        VectorMapSize.push_back(vs);
        
        vector<VectorXd> v_;
		VectorCollisionMap.push_back(v_);
		
        vector<VectorXd> v_obs;
		VectorObstacleMap.push_back(v_obs);
        
        for (int y = 0; y < grid_dim_y; y++)
        {
            VectorXd vec = VectorXd::Zero(5);
            VectorMap[x].push_back(vec);

            VectorXd vecObs = VectorXd::Zero(5);
            VectorObstacleMap[x].push_back(vecObs);
            
            VectorXd vecCollision = VectorXd::Zero(5);
            VectorCollisionMap[x].push_back(vecCollision);

            VectorMapSize[x].push_back(0);
        }
    }
    
    for (int x = 0; x < grid_dim_x; x++) {
        for (int y = 0; y < grid_dim_y; y++) {
            num_obs[x][y]=0;
        }
    }
    
    for (unsigned int k=0; k<OBSTACLE.size(); k++)
    {
        int i= int((OBSTACLE[k](0) +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
        int j= int((OBSTACLE[k](1) +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
        if( i >= 0 && i < grid_dim_x && j >= 0 && j < grid_dim_y )
        {
            num_obs[i][j]++;
        }
    }
    
	
    for (int i = 0; i<grid_dim_x; i++)
    {
        for (int j = 0; j<grid_dim_y; j++)
        {
            // transform index to local coordinate (x,y)
            double x = -grid_offset_x + (i*m_per_cell_+m_per_cell_/2.0);
            double y = -grid_offset_y + (j*m_per_cell_+m_per_cell_/2.0);
            VectorMap[i][j](0) = x;
            VectorMap[i][j](1) = y;
            VectorCollisionMap[i][j](0) = x;
            VectorCollisionMap[i][j](1) = y;
            VectorObstacleMap[i][j](0) = x;
            VectorObstacleMap[i][j](1) = y;

            
            double x_att = computeAttractiveForce(K_att, x, goal[0]);
            double y_att = computeAttractiveForce(K_att, y, goal[1]);

            //double goal_dist = sqrt(((x - goal[0]) * (x - goal[0]))
            //      + ((y - goal[1]) * (y - goal[1])));
            //double x_att = -computeRepulsiveForce(K_att, goal_dist,
            //    Range_att, x, goal[0]);
            // double y_att = -computeRepulsiveForce(K_att, goal_dist,
            //  Range_att, y, goal[1]);

            double start_dist = sqrt(((x - start[0]) * (x - start[0]))
                    + ((y - start[1]) * (y - start[1])));
            double x_rep = computeRepulsiveForce(K_rep, start_dist,
                    Range_rep, x, start[0]);
            double y_rep = computeRepulsiveForce(K_rep, start_dist,
                    Range_rep, y, start[1]);

            // Vector Force (Unit Vector)
            VectorMap[i][j](2) +=(x_att + x_rep);
            VectorMap[i][j](3) +=(y_att + y_rep);

            // Generate obstacle potential
            for (unsigned int k = 0; k < OBSTACLE.size(); k++)
            {
                double obs_x = OBSTACLE[k](0);
                double obs_y = OBSTACLE[k](1);

                if(obs_x == 0.0 && obs_y == 0.0)
                    continue;

                double obs_dist = sqrt(((x - obs_x) * (x - obs_x))
                        + ((y - obs_y) * (y - obs_y)));

                if (obs_dist > SafeRegion && obs_dist < Range_obs)
                {
//                    double obs_pos[2]={obs_x, obs_y};
//                    double tar_pos[2]={x,y};
//                    double B[2]={0,0};
//                    vortexVectorForce(obs_pos,tar_pos, B);
//                     VectorMap[i][j](2) += B[0];                    
//                     VectorMap[i][j](3) += B[1];                   
                    double rep_x = computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, x, obs_x);
                    double rep_y = computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, y, obs_y);

                    VectorMap[i][j](2) += rep_x; 
                    VectorMap[i][j](3) += rep_y; 
                    
                    VectorObstacleMap[i][j](2) += rep_x; 
                    VectorObstacleMap[i][j](3) += rep_y; 
                    
                    VectorMapSize[i][j] += sqrt(pow(rep_x,2)+pow(rep_y,2));
                }
                else if(obs_dist <= SafeRegion)
                {
                   // VectorMap[i][j](2) = 0.0;
                   // VectorMap[i][j](3) = 0.0;

                    VectorCollisionMap[i][j](2) = 1;
                }
            }
            ////////////////////////////////////////
            /* RVIZ */ 
            geometry_msgs::PoseStamped poseStamped;

            poseStamped.pose.position.x = VectorMap[i][j](0);
            poseStamped.pose.position.y = VectorMap[i][j](1);
            poseStamped.pose.position.z = 0.0;

            double vTheta = atan2(VectorMap[i][j](3),VectorMap[i][j](2));
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vTheta);

            poseStamped.pose.orientation = odom_quat;
            poseArray2.poses.push_back(poseStamped.pose);
 
        }
    }

   

}
//////////////////////////////////////////////////////////////////////////////////////////////////
// DO
bool DO( double* from_d, double* to_d, double* target_d)
{
    VectorXd ret = VectorXd::Zero(2);
    
    if( isOriginal || (to_d[0] == GOAL[0] && to_d[1] == GOAL[1]) )
    {
        return false;
    }
 
    int i= int((target_d[0] +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((target_d[1] +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        double target_ori[3];
        target_ori[0] = target_d[0];
        target_ori[1] = target_d[1];
        target_ori[2] = target_d[2];

        double w1 = 0.1;
        double w2 = 0.3;
        int ITERMAX = 50;
        double GRADLIMIT = 0.05;
        int iter = 0;
        Vector3d vGradient = Vector3d::Zero(3);
        Vector3d vMomentum = Vector3d::Zero(3);
        Vector3d vFeasibleDirection = Vector3d::Zero(3);
        while(iter < ITERMAX)
        {
            // Update Gradient
            vGradient(0) = VectorMap[i][j](2);
            vGradient(1) = VectorMap[i][j](3);
            double vGradientSize = VectorMapSize[i][j];
            vGradient.normalize();
            
            // Update Momentum
            double B[2]={0,0};
            magneticVectorForce(from_d, target_d, B);
            magneticVectorForce(to_d, target_d, B);
            //target_d[2] =AngleUtils::toRange_PItoPI( atan2(B[1], B[0]));

            
            vMomentum(0) = cos(target_d[2]);
            vMomentum(1) = sin(target_d[2]);

            Vector3d vDir = Vector3d::Zero(3);
            vDir = vMomentum.cross(vGradient);

            double vDirAng = vDir(2);
            if( fabs(vDir(2)) > RADIANS(1.0))
            {

                if( vDir(2) > 0 )
                {
                    vDirAng = RADIANS(1.0);
                }
                else
                {
                    vDirAng =-RADIANS(1.0);
                }

            }
            target_d[2] = AngleUtils::toRange_PItoPI(target_d[2]+vDirAng);
            vFeasibleDirection(0) = cos(target_d[2]);
            vFeasibleDirection(1) = sin(target_d[2]);
            target_d[0] += w1*vFeasibleDirection(0);
            target_d[1] += w1*vFeasibleDirection(1);

//            target_d[0] += w1*vGradientSize*vGradient(0)+w2*vMomentum(0);
//            target_d[1] += w1*vGradientSize*vGradient(1)+w2*vMomentum(1);

           
            i= int((target_d[0] +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
            j= int((target_d[1] +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
            if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
            {
           
          //      cout <<iter<<" "<<vGradientSize <<" "<<GRADLIMIT <<endl;
                if( vGradientSize < GRADLIMIT )
                    break;
                
                iter++;
                ////////////////////////////////////////
                geometry_msgs::PoseStamped poseStamped;

                poseStamped.pose.position.x = target_d[0];
                poseStamped.pose.position.y = target_d[1];
                poseStamped.pose.position.z = 0.0;

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);

                poseStamped.pose.orientation = odom_quat;
                poseArray3.poses.push_back(poseStamped.pose);
                ////////////////////////////////////////
            }
            else
            {
                target_d[0] = target_ori[0];
                target_d[1] = target_ori[1];
                target_d[2] = target_ori[2];
                return false;
            }

        }
        if( std::isnan(target_d[0])|| std::isnan(target_d[1])||std::isnan(target_d[2]))
        {

            target_d[0] = target_ori[0];
            target_d[1] = target_ori[1];
            target_d[2] = target_ori[2];
            return false;
        }
        else
        {
            ////////////////////////////////////////
            geometry_msgs::PoseStamped poseStamped;

            poseStamped.pose.position.x = target_d[0];
            poseStamped.pose.position.y = target_d[1];
            poseStamped.pose.position.z = 0.0;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);

            poseStamped.pose.orientation = odom_quat;
            poseArray.poses.push_back(poseStamped.pose);
            ////////////////////////////////////////
            return true;
        }
    }
    return false;
}
bool isFreeSpace(const vector<vector<VectorXd> >& VectorCollisionMap,int i, int j)
{
    bool isFreeSpace = true;
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        if( VectorCollisionMap[i][j](2) == 1 )
        {
            isFreeSpace = false;
        }
    }
    return isFreeSpace;

}
bool isValid(const vector<vector<VectorXd> >& VectorCollisionMap,double x, double y, double yaw)
{
    double x_ = x;
    double y_ = y;

    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);

    bool isFreeSpace1 = isFreeSpace(VectorCollisionMap, i, j);
   
    double vehicle_front_x = x + 3.2*cos(yaw);
    double vehicle_front_y = y + 3.2*sin(yaw);

    i= int((vehicle_front_x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((vehicle_front_y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);

    double vehicle_back_x = x - 0.8*cos(yaw);
    double vehicle_back_y = y - 0.8*sin(yaw);
    
    bool isFreeSpace2 = isFreeSpace(VectorCollisionMap, i, j);
    
    i= int((vehicle_back_x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((vehicle_back_y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    bool isFreeSpace3 = isFreeSpace(VectorCollisionMap, i, j);
    
    //////////////////////////////////////////////////////////////////////////////
    x = x_ + cos(yaw+RADIANS(90.0));
    y = y_ + sin(yaw+RADIANS(90.0));
        
    i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);

    bool isFreeSpace4 = isFreeSpace(VectorCollisionMap, i, j);
   
    vehicle_front_x = x + 3.2*cos(yaw);
    vehicle_front_y = y + 3.2*sin(yaw);

    i= int((vehicle_front_x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((vehicle_front_y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);

    vehicle_back_x = x - 0.8*cos(yaw);
    vehicle_back_y = y - 0.8*sin(yaw);
    
    bool isFreeSpace5 = isFreeSpace(VectorCollisionMap, i, j);
    
    i= int((vehicle_back_x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((vehicle_back_y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    bool isFreeSpace6 = isFreeSpace(VectorCollisionMap, i, j);
    //////////////////////////////////////////////////////////////////////////////
    x = x_ + cos(yaw-RADIANS(90.0));
    y = y_ + sin(yaw-RADIANS(90.0));
    
    i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);

    bool isFreeSpace7 = isFreeSpace(VectorCollisionMap, i, j);
   
    vehicle_front_x = x + 3.2*cos(yaw);
    vehicle_front_y = y + 3.2*sin(yaw);

    i= int((vehicle_front_x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((vehicle_front_y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);

    vehicle_back_x = x - 0.8*cos(yaw);
    vehicle_back_y = y - 0.8*sin(yaw);
    
    bool isFreeSpace8 = isFreeSpace(VectorCollisionMap, i, j);
    
    i= int((vehicle_back_x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    j= int((vehicle_back_y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    bool isFreeSpace9 = isFreeSpace(VectorCollisionMap, i, j);
   
    return isFreeSpace1 && isFreeSpace2 && isFreeSpace3 && isFreeSpace4 && isFreeSpace5 && isFreeSpace6 &&isFreeSpace7 && isFreeSpace8 && isFreeSpace9;
}


VectorXd DesiredOrientation( ob::State* from, ob::State* to, ob::State* target)
{
    
    ob::ClothoidStateSpace::StateType *from_ = from->as<ob::ClothoidStateSpace::StateType>();
    ob::ClothoidStateSpace::StateType *to_ = to->as<ob::ClothoidStateSpace::StateType>();
    ob::ClothoidStateSpace::StateType *target_ = target->as<ob::ClothoidStateSpace::StateType>();
   
    double from_d[3]; from_d[0]=from_->getX(); from_d[1]=from_->getY(); from_d[2]=from_->getYaw();
    double to_d[3]; to_d[0]=to_->getX(); to_d[1]=to_->getY(); to_d[2]=to_->getYaw();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( DO(from_d, to_d, target_d) )
    {
        target_->setX(target_d[0]);
        target_->setY(target_d[1]);
        target_->setYaw(target_d[2]);
    }
    return VectorXd::Zero(2);
}

VectorXd DesiredOrientation_(ob::State* from, ob::State* to, ob::State* target)
{
    ob::SE2StateSpace::StateType *from_ = from->as<ob::SE2StateSpace::StateType>();
    ob::SE2StateSpace::StateType *to_ = to->as<ob::SE2StateSpace::StateType>();
    ob::SE2StateSpace::StateType *target_ = target->as<ob::SE2StateSpace::StateType>();
   
    double from_d[3]; from_d[0]=from_->getX(); from_d[1]=from_->getY(); from_d[2]=from_->getYaw();
    double to_d[3]; to_d[0]=to_->getX(); to_d[1]=to_->getY(); to_d[2]=to_->getYaw();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( DO(from_d, to_d, target_d) )
    {
        target_->setX(target_d[0]);
        target_->setY(target_d[1]);
        target_->setYaw(target_d[2]);
    }
    return VectorXd::Zero(2);
}

VectorXd magneticfield(ob::State* target, ob::State* nouse1, ob::State* nouse2)
{

//    return VectorXd::Zero(2);
    ob::ClothoidStateSpace::StateType *target_ = target->as<ob::ClothoidStateSpace::StateType>();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();
    
 
    int i= int((target_d[0] +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((target_d[1] +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        double target_ori[3];
        target_ori[0] = target_d[0];
        target_ori[1] = target_d[1];
        target_ori[2] = target_d[2];

        VectorXd vGradient = VectorXd::Zero(2);
        vGradient(0) = VectorMap[i][j](2);
        vGradient(1) = VectorMap[i][j](3);
        vGradient.normalize();
   
        target_d[2] =AngleUtils::toRange_PItoPI( atan2(vGradient(1), vGradient(0)));
        if( isOriginal || (target_d[0] == GOAL[0] && target_d[1] == GOAL[1]) )
        {
            return VectorXd::Zero(2);
        }
        else
        {
        //    if( rand()<0.8 )
            {
                target_->setYaw(target_d[2]);
            }
            return VectorXd::Zero(2);
        }
    }
    return VectorXd::Zero(2);
}
VectorXd magneticfield_(ob::State* target, ob::State* nouse1, ob::State* nouse2)
{
//    return VectorXd::Zero(2);
    ob::SE2StateSpace::StateType *target_ = target->as<ob::SE2StateSpace::StateType>();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();
    
 
    int i= int((target_d[0] +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((target_d[1] +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        double target_ori[3];
        target_ori[0] = target_d[0];
        target_ori[1] = target_d[1];
        target_ori[2] = target_d[2];

        VectorXd vGradient = VectorXd::Zero(2);
        vGradient(0) = VectorMap[i][j](2);
        vGradient(1) = VectorMap[i][j](3);
        vGradient.normalize();
   
        target_d[2] =AngleUtils::toRange_PItoPI( atan2(vGradient(1), vGradient(0)));
        if( isOriginal || (target_d[0] == GOAL[0] && target_d[1] == GOAL[1]) )
        {
            return VectorXd::Zero(2);
        }
        else
        {
          //  if( rand()<0.8 )
            {
                target_->setYaw(target_d[2]);
            }
            return VectorXd::Zero(2);
        }
    }
    return VectorXd::Zero(2);
}

bool randomCheck_(ob::State* rand)
{

    ob::SE2StateSpace::StateType *rand_ = rand->as<ob::SE2StateSpace::StateType>();
    
    double x = rand_->getX();
    double y =rand_->getY();
    
    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
 
    if( isOriginal )
       return false;
    else
        return !isFreeSpace(VectorCollisionMap, i, j);
   
}
bool randomCheck(ob::State* rand)
{
    ob::ClothoidStateSpace::StateType *rand_ = rand->as<ob::ClothoidStateSpace::StateType>();

    double x = rand_->getX();
    double y =rand_->getY();
    
    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    

    if( isOriginal )
       return false;
    else
        return !isFreeSpace(VectorCollisionMap, i, j);

}


bool isStateValid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& VectorCollisionMap, const ob::State *state)
{
    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    return si->satisfiesBounds(s) && isValid(VectorCollisionMap, s->getX(),s->getY(),s->getYaw());
}

bool isStateValid_Clothoid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& VectorCollisionMap, const ob::State *state)
{
    const ob::ClothoidStateSpace::StateType *s = state->as<ob::ClothoidStateSpace::StateType>();
    return si->satisfiesBounds(s) && isValid(VectorCollisionMap, s->getX(),s->getY(),s->getYaw());
}

void plan(ob::StateSpacePtr space,double time, bool isClothoid)
{
    ob::ScopedState<> start(space), goal(space);
    // set the start and goal states
    start[0] =START[0];
    start[1] =START[1]; 
    start[2] = START[2];//0.99*boost::math::constants::pi<double>();
    goal[0] = GOAL[0];
    goal[1] = GOAL[1]; 
    goal[2] = GOAL[2];//0.99*boost::math::constants::pi<double>();

   
   VectorFieldGenerator(START, GOAL, BOUNDS);

    ////////////////////////////////////////////////////////////////////

    if( isClothoid )
        space->as<ob::ClothoidStateSpace>()->setBounds(BOUNDS);
    else
        space->as<ob::SE2StateSpace>()->setBounds(BOUNDS);

    for(int i=0; i<ITER; i++)
    {
        // define a simple setup class
        og::SimpleSetup ss(space);

        // set state validity checking for this space
        ob::SpaceInformationPtr si(ss.getSpaceInformation());
        ss.setStateValidityChecker(std::bind(
                    isClothoid ? &isStateValid_Clothoid : &isStateValid, si.get(),
                    VectorCollisionMap, std::placeholders::_1));


        if( isClothoid )
        {
            start[3] = 0.0; //Curvature Initialization
            goal[3] = 0.0;
        }

        ss.setStartAndGoalStates(start, goal);

        if( isClothoid )
        {
            ss.setPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        }
        else
        {
            ss.setPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        }
        //b, l::base::PlannerPtr(new ompl::geometric::KPIECE1(ss.getiSpaceInformation())), range);
        // this call is optional, but we put it in to get more output information
        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
        ss.setup();
    //    ss.print();

        // attempt to solve the problem within 30 seconds of planning time
        ob::PlannerStatus solved = ss.solve(time);

        if (solved)
        {
            std::vector<double> reals;

//            std::cout << "Found solution:" << std::endl;
//            ss.simplifySolution();
            og::PathGeometric path = ss.getSolutionPath();
            path.interpolate(1000);

            /////////////////////////////////////////////////////////////////////
            /* RVIZ */
            poseArray.header.frame_id = "/camera_init";//scan->header.frame_id;
            potential_publisher_.publish(poseArray);

            poseArray2.header.frame_id = "/camera_init";//scan->header.frame_id;
            potential2_publisher_.publish(poseArray2);

            poseArray3.header.frame_id = "/camera_init";//scan->header.frame_id;
            potential3_publisher_.publish(poseArray3);
            
            nav_msgs::Path msg;
            std::vector<geometry_msgs::PoseStamped> waypoints;
            msg.header.frame_id = "/camera_init";
            msg.header.stamp = ros::Time::now();


            if( isClothoid )
            {
                for(int i=0;i<path.getStateCount(); i++)
                {

                    const ob::ClothoidStateSpace::StateType *s = path.getState(i)->as<ob::ClothoidStateSpace::StateType>();
                    double x=s->getX(), y=s->getY(), yaw=s->getYaw();


                    geometry_msgs::PoseStamped waypt;
                    waypt.header.frame_id = "/camera_init";
                    waypt.header.stamp = ros::Time::now();


                    waypt.pose.position.x = x;
                    waypt.pose.position.y = y;
                    waypt.pose.position.z = 0;
                    waypoints.push_back(waypt);


                }
            }
            else
            {
                for(int i=0;i<path.getStateCount(); i++)
                {

                    const ob::SE2StateSpace::StateType *s = path.getState(i)->as<ob::SE2StateSpace::StateType>();
                    double x=s->getX(), y=s->getY(), yaw=s->getYaw();

                    geometry_msgs::PoseStamped waypt;
                    waypt.header.frame_id = "/camera_init";
                    waypt.header.stamp = ros::Time::now();


                    waypt.pose.position.x = x;
                    waypt.pose.position.y = y;
                    waypt.pose.position.z = 0;
                    waypoints.push_back(waypt);



                }
            }

            msg.poses.resize(waypoints.size());

            // Extract the plan in world coordinates, we assume the path is all in the same frame
            for(unsigned int i=0; i < waypoints.size(); i++)
            {
                msg.poses[i] = waypoints[i];
            }

            msgpub3.publish(msg);


            /////////////////////////////////////////////////////////////////////

      //      path.printAsMatrix(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }
}

void printTrajectory(ob::StateSpacePtr space, const std::vector<double>& pt)
{
    if (pt.size()!=3) throw ompl::Exception("3 arguments required for trajectory option");
    const unsigned int num_pts = 50;
    ob::ScopedState<> from(space), to(space), s(space);
    std::vector<double> reals;

    from[0] = from[1] = from[2] = 0.;

    to[0] = pt[0];
    to[1] = pt[1];
    to[2] = pt[2];

    std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";
    for (unsigned int i=0; i<=num_pts; ++i)
    {
        space->interpolate(from(), to(), (double)i/num_pts, s());
        reals = s.reals();
        std::cout << "path " << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;
    }
}

void printDistanceGrid(ob::StateSpacePtr space)
{
    // print the distance for (x,y,theta) for all points in a 3D grid in SE(2)
    // over [-5,5) x [-5, 5) x [-pi,pi).
    //
    // The output should be redirected to a file, say, distance.txt. This
    // can then be read and plotted in Matlab like so:
    //     x = reshape(load('distance.txt'),200,200,200);
    //     for i=1:200,
    //         contourf(squeeze(x(i,:,:)),30);
    //         axis equal; axis tight; colorbar; pause;
    //     end;
    const unsigned int num_pts = 200;
    ob::ScopedState<> from(space), to(space);
    from[0] = from[1] = from[2] = 0.;

    for (unsigned int i=0; i<num_pts; ++i)
        for (unsigned int j=0; j<num_pts; ++j)
            for (unsigned int k=0; k<num_pts; ++k)
            {
                to[0] = 5. * (2. * (double)i/num_pts - 1.);
                to[1] = 5. * (2. * (double)j/num_pts - 1.);
                to[2] = boost::math::constants::pi<double>() * (2. * (double)k/num_pts - 1.);
                std::cout << space->distance(from(), to()) << '\n';
            }

}
/*
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "heightmap_node");  //노드명 초기화
    
    ros::NodeHandle node; //노드 핸들 선언
    ros::NodeHandle priv_nh("~");
    
    double from_d[3];
    from_d[0]=0;
    from_d[1]=0;
    from_d[2]=0;
    
    double to_d[3];
    to_d[0]=10;
    to_d[1]=10;
    to_d[2]=0;
    
    potential_publisher_= node.advertise<geometry_msgs::PoseArray>("RANDOMVEC", 1);
    
    double target_d[3];
    
    for( double i=0; i<30; i+=0.3)
    {
        for( double j=0; j<30; j+=0.3)
        {
            VectorXd ret = VectorXd::Zero(2);
            target_d[0]=i;
            target_d[1]=j;
            

            double B[2]={0,0};
            
            magneticVectorForce(from_d, target_d, B);
            magneticVectorForce(to_d, target_d, B);
            
            ret(0) = B[0];
            ret(1) = B[1];
            ret.normalize();
    
            double vTheta = atan2(ret(1), ret(0));

            ////////////////////////////////////////
            geometry_msgs::PoseStamped poseStamped;

            poseStamped.pose.position.x = target_d[0];
            poseStamped.pose.position.y = target_d[1];
            poseStamped.pose.position.z = 0.0;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vTheta);

            poseStamped.pose.orientation = odom_quat;
            poseArray.poses.push_back(poseStamped.pose);
            ////////////////////////////////////////


        }
    }
    poseArray.header.frame_id = "/camera_init";//scan->header.frame_id;
    potential_publisher_.publish(poseArray);


    
}
*/
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "heightmap_node");  //노드명 초기화
    
    ros::NodeHandle node; //노드 핸들 선언
    ros::NodeHandle priv_nh("~");
    potential_publisher_= node.advertise<geometry_msgs::PoseArray>("RANDOMVEC", 1);
    potential3_publisher_= node.advertise<geometry_msgs::PoseArray>("RANDOMVEC2", 1);
    potential2_publisher_= node.advertise<geometry_msgs::PoseArray>("POTENTIAL", 1);
    map_pub= node.advertise<nav_msgs::OccupancyGrid>("GRIDMAP", 10);
    msgpub3 = node.advertise<nav_msgs::Path>("RESULTPATH", 1);
    try
    {
        po::options_description desc("Options");
        desc.add_options()
            ("help", "show help message")
            ("clothoid", "use Clothoid state space")
            ("clothoidrev", "use reverse Clothoid state space")
            ("dubins", "use Dubins state space")
            ("dubinssym", "use symmetrized Dubins state space")
            ("reedsshepp", "use Reeds-Shepp state space (default)")
            ("time", po::value<double>(),"plannig time")
            ("trajectory", po::value<std::vector<double > >()->multitoken(),
                "print trajectory from (0,0,0) to a user-specified x, y, and theta")
            ("distance", "print distance grid")
            ("original", "original RRTstar")
            ("iter", po::value<int>(), "iter RRTstar")
            ("env", po::value<int>(), "EnvironmentType")
            ("envdraw", po::value<int>(), "EnvironmentTypeForDraw")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
            po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        if (vm.count("help") || argc==1)
        {
            std::cout << desc << "\n";
            return 1;
        }

        if( vm.count("envdraw"))
        {
            SetEnvironment(vm["envdraw"].as<int>());
            return 1;
        }
        
        if( vm.count("iter"))
        {
            ITER = vm["iter"].as<int>();
        }
        ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(5.88));

		bool isClothoid = false;

        string strClothoid;
        if (vm.count("clothoidrev"))
		{
            space = ob::StateSpacePtr(new ob::ClothoidStateSpace(0.17, true));
			isClothoid = true;
            strClothoid ="Clothoid-Reverse";
		}
		
		if (vm.count("clothoid"))
		{
			space = ob::StateSpacePtr(new ob::ClothoidStateSpace(0.17, false));
			isClothoid = true;
            strClothoid ="Clothoid";
		}
		
		if (vm.count("dubins"))
        {
            space = ob::StateSpacePtr(new ob::DubinsStateSpace(5.88, false));
			isClothoid = false;
            strClothoid ="Dubins";
        }
        if (vm.count("dubinssym"))
        {
            space = ob::StateSpacePtr(new ob::DubinsStateSpace(5.88, true));
			isClothoid = false;
            strClothoid ="Dubins_Reverse";
        }
        if (vm.count("original"))
            isOriginal = true;
		
        double time = 30.0;
		if (vm.count("time"))
			time = vm["time"].as<double>();

        int env=1;
        if( vm.count("env"))
        {
            env = vm["env"].as<int>();
        }

        SetEnvironment(env);
        if( isOriginal )            
            cout <<"RRT*"<<"\tType:\t"<<env<<"\tTime:\t"<<time<<"\tSteer:\t"<<strClothoid<<endl;
        else
            cout <<"DO-RRT*"<<"\tType:\t"<<env<<"\tTime:\t"<<time<<"\tSteer:\t"<<strClothoid<<endl;

        plan(space,time,isClothoid);


        if (vm.count("trajectory"))
            printTrajectory(space, vm["trajectory"].as<std::vector<double> >());
        if (vm.count("distance"))
            printDistanceGrid(space);
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
/*
int main(int argc, char** argv)
{
    double start[3]={85.0, -8.0, 3.1416};
    double goal[3]={77.36, -1.56, 2.2166};
    double target[3]={74.852,0.0876};

    double B[2]={0.0, 0.0};
    magneticVectorForce(goal, target, B);
    magneticVectorForce(start, target, B);

    cout << B[0] << " "<<B[1] <<" "<<endl;

    return 0;
}

 VectorXd magneticfield(ob::State* from, ob::State* to, ob::State* target)
{
    VectorXd ret = VectorXd::Zero(2);
    
    ob::ClothoidStateSpace::StateType *from_ = from->as<ob::ClothoidStateSpace::StateType>();
    ob::ClothoidStateSpace::StateType *to_ = to->as<ob::ClothoidStateSpace::StateType>();
    ob::ClothoidStateSpace::StateType *target_ = target->as<ob::ClothoidStateSpace::StateType>();
   
    if( isOriginal ) return ret;
    
    if( to_->getX() == GOAL[0] && to_->getY() == GOAL[1] )
        return ret;
    double from_d[3];
    from_d[0]=from_->getX();
    from_d[1]=from_->getY();
    from_d[2]=from_->getYaw();
    
    double to_d[3];
    to_d[0]=to_->getX();
    to_d[1]=to_->getY();
    to_d[2]=to_->getYaw();
    
    double target_d[3];
    target_d[0]=target_->getX();
    target_d[1]=target_->getY();
    target_d[2]=target_->getYaw();

    double B[2]={0,0};

    double vTheta = 0.0;

    {
        double randValue = double(rand()%(RAND_ANGLE_RANGE*100))/100.0*3.1415/180.0;
        int randSign = rand()%2;
        if( randSign == 1 )
        {
            vTheta += randValue;
        }
        else
        {
            vTheta -= randValue;
        }

       magneticVectorForce(from_d, target_d, B);
        magneticVectorForce(to_d, target_d, B);

        ret(0) = B[0];
        ret(1) = B[1];
        ret.normalize();

        vTheta += atan2(ret(1), ret(0));

        //vTheta += target_d[2];
    }

   // cout << target_->getYaw() <<" " <<conv2pi(vTheta)<<endl;
    target_->setYaw(AngleUtils::toRange_PItoPI(vTheta));
    ////////////////////////////////////////
    geometry_msgs::PoseStamped poseStamped;

    poseStamped.pose.position.x = target_d[0];
    poseStamped.pose.position.y = target_d[1];
    poseStamped.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vTheta);

    poseStamped.pose.orientation = odom_quat;
    poseArray.poses.push_back(poseStamped.pose);
    ////////////////////////////////////////

    return ret;
}

VectorXd magneticfield_(ob::State* from, ob::State* to, ob::State* target)
{
    VectorXd ret = VectorXd::Zero(2);
    
    ob::SE2StateSpace::StateType *from_ = from->as<ob::SE2StateSpace::StateType>();
    ob::SE2StateSpace::StateType *to_ = to->as<ob::SE2StateSpace::StateType>();
    ob::SE2StateSpace::StateType *target_ = target->as<ob::SE2StateSpace::StateType>();
   
    if( isOriginal ) return ret;
     if( to_->getX() == GOAL[0] && to_->getY() == GOAL[1] )
    {
        return ret;
    }
    
 
    double from_d[3];
    from_d[0]=from_->getX();
    from_d[1]=from_->getY();
    from_d[2]=from_->getYaw();
    
    double to_d[3];
    to_d[0]=to_->getX();
    to_d[1]=to_->getY();
    to_d[2]=to_->getYaw();
    
    double target_d[3];
    target_d[0]=target_->getX();
    target_d[1]=target_->getY();
    target_d[2]=target_->getYaw();

    double B[2]={0,0};

    double vTheta = 0.0;
    double randValue = double(rand()%(RAND_ANGLE_RANGE*100))/100.0*3.1415/180.0;
    int randSign = rand()%2;
    if( randSign == 1 )
    {
        vTheta += randValue;
    }
    else
    {
        vTheta -= randValue;
    }

   {
        magneticVectorForce(from_d, target_d, B);
        magneticVectorForce(to_d, target_d, B);

        ret(0) = B[0];
        ret(1) = B[1];
        ret.normalize();

        vTheta += atan2(ret(1), ret(0));

    }

    target_->setYaw(AngleUtils::toRange_PItoPI(vTheta));
    
    ////////////////////////////////////////
    geometry_msgs::PoseStamped poseStamped;

    poseStamped.pose.position.x = target_d[0];
    poseStamped.pose.position.y = target_d[1];
    poseStamped.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vTheta);

    poseStamped.pose.orientation = odom_quat;
    poseArray.poses.push_back(poseStamped.pose);
    ////////////////////////////////////////

    
    return ret;
}

VectorXd vectorfield_(const vector<vector<VectorXd> >&VectorMap, ob::State* from, ob::State* target)
{
    VectorXd ret = VectorXd::Zero(2);
    
    ob::SE2StateSpace::StateType *f = from->as<ob::SE2StateSpace::StateType>();
    ob::SE2StateSpace::StateType *t = target->as<ob::SE2StateSpace::StateType>();
    
    if( isOriginal ) return ret;
    
    if( t->getX() == GOAL[0] && t->getY() == GOAL[1] )
    {
        return ret;
    }
    //if( fabs(t->getX() - GOAL[0])<3.0 &&fabs( t->getY()- GOAL[1])<3.0 )

    double x=t->getX(), y=t->getY(), yaw=t->getYaw();
    double x_=f->getX(), y_=f->getY();
    
    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
   
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        ret(0) = VectorMap[i][j](2);
        ret(1) = VectorMap[i][j](3);
        ret.normalize();
        t->setX( x + (x-x_)*vfMgain_+ ret(0)*vfgain_);
        t->setY( y + (y-y_)*vfMgain_+ ret(1)*vfgain_);
        t->setYaw( atan2(y-y_,x-x_) );
    }
   return ret;
}


// Clothoid Space
VectorXd vectorfield(const vector<vector<VectorXd> >&VectorMap, ob::State* from, ob::State* target)
{
    VectorXd ret = VectorXd::Zero(2);
    
    ob::ClothoidStateSpace::StateType *f = from->as<ob::ClothoidStateSpace::StateType>();
    ob::ClothoidStateSpace::StateType *t = target->as<ob::ClothoidStateSpace::StateType>();
    
    if( isOriginal ) return ret;
   
    if( t->getX() == GOAL[0] && t->getY() == GOAL[1] )
    {
   // if( fabs(t->getX() - GOAL[0])<3.0 &&fabs( t->getY()- GOAL[1])<3.0 )
        return ret;
    }
    double x=t->getX(), y=t->getY(), yaw=t->getYaw();
    double x_=f->getX(), y_=f->getY();
    
    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
   
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        ret(0) = VectorMap[i][j](2);
        ret(1) = VectorMap[i][j](3);
        ret.normalize();

        double momsize =sqrt(pow(x-x_,2)+pow(y-y_,2));

        t->setX( x + (x-x_)/momsize*vfMgain_+ ret(0)*vfgain_);
        t->setY( y + (y-y_)/momsize*vfMgain_+ ret(1)*vfgain_);
        //t->setYaw( atan2(t->getY()-y_,t->getX()-x_) );
    }

    //cout << "_VF: "<<s->getX() <<" " <<s->getY() << " "<<s->getYaw()<<endl;
   //cout << "VF_: "<<s->getX() <<" " <<s->getY() << " "<<s->getYaw()<<endl;
    return ret;
}



 
 */

/*
VectorXd magneticfield(ob::State* from, ob::State* to, ob::State* target)
{
    VectorXd ret = VectorXd::Zero(2);
    
    ob::ClothoidStateSpace::StateType *from_ = from->as<ob::ClothoidStateSpace::StateType>();
    ob::ClothoidStateSpace::StateType *to_ = to->as<ob::ClothoidStateSpace::StateType>();
    ob::ClothoidStateSpace::StateType *target_ = target->as<ob::ClothoidStateSpace::StateType>();
   
    double from_d[3];
    from_d[0]=from_->getX();
    from_d[1]=from_->getY();
    from_d[2]=from_->getYaw();
    
    double to_d[3];
    to_d[0]=to_->getX();
    to_d[1]=to_->getY();
    to_d[2]=to_->getYaw();
    
    double target_d[3];
    target_d[0]=target_->getX();
    target_d[1]=target_->getY();
    target_d[2]=target_->getYaw();

    cout << from_d[0] <<" "<< from_d[1] <<" " <<from_d[2] <<" " <<target_d[0] <<" "<<target_d[1]<<" " <<target_d[2] <<endl;
}
*/

/*
{
    VectorXd ret = VectorXd::Zero(2);
    
    ob::SE2StateSpace::StateType *from_ = from->as<ob::SE2StateSpace::StateType>();
    ob::SE2StateSpace::StateType *to_ = to->as<ob::SE2StateSpace::StateType>();
    ob::SE2StateSpace::StateType *target_ = target->as<ob::SE2StateSpace::StateType>();
   
    if( isOriginal ) return ret;
     if( to_->getX() == GOAL[0] && to_->getY() == GOAL[1] )
    {
        return ret;
    }
    
 
    double from_d[3];
    from_d[0]=from_->getX();
    from_d[1]=from_->getY();
    from_d[2]=from_->getYaw();
    
    double to_d[3];
    to_d[0]=to_->getX();
    to_d[1]=to_->getY();
    to_d[2]=to_->getYaw();
    
    double target_d[3];
    target_d[0]=target_->getX();
    target_d[1]=target_->getY();
    target_d[2]=target_->getYaw();

    int i= int((target_d[0] +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((target_d[1] +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
   
    
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        double w1 = 2.0;
        double w2 = 0.3;
        int ITERMAX = 5;
        int GRADLIMIT = 0.1;
        int iter = 0;
        VectorXd vGradient = VectorXd::Zero(2);
        VectorXd vMomentum = VectorXd::Zero(2);
        while(iter < ITERMAX)
        {
            vGradient(0) = VectorMap[i][j](2);
            vGradient(1) = VectorMap[i][j](3);
            double vGradientSize = vGradient.norm();
         
            if( vGradientSize < GRADLIMIT )
                break;

            vGradient.normalize();

            vMomentum(0) = cos(target_d[2]);
            vMomentum(1) = sin(target_d[2]);

            target_d[0] += w1*vGradientSize*vGradient(0)+w2*vMomentum(0);
            target_d[1] += w1*vGradientSize*vGradient(1)+w2*vMomentum(1);

            double B[2]={0,0};
            magneticVectorForce(from_d, target_d, B);
            magneticVectorForce(to_d, target_d, B);
            
            target_d[2] =AngleUtils::toRange_PItoPI( atan2(B[1], B[0]));
            
            iter++;

        }

        target_->setX(target_d[0]);
        target_->setY(target_d[1]);
        target_->setYaw(target_d[2]);
    }
 
    ////////////////////////////////////////
    geometry_msgs::PoseStamped poseStamped;

    poseStamped.pose.position.x = target_d[0];
    poseStamped.pose.position.y = target_d[1];
    poseStamped.pose.position.z = 0.0;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);

    poseStamped.pose.orientation = odom_quat;
    poseArray.poses.push_back(poseStamped.pose);
    ////////////////////////////////////////

    
    return ret;
}
*/
void vortexVectorForce(const double* init, const double* target, double *B)
{
	double ln = 60.0; 	// Length of conductors
	double d = 0;		// Distance between the conductors m & n
	int m = 1; 		// Direction of current through the conductor, m(Right) 1(INTO), -1(OUT)
	int n = -1; 		// Direction of current through the conductor, n(Left) 1(INTO), -1(OUT)
	int N = 3; 		// Number sections/elements in the conductors
	int dl = ln/N; 		// Length of each element

    double xxP = target[0];
    double zzP = target[1];

    double q[2]={init[0], init[1]};

    MatrixXd Cm = Vector2d(q[0], q[1]); // Cm_ --> (2,1)
    //cout << Cm << endl;
    //cout << Cn << endl;
 
	MatrixXd xCm = Cm(0)*MatrixXd::Ones(1,N); 
    
    // cout <<xCm <<endl;
    // cout <<xCn <<endl;

	// Y Coordinate of each element from origin, half on +Y & other half on -Y and also sam for both conductors
    double low = -ln/2.0+dl/2.0;
    double high = ln/2.0-dl/2.0;

	VectorXd yC = VectorXd::LinSpaced( ((high-low)/dl+1),low, low+dl*(N-1) );
    //cout << yC <<endl;

    // zC remains 0 throughout the length, as conductors are lying on XY plane
	MatrixXd zCm = Cm(1)*MatrixXd::Ones(1,N);
	
    //cout << zCm << endl;
    //cout << zCn << endl;

	// Length(Projection) 7 Direction of each current element in Vector form
	MatrixXd Lx = MatrixXd::Zero(1,N);	// Length of each element is zero on X axis
	MatrixXd Ly = dl*MatrixXd::Ones(1,N);	// Length of each element is dl on Y axis
	MatrixXd Lz = MatrixXd::Zero(1,N);	// Length of each element is zero on Z axis

    
    double Bx = 0;
    double By = 0;
    double Bz = 0;

	for( int i=0; i<N; i++ )
	{
		double rxm = xxP - xCm(i); // Displacement Vector along X direction, from cond m..
		
		double ry = yC(i);	// Same for m & n, no detector points on Y direction..
		
		double rzm = zzP - zCm(i); // Same for m & n..
		
        double rm = sqrt(rxm*rxm + ry*ry + rzm*rzm);// Displacement Magnitude for an element on cond m..

		double r3m = rm*rm*rm;
		
		Bx = Bx + m*Ly(i)*rzm/r3m;	// m & n, direction of current element..
		Bz = Bz - m*Ly(i)*rxm/r3m;	// m & n, direction of current element..
		// By = 0;
    //    cout << Bx <<" "<< Bz <<" " <<endl;
	}
    B[0] += Bx;
    B[1] += Bz;

}

