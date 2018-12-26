#include <iostream>
//#include "../inc/ParkingPlannerThread.h"
#include <ros/ros.h>    //ROS湲곕낯 ?ㅻ뜑 ?뚯씪
#include <cmath>
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
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <boost/program_options.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/timer/timer.hpp>
#include <boost/thread/thread.hpp>
#include<ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <std_msgs/Float32MultiArray.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <fstream>
#define DRAW
//#define DRAWVEHICLE
//#define CONFIG_ANALYSIS

//#define ODEDEMO
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;
namespace po = boost::program_options;
using namespace Eigen;


//bool CLOTHOID = true;
//ob::StateSpacePtr space(new ob::ClothoidStateSpace(0.17, true)); // false: forward
//typedef ob::ClothoidStateSpace::StateType STATETYPE;
//typedef ob::ClothoidStateSpace STATESPACE;

int MODEDORRTSTAR = 1;
og::SimpleSetup* ss_g;
og::SimpleSetup* ss_l;
vector<Vector3d> vPath_g;

vector<Vector3d> vTrajec_g;

ofstream outFile("output.txt");
bool CLOTHOID = false;
ob::StateSpacePtr g_space(new ob::DubinsStateSpace(5.88, true)); // false: forward
ob::StateSpacePtr g_space_local(new ob::DubinsStateSpace(5.88, true)); // false: forward
typedef ob::SE2StateSpace::StateType STATETYPE;
typedef ob::SE2StateSpace STATESPACE;
///////////////////////////////////////////////////////////////////////////////////////////////////////////
string LOGFILENAME="DEFAULT";
bool UPDATINGMAP = false;
int TMPMOVINGCNT=0;
int TMPMOVINGPOSCNT=0;
int ITER = 1;
int LOGFLAG=0;
double PLANNINGTIME = 5.0;
double PLANNINGTIME_L = 1.0;
double COSTTHRESHOLD=0.0;

vector<Vector2d> OBSTACLE;
double START_GORI[3]={0.0,0.0,RADIANS(0)};
double GOAL_GORI[3]={0.0,0.0,RADIANS(0)};
double START_G[3]={0.0,0.0,RADIANS(0)};
double GOAL_G[3]={0.0,0.0,RADIANS(0)};
//For local planner
double START_L[3]={0.0,0.0,RADIANS(0)};
double GOAL_L[3]={0.0,0.0,RADIANS(0)};
ob::RealVectorBounds BOUNDS(2);
double SAFEREGION = 0.6;

double RANGE_OBS = 4.0;  
double K_REP_OBS = 1.00f;
double RANGE_REP=180.0;
double K_REP = 100.0;    
double K_ATT = 0.002;

double CAR_C2F = 3.2;
double CAR_C2R = 0.8;
double CAR_WIDTH = 0.9;

vector<vector<VectorXd> > g_map; //unuse
ob::PlannerStatus g_solved;
bool g_replan = false;
bool g_solved_init = false;
double g_localization_map[4]={0.0,0.0,RADIANS(0),0.0};
double g_localization[4]={0.0,0.0,RADIANS(0),0.0};
double VEHICLEPOS_TMP[4]={0,0,0,0};
// KD TREE : Obstacle
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree;

// KD TREE : Path
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree_Path;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree_Path;

// Publish Msg
ros::Publisher g_msgpub1;
ros::Publisher g_msgpub2;
ros::Publisher g_msgpub3;
ros::Publisher g_msgpub4;
ros::Publisher g_msgpub5;

ros::Subscriber g_msgsub1;
//ros::Subscriber g_msgsub2;

geometry_msgs::PoseArray g_posArray1;
geometry_msgs::PoseArray g_posArray2;
geometry_msgs::PoseArray g_posArray3;

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(COSTTHRESHOLD));
    return obj;
}

int SearchNearestNodeIdxByRadius(pcl::PointXYZ searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::vector<pcl::PointXYZ> pvNode;

    if( g_kdTree_Path.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
    {
        return pointIdxRadiusSearch[0];
    }
    return -1;
}

std::vector<pcl::PointXYZ> SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::vector<pcl::PointXYZ> pvNode;

    if( g_kdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            pvNode.push_back(g_pTree->points[pointIdxRadiusSearch[i]]);
        }
    }
    return pvNode;
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

#pragma omp parallel reduction(+:Bx, Bz)	
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

        Bx += + m*Ly(i)*rzm/r3m + n*Ly(i)*rzn/r3n;	// m & n, direction of current element..
        Bz += - m*Ly(i)*rxm/r3m - n*Ly(i)*rxn/r3n;	// m & n, direction of current element..
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
        return K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist))*(x-tar_x);
    else
        return 0;

}

double computeAttractiveForce(double K, double x, double tar_x)
{
    return -1.0f * K * (x - tar_x);
}


VectorXd ComputePotentialField2(double x, double y, double yaw, double* start, double* goal)
{
    Vector3d ret = Vector3d::Zero(3);

    Vector3d ran = Vector3d::Zero(3);
    ran(0) = cos(yaw);
    ran(1) = sin(yaw);

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),RANGE_OBS);
    double sumX=0,sumY=0;
#pragma omp parallel reduction(+:sumX, sumY)	
    for( int i=0; i<obs.size(); i++)
    {
        double obs_x = obs[i].x;
        double obs_y = obs[i].y;

        if(obs_x == 0.0 && obs_y == 0.0)
            continue;

        double obs_dist = sqrt(((x - obs_x) * (x - obs_x))
                + ((y - obs_y) * (y - obs_y)));

        sumX += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, x, obs_x);
        sumY += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, y, obs_y);
    }
    Vector3d pot = Vector3d::Zero(3);


    double rho = sqrt(sumX*sumX+sumY*sumY)*10.0;
    if( rho > 0 )
    {
        pot(0) = sumX;
        pot(1) = sumY;
        pot.normalize();

        Vector3d vDir = Vector3d::Zero(3);
        vDir = pot.cross(ran);

        {
            double yaw=0;
            if( vDir(2) > 0 )
            {
                yaw = boost::math::constants::pi<double>()*0.5;
            }
            else             
            {
                yaw = -boost::math::constants::pi<double>()*0.5;
            }
            double pot_x =pot(0)*cos(yaw)-pot(1)*sin(yaw);
            double pot_y =pot(0)*sin(yaw)+pot(1)*cos(yaw);

            pot(0) = pot_x;
            pot(1) = pot_y;
        }

        if( rho > 1 ) rho = 1.0;
    }
    else
    {
        rho = 0;

        if( rand()%10 > 2 )
        {
            double start_dist = sqrt(((x - start[0]) * (x - start[0]))
                    + ((y - start[1]) * (y - start[1])));
            double x_rep = computeRepulsiveForce(K_REP, start_dist,
                    RANGE_REP, x, start[0]);
            double y_rep = computeRepulsiveForce(K_REP, start_dist,
                    RANGE_REP, y, start[1]);

            // Vector Force (Unit Vector)
            ran(0) =sumX+(x_rep);
            ran(1) =sumY+(y_rep);

            double x_att = computeAttractiveForce(K_ATT, x, goal[0]);
            double y_att = computeAttractiveForce(K_ATT, y, goal[1]);

            ran(0) +=(x_att);
            ran(1) +=(y_att);

            ran.normalize();
        }
    }


    ran(0) =rho*pot(0)+(1-rho)*ran(0);
    ran(1) =rho*pot(1)+(1-rho)*ran(1);

    Vector3d mag = Vector3d::Zero(3);
    {
        double B[2]={0,0};
        double target[2]={x,y};
        magneticVectorForce(start, target, B);

        mag(0) += B[0]*5.0;
        mag(1) += B[1]*5.0;
    }
    {
        double B[2]={0,0};
        double target[2]={x,y};
        magneticVectorForce(goal, target, B);

        mag(0) += B[0]*5.0;
        mag(1) += B[1]*5.0;
    }
    double beta = sqrt(mag(0)*mag(0)+mag(1)*mag(1));
    if( beta > 0 )
    {
        mag(0) = mag(0)/beta;
        mag(1) = mag(1)/beta;
        if( beta > 1 ) beta = 1.0;
    }
    else
    {
        beta = 0;
    }
    ret(0) = beta*mag(0)+(1-beta)*ran(0);
    ret(1) = beta*mag(1)+(1-beta)*ran(1);
    return ret;
}

VectorXd ComputeObstacleField(double x, double y)
{
    VectorXd ret = VectorXd::Zero(5);

    ret(0) = x;
    ret(1) = y;

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),RANGE_OBS);

    double sumX=ret(2),sumY=ret(3);
#pragma omp parallel reduction(+:sumX, sumY)	
    for( int i=0; i<obs.size(); i++)
    {
        double obs_x = obs[i].x;
        double obs_y = obs[i].y;

        if(obs_x == 0.0 && obs_y == 0.0)
            continue;

        double obs_dist = sqrt(((x - obs_x) * (x - obs_x))
                + ((y - obs_y) * (y - obs_y)));

        sumX += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, x, obs_x);
        sumY += computeRepulsiveForce(K_REP_OBS, obs_dist, RANGE_OBS, y, obs_y);
    }
    ret(2)=sumX;
    ret(3)=sumY;
    ret(4) = sqrt(ret(2)*ret(2)+ret(3)*ret(3));
    return ret;
}

bool isFreeSpace(float x, float y)
{
    bool isFreeSpace = true;

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),SAFEREGION);

    if( obs.size() > 0 )
        isFreeSpace = false;

    return isFreeSpace;
}

bool isFreeSpace_(float x, float y)
{
    bool isFreeSpace = true;

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),1.414);

    if( obs.size() > 0 )
        isFreeSpace = false;

    return isFreeSpace;
}

bool isValid(double x, double y, double yaw)
{
    double _x = x + 0.2*cos(yaw);
    double _y = y + 0.2*sin(yaw);

    bool isFreeSpace1 = isFreeSpace_(_x,_y);

    _x = x + 1.2*cos(yaw);
    _y = y + 1.2*sin(yaw);

    bool isFreeSpace2 = isFreeSpace_(_x,_y);
    
    _x = x + 2.2*cos(yaw);
    _y = y + 2.2*sin(yaw);

    bool isFreeSpace3 = isFreeSpace_(_x,_y);

    return isFreeSpace1 && isFreeSpace2 && isFreeSpace3;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// DO
bool DO( double* from_d, double* to_d, double* target_d, bool isLocal = false)
{
    //  return false;
    if( isLocal )
    {
        if( (to_d[0] == GOAL_L[0] && to_d[1] == GOAL_L[1]) )
        {
            return false;
        }
    }
    else
    {
        if( (to_d[0] == GOAL_G[0] && to_d[1] == GOAL_G[1]) )
        {
            return false;
        }
    }

    //    if( sqrt((from_d[0]-to_d[0])*(from_d[0]-to_d[0])+(from_d[1]-to_d[1])*(from_d[1]-to_d[1]))>0 )
    //    {
    //cout <<"out"<<endl;
    //       return false;
    //    }
    // else
    // {
    //  cout << "in"<<endl;
    // }

    double target_ori[3];
    target_ori[0] = target_d[0];
    target_ori[1] = target_d[1];
    target_ori[2] = target_d[2];
    double w2 = 0.3;
    int ITERMAX = 20;
    double GRADLIMIT = 0.01;
    int iter = 0;
    Vector3d vGradient = Vector3d::Zero(3);
    Vector3d vMomentum = Vector3d::Zero(3);
    Vector3d vFeasibleDirection = Vector3d::Zero(3);

    // Update Gradient
    VectorXd VectorField = ComputeObstacleField(target_d[0],target_d[1]);

    double w1= log(VectorField(4)+1)*0.5+0.001;       

    if( VectorField(4) > 0 )
    {
        vGradient(0) = VectorField(2);
        vGradient(1) = VectorField(3);
        vGradient.normalize();

        vMomentum(0) = cos(target_d[2]);
        vMomentum(1) = sin(target_d[2]);

        Vector3d vDir = Vector3d::Zero(3);
        vDir = vMomentum.cross(vGradient);

        double vDirAng = vDir(2);
        if( fabs(vDir(2)) > RADIANS(90.0))
        {

            if( vDir(2) > 0 )
            {
                vDirAng = RADIANS(90.0);
            }
            else
            {
                vDirAng =-RADIANS(90.0);
            }

            double vFeasibleAngle = AngleUtils::toRange_PItoPI(target_d[2]+vDirAng);

            vGradient(0)  = cos(vFeasibleAngle);
            vGradient(1)  = sin(vFeasibleAngle);
        }
    }
    else
    {
#ifdef DRAW1 
        ////////////////////////////////////////
        geometry_msgs::PoseStamped poseStamped;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
        header.frame_id = "/camera_init_global";
        poseStamped.pose.position.x = target_d[1];
        poseStamped.pose.position.y = -target_d[0];
        poseStamped.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
        header.frame_id = "/camera_init";
        poseStamped.pose.position.x = target_d[0];
        poseStamped.pose.position.y = target_d[1];
        poseStamped.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif
        poseStamped.header = header;



        poseStamped.pose.orientation = odom_quat;
        g_posArray2.poses.push_back(poseStamped.pose);
        ////////////////////////////////////////
#endif
        return true;
    }


    bool ret = false;
    while(iter < ITERMAX)
    {
        //    cout <<"ITER :" <<iter<<endl;
        //    cout << target_d[0] <<" "<<target_d[1] <<" "<<target_d[2] <<endl;
        //    cout << vGradient(0) <<" "<<vGradient(1)<<" " <<VectorField(4)<<endl;
        // Update Momentum
        double B[2]={0,0};
        magneticVectorForce(from_d, target_d, B);
        magneticVectorForce(to_d, target_d, B);


        double randValue = 0;//double(rand()%(5*100))/100.0*3.1415/180.0;
        int randSign = rand()%2;
        if( randSign != 1 )
        {
            randValue*=-1.0;
        }

        //randValue=0.0;
        target_d[2] =AngleUtils::toRange_PItoPI( atan2(B[1], B[0])+randValue);

        if( isValid(target_d[0], target_d[1], target_d[2]) )
        {
            //    cout <<"ITER OUT" <<iter<<endl; 
            ret = true;
            break;
        }
        target_d[0] += w1*vGradient(0);
        target_d[1] += w1*vGradient(1);

#ifdef DRAW 
        ////////////////////////////////////////
        geometry_msgs::PoseStamped poseStamped;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
        header.frame_id = "/camera_init_global";
        poseStamped.pose.position.x = target_d[1];
        poseStamped.pose.position.y = -target_d[0];
        poseStamped.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
        header.frame_id = "/camera_init";
        poseStamped.pose.position.x = target_d[0];
        poseStamped.pose.position.y = target_d[1];
        poseStamped.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif

        poseStamped.header = header;



        poseStamped.pose.orientation = odom_quat;
        g_posArray3.poses.push_back(poseStamped.pose);
        ////////////////////////////////////////
#endif
        iter++;
    }
    
    if( ret )
    {
#ifdef DRAW 
        ////////////////////////////////////////
        geometry_msgs::PoseStamped poseStamped;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
        header.frame_id = "/camera_init_global";
        poseStamped.pose.position.x = target_d[1];
        poseStamped.pose.position.y = -target_d[0];
        poseStamped.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
        header.frame_id = "/camera_init";
        poseStamped.pose.position.x = target_d[0];
        poseStamped.pose.position.y = target_d[1];
        poseStamped.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif
        poseStamped.header = header;


        poseStamped.pose.orientation = odom_quat;
        g_posArray2.poses.push_back(poseStamped.pose);
        ////////////////////////////////////////
#endif
    }
    return ret;
}


bool DesiredOrientation_local(ob::State* from, ob::State* to, ob::State* target)
{
    STATETYPE *from_ = from->as<STATETYPE>();
    STATETYPE *to_ = to->as<STATETYPE>();
    STATETYPE *target_ = target->as<STATETYPE>();

    double from_d[3]; from_d[0]=from_->getX(); from_d[1]=from_->getY(); from_d[2]=from_->getYaw();
    double to_d[3]; to_d[0]=to_->getX(); to_d[1]=to_->getY(); to_d[2]=to_->getYaw();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( DO(from_d, to_d, target_d,true) )
    {
        target_->setX(target_d[0]);
        target_->setY(target_d[1]);
        target_->setYaw(target_d[2]);
        return true;
    }
    return false;
}
bool DesiredOrientation(ob::State* from, ob::State* to, ob::State* target)
{
    STATETYPE *from_ = from->as<STATETYPE>();
    STATETYPE *to_ = to->as<STATETYPE>();
    STATETYPE *target_ = target->as<STATETYPE>();

    double from_d[3]; from_d[0]=from_->getX(); from_d[1]=from_->getY(); from_d[2]=from_->getYaw();
    double to_d[3]; to_d[0]=to_->getX(); to_d[1]=to_->getY(); to_d[2]=to_->getYaw();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( DO(from_d, to_d, target_d) )
    {
        target_->setX(target_d[0]);
        target_->setY(target_d[1]);
        target_->setYaw(target_d[2]);
        return true;
    }
    return false;
}

VectorXd magneticfield(ob::State* target, ob::State* nouse1, ob::State* nouse2)
{
    //    return VectorXd::Zero(2);
    STATETYPE *target_ = target->as<STATETYPE>();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( (target_d[0] == GOAL_G[0] && target_d[1] == GOAL_G[1]) )
    {
#ifdef DRAW 
        {
            geometry_msgs::PoseStamped poseStamped;

            std_msgs::Header header;
            header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
            header.frame_id = "/camera_init_global";
            poseStamped.pose.position.x = target_d[1];
            poseStamped.pose.position.y = -target_d[0];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
            header.frame_id = "/camera_init";
            poseStamped.pose.position.x = target_d[0];
            poseStamped.pose.position.y = target_d[1];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif

            poseStamped.header =header;


            poseStamped.pose.orientation = odom_quat;
            g_posArray1.poses.push_back(poseStamped.pose);
        }
#endif
        return VectorXd::Zero(2);
    }
    else
    {
        double target_ori[3];
        target_ori[0] = target_d[0];
        target_ori[1] = target_d[1];
        target_ori[2] = target_d[2];
        VectorXd VectorField;
        if( nouse1 == NULL )
        {
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],START_G,GOAL_G);
        }
        else
        {
            double NEAR[3];
            STATETYPE *nouse1_ = nouse1->as<STATETYPE>();
            NEAR[0]=nouse1_->getX();
            NEAR[1]=nouse1_->getY();
            NEAR[2]=nouse1_->getYaw();
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],NEAR,GOAL_G);
        }
        target_d[2] =AngleUtils::toRange_PItoPI( atan2(VectorField(1), VectorField(0)));

        target_->setYaw(target_d[2]);
#ifdef CONFIG_ANALYSIS
        cout << target_d[0] << "\t"<<target_d[1] << "\t"<<target_d[2] << "\t"<<endl;
#endif
#ifdef DRAW 

        {
            geometry_msgs::PoseStamped poseStamped;

            std_msgs::Header header;
            header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
            header.frame_id = "/camera_init_global";
            poseStamped.pose.position.x = target_d[1];
            poseStamped.pose.position.y = -target_d[0];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
            header.frame_id = "/camera_init";
            poseStamped.pose.position.x = target_d[0];
            poseStamped.pose.position.y = target_d[1];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif


            poseStamped.header =header;


            poseStamped.pose.orientation = odom_quat;
            g_posArray1.poses.push_back(poseStamped.pose);
        }
#endif
        return VectorXd::Zero(2);
    }
    return VectorXd::Zero(2);
}

VectorXd magneticfield_local(ob::State* target, ob::State* nouse1, ob::State* nouse2)
{
    //    return VectorXd::Zero(2);
    STATETYPE *target_ = target->as<STATETYPE>();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( (target_d[0] == GOAL_L[0] && target_d[1] == GOAL_L[1]) )
    {
#ifdef DRAW 
        {
            geometry_msgs::PoseStamped poseStamped;

            std_msgs::Header header;
            header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
            header.frame_id = "/camera_init_global";
            poseStamped.pose.position.x = target_d[1];
            poseStamped.pose.position.y = -target_d[0];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
            header.frame_id = "/camera_init";
            poseStamped.pose.position.x = target_d[0];
            poseStamped.pose.position.y = target_d[1];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif

            poseStamped.header =header;


            poseStamped.pose.orientation = odom_quat;
            g_posArray1.poses.push_back(poseStamped.pose);
        }
#endif
        return VectorXd::Zero(2);
    }
    else
    {
        double target_ori[3];
        target_ori[0] = target_d[0];
        target_ori[1] = target_d[1];
        target_ori[2] = target_d[2];
        VectorXd VectorField;
        if( nouse1 == NULL )
        {
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],START_L,GOAL_L);
        }
        else
        {
            double NEAR[3];
            STATETYPE *nouse1_ = nouse1->as<STATETYPE>();
            NEAR[0]=nouse1_->getX();
            NEAR[1]=nouse1_->getY();
            NEAR[2]=nouse1_->getYaw();
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],NEAR,GOAL_L);
        }
        target_d[2] =AngleUtils::toRange_PItoPI( atan2(VectorField(1), VectorField(0)));

        target_->setYaw(target_d[2]);
#ifdef CONFIG_ANALYSIS
        cout << target_d[0] << "\t"<<target_d[1] << "\t"<<target_d[2] << "\t"<<endl;
#endif
#ifdef DRAW 

        {
            geometry_msgs::PoseStamped poseStamped;

            std_msgs::Header header;
            header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
            header.frame_id = "/camera_init_global";
            poseStamped.pose.position.x = target_d[1];
            poseStamped.pose.position.y = -target_d[0];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]-RADIANS(90));
#else
            header.frame_id = "/camera_init";
            poseStamped.pose.position.x = target_d[0];
            poseStamped.pose.position.y = target_d[1];
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(target_d[2]);
#endif


            poseStamped.header =header;


            poseStamped.pose.orientation = odom_quat;
            g_posArray1.poses.push_back(poseStamped.pose);
        }
#endif
        return VectorXd::Zero(2);
    }
    return VectorXd::Zero(2);
}


bool randomCheck_(ob::State* rand)
{
    STATETYPE *rand_ = rand->as<STATETYPE>();

    double x = rand_->getX();
    double y =rand_->getY();
    double yaw =rand_->getYaw();

    return !isValid(x,y,yaw);

}
bool randomCheck(ob::State* rand)
{
    STATETYPE *rand_ = rand->as<STATETYPE>();

    double x = rand_->getX();
    double y =rand_->getY();

    return !isFreeSpace(x,y);

}

bool isStateValid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& map, const ob::State *state)
{
    const STATETYPE *s = state->as<STATETYPE>();
    return si->satisfiesBounds(s) && isValid(s->getX(),s->getY(),s->getYaw());
}

void UpdateGlobalPathData()
{
    if (g_solved && g_solved_init)
    {
        og::PathGeometric path= ss_g->getSolutionPath();
        path.interpolate(100);

        vPath_g.clear();
        if( g_pTree_Path != NULL )
            g_pTree_Path->clear();
        
        g_pTree_Path  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        for(int i=0;i<path.getStateCount(); i++)
        {
            const STATETYPE *s = path.getState(i)->as<STATETYPE>();
            double x=s->getX(), y=s->getY(), yaw=s->getYaw();

            g_pTree_Path->push_back(pcl::PointXYZ(x, y, 0.0));
            vPath_g.push_back(Vector3d(x,y,yaw));
        }

        g_kdTree_Path.setInputCloud(g_pTree_Path);

    }
}

void MergeLocal2GlobalPathData(int startIdx, int goalIdx, og::PathGeometric* path_l)
{
    if (g_solved && g_solved_init)
    {
        for(int i=0;i<goalIdx-startIdx+1; i++)
        {
            const STATETYPE *s = path_l->getState(i)->as<STATETYPE>();
            double x=s->getX(), y=s->getY(), yaw=s->getYaw();

            vPath_g[i+startIdx]=Vector3d(x,y,yaw);
            g_pTree_Path->points[i+startIdx] = pcl::PointXYZ(x, y, 0.0);
        }
        g_kdTree_Path.setInputCloud(g_pTree_Path);
    }
}

void plan_init(og::SimpleSetup* ss,double* start, double* goal, bool isLocal = false)
{
    if( isLocal )
    {
        ob::ScopedState<> ss_start(g_space_local), ss_goal(g_space_local);
        ////////////////////////////////
        // set the start and goal states
        ss_start[0] =start[0];
        ss_start[1] =start[1]; 
        ss_start[2] =start[2];
        ss_goal[0] = goal[0];
        ss_goal[1] = goal[1]; 
        ss_goal[2] = goal[2];

        ss->setStartAndGoalStates(ss_start, ss_goal,0.05);
    }
    else
    {
        ob::ScopedState<> ss_start(g_space), ss_goal(g_space);
        ////////////////////////////////
        // set the start and goal states
        ss_start[0] =start[0];
        ss_start[1] =start[1]; 
        ss_start[2] =start[2];
        ss_goal[0] = goal[0];
        ss_goal[1] = goal[1]; 
        ss_goal[2] = goal[2];

        ss->setStartAndGoalStates(ss_start, ss_goal,0.05);
    }
    // set state validity checking for this space
    ob::SpaceInformationPtr si(ss->getSpaceInformation());
    ss->setStateValidityChecker(std::bind(
                &isStateValid, si.get(),
                g_map, std::placeholders::_1));

    //ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation,false,false,false,"DORRTstart"));
    if( isLocal )
    {
        //ss->setPlanner(ob::PlannerPtr(new og::CForest(ss->getSpaceInformation())));
        //ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation()));
       // ss->setPlanner(ob::PlannerPtr(new og::CForest(ss->getSpaceInformation(),randomCheck,magneticfield_local, DesiredOrientation_local, true, false, true, "CFDO*(TFT_)")));
        ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation(),randomCheck,magneticfield_local, DesiredOrientation_local,true,false,true));
    }
    else
    {
 //       ss->setPlanner(ob::PlannerPtr(new og::CForest(ss->getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation, true, false, true, "CFDO*(TFT)")));


      if( MODEDORRTSTAR != 0 )
      {

          cout <<"DORRT MODE"<<endl;
        ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation,true,false,true));
      }
      else
      {
          cout <<"RRT MODE"<<endl;
        ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation()));
      }
    }

    // this call is optional, but we put it in to get more output information
    ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
    ss->setup();
}
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool plan(og::SimpleSetup* ss, double time, bool isLocal = false)
{
    //    ss.print();
    // attempt to solve the problem within 30 seconds of planning time
    if( isLocal )
    {
        cout <<"### LOCAL ### "<<time<<endl;
        return ss->solve(time);
    }
    else
    {
        //        cout <<"### GLOBAL ### "<<time<<endl;
        g_solved = ss->solve(time);
        g_solved_init = true;

        return g_solved;
    }
}

double PurePursuit(Vector2d m_lookAheadPos, Vector2d m_pos, double m_heading)
{
    double m_len_c2r = 1.1;
    double m_len_f2r = 2.84;
    double m_ratio_s2w=20.0;
    double m_limit_steerAngle = RADIANS(539);
    double m_steerAngle = 0;
    // Pure Pursuit Algorithm
    m_pos[0] = m_pos[0] - m_len_c2r*cos(m_heading);
    m_pos[1] = m_pos[1] - m_len_c2r*sin(m_heading);
    Vector2d L = m_lookAheadPos-m_pos;
    Vector2d Yv(-sin(m_heading),cos(m_heading));
    Vector2d YL(L[0]*Yv[0],L[1]*Yv[1]);


    float YLtemp = YL[0]+YL[1];


    if( YLtemp != 0 )
    {
        double m_lookAheadLength = L.norm();

        m_steerAngle = (m_ratio_s2w*m_len_f2r*2.0*YLtemp)/(m_lookAheadLength*m_lookAheadLength);
        //                        m_steerAngle *= -1.0;
        if(m_steerAngle>m_limit_steerAngle)
        {
            m_steerAngle=m_limit_steerAngle;
        }
        else if(m_steerAngle<-m_limit_steerAngle)
        {
            m_steerAngle=-m_limit_steerAngle;
        }
    }
    else
    {
        m_steerAngle = 0;
    }

    // u
    return m_steerAngle/m_ratio_s2w;

}

void forwardsimulation(Vector2d m_lookAheadPos, Vector2d& m_pos,double& m_heading, double dT, double T)
{
    //cout <<m_lookAheadPos[0] <<" "<<m_lookAheadPos[1] <<" "<<m_heading<<" pos: "<<m_pos[0] <<" "<<m_pos[1]<<" u: "<<u<<" "<<DEGREES(u*20)<<endl;
    for( int i=0; i<T/dT; i++)
    {
        double u = PurePursuit(m_lookAheadPos, m_pos, m_heading);
        double v = 4.0*0.278;
        double qdotX = v*cos(m_heading);
        double qdotY = v*sin(m_heading);
        double qdotT = v*tan(u)/3.8;

        m_pos[0] += qdotX *dT;
        m_pos[1] += qdotY *dT;
        m_heading += qdotT *dT;
        
        outFile << u<<" "<<g_localization_map[0] <<" "<<g_localization_map[1]<<" "<< g_localization_map[2]<< endl;
    }
}
void planner_global()
{
    // Global Path Planning
    // Update Global Information
    //////////////////////////////////////////////////////
    START_G[0] = g_localization_map[0];
    START_G[1] = g_localization_map[1];
    START_G[2] = g_localization_map[2];


    g_space->as<STATESPACE>()->setBounds(BOUNDS);
   
  //  cout << START_G[0] << " " <<START_G[1] <<" "<<START_G[2]<<" "<<GOAL_G[0] <<" "<<GOAL_G[1] << " "<<GOAL_G[2]<<endl;
    plan_init(ss_g,START_G,GOAL_G, false);
    
    plan(ss_g,PLANNINGTIME);
    UpdateGlobalPathData();
}

// 0 : No Global Path Data
// -1: Fail to Find NN
// -2: Fail to Goal Index ( collision )
// -3: Fail to Local Path Generation ( Cannot solve )
// 1: Success to Local Path
int planner_local()
{
    if (g_solved && g_solved_init  )
    {
        if( g_replan == false )
        {
            return 2;
        }
        cout <<"PATH REPLANNING - LOCAL"<<endl;
        // Find Nearest Node Index from the global path w.r.t current pos (g_localization_map)

        double goal[3]={0,0,0};
        double start[3]={0,0,0};

        //START_G - global map 
        // 기준좌표계: x, y, theta, 대상좌표: _x, _y
        //GeometricUtils::TransRelativeCoord(double ref_x, double ref_y, double ref_h, double _x, double _y, double _h, double& ret_x, double& ret_y, double& ret_h)
        //GeometricUtils::TransRelativeCoord(START_G[0], START_G[1], START_G[2], g_localization_map[0], g_localization_map[1], g_localization_map[2], start[0], start[1], start[2]);
       
        start[0] = g_localization_map[0];
        start[1] = g_localization_map[1];
        start[2] = g_localization_map[2];

        int startIndex  =  SearchNearestNodeIdxByRadius(pcl::PointXYZ(start[0], start[1], 0), 10.0);

        if( startIndex == -1 )
        {
            cout <<"local -1" <<endl;
            return -1;  
        }

        int goalIndex = -1;
        int minGoalIndex = 40;
        int maxGoalIndex = 80;

        for(int i=startIndex+minGoalIndex; i<vPath_g.size();i++)
        {
            if( i > maxGoalIndex + startIndex )
            {
                break;
            }

            double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

            std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),0.5);
            if( obs.size() == 0 )
            {
                goal[0] = x;
                goal[1] = y;
                goal[2] = yaw;
                goalIndex = i;
                break;
            
            }
            else
            {
            }

        }

        cout <<"local : startidx["<<startIndex<<"] goalidx["<<goalIndex<<"]" <<endl;

        if(goalIndex == -1)
        {
            cout <<"local -2 : startidx["<<startIndex<<"] goalidx["<<goalIndex<<"]" <<endl;
            return -2; 
        }

        //cout <<"LOCAL "<<start[0] <<" "<< start[1]<<" " <<start[2] <<" "<<goal[0] <<" "<<goal[1] <<" "<< goal[2] <<endl;


        // For magnetic field, obstacle field
        START_L[0]=start[0];
        START_L[1]=start[1];
        START_L[2]=start[2];
        GOAL_L[0]=goal[0];
        GOAL_L[1]=goal[1];
        GOAL_L[2]=goal[2];

        vector<double> vX;
        vector<double> vY;
        double bound = 20.0;

        for(int i=startIndex;i<=goalIndex; i++)
        {
            double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

            vX.push_back(x+bound);
            vX.push_back(x-bound);
            vY.push_back(y+bound);
            vY.push_back(y-bound);
        }

        ob::RealVectorBounds bounds(2);

        bounds.low[0] = *min_element(vX.begin(),vX.end());
        bounds.high[0] =*max_element(vX.begin(),vX.end());
        bounds.low[1] = *min_element(vY.begin(),vY.end());
        bounds.high[1] =*max_element(vY.begin(),vY.end());

        g_space_local->as<STATESPACE>()->setBounds(bounds);
        
        plan_init(ss_l,start, goal, true);
        ///////////////////////////////////////////////////
        bool retplan = plan(ss_l, PLANNINGTIME_L, true);
        int ret = -3;
        if( retplan )
        {
            og::PathGeometric path = ss_l->getSolutionPath();
            path.interpolate(goalIndex - startIndex + 1);
            
            MergeLocal2GlobalPathData(startIndex, goalIndex, &path);
            ret = 1;
        }
        

        cout <<"local :"<<ret <<endl;
        cout <<"HH"<<endl;
        return ret;
    }
    return 0;
}

int GetLookAheadIndex()
{
    double goal[3]={0,0,0};
    double start[3]={0,0,0};

    //START_G - global map 
    // 기준좌표계: x, y, theta, 대상좌표: _x, _y
    //GeometricUtils::TransRelativeCoord(double ref_x, double ref_y, double ref_h, double _x, double _y, double _h, double& ret_x, double& ret_y, double& ret_h)
    //GeometricUtils::TransRelativeCoord(START_G[0], START_G[1], START_G[2], g_localization_map[0], g_localization_map[1], g_localization_map[2], start[0], start[1], start[2]);

    start[0] = g_localization_map[0];
    start[1] = g_localization_map[1];
    start[2] = g_localization_map[2];

    int startIndex  =  SearchNearestNodeIdxByRadius(pcl::PointXYZ(start[0], start[1], 0), 10.0);

    if( startIndex == -1 )
    {
        cout <<"local -1" <<endl;
        return -1;  
    }

    int goalIndex = -1;
    int minGoalIndex = 0;
    int maxGoalIndex = 80;

    for(int i=startIndex+minGoalIndex; i<vPath_g.size();i++)
    {
        if( i > maxGoalIndex + startIndex )
        {
            goalIndex = vPath_g.size()-1;
            break;
        }

        double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];
      if( sqrt((vPath_g[startIndex][0]-vPath_g[i][0])*(vPath_g[startIndex][0]-vPath_g[i][0])+ (vPath_g[startIndex][1]-vPath_g[i][1])*(vPath_g[startIndex][1]-vPath_g[i][1])) < 4 )
      {
          continue;
      }


        std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),0.5);
        if( obs.size() == 0 )
        {
            goal[0] = x;
            goal[1] = y;
            goal[2] = yaw;
            goalIndex = i;
            break;

        }

    }
    return goalIndex;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// From HeightMap Module : Local Path Update : 5hz
void SubTopicProcessTmp()
{
    TMPMOVINGCNT++;
  
    UPDATINGMAP=true;
    // Global Coordinate Obstacle Data and Position Data
    vector<Vector2d> vObstacle;
    for(double x=-20; x<90; x+=0.4 )
        for(double y=-24; y<-22; y+=0.4)
            vObstacle.push_back(Vector2d(x,y));
    for(double x=-20; x<90; x+=0.4 )
        for(double y=16; y<18; y+=0.4)
            vObstacle.push_back(Vector2d(x,y));
    //for(double x=25; x<27; x+=0.1 )
      //  for(double y=-15; y<10; y+=0.1)
        //    OBSTACLE.push_back(Vector2d(x,y));
   
    for(double x=50; x<54; x+=0.1 )
        for(double y=-5; y<5; y+=0.1)
            vObstacle.push_back(Vector2d(x,y+TMPMOVINGCNT*0.01));


    double randPos = 1.0-(rand()%40)/20.0;
    double randPos2= 1.0-(rand()%40)/20.0;
    vObstacle.push_back(Vector2d(-99999.0,-99999.0));
   
        GOAL_G[0] = GOAL_GORI[0]+randPos;
        GOAL_G[1] = GOAL_GORI[1]+randPos2;
        GOAL_G[2] = GOAL_GORI[2];
    if( g_pTree != NULL )
        g_pTree->clear();
    
    g_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(int k=0; k<vObstacle.size(); k++)
    {
        g_pTree->push_back(pcl::PointXYZ(vObstacle[k](0), vObstacle[k](1), 0.0));
    }

    g_kdTree.setInputCloud(g_pTree);

    /////////////////////////////////////////////////////////////////////////////
    // localization data (SLAM)
    if( vPath_g.size()==0)
    {
        g_localization_map[0] = VEHICLEPOS_TMP[0];
        g_localization_map[1] = VEHICLEPOS_TMP[1];
        g_localization_map[2] = VEHICLEPOS_TMP[2];
        g_localization_map[2]=AngleUtils::toRange_PItoPI(g_localization_map[2]);
        g_localization_map[3] = 1.0;
    }


    ////////////////////////////////////
    // 파일 입력 (쓰기)


        vTrajec_g.push_back(Vector3d(g_localization_map[0],g_localization_map[1],g_localization_map[2]));
    ///////////////////////////////////
        if( sqrt((GOAL_G[0] - GOAL_GORI[0])*(GOAL_G[0] - GOAL_GORI[0])+(GOAL_G[1] - GOAL_GORI[1])*(GOAL_G[1] - GOAL_GORI[1] )) >0.5 )
            planner_global();

    
    if( vPath_g.size()>0)
    {
        int lookIndex = GetLookAheadIndex()/2;
        if( lookIndex > -1 )
        {
            Vector2d m_look(vPath_g[lookIndex][0],vPath_g[lookIndex][1]);
            Vector2d m_pos(g_localization_map[0],g_localization_map[1]);

            double m_heading = g_localization_map[2];

            

            forwardsimulation(m_look, m_pos, m_heading, 0.1, 1);

            g_localization_map[0] = m_pos[0];
            g_localization_map[1] = m_pos[1];
            g_localization_map[2] = m_heading;
            g_localization_map[2]=AngleUtils::toRange_PItoPI(g_localization_map[2]);
            g_localization_map[3] = 1.0;
        }
    }
   /* 
    g_replan = false;
    for(int i=0; i<vPath_g.size(); i++)
    {
        double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

        std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),0.5);
        if( obs.size() > 0 )
        {
            g_replan = true;
            break;
        }
    }

    if ( g_solved_init == false )
    {
        planner_global();
        g_replan = true;
    }
    else
    {
        int startIndex  =  SearchNearestNodeIdxByRadius(pcl::PointXYZ(g_localization_map[0], g_localization_map[1], 0), 1.0);

        if( startIndex == -1 )
        {
            g_replan = true;
        }

        int iterLocal = 0;
        int retLocal = planner_local();
        if( retLocal == -3 )
        {
            while( iterLocal < 5 )
            {
                retLocal = planner_local();
                if( retLocal > 1 )
                    break;
                iterLocal++;
            }
        }
        if( retLocal < 1 )
        {
            cout <<"PATH REPLANNING - GLOBAL"<<endl;
            planner_global();
        }
        else
        {
            cout <<"PATH Tracking ........"<<TMPMOVINGCNT<<" "<<TMPMOVINGPOSCNT<<endl;
        }
    }

    */
    /////////////////////////////////////////////////////////////////////
    // Pub local planner path to controller ( Global Coordinate )
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
        header.frame_id = "/camera_init_global";
#else
        header.frame_id = "/camera_init";
#endif

        nav_msgs::Path msg2;
        msg2.header = header;

        g_posArray1.header = header;
        g_msgpub1.publish(g_posArray1);
        g_posArray1.poses.clear();

        g_posArray2.header = header;
        g_msgpub2.publish(g_posArray2);
        g_posArray2.poses.clear();

        g_posArray3.header = header;
        g_msgpub3.publish(g_posArray3);
        g_posArray3.poses.clear();

        for(int i=0;i<vPath_g.size(); i++)
        {

            double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

            
            geometry_msgs::PoseStamped waypt2;
            waypt2.header = header;
            waypt2.pose.position.x = x;
            waypt2.pose.position.y = y;
            waypt2.pose.position.z = 0;
            msg2.poses.push_back(waypt2);
        }

        g_msgpub5.publish(msg2);
    }

    UPDATINGMAP=false;

    UPDATINGMAP=false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// From HeightMap Module : Local Path Update : 5hz
void SubTopicProcess1(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    UPDATINGMAP=true;
    // Global Coordinate Obstacle Data and Position Data
    vector<Vector2d> vObstacle;
    vObstacle.push_back(Vector2d(-99999.0,-99999.0));

    for(int i=0; i<int((msg->data.size()-4)/2.0); i++)
    {
        vObstacle.push_back(Vector2d(msg->data.at(i*2+4),msg->data.at(i*2+5)));
    }
    cout <<"SUBDATA"<<endl;
    
    if( g_pTree != NULL )
        g_pTree->clear();
    
    g_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(int k=0; k<vObstacle.size(); k++)
    {
        g_pTree->push_back(pcl::PointXYZ(vObstacle[k](0), vObstacle[k](1), 0.0));
    }

    g_kdTree.setInputCloud(g_pTree);

    /////////////////////////////////////////////////////////////////////////////
    // localization data (SLAM)
    g_localization_map[0] = msg->data.at(0);
    g_localization_map[1] = msg->data.at(1);
    g_localization_map[2] = msg->data.at(2);
    g_localization_map[2]=AngleUtils::toRange_PItoPI(g_localization_map[2]);
    g_localization_map[3] = msg->data.at(3);

   
    g_replan = false;
    for(int i=0; i<vPath_g.size(); i++)
    {
        double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

        std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),0.2);
        if( obs.size() > 0 )
        {
            g_replan = true;
            break;
        }
    }

    if ( g_solved_init == false )
    {
        g_replan = true;
    }

    int iterLocal = 0;
    int retLocal = planner_local();
    if( retLocal == -3 )
    {
        while( iterLocal < 5 )
        {
            retLocal = planner_local();
            if( retLocal > 1 )
                break;
            iterLocal++;
        }
    }
    if( retLocal < 1 )
    {
        cout <<"PATH REPLANNING - GLOBAL"<<endl;
        planner_global();
    }
    else
    {
        cout << "path size["<<vPath_g.size()<<"] "<<msg->data.at(0) << " "<<msg->data.at(1) <<endl;
        cout <<"PATH Tracking ........"<<endl;
    }
    
    /////////////////////////////////////////////////////////////////////
    // Pub local planner path to controller ( Global Coordinate )
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
        header.frame_id = "/camera_init_global";
#else
        header.frame_id = "/camera_init";
#endif
        nav_msgs::Path msg;
        msg.header = header;

        nav_msgs::Path msg2;
        msg2.header = header;

        g_posArray1.header = header;
        g_msgpub1.publish(g_posArray1);
        g_posArray1.poses.clear();

        g_posArray2.header = header;
        g_msgpub2.publish(g_posArray2);
        g_posArray2.poses.clear();

        g_posArray3.header = header;
        g_msgpub3.publish(g_posArray3);
        g_posArray3.poses.clear();

        for(int i=0;i<vPath_g.size(); i++)
        {

            double x=vPath_g[i][0], y=vPath_g[i][1], yaw=vPath_g[i][2];

            geometry_msgs::PoseStamped waypt;
            waypt.header = header;
#ifdef DRAWVEHICLE
            waypt.pose.position.x = y;
            waypt.pose.position.y = -x;
            waypt.pose.position.z = 0;
#else
            waypt.pose.position.x = x;
            waypt.pose.position.y = y;
            waypt.pose.position.z = 0;
#endif
            msg.poses.push_back(waypt);
            
            geometry_msgs::PoseStamped waypt2;
            waypt2.header = header;
            waypt2.pose.position.x = x;
            waypt2.pose.position.y = y;
            waypt2.pose.position.z = 0;
            msg2.poses.push_back(waypt2);
        }

        g_msgpub5.publish(msg2);
        g_msgpub4.publish(msg);
    }

    UPDATINGMAP=false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PLANNER_FRAMEWORK()
{
    int argc=0;
    char** argv;
    ros::init(argc, argv, "DecisionMaker");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");
    g_msgpub1 = node.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_1", 1);
    g_msgpub2 = node.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_2", 1);
    g_msgpub3 = node.advertise<geometry_msgs::PoseArray>("PoseArray_RRT_3", 1);
    g_msgpub4 = node.advertise<nav_msgs::Path>("Path_RRT", 1);
    g_msgpub5 = node.advertise<nav_msgs::Path>("RNDFPathData", 1);
    
    //g_msgsub1 = node.subscribe("velodyne_potential_array", 1, SubTopicProcess1);
    
    cout << "START of DO-RRT*: ROS Version"<<endl;
    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        if( UPDATINGMAP==false)
           {
               if( g_solved_init )
               {
                   /////////////////////
                   //FOR DEBUG
                   double dist = sqrt((g_localization_map[0]-GOAL_G[0])*(g_localization_map[0]-GOAL_G[0])+ (g_localization_map[1]-GOAL_G[1])*(g_localization_map[1]-GOAL_G[1]) );
                   double ang = fabs(g_localization_map[2] - GOAL_G[2]);

                   if( dist < 5.2 )
                   {
                        outFile.close();
                       cout <<" GOAL " <<endl;
                       
                       std_msgs::Header header;
                       header.stamp = ros::Time::now();
                       header.frame_id = "/camera_init";
                       nav_msgs::Path msg;
                       msg.header = header;

                       for( int i=0; i<vTrajec_g.size();i++)
                       {
                           geometry_msgs::PoseStamped waypt;
                           waypt.header = header;
                           waypt.pose.position.x = vTrajec_g[i][0];
                           waypt.pose.position.y = vTrajec_g[i][1];
                           waypt.pose.position.z = 0;
                           msg.poses.push_back(waypt);
                       }
                       g_msgpub4.publish(msg);
                       
                       break;
                   }
                   else
                   {
                       SubTopicProcessTmp();
                   }
               }
               else
               {
                   SubTopicProcessTmp();
               }
           }
        /////////////////////
        ros::spinOnce();
        sleep(0.01);
    }
    cout << "END of DO-RRT*: ROS Version"<<endl;
}

int main(int argc, char* argv[])
{

    try
    {
        po::options_description desc("Options");
        desc.add_options()
            ("x", po::value<double>(),"goalX")
            ("y", po::value<double>(),"goalY")
            ("yaw", po::value<double>(),"goalYaw")
            ("globalT", po::value<double>(),"globalT")
            ("localT", po::value<double>(),"localT")
            ("bminx", po::value<double>(),"BminX")
            ("bminy", po::value<double>(),"BminY")
            ("bmaxx", po::value<double>(),"BmaxX")
            ("bmaxy", po::value<double>(),"BmaxY")
            ("rrt", po::value<double>(),"rrtmode")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc,
                    po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
        po::notify(vm);

        double x, y, yaw; 
        if (vm.count("x"))
        {
            GOAL_G[0] = vm["x"].as<double>();
        }
        if (vm.count("y"))
        {
            GOAL_G[1] = vm["y"].as<double>();
        }
        if (vm.count("yaw"))
        {
            GOAL_G[2] = RADIANS(vm["yaw"].as<double>());
            GOAL_G[2]=AngleUtils::toRange_PItoPI(GOAL_G[2]);
        }
        if (vm.count("globalT"))
        {
            PLANNINGTIME = vm["globalT"].as<double>();
        }
        if (vm.count("localT"))
        {
            PLANNINGTIME_L = vm["localT"].as<double>();
        }
        if (vm.count("bminx"))
        {
            BOUNDS.low[0] = vm["bminx"].as<double>();
        }
        if (vm.count("bmaxx"))
        {
            BOUNDS.high[0] = vm["bmaxx"].as<double>();
        }
        if (vm.count("bminy"))
        {
            BOUNDS.low[1] = vm["bminy"].as<double>();
        }
        if (vm.count("bmaxy"))
        {
            BOUNDS.high[1] = vm["bmaxy"].as<double>();
        }
        if (vm.count("rrt"))
        {
            MODEDORRTSTAR = 0;
        }
        VEHICLEPOS_TMP[0]=85.0;
        VEHICLEPOS_TMP[1]=0.0;
        VEHICLEPOS_TMP[2]=RADIANS(179);
        cout <<"BOUND " <<BOUNDS.low[0]<<" "<<BOUNDS.high[0]<<" "<<BOUNDS.low[1]<<" "<<BOUNDS.high[1]<<endl;

        GOAL_GORI[0] = GOAL_G[0];
        GOAL_GORI[1] = GOAL_G[1];
        GOAL_GORI[2] = GOAL_G[2];
/*
        //GOAL_G[0] = -10.0;
        //GOAL_G[1] = 0.0;
        //GOAL_G[2] = RADIANS(179);

        BOUNDS.low[0] = bminx;
        BOUNDS.high[0] = bmaxy;

        BOUNDS.low[1] = bminy;
        BOUNDS.high[1] = bmaxy;
        */
        ss_g = new og::SimpleSetup(g_space);
        
        ss_l = new og::SimpleSetup(g_space_local);

        boost::thread t=boost::thread(boost::bind(&PLANNER_FRAMEWORK));
        //boost::thread t=boost::thread(PLANNER_FRAMEWORK, "DORRT");

        t.join();
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
