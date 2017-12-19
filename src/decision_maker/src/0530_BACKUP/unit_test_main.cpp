#include <iostream>
//#include "../inc/ParkingPlannerThread.h"
#include <ros/ros.h>    //ROS湲곕낯 ?ㅻ뜑 ?뚯씪
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
#include <ompl/geometric/planners/rrt/DORRTstar_1.h>
#include <ompl/geometric/planners/rrt/DORRTstar_2.h>
#include <ompl/geometric/planners/rrt/DORRTstar_3.h>
#include <ompl/geometric/planners/rrt/InformedDORRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/DORRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/pDORRT.h>
#include <ompl/geometric/planners/rrt/DORRT_1.h>
#include <ompl/geometric/planners/rrt/DORRT_2.h>
#include <ompl/geometric/planners/rrt/DORRT_3.h>
#include <boost/program_options.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/timer/timer.hpp>
#include<ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

//#define DRAW
//#define DRAWVEHICLE
//#define CONFIG_ANALYSIS
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ITER = 1;
string LOGFILENAME="DEFAULT";
int LOGFLAG=0;
bool GDOMODE= true;
bool isOriginal = false;
double START[3]={85.0,-8.0,RADIANS(179)};
double GOAL[3]={20.0,10.0,RADIANS(-90)};
double costThresold=0.0;
vector<Vector2d> OBSTACLE;
ob::RealVectorBounds BOUNDS(2);
//vector<vector<VectorXd> > VectorMap;
//vector<vector<VectorXd> > VectorObstacleMap;
//vector<vector<double> > VectorMapSize;
vector<vector<VectorXd> > VectorCollisionMap;
VectorXd VectorMap;


ros::Publisher potential_publisher_;
ros::Publisher potential2_publisher_;
ros::Publisher potential3_publisher_;
ros::Publisher msgpub3;
ros::Publisher map_pub;
geometry_msgs::PoseArray poseArray;
geometry_msgs::PoseArray poseArray2;
geometry_msgs::PoseArray poseArray3;

// KD TREE : Obstacle
pcl::PointCloud<pcl::PointXYZ>::Ptr m_pTree;
pcl::KdTreeFLANN<pcl::PointXYZ> m_kdTree;

// GRID
double grid_offset_x=20.0;
double grid_offset_y=22.0;

double m_per_cell_ = 0.5;

int grid_dim_x_=0.0;
int grid_dim_y_=0.0;

//Potential Field
double SafeRegion = 0.6;

double Range_obs = 3.0;  
double K_rep_obs = 1.00f;

double Range_rep=180.0;
double Range_att=180.0;

double K_rep = 100.0;    
double K_att = 0.002;


double car_c2f = 3.2;
double car_c2r = 0.8;
double car_width = 0.9;
//NAG
//double vfgain_ = 1.0;
//double vfMgain_ = 0.5;

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(costThresold));
    return obj;
}


std::vector<pcl::PointXYZ> SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::vector<pcl::PointXYZ> pvNode;

    if( m_kdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            pvNode.push_back(m_pTree->points[pointIdxRadiusSearch[i]]);
        }
    }
    return pvNode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetEnvironment(int type)
{
    cout << "type:"<< type<<endl;
    vector<double> obs_x_min;
    vector<double> obs_x_max;
    vector<double> obs_y_min;
    vector<double> obs_y_max;
    double res = 0.5;
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
            costThresold=116.0;
            START[0]=85.0;
            START[1]=0.0;
            START[2]=RADIANS(179);
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
                for(double y=-5; y<15; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));

            for(double x=25; x<27; x+=0.4 )
                for(double y=-15; y<10; y+=0.4)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=65; x<67; x+=0.4 )
                for(double y=-13; y<10; y+=0.4)
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
            costThresold=43.0;
            START[0]=28.0;
            START[1]=8.0;
            START[2]=RADIANS(-90);
            GOAL[0]=5.0;
            GOAL[1]=5.0;
            GOAL[2]=RADIANS(-90);

            BOUNDS.low[0] = -20.0;
            BOUNDS.high[0] = 30.0;

            BOUNDS.low[1] = -20;
            BOUNDS.high[1] = 20.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            // LEFT CENTER
            for(double x=-12; x<25; x+=0.5 )
            {

                // CENTER
                if( x > 2.5 && x < 7.5 ) continue; // width of parking space
                for(double y=1.0; y<10.0; y+=0.5)
                {
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            /*
            // LEFT
            for(double x=-14; x<-12; x+=0.5 )
            for(double y=-12; y<12; y+=0.5)
            OBSTACLE.push_back(Vector2d(x,y));
            // RIGHT BARRIER
            for(double x=32; x<40; x+=0.5 )
            for(double y=-12; y<12; y+=0.5)
            OBSTACLE.push_back(Vector2d(x,y));
             */
            //BOTTOM
            for(double x=-12; x<25; x+=0.5 )
                for(double y=-12; y<-8; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            // UPPER
            for(double x=-12; x<25; x+=0.5 )
                for(double y=10; y<12; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-4; x<0.0; x+=0.5 )
                for(double y=-2; y<-1; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-4; x<0.0; x+=0.5 )
                for(double y=-10; y<-6; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=8; x<12; x+=0.5 )
                for(double y=-10; y<-6; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            break;
        case 3: // Forward Driving
            costThresold=45.0;
            START[0]=28.0;
            START[1]=8.0;
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

            // LEFT CENTER
            for(double x=-12; x<25; x+=0.2 )
            {

                // CENTER
                if( x > 2.5 && x < 7.5 ) continue; // width of parking space
                for(double y=1.0; y<10.0; y+=0.2)
                {
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }

            /*
               for(double x=-14; x<-12; x+=0.2 )
               for(double y=-12; y<12; y+=0.2)
               OBSTACLE.push_back(Vector2d(x,y));
            // RIGHT BARRIER
            for(double x=32; x<40; x+=0.2 )
            for(double y=-12; y<12; y+=0.2)
            OBSTACLE.push_back(Vector2d(x,y));
             */ 
            for(double x=-12; x<25; x+=0.2 )
                for(double y=10; y<12; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));

            for(double x=-12; x<25; x+=0.2 )
                for(double y=-12; y<-8; y+=0.2)
                    OBSTACLE.push_back(Vector2d(x,y));

            for(double x=-4; x<0.1; x+=0.1 )
                for(double y=-10; y<-6; y+=0.1)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=8; x<12; x+=0.1 )
                for(double y=-10; y<-6; y+=0.1)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=8; x<12; x+=0.1 )
                for(double y=-1; y<1; y+=0.1)
                    OBSTACLE.push_back(Vector2d(x,y));
            break;
            break;
        case 4:
            costThresold=40.45;
            START[0]=0.0;
            START[1]=0.0;
            START[2]=RADIANS(0);
            GOAL[0]=35.0;
            GOAL[1]=5.0;
            GOAL[2]=RADIANS(-90);

            BOUNDS.low[0] = -10.0;
            BOUNDS.high[0] = 50.0;

            BOUNDS.low[1] = -10;
            BOUNDS.high[1] = 15.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];
            OBSTACLE.push_back(Vector2d(100,100));
            break;
        case 5:
            costThresold=37.0;
            START[0]=24.0;
            START[1]=-2.0;
            START[2]=RADIANS(179);
            GOAL[0]=-5.0;
            GOAL[1]=-6.0;
            GOAL[2]=RADIANS(0);


            BOUNDS.low[0] = -20.0;
            BOUNDS.high[0] = 40.0;

            BOUNDS.low[1] = -10;
            BOUNDS.high[1] = 10.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            // LEFT CENTER
            for(double x=-12; x<25; x+=0.5 )
            {

                // CENTER
                if( x > 2.5 && x < 7.5 ) continue; // width of parking space
                for(double y=1.0; y<10.0; y+=0.5)
                {
                    OBSTACLE.push_back(Vector2d(x,y));
                }
            }
            for(double x=-14; x<-8; x+=0.5 )
                for(double y=-12; y<12; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            // RIGHT BARRIER
            for(double x=32; x<40; x+=0.5 )
                for(double y=-12; y<12; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=-12; x<25; x+=0.5 )
                for(double y=10; y<12; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));

            for(double x=-12; x<25; x+=0.5 )
                for(double y=-12; y<-8; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));

            for(double x=8; x<40.0; x+=0.5 )
                for(double y=-12; y<-4; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=3; x<40; x+=0.5 )
                for(double y=-10; y<-6; y+=0.5)
                    OBSTACLE.push_back(Vector2d(x,y));
            break;
        case 6:
            START[0]=104.33;
            START[1]=2.377;
            START[2]=RADIANS(DEGREES(4.759)+90);//RADIANS(DEGREES(4.759));
            GOAL[0]=118.8;
            GOAL[1]=7.65;
            GOAL[2]=RADIANS(DEGREES(3.1977)+90);//RADIANS(DEGREES(3.1977));

            cout << DEGREES(START[2]) <<" "<< DEGREES(GOAL[2]) << endl; 
            START[2]=AngleUtils::toRange_PItoPI(START[2]);
            GOAL[2]=AngleUtils::toRange_PItoPI(GOAL[2]);

            cout << DEGREES(START[2]) <<" "<< DEGREES(GOAL[2]) << endl; 
            BOUNDS.low[0] = 100.0;
            BOUNDS.high[0] = 140.0;

            BOUNDS.low[1] = -15;
            BOUNDS.high[1] = 15.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            ////////////////////////////////////
            obs_x_min.push_back(113.75+SafeRegion);
            obs_x_max.push_back(117.15-SafeRegion);
            obs_y_min.push_back(5.35+SafeRegion);
            obs_y_max.push_back(10.35-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(113.75+SafeRegion);
            obs_x_max.push_back(138.45-SafeRegion);
            obs_y_min.push_back(10.35+SafeRegion);
            obs_y_max.push_back(12.35-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(120.45+SafeRegion);
            obs_x_max.push_back(138.45-SafeRegion);
            obs_y_min.push_back(5.35+SafeRegion);
            obs_y_max.push_back(10.35-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(104.33+SafeRegion);
            obs_x_max.push_back(124.79-SafeRegion);
            obs_y_min.push_back(-5.3+SafeRegion);
            obs_y_max.push_back(-0.3-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(127.1+SafeRegion);
            obs_x_max.push_back(129.4-SafeRegion);
            obs_y_min.push_back(-5.3+SafeRegion);
            obs_y_max.push_back(-0.3-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(104.33+SafeRegion);
            obs_x_max.push_back(129.4-SafeRegion);
            obs_y_min.push_back(-7.3+SafeRegion);
            obs_y_max.push_back(-5.3-SafeRegion);

            for(int i=0;i<obs_x_min.size();i++)
            {
                for(double x=obs_x_min[i]; x<=obs_x_max[i]; x+=res )
                {
                    for(double y=obs_y_min[i]; y<=obs_y_max[i]; y+=res )
                    {
                        OBSTACLE.push_back(Vector2d(x,y));
                    }
                }
            }
            break;
        case 7:

            START[0]=26.5;
            START[1]=13.3;
            START[2]=RADIANS(DEGREES(4.759)+90);//RADIANS(DEGREES(4.759));
            GOAL[0]=51.7;   
            GOAL[1]=7.65;
            GOAL[2]=RADIANS(DEGREES(3.1977)+90);//RADIANS(DEGREES(3.1977));

            cout << DEGREES(START[2]) <<" "<< DEGREES(GOAL[2]) << endl; 
            START[2]=AngleUtils::toRange_PItoPI(START[2]);
            GOAL[2]=AngleUtils::toRange_PItoPI(GOAL[2]);

            cout << DEGREES(START[2]) <<" "<< DEGREES(GOAL[2]) << endl; 
            BOUNDS.low[0] = 20;
            BOUNDS.high[0] = 60.0;

            BOUNDS.low[1] = 0;
            BOUNDS.high[1] = 25.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];

            ////////////////////////////////////
            obs_x_min.push_back(20.45+SafeRegion);
            obs_x_max.push_back(80.45-SafeRegion);
            obs_y_min.push_back(17.35+SafeRegion);
            obs_y_max.push_back(24.35-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(10.15+SafeRegion);
            obs_x_max.push_back(41.15-SafeRegion);
            obs_y_min.push_back(-4.65+SafeRegion);
            obs_y_max.push_back(11.85-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(48.95+SafeRegion);
            obs_x_max.push_back(80.15-SafeRegion);
            obs_y_min.push_back(10.35+SafeRegion);
            obs_y_max.push_back(11.85-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(47.15+SafeRegion);
            obs_x_max.push_back(48.95-SafeRegion);
            obs_y_min.push_back(5.35+SafeRegion);
            obs_y_max.push_back(12.35-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(53.75+SafeRegion);
            obs_x_max.push_back(80.45-SafeRegion);
            obs_y_min.push_back(5.35+SafeRegion);
            obs_y_max.push_back(10.35-SafeRegion);
            ////////////////////////////////////
            obs_x_min.push_back(40.50+SafeRegion);
            obs_x_max.push_back(64.15-SafeRegion);
            obs_y_min.push_back(-4.65+SafeRegion);
            obs_y_max.push_back(0.35-SafeRegion);

            for(int i=0;i<obs_x_min.size();i++)
            {
                for(double x=obs_x_min[i]; x<=obs_x_max[i]; x+=res )
                {
                    for(double y=obs_y_min[i]; y<=obs_y_max[i]; y+=res )
                    {
                        OBSTACLE.push_back(Vector2d(x,y));
                    }
                }
            }
            break;
        case 8:

            START[0]=0.0;
            START[1]=0.0;
            START[2]=RADIANS(0);//RADIANS(DEGREES(4.759));
            GOAL[0]=150;   
            GOAL[1]=130;
            GOAL[2]=RADIANS(DEGREES(3.1977)+90);//RADIANS(DEGREES(3.1977));

            cout << DEGREES(START[2]) <<" "<< DEGREES(GOAL[2]) << endl; 
            START[2]=AngleUtils::toRange_PItoPI(START[2]);
            GOAL[2]=AngleUtils::toRange_PItoPI(GOAL[2]);

            cout << DEGREES(START[2]) <<" "<< DEGREES(GOAL[2]) << endl; 
            BOUNDS.low[0] = 0;
            BOUNDS.high[0] = 200.0;

            BOUNDS.low[1] = 0;
            BOUNDS.high[1] = 200.0;

            // GRID
            grid_offset_x=-BOUNDS.low[0];
            grid_offset_y=-BOUNDS.low[1];



            ////////////////////////////////////
            for(int j=0;j<10;j++)
            {
                for(int i=0;i<50;i++)
                {
                    double car_x = i*(2*car_width+0.8)+10;
                    double car_y = j*12;

                    obs_x_min.push_back(car_x-car_width);
                    obs_x_max.push_back(car_x+car_width);
                    obs_y_min.push_back(car_y-car_c2r);
                    obs_y_max.push_back(car_y+car_c2f);
                }
            }
            for(int i=0;i<obs_x_min.size();i++)
            {
                for(double x=obs_x_min[i]; x<=obs_x_max[i]; x+=res )
                {
                    for(double y=obs_y_min[i]; y<=obs_y_max[i]; y+=res )
                    {
                        OBSTACLE.push_back(Vector2d(x,y));
                    }
                }
            }
            break;
        default:
            break;
    }

    cout <<"end of alloc obstacles"<<endl;
    grid_dim_x_ = ceil((BOUNDS.high[0] - BOUNDS.low[0])/m_per_cell_);
    grid_dim_y_ = ceil((BOUNDS.high[1] - BOUNDS.low[1])/m_per_cell_);

    nav_msgs::MapMetaData info;
    info.width = grid_dim_x_;
    info.height = grid_dim_y_;
    info.resolution = m_per_cell_;

    info.origin.position.x = -grid_offset_x;
    info.origin.position.y = -grid_offset_y;

    ///////////////////////////////////////////////////////////
    m_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //m_pTree->clear();

    for(int k=0; k<OBSTACLE.size(); k++)
    {
        m_pTree->push_back(pcl::PointXYZ(OBSTACLE[k](0), OBSTACLE[k](1), 0.0));
#ifdef CONFIG_ANALYSIS
        cout << "OBS:" << "\t"<<OBSTACLE[k](0)<< "\t"<<OBSTACLE[k](1) << "\t"<<endl;
#endif
    }

    //cout<< "kdtree"<<endl;
    ////////////////////////////////
    m_kdTree.setInputCloud(m_pTree);
    ////////////////////////////////
    //cout<< info.width << " " <<info.height <<" " <<info.resolution << " " << info.origin.position.x <<" " <<info.origin.position.y <<endl;
    ///////////////////////////////////////////////////////////
    /*
       QuadTree::Map map(info.width, info.height);

       for (int x = 0; x < info.width; x++)
       {
       for (int y = 0; y < info.height; y++)
       {
       int obsval =0;
       map.Insert(QuadTree::Cell(x,y,info.width,obsval));

       }
       }
       for (int x = 0; x < info.width; x++)
       {
       for (int y = 0; y < info.height; y++)
       {
       double dx = -grid_offset_x + (x*m_per_cell_+m_per_cell_/2.0);
       double dy = -grid_offset_y + (y*m_per_cell_+m_per_cell_/2.0);
       std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(dx,dy,0),m_per_cell_);
       if( obs.size() > 0 )
       {
       map.Insert(QuadTree::Cell(x,y,info.width,1));
       }

       }
       }

       nav_msgs::OccupancyGrid* newGrid = map.Grid();

       newGrid->header.frame_id = "/camera_init";
       newGrid->header.stamp = ros::Time::now();
       newGrid->info = info;
       map_pub.publish(*newGrid);
     */
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
        return K*((1.0f/dist)-(1.0f/range))*(1.0f/(dist*dist*dist))*(x-tar_x);
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

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),Range_obs);
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

        sumX += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, x, obs_x);
        sumY += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, y, obs_y);
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
            double x_rep = computeRepulsiveForce(K_rep, start_dist,
                    Range_rep, x, start[0]);
            double y_rep = computeRepulsiveForce(K_rep, start_dist,
                    Range_rep, y, start[1]);

            // Vector Force (Unit Vector)
            ran(0) =sumX+(x_rep);
            ran(1) =sumY+(y_rep);

            double x_att = computeAttractiveForce(K_att, x, goal[0]);
            double y_att = computeAttractiveForce(K_att, y, goal[1]);

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

VectorXd ComputePotentialField(double x, double y, double* start, double* goal)
{
    VectorXd ret = VectorXd::Zero(5);

    ret(0) = x;
    ret(1) = y;


    double start_dist = sqrt(((x - start[0]) * (x - start[0]))
            + ((y - start[1]) * (y - start[1])));

    //    if( start_dist < 5.0 )
    {
        double B[2]={0,0};
        double target[2]={ret(0),ret(1)};
        magneticVectorForce(start, target, B);

        ret(2) += B[0]*2.0;
        ret(3) += B[1]*2.0;
    }
    double x_rep = computeRepulsiveForce(K_rep, start_dist,
            Range_rep, x, start[0]);
    double y_rep = computeRepulsiveForce(K_rep, start_dist,
            Range_rep, y, start[1]);

    // Vector Force (Unit Vector)
    ret(2) +=(x_rep);
    ret(3) +=(y_rep);

    double goal_dist = sqrt(((x - goal[0]) * (x - goal[0]))
            + ((y - goal[1]) * (y - goal[1])));
    //    if( goal_dist < 10.0 )
    {
        double B[2]={0,0};
        double target[2]={ret(0),ret(1)};
        magneticVectorForce(goal, target, B);

        ret(2) += B[0]*2.0;
        ret(3) += B[1]*2.0;
    }
    //  else
    {
        double x_att = computeAttractiveForce(K_att, x, goal[0]);
        double y_att = computeAttractiveForce(K_att, y, goal[1]);

        ret(2) +=(x_att);
        ret(3) +=(y_att);

    }
    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),Range_obs);
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

        sumX += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, x, obs_x);
        sumY += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, y, obs_y);

        //    cout << sumX <<" " <<sumY<<endl;
    }
    //cout << sumX <<" " <<sumY<<endl;
    ret(2) = sumX;
    ret(3) = sumY;
    ret(4) = sqrt(ret(2)*ret(2)+ret(3)*ret(3));
    return ret;
}

VectorXd ComputeObstacleField(double x, double y)
{
    VectorXd ret = VectorXd::Zero(5);

    ret(0) = x;
    ret(1) = y;

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),Range_obs);

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

        sumX += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, x, obs_x);
        sumY += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, y, obs_y);
    }
    ret(2)=sumX;
    ret(3)=sumY;
    ret(4) = sqrt(ret(2)*ret(2)+ret(3)*ret(3));
    return ret;
}

bool isFreeSpace(float x, float y)
{
    bool isFreeSpace = true;

    std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(x,y,0),SafeRegion);

    if( obs.size() > 0 )
        isFreeSpace = false;

    return isFreeSpace;
}

bool isValid(double x, double y, double yaw)
{
    double x_ = x;
    double y_ = y;

    double _x = x;
    double _y = x;

    bool isFreeSpace1 = isFreeSpace(x_,y_);

    _x = x_ + 3.2*cos(yaw);
    _y = y_ + 3.2*sin(yaw);

    bool isFreeSpace2 = isFreeSpace(_x,_y);

    _x = x_ - 0.8*cos(yaw);
    _y = y_ - 0.8*sin(yaw);

    bool isFreeSpace3 = isFreeSpace(_x,_y);

    //////////////////////////////////////////////////////////////////////////////
    x_ = x + 0.9*cos(yaw+RADIANS(90.0));
    y_ = y + 0.9*sin(yaw+RADIANS(90.0));

    bool isFreeSpace4 = isFreeSpace(x_,y_);

    _x = x_ + 3.2*cos(yaw);
    _y = y_ + 3.2*sin(yaw);

    bool isFreeSpace5 = isFreeSpace(_x,_y);

    _x = x_ - 0.8*cos(yaw);
    _y = y_ - 0.8*sin(yaw);

    bool isFreeSpace6 = isFreeSpace(_x,_y);
    //////////////////////////////////////////////////////////////////////////////
    x_ = x + 0.9*cos(yaw-RADIANS(90.0));
    y_ = y + 0.9*sin(yaw-RADIANS(90.0));

    bool isFreeSpace7 = isFreeSpace(x_,y_);

    _x = x_ + 3.2*cos(yaw);
    _y = y_ + 3.2*sin(yaw);

    bool isFreeSpace8 = isFreeSpace(_x,_y);

    _x = x_ - 0.8*cos(yaw);
    _y = y_ - 0.8*sin(yaw);

    bool isFreeSpace9 = isFreeSpace(_x,_y);
    return isFreeSpace1 && isFreeSpace2 && isFreeSpace3 && isFreeSpace4 && isFreeSpace5 && isFreeSpace6 &&isFreeSpace7 && isFreeSpace8 && isFreeSpace9;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// DO
bool DO( double* from_d, double* to_d, double* target_d)
{

    //  return false;

    if( isOriginal || (to_d[0] == GOAL[0] && to_d[1] == GOAL[1]) )
    {
        return false;
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
        poseArray2.poses.push_back(poseStamped.pose);
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
        poseArray3.poses.push_back(poseStamped.pose);
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
        poseArray2.poses.push_back(poseStamped.pose);
        ////////////////////////////////////////
#endif
    }
    return ret;
}

bool DesiredOrientation( ob::State* from, ob::State* to, ob::State* target)
{

    if( GDOMODE == true )
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
            return true;
        }
        return false;
    }
    else
    {
        return false;
    }
}

bool DesiredOrientation_(ob::State* from, ob::State* to, ob::State* target)
{
    if( GDOMODE == true )
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
            return true;
        }
        return false;
    }
    else
    {
        return false;
    }
}

VectorXd magneticfield(ob::State* target, ob::State* nouse1, ob::State* nouse2)
{

    //    return VectorXd::Zero(2);
    ob::ClothoidStateSpace::StateType *target_ = target->as<ob::ClothoidStateSpace::StateType>();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( isOriginal || (target_d[0] == GOAL[0] && target_d[1] == GOAL[1]) )
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
            poseArray.poses.push_back(poseStamped.pose);
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
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],START,GOAL);
        }
        else
        {
            double NEAR[3];
            ob::ClothoidStateSpace::StateType *nouse1_ = nouse1->as<ob::ClothoidStateSpace::StateType>();
            NEAR[0]=nouse1_->getX();
            NEAR[1]=nouse1_->getY();
            NEAR[2]=nouse1_->getYaw();
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],NEAR,GOAL);
        }
        target_d[2] =AngleUtils::toRange_PItoPI( atan2(VectorField(1), VectorField(0)));
        target_->setYaw(target_d[2]);

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
            poseArray.poses.push_back(poseStamped.pose);
        }
#endif
        return VectorXd::Zero(2);
    }
    return VectorXd::Zero(2);

}
VectorXd magneticfield_(ob::State* target, ob::State* nouse1, ob::State* nouse2)
{
    //    return VectorXd::Zero(2);
    ob::SE2StateSpace::StateType *target_ = target->as<ob::SE2StateSpace::StateType>();
    double target_d[3]; target_d[0]=target_->getX(); target_d[1]=target_->getY(); target_d[2]=target_->getYaw();

    if( isOriginal || (target_d[0] == GOAL[0] && target_d[1] == GOAL[1]) )
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
            poseArray.poses.push_back(poseStamped.pose);
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
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],START,GOAL);
        }
        else
        {
            double NEAR[3];
            ob::SE2StateSpace::StateType *nouse1_ = nouse1->as<ob::SE2StateSpace::StateType>();
            NEAR[0]=nouse1_->getX();
            NEAR[1]=nouse1_->getY();
            NEAR[2]=nouse1_->getYaw();
            VectorField = ComputePotentialField2(target_d[0],target_d[1],target_d[2],NEAR,GOAL);
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
            poseArray.poses.push_back(poseStamped.pose);
        }
#endif
        return VectorXd::Zero(2);
    }
    return VectorXd::Zero(2);
}

bool randomCheck_(ob::State* rand)
{

    ob::SE2StateSpace::StateType *rand_ = rand->as<ob::SE2StateSpace::StateType>();

    double x = rand_->getX();
    double y =rand_->getY();

    if( isOriginal )
        return false;
    else
        return !isFreeSpace(x,y);

}
bool randomCheck(ob::State* rand)
{
    ob::ClothoidStateSpace::StateType *rand_ = rand->as<ob::ClothoidStateSpace::StateType>();

    double x = rand_->getX();
    double y =rand_->getY();

    if( isOriginal )
        return false;
    else
        return !isFreeSpace(x,y);
}


bool isStateValid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& VectorCollisionMap, const ob::State *state)
{
    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    return si->satisfiesBounds(s) && isValid(s->getX(),s->getY(),s->getYaw());
}

bool isStateValid_Clothoid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& VectorCollisionMap, const ob::State *state)
{
    const ob::ClothoidStateSpace::StateType *s = state->as<ob::ClothoidStateSpace::StateType>();
    return si->satisfiesBounds(s) && isValid(s->getX(),s->getY(),s->getYaw());
}

void benchmark_RRT(ob::StateSpacePtr space,double time, bool isClothoid)
{

    ob::ScopedState<> start(space), goal(space);
    // set the start and goal states
    start[0] =START[0];
    start[1] =START[1]; 
    start[2] = START[2];//0.99*boost::math::constants::pi<double>();
    goal[0] = GOAL[0];
    goal[1] = GOAL[1]; 
    goal[2] = GOAL[2];//0.99*boost::math::constants::pi<double>();


    //VectorFieldGenerator(START, GOAL, BOUNDS);

    ////////////////////////////////////////////////////////////////////

    if( isClothoid )
        space->as<ob::ClothoidStateSpace>()->setBounds(BOUNDS);
    else
        space->as<ob::SE2StateSpace>()->setBounds(BOUNDS);

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

    /*    if( isClothoid )
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
     */
    ompl::tools::Benchmark b(ss, "my experiment");
    // We add the planners to evaluate.
    if( isClothoid )
    {
        b.addPlanner(std::make_shared<ompl::geometric::DORRT>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        b.addPlanner(std::make_shared<ompl::geometric::DORRT_1>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        b.addPlanner(std::make_shared<ompl::geometric::DORRT_2>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        b.addPlanner(std::make_shared<ompl::geometric::DORRT_3>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        b.addPlanner(std::make_shared<ompl::geometric::pDORRT>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
    }
    else
    {
        b.addPlanner(std::make_shared<ompl::geometric::DORRT>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        b.addPlanner(std::make_shared<ompl::geometric::DORRT_1>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        b.addPlanner(std::make_shared<ompl::geometric::DORRT_2>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        b.addPlanner(std::make_shared<ompl::geometric::DORRT_3>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        b.addPlanner(std::make_shared<ompl::geometric::pDORRT>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
    }
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::pRRT>(ss.getSpaceInformation()));
    // etc
    // For planners that we want to configure in specific ways,
    // the ompl::base::PlannerAllocator should be used:
    //b.addPlannerAllocator(boost::bind(&myConfiguredPlanner, _1));
    // etc.
    // Now we can benchmark: 5 second time limit for each plan computation,
    // 100 MB maximum memory usage per plan computation, 50 runs for each planner
    // and true means that a text-mode progress bar should be displayed while
    // computation is running.
    ompl::tools::Benchmark::Request req;
    req.maxTime = time;
    req.maxMem = 100.0;
    req.runCount = ITER;
    req.displayProgress = true;
    b.benchmark(req);
    // This will generate a file of the form ompl_host_time.log
    if( LOGFLAG == 1 )
        b.saveResultsToFile(LOGFILENAME.c_str());
    else
        b.saveResultsToFile();

}
void benchmark(ob::StateSpacePtr space,double time, bool isClothoid)
{

    ob::ScopedState<> start(space), goal(space);
    // set the start and goal states
    start[0] =START[0];
    start[1] =START[1]; 
    start[2] = START[2];//0.99*boost::math::constants::pi<double>();
    goal[0] = GOAL[0];
    goal[1] = GOAL[1]; 
    goal[2] = GOAL[2];//0.99*boost::math::constants::pi<double>();


    //VectorFieldGenerator(START, GOAL, BOUNDS);

    ////////////////////////////////////////////////////////////////////

    if( isClothoid )
        space->as<ob::ClothoidStateSpace>()->setBounds(BOUNDS);
    else
        space->as<ob::SE2StateSpace>()->setBounds(BOUNDS);

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
    ss.setOptimizationObjective(getThresholdPathLengthObj(si));
    /*    if( isClothoid )
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
     */
    ompl::tools::Benchmark b(ss, "my experiment");
    // We add the planners to evaluate.

    //b.addPlanner(std::make_shared<ompl::geometric::InformedRRTstar>(ss.getSpaceInformation()));
    //b.addPlanner(std::make_shared<ompl::geometric::BITstar>(ss.getSpaceInformation()));
    //b.addPlanner(std::make_shared<ompl::geometric::FMT>(ss.getSpaceInformation()));
    if( isClothoid )
    {
        //b.addPlanner(std::make_shared<ompl::geometric::InformedDORRTstar>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        b.addPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        //b.addPlanner(std::make_shared<ompl::geometric::DORRTstar_1>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        //b.addPlanner(std::make_shared<ompl::geometric::DORRTstar_2>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
        //b.addPlanner(std::make_shared<ompl::geometric::DORRTstar_3>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
    }
    else
    {
        //b.addPlanner(std::make_shared<ompl::geometric::InformedDORRTstar>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        b.addPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        //b.addPlanner(std::make_shared<ompl::geometric::DORRTstar_1>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        //b.addPlanner(std::make_shared<ompl::geometric::DORRTstar_2>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
        //b.addPlanner(std::make_shared<ompl::geometric::DORRTstar_3>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
    }
    b.addPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));

    // etc
    // For planners that we want to configure in specific ways,
    // the ompl::base::PlannerAllocator should be used:
    //b.addPlannerAllocator(boost::bind(&myConfiguredPlanner, _1));
    // etc.
    // Now we can benchmark: 5 second time limit for each plan computation,
    // 100 MB maximum memory usage per plan computation, 50 runs for each planner
    // and true means that a text-mode progress bar should be displayed while
    // computation is running.
    ompl::tools::Benchmark::Request req;
    req.maxTime = time;
    req.maxMem = 100.0;
    req.runCount = ITER;
    req.displayProgress = true;
    b.benchmark(req);
    // This will generate a file of the form ompl_host_time.log
    if( LOGFLAG == 1 )
        b.saveResultsToFile(LOGFILENAME.c_str());
    else
        b.saveResultsToFile();
}
void plan(ob::StateSpacePtr space,double time, bool isClothoid, bool isRRT)
{
    ob::ScopedState<> start(space), goal(space);
    // set the start and goal states
    start[0] =START[0];
    start[1] =START[1]; 
    start[2] = START[2];//0.99*boost::math::constants::pi<double>();
    goal[0] = GOAL[0];
    goal[1] = GOAL[1]; 
    goal[2] = GOAL[2];//0.99*boost::math::constants::pi<double>();


    //VectorFieldGenerator(START, GOAL, BOUNDS);

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

        if( isRRT )
        {
            if( isClothoid )
            {
                ss.setPlanner(std::make_shared<ompl::geometric::DORRT_1>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
            }
            else
            {
                ss.setPlanner(std::make_shared<ompl::geometric::DORRT_1>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
            }
        }
        else
        {
            if( isClothoid )
            {
                ss.setPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),randomCheck,magneticfield, DesiredOrientation));
            }
            else
            {
                ss.setPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),randomCheck_,magneticfield_, DesiredOrientation_));
            }
        }
        //b, l::base::PlannerPtr(new ompl::geometric::KPIECE1(ss.getiSpaceInformation())), range);
        // this call is optional, but we put it in to get more output information
        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
        ss.setup();
        //    ss.print();
        // attempt to solve the problem within 30 seconds of planning time
        ob::PlannerStatus solved = ss.solve(time);

        std_msgs::Header header;
        header.stamp = ros::Time::now();
#ifdef DRAWVEHICLE
        header.frame_id = "/camera_init_global";
#else
        header.frame_id = "/camera_init";
#endif
        poseArray.header = header;
        potential_publisher_.publish(poseArray);

        poseArray2.header = header;
        potential2_publisher_.publish(poseArray2);

        poseArray3.header = header;
        potential3_publisher_.publish(poseArray3);
        if (solved)
        {
            std::vector<double> reals;

            //            std::cout << "Found solution:" << std::endl;
            //            ss.simplifySolution();
            og::PathGeometric path = ss.getSolutionPath();
            path.interpolate(1000);

            /////////////////////////////////////////////////////////////////////
            /* RVIZ */

            nav_msgs::Path msg;
            msg.header = header;

            if( isClothoid )
            {
                for(int i=0;i<path.getStateCount(); i++)
                {

                    geometry_msgs::PoseStamped waypt;
                    waypt.header = header;
                    const ob::ClothoidStateSpace::StateType *s = path.getState(i)->as<ob::ClothoidStateSpace::StateType>();
                    double x=s->getX(), y=s->getY(), yaw=s->getYaw();

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

                }
            }
            else
            {
                for(int i=0;i<path.getStateCount(); i++)
                {

                    geometry_msgs::PoseStamped waypt;
                    waypt.header = header;
                    const ob::SE2StateSpace::StateType *s = path.getState(i)->as<ob::SE2StateSpace::StateType>();
                    double x=s->getX(), y=s->getY(), yaw=s->getYaw();

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


                }
            }

            // Extract the plan in world coordinates, we assume the path is all in the same frame

            //    path.printAsMatrix(std::cout);
            msgpub3.publish(msg);
            ros::spinOnce();
            /////////////////////////////////////////////////////////////////////

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

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "DORRT");  //?몃뱶紐?珥덇린??    
    ros::NodeHandle node; //?몃뱶 ?몃뱾 ?좎뼵
    ros::NodeHandle priv_nh("~");
    potential_publisher_= node.advertise<geometry_msgs::PoseArray>("randomvec", 1);
    potential2_publisher_= node.advertise<geometry_msgs::PoseArray>("potential", 1);
    potential3_publisher_= node.advertise<geometry_msgs::PoseArray>("potential2", 1);
    map_pub= node.advertise<nav_msgs::OccupancyGrid>("/gridmap", 1);
    msgpub3 = node.advertise<nav_msgs::Path>("/resultpath", 1);
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
            ("logfilename", po::value<string>(),"log filename")
            ("trajectory", po::value<std::vector<double > >()->multitoken(),
             "print trajectory from (0,0,0) to a user-specified x, y, and theta")
            ("distance", "print distance grid")
            ("benchmark", "Benchmark")
            ("rrt", "rrt")
            ("gdo", "GDO Mode")
            ("nogdo", "No GDO Mode")
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

        if( vm.count("logfilename"))
        {
            LOGFLAG = 1;
            LOGFILENAME = vm["logfilename"].as<string>();
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

        if (vm.count("nogdo"))
            GDOMODE = false;

        if (vm.count("gdo"))
            GDOMODE = true;

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

        if( vm.count("benchmark"))
        {
            if( vm.count("rrt"))
                benchmark_RRT(space,time,isClothoid);
            else               
                benchmark(space,time,isClothoid);
        }
        else
        {            
            if( vm.count("rrt"))
            {
                plan(space,time,isClothoid,true);
            }
            else
            {
                plan(space,time,isClothoid,false);
            }
        }
        //        plan(space,time,isClothoid);
        // benchmark(space,time,isClothoid);


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
*/
