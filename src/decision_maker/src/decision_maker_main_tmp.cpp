#include <iostream>
//#include "../inc/ParkingPlannerThread.h"
#include <ros/ros.h>    //ROS湲곕낯 ?ㅻ뜑 ?뚯씪
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeometricUtils.h>
#include <MathParam.h>
#include <AngleUtils.h>
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;
using namespace Eigen;
///////////////////////////////////////////
ros::Publisher g_msgpub1;
ros::Subscriber g_msgsub1; //LocalizationData
ros::Subscriber g_msgsub2; //HeightMapData
///////////////////////////////////////////


// From Localization Module
void SubTopicProcess1(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    m_pos[0] = msg->data.at(0);
    m_pos[1] = msg->data.at(1);
    m_pos[2] = msg->data.at(2);
    m_vel = msg->data.at(3); // m/s
}

// From HeightMap Module
void SubTopicProcess2(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if( g_updatePlanner )
    {
        g_updatePlanner = false;
        m_pTree->clear();
        m_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        START_G[0] = msg->data.at(0); // x
        START_G[1] = msg->data.at(1); // y
        START_G[2] = msg->data.at(2); // theta
        START_G[2]=AngleUtils::toRange_PItoPI(START_G[2]);
        
        GOAL_G[0] = msg->data.at(3); // goal x
        GOAL_G[1] = msg->data.at(4); // goal y
        GOAL_G[2] = msg->data.at(5); // goal theta
        GOAL_G[2]=AngleUtils::toRange_PItoPI(GOAL_G[2]);
        
        OBSTACLE.clear();

        for(int i=0; i<int((msg->data.size()-6)/4.0); i++)
        {
            OBSTACLE.push_back(Vector2d(msg->data.at(i*4+6),msg->data.at(i*4+7)));
        }

        for(int k=0; k<OBSTACLE.size(); k++)
        {
            m_pTree->push_back(pcl::PointXYZ(OBSTACLE[k](0), OBSTACLE[k](1), 0.0));
        }

        m_kdTree.setInputCloud(m_pTree);

        // solve
    }
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "DecisionMaker");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");
    g_msgpub1 = node.advertise<geometry_msgs::PoseArray>("randomvec", 1);

    g_msgsub1 = node.subscribe("LocalizationData", 1, &SubTopicProcess1, this);
    g_msgsub2 = node.subscribe("velodyne_potential_array", 1, &SubTopicProcess2, this);
    return 0;
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
            COSTTHRESHOLD=116.0;
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
            COSTTHRESHOLD=43.0;
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
            COSTTHRESHOLD=45.0;
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
            COSTTHRESHOLD=40.45;
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

            OBSTACLE.push_back(Vector2d(100,100));
            break;
        case 5:
            COSTTHRESHOLD=37.0;
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
    ///////////////////////////////////////////////////////////
}
