#include <iostream>
//#include "../inc/ParkingPlannerThread.h"
#include "../inc/VectorPursuit.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <GeometricUtils.h>
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

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////////////////////////////////

double grid_offset_x=0.0;
double grid_offset_y=0.0;

double m_per_cell_ = 0.3;

int grid_dim_x_=0.0;
int grid_dim_y_=0.0;
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

void VectorFieldGenerator(double start[2], double goal[2], ob::RealVectorBounds
        bounds, vector<Vector2d> obs, vector<vector<VectorXd> >& VectorMap,
        vector<vector<VectorXd> >& VectorObstacleMap)
{
    ///////////////////////////////////////////////////////////////////
    // 1. Potential Field

    // Parameters
    double SafeRegion = 0.3;  
    
    double Range_obs = 10.0;  
    double K_rep_obs = 5.2f;

    double Range_rep=180.0;
    double Range_att=180.0;

    double K_rep = 200.0;    
    double K_att = 0.005;

    int grid_dim_x = ceil((bounds.high[0] - bounds.low[0])/m_per_cell_);
    int grid_dim_y = ceil((bounds.high[1] - bounds.low[1])/m_per_cell_);

    grid_dim_x_ = ceil((bounds.high[0] - bounds.low[0])/m_per_cell_);
    grid_dim_y_ = ceil((bounds.high[1] - bounds.low[1])/m_per_cell_);

    double num_obs[grid_dim_x][grid_dim_y];
    
	// Initialize Potential Map
    for (int x = 0; x < grid_dim_x; x++)
    {
		vector<VectorXd> v;
		VectorMap.push_back(v);
		
        vector<VectorXd> v_;
		VectorObstacleMap.push_back(v_);
		
        for (int y = 0; y < grid_dim_y; y++)
        {
            VectorXd vec = VectorXd::Zero(5);
            VectorMap[x].push_back(vec);

            VectorXd vecObs = VectorXd::Zero(5);
            VectorObstacleMap[x].push_back(vecObs);
        }
    }
    
    for (int x = 0; x < grid_dim_x; x++) {
        for (int y = 0; y < grid_dim_y; y++) {
            num_obs[x][y]=0;
        }
    }
    
    for (unsigned int k=0; k<obs.size(); k++)
    {
        int i= int((obs[k](0) +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
        int j= int((obs[k](1) +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
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

            {
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
/*
                cout << i <<" "<<j <<" "<<x <<" "<<y <<" "<<start[0]<<
                   " "<<start[1]<<" "<<goal[0] << " "<<goal[1] <<" "
                   <<start_dist <<" "<<K_rep<< " "<<K_att <<" " <<x_rep<<" "
                   <<y_rep<<" "<<x_att <<" " << y_att <<endl;
*/
                // Vector Force (Unit Vector)
                VectorMap[i][j](0) = x;
                VectorMap[i][j](1) = y;
                VectorMap[i][j](2) +=(x_att + x_rep);
                VectorMap[i][j](3) +=(y_att + y_rep);

                VectorObstacleMap[i][j](0) = x;
                VectorObstacleMap[i][j](1) = y;

            }
            // Generate obstacle potential
            for (unsigned int k = 0; k < obs.size(); k++)
            {
                double obs_x = obs[k](0);
                double obs_y = obs[k](1);

                if(obs_x == 0.0 && obs_y == 0.0)
                    continue;

                double obs_dist = sqrt(((x - obs_x) * (x - obs_x))
                        + ((y - obs_y) * (y - obs_y)));

                if (obs_dist > SafeRegion && obs_dist < Range_obs)
                {
                    VectorMap[i][j](2) += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, x, obs_x);
                    VectorMap[i][j](3) += computeRepulsiveForce(K_rep_obs, obs_dist, Range_obs, y, obs_y);
                }
                else if(obs_dist <= SafeRegion)
                {
                   // VectorMap[i][j](2) = 0.0;
                   // VectorMap[i][j](3) = 0.0;

                    VectorObstacleMap[i][j](2) = 1;
                }
            }

        }
    }

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

double conv2pi(double x){
   const double twopi = 2. * boost::math::constants::pi<double>();
   x = fmod(x,twopi);
    if (x < 0)
        x += twopi;
    return x;
}

VectorXd vectorfield_(const vector<vector<VectorXd> >&VectorMap, ob::State* state)
{
    ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    double x=s->getX(), y=s->getY(), yaw=s->getYaw();
    
    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
   
    VectorXd ret = VectorXd::Zero(2);
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        ret(0) = VectorMap[i][j](2);
        ret(1) = VectorMap[i][j](3);
        ret.normalize();
    }
    float vTheta = atan2(ret(1), ret(0));
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vTheta);

    double randValue = double(rand()%300)/10.0*3.1415/180.0;
    int randSign = rand()%2;
    if( randSign == 1 )
    {
        vTheta += randValue;
    }
    else
    {
        vTheta -= randValue;
    }


    double gain = 0.3;
    //cout << x << " " << y<<" " <<x+ret(0)<< " " << y+ret(1)<<endl;
    //s->setX( x + ret(0)*gain);
    //s->setY( y + ret(1)*gain);
    //s->setYaw(conv2pi(vTheta));

    return ret;
}
VectorXd vectorfield(const vector<vector<VectorXd> >&VectorMap, ob::State* state)
{
    ob::ClothoidStateSpace::StateType *s = state->as<ob::ClothoidStateSpace::StateType>();
    double x=s->getX(), y=s->getY(), yaw=s->getYaw();
    
    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
   
    VectorXd ret = VectorXd::Zero(2);
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        ret(0) = VectorMap[i][j](2);
        ret(1) = VectorMap[i][j](3);
        ret.normalize();
    }
    float vTheta = atan2(ret(1), ret(0));
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vTheta);

    double randValue = double(rand()%300)/10.0*3.1415/180.0;
    int randSign = rand()%2;
    if( randSign == 1 )
    {
        cout << x << " " << y<<" "<<yaw<< " " <<i <<" " <<j << " " <<ret(0)<< " "
        << ret(1)<<" "<<vTheta<<" "<<randValue<<endl;
     
        vTheta += randValue;
    }
    else
    {
    cout << x << " " << y<<" "<<yaw<< " " <<i <<" " <<j << " " <<ret(0)<< " "
        << ret(1)<<" "<<vTheta<<" "<<-randValue<<endl;
        vTheta -= randValue;
    }


    double gain = 0.3;
    //s->setX( x + ret(0)*gain);
    //s->setY( y + ret(1)*gain);
    //s->setYaw(conv2pi(vTheta));

    return ret;
}

bool isStateValid(const ob::SpaceInformation *si, const vector<vector<VectorXd>
        >&
        VectorObstacleMap, const ob::State *state)
{
    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    double x=s->getX(), y=s->getY(), yaw=s->getYaw();

    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    bool isFreeSpace = true;
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        if( VectorObstacleMap[i][j](2) == 1 )
        {
            isFreeSpace = false;
        }
    }
    return si->satisfiesBounds(s) && isFreeSpace;
}

bool isStateValid_Clothoid(const ob::SpaceInformation *si,
        const vector<vector<VectorXd> >& VectorObstacleMap, const ob::State *state)
{
    const ob::ClothoidStateSpace::StateType *s = state->as<ob::ClothoidStateSpace::StateType>();
    double x=s->getX(), y=s->getY(), yaw=s->getYaw();

    int i= int((x +grid_offset_x - m_per_cell_/2.0 )/m_per_cell_);
    int j= int((y +grid_offset_y - m_per_cell_/2.0 )/m_per_cell_);
    
    bool isFreeSpace = true;
    if( i >= 0 && i < grid_dim_x_ && j >= 0 && j < grid_dim_y_ )
    {
        if( VectorObstacleMap[i][j](2) == 1 )
        {
            isFreeSpace = false;
        }
    }

    return si->satisfiesBounds(s) && isFreeSpace;
}
void plan(ob::StateSpacePtr space,double time, bool isClothoid)
{
	// 0. Build KdTree
	// =BuildTree()

	ob::ScopedState<> start(space), goal(space);
    // set the start and goal states
	start[0] =85.0;
	start[1] =20.0; 
	start[2] = 0.99*boost::math::constants::pi<double>();
	goal[0] = 10.0;
	goal[1] = 50.0; 
	//goal[2] = 0.99*boost::math::constants::pi<double>();
	goal[2] = 0.99*boost::math::constants::pi<double>();
    
    ob::RealVectorBounds bounds(2);
    bounds.low[0] = 0;
    bounds.high[0] = 100.0;
	
    bounds.low[1] = 0;
    bounds.high[1] = 50.0;

    ////////////////////////////////////////////////////////////////////
    //Potential

    vector<Vector2d> obs;
    for(double x=40; x<60; x+=0.3 )
        for(double y=25; y<30; y+=0.3)
            obs.push_back(Vector2d(x,y));

    vector<vector<VectorXd> > VectorMap;
    vector<vector<VectorXd> > VectorObstacleMap;
    double start_[2];
    double goal_[2];
    start_[0] = start[0];
    start_[1] = start[1];
    goal_[0] = goal[0];
    goal_[1] = goal[1];

    VectorFieldGenerator(start_, goal_, bounds, obs, VectorMap,
            VectorObstacleMap);
    
    ////////////////////////////////////////////////////////////////////
	
    if( isClothoid )
    	space->as<ob::ClothoidStateSpace>()->setBounds(bounds);
	else
    	space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ob::SpaceInformationPtr si(ss.getSpaceInformation());
	ss.setStateValidityChecker(std::bind(
        isClothoid ? &isStateValid_Clothoid : &isStateValid, si.get(),
        VectorObstacleMap, std::placeholders::_1));

	
    if( isClothoid )
	{
		start[3] = 0.0;
		goal[3] = 0.0;
	}
    
	ss.setStartAndGoalStates(start, goal);

    if( isClothoid )
    {
        ss.setPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),VectorMap,vectorfield));
    }
    else
    {
        ss.setPlanner(std::make_shared<ompl::geometric::DORRTstar>(ss.getSpaceInformation(),VectorMap,vectorfield_));
        //ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
    }
	//b, l::base::PlannerPtr(new ompl::geometric::KPIECE1(ss.getiSpaceInformation())), range);
    // this call is optional, but we put it in to get more output information
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();
    ss.print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = ss.solve(time);

    if (solved)
    {
        std::vector<double> reals;

        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        og::PathGeometric path = ss.getSolutionPath();
        path.interpolate(1000);
        path.printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
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

        ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(5.88));

		bool isClothoid = false;

        if (vm.count("clothoidrev"))
		{
            space = ob::StateSpacePtr(new ob::ClothoidStateSpace(0.17, true));
			isClothoid = true;
		}
		
		if (vm.count("clothoid"))
		{
			space = ob::StateSpacePtr(new ob::ClothoidStateSpace(0.17, false));
			isClothoid = true;
		}
		
		if (vm.count("dubins"))
            space = ob::StateSpacePtr(new ob::DubinsStateSpace(5.88, false));
        if (vm.count("dubinssym"))
            space = ob::StateSpacePtr(new ob::DubinsStateSpace(5.88, true));
    
		double time = 30.0;

		if (vm.count("time"))
			time = vm["time"].as<double>();

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
