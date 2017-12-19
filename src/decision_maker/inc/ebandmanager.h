#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeometricUtils.h>
#include <MathParam.h>
#include <AngleUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeometricUtils.h>
#include <MathParam.h>
#include <AngleUtils.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;
class EbandManager
{


    public:
    vector<Vector2d> mNodes_global;
    vector<Vector2d> mNodes_local;
    vector<Vector3d> mNodes_local1;
    MatrixXd mJ;
    MatrixXd mF;
    MatrixXd mdX_pre;
    
    // KD TREE : Obstacle
    vector<Vector2d> OBSTACLE;
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree;
    pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree;

    EbandManager();
    double FixDouble(double a, int n);
    void SetLocalNodes(vector<Vector2d> path,vector<Vector3d> path1, vector<Vector2d> obs);


    std::vector<pcl::PointXYZ> SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius);
    void setEnvironment(int type, double* start, double* goal);
    void SetLocalNodes();

    bool ComputeNewtonRaphton();

    void ComputeIntForce(Vector2d node, Vector2d node_, Vector2d _node, double k, double li_m,double li_p, MatrixXd &F);

    void ComputeIntJacobian(Vector2d node, Vector2d node_, Vector2d _node, double k, double li_m, double li_p,MatrixXd& J,MatrixXd& _J,MatrixXd& J_);

    void ComputeObsForce(Vector2d node, Vector2d obstacle, double k, double r, MatrixXd& F);

    void ComputeObsJacobian(Vector2d node, Vector2d obstacle, double k, double r, MatrixXd& J);

    void ComputeKammForce(Vector2d node, Vector2d kammNode, double k,MatrixXd& F);

    void ComputeKammJacobian(Vector2d node, Vector2d kammNode, double k,MatrixXd& J);
};

