#pragma once
#ifndef _DORRT_H_
#define _DORRT_H_

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <ctime>
using namespace Eigen;
using namespace std;

class DORRT
{
    private:

        ///////////////////////////////////////////
        // KD TREE : Obstacle
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_pTree;
        pcl::KdTreeFLANN<pcl::PointXYZ> m_kdTree;
    public :
        DORRT();
        ~DORRT();


        std::vector<pcl::PointXYZ> SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius);

        void magneticVectorForce(const double* init, const double* target, double *B);


        double computeRepulsiveForce(double K, double dist, double range, double x, double tar_x);

        double computeAttractiveForce(double K, double x, double tar_x);

        VectorXd ComputePotentialField2(double x, double y, double yaw, double* start, double* goal);

        VectorXd ComputeObstacleField(double x, double y);


        bool isFreeSpace(float x, float y)

            bool isValid(double x, double y, double yaw)

            bool DO( double* from_d, double* to_d, double* target_d)

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
