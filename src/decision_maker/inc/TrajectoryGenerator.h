#pragma once
#ifndef _TrajectoryGenerator_H_
#define _TrajectoryGenerator_H_

#include <iostream>
#include <vector>
#include <DataStructures.h>
#include <Eigen/Core>
using namespace Eigen;
using namespace std;

class TrajectoryGenerator
{
private:
	Vector3d m_vehiclePos;
	Vector3d m_targetPos;
	float m_fLookAheadDist;
	float m_v_d;
	float m_T;
	float m_dT;
	float m_fPathLength;
	float m_fCost;
	bool m_bValid;
	vector<Vector3d> m_vNodes;
	vector<Vector2d> m_vControls;
public :
   TrajectoryGenerator(Vector3d initPos, Vector3d goalPos, float fLookAheadDist, float v_d, float T, float dT );
   ~TrajectoryGenerator();
   
   void Compute();
   void TransformXYtoYX(Vector3d pos, Vector3d& ret); // vector pursuit coordinate : north(x) east(y)    Convention: east(x) north(y) 
   void GetLookAheadPt(Vector3d targetPos, Vector3d vehiclePos, double dLookAheadDist, double dir, Vector3d& lookAheadPos);
   vector<Vector3d> GetNodes(){ return m_vNodes; }
   vector<Vector2d> GetControls(){ return m_vControls; }
};

#endif
