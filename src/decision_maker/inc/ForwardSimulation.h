#pragma once
#ifndef _ForwardSimulation_H_
#define _ForwardSimulation_H_

#include <iostream>
#include <vector>
#include <Eigen/Core>

using namespace std;

class ForwardSimulation
{
public:
	ForwardSimulation(vector<Vector3d>& vRetNodes, float x, float y, float h, float v, float wa, float a, float dT, float T);
	bool Compute();

protected:
	vector<VectorXd> m_vNodes;
	vector<Vector3d>& m_vRetNodes;

	float m_dT;
	float m_T;
	float m_YawRate;
	float m_AccLat;
};

#endif
