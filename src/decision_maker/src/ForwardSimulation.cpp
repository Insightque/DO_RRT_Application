#include <MathParam.h>
#include <AngleUtils.h>
#include "../inc/Parameters.h"
#include "../inc/ForwardSimulation.h"

ForwardSimulation::ForwardSimulation(vector<Vector3d>& vRetNodes, float x, float y, float h, float v, float wa, float a, float dT, float T): m_dT(dT), m_T(T), m_vRetNodes(vRetNodes)	
{
	VectorXd vec(6);
	vec << x,y,h,v,wa,a;
	m_vRetNodes.push_back(vec.head(3));
	m_vNodes.push_back(vec);
	
	float R = 0.0;

	if( v < 0 )
	{
		wa *= -1.0;
	}
	if( wa != 0.0 )
	{
		R = sqrt( ( _PARAM_DIST_C2R * _PARAM_DIST_C2R )+( _PARAM_DIST_F2R * _PARAM_DIST_F2R )*cot(wa)*cot(wa) );
	}

	if( wa < 0 )
	{
		R *= -1.0;
	}

	if( fabs(wa) > 0.01 )
	{
		m_YawRate = v / R;
		m_AccLat = v*v / R;
	}
	else
	{
		m_YawRate = 0;
		m_AccLat = 0;
	}

	Compute();

}

bool ForwardSimulation::Compute()
{
    bool bValid = true;

  //  cout << "Forward Compute():"<< m_dT << " " << m_T <<endl;

    for( float t=m_dT; t<=m_T; t+=m_dT)
    {
     
	VectorXd vNode_pre(6);
        vNode_pre = m_vNodes.back();

	VectorXd vNode(6);

        if( fabs(vNode_pre[4]) < 0.01 ) // wheel angle is near zero
        {
            vNode[2] = vNode_pre[2] + m_YawRate * m_dT;
            vNode[0] = vNode_pre[0] + vNode_pre[3]*cos(vNode_pre[2])*m_dT + 0.5*vNode_pre[5]*cos(vNode_pre[2])*m_dT*m_dT;
            vNode[1] = vNode_pre[1] + vNode_pre[3]*sin(vNode_pre[2])*m_dT + 0.5*vNode_pre[5]*sin(vNode_pre[2])*m_dT*m_dT;
            vNode[3] = vNode_pre[3] + vNode_pre[5] * m_dT;
            vNode[4] = vNode_pre[4]; // wheel angel is constant

        }
        else // vec << x,y,h,v,wa,a; v3 wa4 a5
        {
            if( fabs(vNode_pre[3]) < 0.01 ) // velocity is near zero
            {
                vNode[0] = vNode_pre[0];
                vNode[1] = vNode_pre[1];
                vNode[2] = vNode_pre[2];
                vNode[3] = vNode_pre[3] + vNode_pre[5] * m_dT;
                vNode[4] = vNode_pre[4]; // wheel angel is constant
            }
            else
            {
                vNode[2] = vNode_pre[2] + m_YawRate * m_dT;
                vNode[0] = vNode_pre[0] +
                        vNode_pre[3]*(sin(vNode[2]) - sin(vNode_pre[2]))*1.0/m_YawRate +
                        (vNode_pre[5]*(cos(vNode[2]) - cos(vNode_pre[2])) + vNode_pre[5]*sin(vNode[2])*m_YawRate*m_dT)*1.0/(m_YawRate*m_YawRate);

                vNode[1] = vNode_pre[1] -
                        vNode_pre[3]*(cos(vNode[2]) - cos(vNode_pre[2]))*1.0/m_YawRate +
                        (vNode_pre[5]*(sin(vNode[2]) - sin(vNode_pre[2])) - vNode_pre[5]*cos(vNode[2])*m_YawRate*m_dT)*1.0/(m_YawRate*m_YawRate);

                vNode[3] = vNode_pre[3] + vNode_pre[5] * m_dT;
                vNode[4] = vNode_pre[4]; // wheel angel is constant
            }
        }
	vNode[5] = vNode_pre[5];

        m_vNodes.push_back(vNode);
        m_vRetNodes.push_back(vNode.head(3));
//	cout << "F:" << vNode[0] << " " << vNode[1] << endl;
    }
}
