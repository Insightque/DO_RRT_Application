#include <MathParam.h>
#include <AngleUtils.h>
#include <GeometricUtils.h>
#include "../inc/TrajectoryGenerator.h"
#include "../inc/Parameters.h"
#include "../inc/ForwardSimulation.h"
TrajectoryGenerator::TrajectoryGenerator(Vector3d vehiclePos, Vector3d targetPos, float fLookAheadDist, float v_d, float T, float dT)
:m_fLookAheadDist(fLookAheadDist), m_v_d(v_d), m_T(T), m_dT(dT)
{
  TransformXYtoYX(vehiclePos, m_vehiclePos);
  TransformXYtoYX(targetPos, m_targetPos);
  m_fPathLength = 0.0;
  m_fCost = 0.0;
  m_bValid = false;
  Compute();
}

TrajectoryGenerator::~TrajectoryGenerator()
{

}

void TrajectoryGenerator::Compute()
{
//	int nChangeCount = 0;
	float v_d = m_v_d;
	
	Vector3d v_t(0.0, 0.0, 0.0);
	
	
	for( int i=0; i<m_T/m_dT; i++)
	{
		
		//cout << "iter " << i << endl;
		////////////////////////////////////////
		GeometricUtils::TransRelativeCoord( m_vehiclePos[0], m_vehiclePos[1], m_vehiclePos[2], m_targetPos[0], m_targetPos[1], m_targetPos[2], v_t[0], v_t[1], v_t[2] );
		
		v_t[2] = AngleUtils::toRange_PItoPI( v_t[2] );
		
	
		////////////////////////////////////////
		
		Vector3d lookAheadPos_global;
		GetLookAheadPt(m_targetPos, m_vehiclePos, m_fLookAheadDist, SIGN(v_d), lookAheadPos_global);
		
		Vector3d lookAheadPos(0.0, 0.0, 0.0); // lookAheadPos local coordinate
		GeometricUtils::TransRelativeCoord( m_vehiclePos[0], m_vehiclePos[1], m_vehiclePos[2]
											, lookAheadPos_global[0], lookAheadPos_global[1], lookAheadPos_global[2]
											, lookAheadPos[0], lookAheadPos[1], lookAheadPos[2] );
		
		lookAheadPos[2] = AngleUtils::toRange_PItoPI( lookAheadPos[2] );	
		
//			cout<< "lookpos :"<<lookAheadPos_global[0]<<" "<<lookAheadPos_global[1]<<" "<<lookAheadPos_global[2]<<" "<<endl;
			
		double w_X_s_d, w_Y_s_d;
		
		if( lookAheadPos[1] == 0.0 )
		{
			w_X_s_d = m_vehiclePos[0] - _PARAM_K_VECTOR_PURSUIT * ( lookAheadPos[1] - m_vehiclePos[1] ) / lookAheadPos[2];
			w_Y_s_d = m_vehiclePos[1] + _PARAM_K_VECTOR_PURSUIT * ( lookAheadPos[0] - m_vehiclePos[0] ) / lookAheadPos[2];
		}
		else
		{
			double d = sqrt(lookAheadPos[0]*lookAheadPos[0] + lookAheadPos[1]*lookAheadPos[1]);
			double phi = atan2( lookAheadPos[1] - d*d/(2.0*lookAheadPos[1]), lookAheadPos[0] ) - atan2( -d*d/(2.0*lookAheadPos[1]), 0.0);
			
			w_X_s_d = m_vehiclePos[0] - _PARAM_K_VECTOR_PURSUIT * phi / ( (_PARAM_K_VECTOR_PURSUIT-1.0)*phi + lookAheadPos[2] ) * ( d*d / (2.0*lookAheadPos[1]) * sin( m_vehiclePos[2] ));
			w_Y_s_d = m_vehiclePos[1] + _PARAM_K_VECTOR_PURSUIT * phi / ( (_PARAM_K_VECTOR_PURSUIT-1.0)*phi + lookAheadPos[2] ) * ( d*d / (2.0*lookAheadPos[1]) * cos( m_vehiclePos[2] ));
		}
		
		double v_X_s_d = m_vehiclePos[0] * cos(m_vehiclePos[2]) + m_vehiclePos[1] * sin(m_vehiclePos[2]) - ( w_X_s_d * cos(m_vehiclePos[2]) + w_Y_s_d * sin(m_vehiclePos[2]) );
		double v_Y_s_d = -m_vehiclePos[0] * sin(m_vehiclePos[2]) + m_vehiclePos[1] * cos(m_vehiclePos[2]) - ( -w_X_s_d * sin(m_vehiclePos[2]) + w_Y_s_d * cos(m_vehiclePos[2]) );
		
		//cout << w_X_s_d <<" "<<w_Y_s_d <<" " <<v_X_s_d<<" " <<v_Y_s_d << endl;
		
		double R = sqrt( (m_vehiclePos[0] - w_X_s_d)*(m_vehiclePos[0] - w_X_s_d) + (m_vehiclePos[1] - w_Y_s_d)*(m_vehiclePos[1] - w_Y_s_d) );
		double wheelAngle = SIGN(v_Y_s_d)*v_d / R;
		//cout << DEGREES(wheelAngle);
		//if( fabs(wheelAngle) > _PARAM_WHEELANGLE_LIMIT )
		//{
		//	wheelAngle = SIGN(wheelAngle)*_PARAM_WHEELANGLE_LIMIT;
		//} 
	
		Vector3d vehiclePos;
		TransformXYtoYX(m_vehiclePos, vehiclePos);
		
		//////////////////////////////////////////////////////////////////////////////
		ForwardSimulation fs(m_vNodes, vehiclePos[0], vehiclePos[1], vehiclePos[2], v_d, wheelAngle, 0.0, m_dT, m_dT);
				
		Vector3d vehicleNewPos = m_vNodes.back();
		//cout << vehicleNewPos[0] << " " << vehicleNewPos[1] << endl;
		TransformXYtoYX(vehicleNewPos,m_vehiclePos);
	}
	
}

void TrajectoryGenerator::GetLookAheadPt(Vector3d targetPos, Vector3d vehiclePos, double dLookAheadDist, double dir, Vector3d& lookAheadPos)
{
	Vector2d ret;
	Vector2d pt(vehiclePos[0], vehiclePos[1]);
	Vector2d lineCenter(targetPos[0], targetPos[1]); 
	GeometricUtils::CloestPointOnLine(lineCenter, targetPos[2], 1000.0, pt, ret );
	lookAheadPos[0] = ret[0] + dir*dLookAheadDist*cos(targetPos[2]);
	lookAheadPos[1] = ret[1] + dir*dLookAheadDist*sin(targetPos[2]);
	lookAheadPos[2] = targetPos[2];
	//cout << targetPos[0] <<" "<<targetPos[1] <<" "<<targetPos[2] <<" "<<vehiclePos[0] <<" "<<vehiclePos[1] <<" "<<vehiclePos[2] <<" "<<lookAheadPos[0] <<" "<<lookAheadPos[1] <<" "<<lookAheadPos[2] <<" "<<endl;
}

void TrajectoryGenerator::TransformXYtoYX(Vector3d pos, Vector3d& ret)
{
	float rad = RADIANS(90.0) - pos[2];
	if( SIGN( rad ) < 0 )
	{
		rad = RADIANS(360.0) + rad;
	}
	
	ret[0] = pos[1];
	ret[1] = pos[0];
	ret[2] = rad;
}
