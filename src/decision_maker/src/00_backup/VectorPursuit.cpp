#include <MathParam.h>
#include <AngleUtils.h>
#include <GeometricUtils.h>
#include "../inc/VectorPursuit.h"
#include "../inc/Parameters.h"
#include "../inc/CTRAModel.h"
VectorPursuit::VectorPursuit(Vector3d vehiclePos, Vector3d targetPos, float fLookAheadDist, float v_d, float T)
:m_fLookAheadDist(fLookAheadDist), m_v_d(v_d), m_T(T)
{
  TransformXYtoYX(vehiclePos, m_vehiclePos);
  TransformXYtoYX(targetPos, m_targetPos);
  m_fPathLength = 0.0;
  m_fCost = 0.0;
  m_bValid = false;
  Compute();
}

VectorPursuit::~VectorPursuit()
{

}

void VectorPursuit::Compute()
{
	int nChangeCount = 0;
	float preV = m_v_d;
	float v_d = m_v_d;
	
	Vector3d v_t(0.0, 0.0, 0.0);
	
	
	for( int i=0; i<m_T; i++)
	{
		
		//cout << "iter " << i << endl;
		////////////////////////////////////////
		GeometricUtils::TransRelativeCoord( m_vehiclePos[0], m_vehiclePos[1], m_vehiclePos[2], m_targetPos[0], m_targetPos[1], m_targetPos[2], v_t[0], v_t[1], v_t[2] );
		
		v_t[2] = AngleUtils::toRange_PItoPI( v_t[2] );
		
		if( m_v_d > 0.0 )
		{
			if( fabs(v_t[2]) < _PARAM_BACKWARD_LOOKAHEAD_ANGLE && v_t[0] < 0.0 )
			{
				v_d = m_v_d * -1.0;
				
			}
			else
			{
				v_d = m_v_d;
			}
		}
		else
		{
			if( fabs(v_t[2]) < _PARAM_BACKWARD_LOOKAHEAD_ANGLE && v_t[0] < 0.0 )
			{
				v_d = m_v_d;
			}
			else
			{
				v_d = m_v_d*-1.0;
			}
		}
		if( preV != v_d )
		{
			//cout << "v_t=" <<v_t[0] <<" " <<v_t[1] <<" " <<v_t[2] <<endl;
			nChangeCount++;
			
		}
	
		////////////////////////////////////////
		
		Vector3d lookAheadPos_global;
		GetLookAheadPt(m_targetPos, m_vehiclePos, m_fLookAheadDist, SIGN(v_d), lookAheadPos_global);
		
		Vector3d lookAheadPos(0.0, 0.0, 0.0); // lookAheadPos local coordinate
		GeometricUtils::TransRelativeCoord( m_vehiclePos[0], m_vehiclePos[1], m_vehiclePos[2]
											, lookAheadPos_global[0], lookAheadPos_global[1], lookAheadPos_global[2]
											, lookAheadPos[0], lookAheadPos[1], lookAheadPos[2] );
		
		lookAheadPos[2] = AngleUtils::toRange_PItoPI( lookAheadPos[2] );	
		
			cout<< "lookpos :"<<lookAheadPos_global[0]<<" "<<lookAheadPos_global[1]<<" "<<lookAheadPos_global[2]<<" "<<endl;
			
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
		
		CTRAModel model( vehiclePos[0], vehiclePos[1], vehiclePos[2], v_d, 0.0, wheelAngle, 1.0, 0.1, false);
		
		//cout << "=========1==========="<<endl;
		
		if( model.GetValid() == true )
		{
			//cout << "=========2==========="<<endl;
			m_fPathLength += model.GetPathLength();
			if( m_vRetEdge.size() == 0 )
			{
				m_vRetEdge = model.GetEdge();
			}
			else
			{
				//cout << "size() 1: " << m_vRetEdge.size() << " , " <<model.GetEdge().size()<<endl;
				vector<Edge*> edges = model.GetEdge();
				
				for( int k=0; k<edges.size(); k++)
				{
					m_vRetEdge.push_back(edges[k]);
				}
				
				//std::copy(model.GetEdge().begin(), model.GetEdge().end(), std::back_inserter(m_vRetEdge));
				//cout << "size() 2: " << m_vRetEdge.size() << " , " <<model.GetEdge().size()<<endl;
			}
			//cout << "=========3==========="<<endl;
			if( nChangeCount >= _PARAM_CHANGE_LIMIT )
			{
				cout << "changeCount LIMIT"<<nChangeCount << endl;
				m_fCost = CostFunction(v_t);
				m_bValid = true;
				break;
			}
			
			if( sqrt((m_targetPos[0]-m_vehiclePos[0])*(m_targetPos[0]-m_vehiclePos[0]) + (m_targetPos[1]-m_vehiclePos[1])*(m_targetPos[1]-m_vehiclePos[1]) ) < _PARAM_GOAL_DIST_RES
				&& fabs(m_targetPos[2]-m_vehiclePos[2]) < _PARAM_GOAL_ANG_RES )
			{
				cout << "=========4==========="<<endl;
				m_fCost = CostFunction(v_t);
				m_bValid = true;
				break;
			}
			//cout << "=========5==========="<<endl;
			if( i == m_T-1 )
			{
				//cout << "=========Valid LOOP "<<i<<" ,"<<m_vRetEdge.size()<<" ==========="<<endl;
				m_fCost = CostFunction(v_t);
				m_bValid = true;
			}
		//	cout << "=========Valid LOOP "<<i<<" ,"<<m_vRetEdge.size()<<" ==========="<<endl;
			Edge* pEdge =  model.GetEdge().back();
			Vector3d vehicleNewPos(pEdge->x,  pEdge->y,  pEdge->h);
			TransformXYtoYX(vehicleNewPos,m_vehiclePos);
			
		}
		else
		{
			cout << "=========inValid LOOP "<<i<<" ,"<<m_vRetEdge.size()<<" ==========="<<endl;
			m_fPathLength = -1.0;
			m_bValid = false;
			model.ClearEdges();
			m_vRetEdge.clear();
			break;
		}
		preV = v_d;
	}
	
}

float VectorPursuit::CostFunction(Vector3d v_t)
{
	float distNorm = sqrt(v_t[0]*v_t[0]+v_t[1]*v_t[1]);
	float cost_k = 100.0/distNorm;
	float cost_k2 = 10.0;
	return sqrt( (cost_k2*distNorm)*(cost_k2*distNorm) +  (cost_k*v_t[2])*(cost_k*v_t[2]) );
}

void VectorPursuit::GetLookAheadPt(Vector3d targetPos, Vector3d vehiclePos, double dLookAheadDist, double dir, Vector3d& lookAheadPos)
{
	Vector2d ret;
	Vector2d pt(vehiclePos[0], vehiclePos[1]);
	Vector2d lineCenter(targetPos[0], targetPos[1]); 
	GeometricUtils::CloestPointOnLine(lineCenter, targetPos[2], 1000.0, pt, ret );
	lookAheadPos[0] = ret[0] + dir*dLookAheadDist*cos(targetPos[2]);
	lookAheadPos[1] = ret[1] + dir*dLookAheadDist*sin(targetPos[2]);
	lookAheadPos[2] = targetPos[2];
}

void VectorPursuit::TransformXYtoYX(Vector3d pos, Vector3d& ret)
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
