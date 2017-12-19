#include <MathParam.h>
#include <AngleUtils.h>
#include "../inc/CTRAModel.h"
#include "../inc/Parameters.h"
CTRAModel::CTRAModel(float x, float y, float h, float v, float a, float wa, float T, float dT, bool bReverseMode)
    :VehicleModel(x,y,h,v,wa,T),m_dT(dT),m_bReverseMode(bReverseMode)
{
    Edge* pEdge = m_vEdge.back();
    pEdge->a = a;
	m_PathLength = 0.0;
    m_R = 0.0;
    if( wa != 0.0 )
    {
        m_R = sqrt( ( _PARAM_DIST_C2R * _PARAM_DIST_C2R )+( _PARAM_DIST_F2R * _PARAM_DIST_F2R )*cot(wa)*cot(wa) );
    }

    if( wa < 0 )
    {
        m_R *= -1.0;
    }

    if( fabs(wa) > 0.01 )
    {
        m_YawRate = v / m_R;
        m_AccLat = v*v / m_R;
    }
    else
    {
        m_YawRate = 0;
        m_AccLat = 0;
    }

    m_bValid = Compute();
	
}

CTRAModel::~CTRAModel()
{

}

bool CTRAModel::CollisionDetection(float x, float y)
{
	return false;
   
}

bool CTRAModel::Compute()
{
    bool bValid = true;
	
	//////////////////////////////////////////////////////////////////////////////////////////
    for( float t=m_dT; t<=m_T; t+=m_dT)
    {
        Edge* pEdge_pre =NULL;

        if( m_bReverseMode == true )
            pEdge_pre = m_vEdge.front();
        else
            pEdge_pre = m_vEdge.back();

        if( (m_AccLat*m_AccLat + pEdge_pre->a*pEdge_pre->a) > (_PARAM_GRAVITY*_PARAM_FRICTION)*(_PARAM_GRAVITY*_PARAM_FRICTION) )
        {
			//cout << "roll over"<<endl;
            bValid = false;
            break;
        }

        Edge* pEdge = new Edge();

        if( fabs(pEdge_pre->wa) < 0.01 ) // wheel angle is near zero
        {
            pEdge->h = pEdge_pre->h + m_YawRate * m_dT;
            pEdge->x = pEdge_pre->x + pEdge_pre->v*cos(pEdge_pre->h)*m_dT + 0.5*pEdge_pre->a*cos(pEdge_pre->h)*m_dT*m_dT;
            pEdge->y = pEdge_pre->y + pEdge_pre->v*sin(pEdge_pre->h)*m_dT + 0.5*pEdge_pre->a*sin(pEdge_pre->h)*m_dT*m_dT;
            pEdge->v = pEdge_pre->v + pEdge_pre->a * m_dT;
            pEdge->wa = pEdge_pre->wa; // wheel angel is constant
        }
        else
        {
            if( fabs(pEdge_pre->v) < 0.01 ) // velocity is near zero
            {
                pEdge->x = pEdge_pre->x;
                pEdge->y = pEdge_pre->y;
                pEdge->h = pEdge_pre->h;
                pEdge->v = pEdge_pre->v + pEdge_pre->a * m_dT;
                pEdge->wa = pEdge_pre->wa; // wheel angel is constant
            }
            else
            {
                pEdge->h = pEdge_pre->h + m_YawRate * m_dT;
                pEdge->x = pEdge_pre->x +
                        pEdge_pre->v*(sin(pEdge->h) - sin(pEdge_pre->h))*1.0/m_YawRate +
                        (pEdge_pre->a*(cos(pEdge->h) - cos(pEdge_pre->h)) + pEdge_pre->a*sin(pEdge->h)*m_YawRate*m_dT)*1.0/(m_YawRate*m_YawRate);

                pEdge->y = pEdge_pre->y -
                        pEdge_pre->v*(cos(pEdge->h) - cos(pEdge_pre->h))*1.0/m_YawRate +
                        (pEdge_pre->a*(sin(pEdge->h) - sin(pEdge_pre->h)) - pEdge_pre->a*cos(pEdge->h)*m_YawRate*m_dT)*1.0/(m_YawRate*m_YawRate);

                pEdge->v = pEdge_pre->v + pEdge_pre->a * m_dT;
                pEdge->wa = pEdge_pre->wa; // wheel angel is constant
            }
        }

        // Collision Detection
        if( CollisionDetection(pEdge->x, pEdge->y) == false)
        {
			m_PathLength = m_PathLength +
						   sqrt( ( pEdge->x - pEdge_pre->x )*( pEdge->x - pEdge_pre->x ) + ( pEdge->y - pEdge_pre->y )*( pEdge->y - pEdge_pre->y ));
            if( m_bReverseMode == true )
                m_vEdge.insert(m_vEdge.begin(),pEdge);
            else
                m_vEdge.push_back(pEdge);

        }
        else // collision
        {
			//cout << "collision"<<endl;
			m_PathLength = -1;
            bValid = 0;
            break;
        }
    }
    
    ////////////////////////////
    /*if( m_bReverseMode == true )
    {
		for( int i=0; i<m_vEdge.size()*4.0/5.0; i++)
		{
			Edge* pEdge_pre =NULL;
            pEdge_pre = m_vEdge.back();
            delete pEdge_pre;
			m_vEdge.pop_back();
		}
    }*/
    return bValid;
}



