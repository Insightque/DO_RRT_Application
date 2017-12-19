#include "../inc/Node.h"

Node::Node(float x, float y, float h, int id, int pId, int dir, float cost, vector<Edge*> vEdge)
:m_x(x), m_y(y), m_h(h), m_id(id), m_pId(pId), m_dir(dir), m_cost(cost), m_vEdge(vEdge)
{
    m_ratio = 0.1; // hueristic value

    // initialize PointXYZI for generating kdTree
    m_q.x = m_x;
    m_q.y = m_y;
    m_q.z = m_h * m_ratio;
    m_q.intensity = m_id;
}

Node::Node(float x, float y, float h, int id, int pId, int dir, float cost)
:m_x(x), m_y(y), m_h(h), m_id(id), m_pId(pId), m_dir(dir), m_cost(cost)
{
    m_ratio = 0.1; // hueristic value

    // initialize PointXYZI for generating kdTree
    m_q.x = m_x;
    m_q.y = m_y;
    m_q.z = m_h * m_ratio;
    m_q.intensity = m_id;
}

Node::~Node()
{
    for( int i=0; i<m_vEdge.size(); i++)
    {
        delete m_vEdge[i];
    }
}
