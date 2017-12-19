#include "../inc/Tree.h"
#include "../inc/Node.h"
#include <AngleUtils.h>
Tree::Tree()
{
  m_pTree  = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

Tree::~Tree()
{
    for( int i=0; i<m_vNodes.size(); i++)
    {
        delete m_vNodes[i];
    }
}

void Tree::ClearTree()
{
    m_pTree->clear();
    for( int i=0; i<m_vNodes.size(); i++)
    {
        delete m_vNodes[i];
    }
}
void Tree::AddNode(Node* pNode)
{
    m_vNodes.push_back(pNode);
    m_pTree->push_back(pNode->getNodePoint());
}

void Tree::SetkdTree()
{
    m_kdTree.setInputCloud(m_pTree);

}

std::vector<Node*> Tree::SearchNode(pcl::PointXYZI searchPoint, int K)
{
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::vector<Node*> pvNode;
    if ( m_kdTree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
	for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	{
		pvNode.push_back(m_vNodes[pointIdxNKNSearch[i]]);
              /*std::cout << "    "  <<   m_pTree->points[ pointIdxNKNSearch[i] ].x
                        << " " << m_pTree->points[ pointIdxNKNSearch[i] ].y
                        << " " << m_pTree->points[ pointIdxNKNSearch[i] ].z
                        << " " << searchPoint.x
                        << " " << searchPoint.y
                        << " " << searchPoint.z
                        << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
	}
    }

    return pvNode;
}

std::vector<Node*> Tree::SearchNodeByRadius(pcl::PointXYZI searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::vector<Node*> pvNode;

    if( m_kdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
	{
 		
              pvNode.push_back(m_vNodes[pointIdxRadiusSearch[i]]);
/*            std::cout << "    "  <<   m_pTree->points[ pointIdxRadiusSearch[i] ].x
                      << " " << m_pTree->points[ pointIdxRadiusSearch[i] ].y
                      << " " << m_pTree->points[ pointIdxRadiusSearch[i] ].z
		      << " " << searchPoint.x
                      << " " << searchPoint.y
                      << " " << searchPoint.z
                      << std::endl;
*/
	}
    }
    return pvNode;
}

bool Tree::IsGoal(float x, float y, float h, float fGoalDistRes, float fGoalAngRes)
{
    for( int i=0; i<m_vNodes.size(); i++)
    {
        Edge* pEdge = m_vNodes[i]->GetBackEdge();

	if( sqrt((pEdge->x - x)*(pEdge->x - x) + (pEdge->y - y)*(pEdge->y - y)) < fGoalDistRes
		    && fabs(AngleUtils::toRange(pEdge->h -h)) < fGoalAngRes )
	{
		return true;
	}
	
    }
    return false;
}
