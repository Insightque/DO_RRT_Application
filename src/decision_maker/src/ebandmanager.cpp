#include "../inc/ebandmanager.h"
using namespace std;
EbandManager::EbandManager()
{

}

void EbandManager::SetLocalNodes(vector<Vector2d> path,vector<Vector3d> path1, vector<Vector2d> obs)
{
    mNodes_local = path;
    mNodes_local1 = path1;
    OBSTACLE = obs;

    g_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int k=0; k<OBSTACLE.size(); k++)
    {
        g_pTree->push_back(pcl::PointXYZ(OBSTACLE[k](0)+2.0, OBSTACLE[k](1), 0.0));
    }
    g_kdTree.setInputCloud(g_pTree);

    int nMatSize = (mNodes_local.size()-2)*2.0;
    mJ.resize(nMatSize,nMatSize);
    mF.resize(nMatSize,1);
    mdX_pre.resize(nMatSize,1);
    for(int i=1;i<mNodes_local.size()-1;i++)
    {
        mdX_pre((i-1)*2) = mNodes_local.at(i)(0);
        mdX_pre((i-1)*2+1) = mNodes_local.at(i)(1);
    }

}

void EbandManager::SetLocalNodes()
{


    double start[3]={0,0,0};
    double goal[3]={15.3982,-2.6363};

    setEnvironment(0, start, goal);

    int nMatSize = (mNodes_local.size()-2)*2.0;
    mJ.resize(nMatSize,nMatSize);
    mF.resize(nMatSize,1);
    mdX_pre.resize(nMatSize,1);
    for(int i=1;i<mNodes_local.size()-1;i++)
    {
        mdX_pre((i-1)*2) = mNodes_local.at(i)(0);
        mdX_pre((i-1)*2+1) = mNodes_local.at(i)(1);
    }

}

std::vector<pcl::PointXYZ> EbandManager::SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::vector<pcl::PointXYZ> pvNode;

    if( g_kdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            pvNode.push_back(g_pTree->points[pointIdxRadiusSearch[i]]);
        }
    }
    return pvNode;
}

void EbandManager::setEnvironment(int type, double* start, double* goal)
{

    mNodes_local.push_back(Vector2d(0.0, 0.0));
    mNodes_local.push_back(Vector2d(1.5, -0.0038));
    mNodes_local.push_back(Vector2d(3.0, -0.0151));
    mNodes_local.push_back(Vector2d(4.5, -0.0340));
    mNodes_local.push_back(Vector2d(6.0, -0.0604));
    mNodes_local.push_back(Vector2d(7.5, -0.0944));
    mNodes_local.push_back(Vector2d(9.0, -0.1359));
    mNodes_local.push_back(Vector2d(10.5, -0.1850));
    mNodes_local.push_back(Vector2d(12.0, -0.2417));
    mNodes_local.push_back(Vector2d(13.5, -0.3059));
    mNodes_local.push_back(Vector2d(15.3982, -0.6363));
    mNodes_local1.push_back(Vector3d(0.0, 0.0,0));
    mNodes_local1.push_back(Vector3d(1.5, -0.0038,0));
    mNodes_local1.push_back(Vector3d(3.0, -0.0151,0));
    mNodes_local1.push_back(Vector3d(4.5, -0.0340,0));
    mNodes_local1.push_back(Vector3d(6.0, -0.0604,0));
    mNodes_local1.push_back(Vector3d(7.5, -0.0944,0));
    mNodes_local1.push_back(Vector3d(9.0, -0.1359,0));
    mNodes_local1.push_back(Vector3d(10.5, -0.1850,0));
    mNodes_local1.push_back(Vector3d(12.0, -0.2417,0));
    mNodes_local1.push_back(Vector3d(13.5, -0.3059,0));
    mNodes_local1.push_back(Vector3d(15.3982, -0.6363,0));
    switch(type)
    {
        case 0:
            start[0]=mNodes_local.at(0)(0);
            start[1]=mNodes_local.at(0)(1);
            start[2]=RADIANS(0);
            goal[0]=mNodes_local.at(mNodes_local.size()-1)(0);
            goal[1]=mNodes_local.at(mNodes_local.size()-1)(1);
            goal[2]=RADIANS(0);

            //OBSTACLE.push_back(Vector2d(6.0,-0.3604));
            
            for(double x=5; x<6; x+=0.2 )
                for(double y=-5; y<-0.1; y+=0.1)
                    OBSTACLE.push_back(Vector2d(x,y));

            break;

        case 1:
            start[0]=0.0;
            start[1]=0.0;
            start[2]=RADIANS(0);
            goal[0]=50.0;
            goal[1]=50.0;
            goal[2]=RADIANS(0);

            for(double x=15; x<35; x+=0.1)
                for(double y=30; y<59.6; y+=0.1)
                    OBSTACLE.push_back(Vector2d(x,y));
            for(double x=15; x<35; x+=0.1)
                for(double y=0.1; y<25; y+=0.1)
                    OBSTACLE.push_back(Vector2d(x,y));
            break;

        default:
            break;
    }

    g_pTree  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int k=0; k<OBSTACLE.size(); k++)
    {
        g_pTree->push_back(pcl::PointXYZ(OBSTACLE[k](0), OBSTACLE[k](1), 0.0));
    }
    g_kdTree.setInputCloud(g_pTree);
}
double EbandManager::FixDouble(double a, int n)
{
    return ((int)(a*pow(10.0,n)))/pow(10,n);
}
bool EbandManager::ComputeNewtonRaphton()
{
    ///////////////////////////////////////////////////////////
    /// properties
    double ki = 10000.0;
    double ki1 =1000.0;
    double kobs = 10.0;
    //double kobs = 0.0;
    double li1 = 1.0;

    double tEpsilon = 0.002;
    bool ret = false;
    int iter =0;
    double epsilon = 9999999;
    while(iter <1000)
    {

        int k=0;
        for( int i=1; i<mNodes_local.size()-1 ; i++)
        {
            double li_m = (mNodes_local1.at(i) - mNodes_local1.at(i-1)).norm()-0.1;
            double li_p = (mNodes_local1.at(i) - mNodes_local1.at(i+1)).norm()-0.1;
            k = i-1;
            
            MatrixXd F_int(2,1);
            MatrixXd J_int(2,2);
            MatrixXd _J_int(2,2);
            MatrixXd J_int_(2,2);
            F_int=MatrixXd::Zero(2,1);
            J_int=MatrixXd::Zero(2,2);
            _J_int=MatrixXd::Zero(2,2);
            J_int_=MatrixXd::Zero(2,2);
            // internal springs
           ComputeIntForce(mNodes_local.at(i),mNodes_local.at(i+1),mNodes_local.at(i-1),ki,li_m,li_p,F_int);
           ComputeIntJacobian(mNodes_local.at(i),mNodes_local.at(i+1),mNodes_local.at(i-1),ki,li_m,li_p,J_int,_J_int,J_int_);

            //F_int *= -1.0;
            //J_int *= -1.0;
            //_J_int *= -1.0;
            //J_int_ *= -1.0;
            //cout <<"J :"<<J_int<<endl;
           ////////////////////////////////////////////////////////// 
            MatrixXd F1_int(2,1);
            MatrixXd J1_int(2,2);
            MatrixXd _J1_int(2,2);
            MatrixXd J1_int_(2,2);
            F1_int=MatrixXd::Zero(2,1);
            J1_int=MatrixXd::Zero(2,2);
            _J1_int=MatrixXd::Zero(2,2);
            J1_int_=MatrixXd::Zero(2,2);
        
           ////////////////////////////////////////////////////////// 

            double deg=90.0;
            double yaw = mNodes_local1.at(i)(2)+RADIANS(deg);
            
            Matrix2d Rot(2,2);
            Rot << cos(yaw), -sin(yaw),
                sin(yaw),  cos(yaw);

            Vector2d vec1 =  Vector2d(mNodes_local1.at(i)(0),mNodes_local1.at(i)(1))+Rot*Vector2d(li1+0.2,0); // Cm_ --> (2,1)
            //Vector2d vec1 =  Vector2d(mNodes_local1.at(i)(0),mNodes_local1.at(i)(1)); // Cm_ --> (2,1)
            
            yaw = mNodes_local1.at(i)(2)-RADIANS(deg);
            
            Rot << cos(yaw), -sin(yaw),
                sin(yaw),  cos(yaw);

            Vector2d vec2 =  Vector2d(mNodes_local1.at(i)(0),mNodes_local1.at(i)(1))+Rot*Vector2d(li1+0.2,0); // Cm_ --> (2,1)
            //Vector2d vec2 =  Vector2d(mNodes_local1.at(i)(0),mNodes_local1.at(i)(1)); // Cm_ --> (2,1)

            //cout <<vec1(0) <<"\t"<< vec1(1) <<"\t"<< mNodes_local1.at(i)(0) <<"\t"<<mNodes_local1.at(i)(1) <<"\t"<<vec2(0)<<"\t"<<vec2(1)<<endl;
            ComputeIntForce(mNodes_local.at(i),vec1,vec2,ki1,li1,li1,F1_int);
            ComputeIntJacobian(mNodes_local.at(i),vec1,vec2,ki1,li1,li1,J1_int,_J1_int,J1_int_);
            
//            F1_int *= -1.0;
//            J1_int *= -1.0;
//            _J1_int *= -1.0;
//            J1_int_ *= -1.0;
            ////////////////////////////////////////////////////////// 
           
//            cout <<"F "<< F1_int(0) << " " << F1_int(1)<<endl;
            std::vector<pcl::PointXYZ> obs = SearchNodeByRadius(pcl::PointXYZ(mNodes_local.at(i)(0),mNodes_local.at(i)(1),0),2.0);

            MatrixXd F_obs(2,1);
            MatrixXd J_obs(2,2);

            F_obs=MatrixXd::Zero(2,1);
            J_obs=MatrixXd::Zero(2,2);
          
            for( int j=0; j<obs.size(); j++)
            {
                Vector2d vObs(obs.at(j).x, obs.at(j).y);
                if( (vObs - mNodes_local.at(i)).norm() < 0.001 )
                {
                    Vector2d tmpVec=Vector2d(mNodes_local1.at(i)(0),mNodes_local1.at(i)(1));
                    vObs += (tmpVec-mNodes_local.at(i))/ (tmpVec-mNodes_local.at(i)).norm()*0.1;
                    
                }
                ComputeObsForce(mNodes_local.at(i),vObs,kobs,2.0,F_obs);
                ComputeObsJacobian(mNodes_local.at(i),vObs,kobs,2.0,J_obs);
            }
            //F_obs *= -1.0;
            //J_obs *= -1.0;
            
//            cout << F_obs << endl;
//            cout << J_obs << endl;
            if( k > 0 )
            {
                mJ(k*2+0,k*2-2) = _J_int(0,0);
                mJ(k*2+0,k*2-1) = _J_int(0,1);
                mJ(k*2+1,k*2-2) = _J_int(1,0);
                mJ(k*2+1,k*2-1) = _J_int(1,1);
            }
            mJ(k*2+0,k*2+0) = J_int(0,0)+J1_int(0,0)+J_obs(0,0);
            mJ(k*2+0,k*2+1) = J_int(0,1)+J1_int(0,1)+J_obs(0,1);
            mJ(k*2+1,k*2+0) = J_int(1,0)+J1_int(1,0)+J_obs(1,0);
            mJ(k*2+1,k*2+1) = J_int(1,1)+J1_int(1,1)+J_obs(1,1);

            if( k < mNodes_local.size()-3 )
            {
                mJ(k*2+0,k*2+2) = J_int_(0,0);
                mJ(k*2+0,k*2+3) = J_int_(0,1);
                mJ(k*2+1,k*2+2) = J_int_(1,0);
                mJ(k*2+1,k*2+3) = J_int_(1,1);
            }
            mF(k*2+0,0) = F1_int(0,0)+F_int(0,0)+F_obs(0,0);
            mF(k*2+1,0) = F1_int(1,0)+F_int(1,0)+F_obs(1,0);

        }
        MatrixXd mm = mJ.inverse() * mF;
        //cout << mJ <<endl;
        //cout <<"tmp "<<mm.rows()<<" "<< mm.cols()<<endl; 
        //cout <<"tmp "<<mdX_pre.rows()<<" "<< mdX_pre.cols()<<endl;
//        cout <<mm<<endl;
        MatrixXd dX = mdX_pre - mm;

        mdX_pre = dX;

        for( int i=1; i<mNodes_local.size()-1; i++)
        {
          //  cout <<dX(2*(i-1))<<" "<<dX(2*(i-1)+1)<<endl;

            mNodes_local.at(i)(0) = dX(2*(i-1));
            mNodes_local.at(i)(1) = dX(2*(i-1)+1);
        }

        epsilon = mm.norm();
        if( epsilon < tEpsilon )
        {
            ret = true;
            cout <<iter<<" F, J computed: "<< epsilon<<endl;
            break;
        }
        iter++;
    }
    cout <<iter<<" computedX "<< epsilon<<endl;
    return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Internal Springs
void EbandManager::ComputeIntForce(Vector2d node, Vector2d node_, Vector2d _node, double k, double li_m,double li_p,MatrixXd& F)
{
    double ri_x=        node.x();
    double ri_y=        node.y();
    double rip1_y=      node_.y();
    double rip1_x=      node_.x();
    double rim1_y=      _node.y();
    double rim1_x=      _node.x();
    double ki_m=          k;
    double ki_p=          k;

    F(0,0) = ki_m*(ri_x*2.0-rim1_x*2.0)*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/2.0)+ki_p*(ri_x*2.0-rip1_x*2.0)*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/2.0);
      F(1,0) = ki_m*(ri_y*2.0-rim1_y*2.0)*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/2.0)+ki_p*(ri_y*2.0-rip1_y*2.0)*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/2.0);

 /*   F(0,0) = FixDouble(ki*(ri_x*2.0-rim1_x*2.0)*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/2.0)+ki*(ri_x*2.0-rip1_x*2.0)*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/2.0),4);
    F(1,0) = FixDouble(ki*(ri_y*2.0-rim1_y*2.0)*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/2.0)+ki*(ri_y*2.0-rip1_y*2.0)*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/2.0),4);
*/
    //cout <<"posX " <<ri_x <<" "<< rip1_x <<" "<<rim1_x <<" "<<ri_x-rip1_x<< " F:  "<<F(0,0)<<" "<<F(1,0)<<endl;
    //cout <<"posY " <<ri_y <<" "<< rip1_y <<" "<<rim1_y <<" "<<ri_y-rip1_y<< " F:  "<<F(0,0)<<" "<<F(1,0)<<"k " <<k <<" l "<<l_init<<endl;

}

void EbandManager::ComputeIntJacobian(Vector2d node, Vector2d node_, Vector2d _node, double k, double li_m,double li_p,MatrixXd& J,MatrixXd& _J,MatrixXd& J_)
{
    double ri_x=        node.x();
    double ri_y=        node.y();
    double rip1_y=      node_.y();
    double rip1_x=      node_.x();
    double rim1_y=      _node.y();
    double rim1_x=      _node.x();
    double ki_m=          k;
    double ki_p=          k;

     J(0,0) = ki_m*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))+ki_p*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))-(ki_m*pow(ri_x*2.0-rim1_x*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))-(ki_p*pow(ri_x*2.0-rip1_x*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))-ki_m*pow(ri_x*2.0-rim1_x*2.0,2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0)-ki_p*pow(ri_x*2.0-rip1_x*2.0,2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);
       J(0,1) = (ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*(-1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))-(ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))-ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0)-ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);
         J(1,0) = (ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*(-1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))-(ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))-ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0)-ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);
           J(1,1) = ki_m*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))+ki_p*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))-(ki_m*pow(ri_y*2.0-rim1_y*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))-(ki_p*pow(ri_y*2.0-rip1_y*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))-ki_m*pow(ri_y*2.0-rim1_y*2.0,2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0)-ki_p*pow(ri_y*2.0-rip1_y*2.0,2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);

  _J(0,0) = -ki_m*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))+(ki_m*pow(ri_x*2.0-rim1_x*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))+ki_m*pow(ri_x*2.0-rim1_x*2.0,2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0);
     _J(0,1) = (ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*(1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))+ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0);
     _J(1,0) = (ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*(1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))+ki_m*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0);
        _J(1,1) = -ki_m*1.0/sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))+(ki_m*pow(ri_y*2.0-rim1_y*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))+ki_m*pow(ri_y*2.0-rim1_y*2.0,2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li_m-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0);

       J_(0,0) = -ki_p*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))+(ki_p*pow(ri_x*2.0-rip1_x*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))+ki_p*pow(ri_x*2.0-rip1_x*2.0,2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);
         J_(0,1) = (ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))+ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);
           J_(1,0) = (ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))+ki_p*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);
             J_(1,1) = -ki_p*1.0/sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))+(ki_p*pow(ri_y*2.0-rip1_y*2.0,2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))+ki_p*pow(ri_y*2.0-rip1_y*2.0,2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li_p-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0);

    /*
    J(0,0) = FixDouble(ki*-2.0+ki*li*(ri_y*ri_y)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)+ki*li*(ri_y*ri_y)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)+ki*li*(rim1_y*rim1_y)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)+ki*li*(rip1_y*rip1_y)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)-ki*li*ri_y*rim1_y*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*2.0-ki*li*ri_y*rip1_y*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*2.0,4);
    J(0,1) = FixDouble((ki*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*(-1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))-(ki*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))-ki*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0)-ki*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0),4);
    J(1,0) =FixDouble( (ki*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*(-1.0/4.0))/(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0))-(ki*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*(1.0/4.0))/(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0))-ki*(ri_x*2.0-rim1_x*2.0)*(ri_y*2.0-rim1_y*2.0)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li-sqrt(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0)))*(1.0/4.0)-ki*(ri_x*2.0-rip1_x*2.0)*(ri_y*2.0-rip1_y*2.0)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li-sqrt(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0)))*(1.0/4.0),4);
    J(1,1) = FixDouble(ki*-2.0+ki*li*(ri_x*ri_x)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)+ki*li*(ri_x*ri_x)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)+ki*li*(rim1_x*rim1_x)*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)+ki*li*(rip1_x*rip1_x)*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)-ki*li*ri_x*rim1_x*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*2.0-ki*li*ri_x*rip1_x*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*2.0,4);


    _J(0,0) = FixDouble(-ki*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li*(ri_y*ri_y)+li*(rim1_y*rim1_y)-pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)-li*ri_y*rim1_y*2.0),4);
    _J(0,1) = FixDouble(ki*li*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(ri_x-rim1_x)*(ri_y-rim1_y),4);
    _J(1,0) = FixDouble(ki*li*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(ri_x-rim1_x)*(ri_y-rim1_y),4);
    _J(1,1) = FixDouble(-ki*1.0/pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)*(li*(ri_x*ri_x)+li*(rim1_x*rim1_x)-pow(pow(ri_x-rim1_x,2.0)+pow(ri_y-rim1_y,2.0),3.0/2.0)-li*ri_x*rim1_x*2.0),4);


    J_(0,0) = FixDouble(-ki*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li*(ri_y*ri_y)+li*(rip1_y*rip1_y)-pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)-li*ri_y*rip1_y*2.0),4);
    J_(0,1) = FixDouble(ki*li*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(ri_x-rip1_x)*(ri_y-rip1_y),4);
    J_(1,0) = FixDouble(ki*li*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(ri_x-rip1_x)*(ri_y-rip1_y),4);
    J_(1,1) = FixDouble(-ki*1.0/pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)*(li*(ri_x*ri_x)+li*(rip1_x*rip1_x)-pow(pow(ri_x-rip1_x,2.0)+pow(ri_y-rip1_y,2.0),3.0/2.0)-li*ri_x*rip1_x*2.0),4);
*/
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Obstacle Springs

void EbandManager::ComputeObsForce(Vector2d node, Vector2d obstacle, double k, double r, MatrixXd &F)
{
    double ri_x=        node.x();
    double ri_y=        node.y();
    double obs_width = 0.3;
    double car_width = 2.0;
    double r_obs_x=     obstacle.x();
    double r_obs_y=     obstacle.y();
    double kobs=        k;

    if( (node - obstacle).norm() < r )
    {
/*        F(0,0) += FixDouble(-kobs*1.0/sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))*(r_obs_x-ri_x)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))),4);
        F(1,0) += FixDouble(-kobs*1.0/sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))*(r_obs_y-ri_y)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))),4);
        */
        F(0,0) += kobs*(r_obs_x*2.0-ri_x*2.0)*1.0/sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))*(-1.0/2.0);
        F(1,0) += kobs*(r_obs_y*2.0-ri_y*2.0)*1.0/sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))*(-1.0/2.0);

    }
    else
    {
        F(0,0) += 0.0;
        F(1,0) += 0.0;
    }
}

void EbandManager::ComputeObsJacobian(Vector2d node, Vector2d obstacle, double k, double r, MatrixXd& J)
{
    double ri_x=        node.x();
    double ri_y=        node.y();
    double obs_width = 0.3;
    double car_width = 2.0;
    //double r_safe=      r;
    double r_obs_x=     obstacle.x();
    double r_obs_y=     obstacle.y();
    double kobs=        k;

    //cout << "ComputeObsJ: "<<J(0,0) <<" "<<J(0,1)<<" "<<J(1,0)<<" "<<J(1,1)<<endl;


    J(0,0) += kobs*1.0/sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))-(kobs*pow(r_obs_x*2.0-ri_x*2.0,2.0)*(1.0/4.0))/(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))-kobs*pow(r_obs_x*2.0-ri_x*2.0,2.0)*1.0/pow(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0),3.0/2.0)*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))*(1.0/4.0);
    J(0,1) += (kobs*(r_obs_x*2.0-ri_x*2.0)*(r_obs_y*2.0-ri_y*2.0)*(-1.0/4.0))/(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))-kobs*(r_obs_x*2.0-ri_x*2.0)*(r_obs_y*2.0-ri_y*2.0)*1.0/pow(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0),3.0/2.0)*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))*(1.0/4.0);
    J(1,0) += (kobs*(r_obs_x*2.0-ri_x*2.0)*(r_obs_y*2.0-ri_y*2.0)*(-1.0/4.0))/(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))-kobs*(r_obs_x*2.0-ri_x*2.0)*(r_obs_y*2.0-ri_y*2.0)*1.0/pow(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0),3.0/2.0)*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))*(1.0/4.0);
    J(1,1) += kobs*1.0/sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))-(kobs*pow(r_obs_y*2.0-ri_y*2.0,2.0)*(1.0/4.0))/(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0))-kobs*pow(r_obs_y*2.0-ri_y*2.0,2.0)*1.0/pow(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0),3.0/2.0)*(car_width+obs_width-sqrt(pow(r_obs_x-ri_x,2.0)+pow(r_obs_y-ri_y,2.0)))*(1.0/4.0);

    /*
    double signKx = 0.0;
    if( r_obs_x > ri_x )
    {
        signKx = 1.0;
    }
    else if( r_obs_x < ri_x )
    {
        signKx = -1.0;
    }

    double signKy = 0.0;
    if( r_obs_y > ri_y )
    {
        signKy = 1.0;
    }
    else if( r_obs_y < ri_y )
    {
        signKy = -1.0;
    }
    J(0,0) += kobs*1.0/sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)))-(kobs*fabs(r_obs_x-ri_x)*(((r_obs_x-ri_x)/fabs(r_obs_x-ri_x)))*(r_obs_x-ri_x))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*fabs(r_obs_x-ri_x)*(((r_obs_x-ri_x)/fabs(r_obs_x-ri_x)))*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_x-ri_x)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)));
    J(0,1) += -(kobs*fabs(r_obs_x-ri_x)*(((r_obs_x-ri_x)/fabs(r_obs_x-ri_x)))*(r_obs_y-ri_y))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*fabs(r_obs_x-ri_x)*(((r_obs_x-ri_x)/fabs(r_obs_x-ri_x)))*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_y-ri_y)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)));
    J(1,0) += -(kobs*fabs(r_obs_y-ri_y)*(((r_obs_y-ri_y)/fabs(r_obs_y-ri_y)))*(r_obs_x-ri_x))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*fabs(r_obs_y-ri_y)*(((r_obs_y-ri_y)/fabs(r_obs_y-ri_y)))*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_x-ri_x)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)));
    J(1,1) += kobs*1.0/sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)))-(kobs*fabs(r_obs_y-ri_y)*(((r_obs_y-ri_y)/fabs(r_obs_y-ri_y)))*(r_obs_y-ri_y))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*fabs(r_obs_y-ri_y)*(((r_obs_y-ri_y)/fabs(r_obs_y-ri_y)))*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_y-ri_y)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)));

    cout << kobs*1.0/sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))<<" "<<(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)))<<" "<<-(kobs*fabs(r_obs_x-ri_x)*(((r_obs_x-ri_x)/fabs(r_obs_x-ri_x)))*(r_obs_x-ri_x))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))<<" "<<-kobs*fabs(r_obs_x-ri_x)*(((r_obs_x-ri_x)/fabs(r_obs_x-ri_x)))*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)<<" "<<(r_obs_x-ri_x)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)));
 
    J(0,0) +=FixDouble(kobs*1.0/sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)))-(kobs*signKx*fabs(r_obs_x-ri_x)*(r_obs_x-ri_x))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*signKx*fabs(r_obs_x-ri_x)*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_x-ri_x)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))),4);
    J(0,1) +=FixDouble( -(kobs*signKx*fabs(r_obs_x-ri_x)*(r_obs_y-ri_y))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*signKx*fabs(r_obs_x-ri_x)*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_y-ri_y)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))),4);
    J(1,0) += FixDouble(-(kobs*signKy*fabs(r_obs_y-ri_y)*(r_obs_x-ri_x))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*signKy*fabs(r_obs_y-ri_y)*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_x-ri_x)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))),4);
    J(1,1) += FixDouble(kobs*1.0/sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0)))-(kobs*signKy*fabs(r_obs_y-ri_y)*(r_obs_y-ri_y))/(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))-kobs*signKy*fabs(r_obs_y-ri_y)*1.0/pow(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0),3.0/2.0)*(r_obs_y-ri_y)*(r_safe-sqrt(pow(fabs(r_obs_x-ri_x),2.0)+pow(fabs(r_obs_y-ri_y),2.0))),4);
*/
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// KAMM Constraints

void EbandManager::ComputeKammForce(Vector2d node, Vector2d kammNode, double k, MatrixXd &F)
{
    F(0,0) = k * ( kammNode.x() - node.x());
    F(1,0) = k * ( kammNode.y() - node.y());
}

void EbandManager::ComputeKammJacobian(Vector2d node, Vector2d kammNode, double k,MatrixXd& J)
{
    J(0,0) = k;
    J(0,1) = 0.0;
    J(1,0) = 0.0;
    J(1,1) = k;
}
