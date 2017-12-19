#include <iostream>
//#include "../inc/ParkingPlannerThread.h"
#include "../inc/VectorPursuit.h"
#include <Eigen/Core>
#include <GeometricUtils.h>
using namespace Eigen;
using namespace std;

void GetLookAheadPt(Vector3d targetPos, Vector3d vehiclePos, double dLookAheadDist, double dir, Vector3d& lookAheadPos)
{
	
}
 
void magneticVectorForce(double& start[3], double& goal[3], double& target[3])
{
	double ln = 40.0; 	// Length of conductors
	double d = 1.61/2.0;		// Distance between the conductors m & n
	int m = 1; 		// Direction of current through the conductor, m(Right) 1(INTO), -1(OUT)
	int n = -1; 		// Direction of current through the conductor, n(Left) 1(INTO), -1(OUT)
	int N = 20; 		// Number sections/elements in the conductors
	int dl = ln/N; 		// Length of each element

	MatrixXd Rot(2,2);
	Rot << cos(start[0]), -sin(start[0]),
	       sin(start[0]),  cos(start[0]);

    MatrixXd Cm_ = Rot*Vector2d(d/2.0;)
/*
	MatrixXd xCm = (d/2.0)*MatrixXd::Ones(1,N); 
	MatrixXd xCn = (-d/2.0)*MatrixXd::Ones(1,N);
	
	// Y Coordinate of each element from origin, half on +Y & other half on -Y and also sam for both conductors
	VectorXd yC = VectorXd::LinSpaced(N, -ln/2.0+dl/2.0, ln/2.0-dl/2.0);

	// zC remains 0 throughout the length, as conductors are lying on XY plane
	MatrixXd zC = MatrixXd::Zero(1,N);
	
	// Length(Projection) 7 Direction of each current element in Vector form
	MatrixXd Lx = MatrixXd::Zero(1,N);	// Length of each element is zero on X axis
	MatrixXd Ly = dl*MatrixXd::Ones(1,N);	// Length of each element is dl on Y axis
	MatrixXd Lz = MatrixXd::Zero(1,N);	// Length of each element is zero on Z axis
		
	// Points/Locations in space (here XZ plane) where B is to be computed
	int NP = 96;	//Detector points..
	int xPmax = 12;	//Dimensions of detector space.., arbitrary..
	int zPmax = 12;

	MatrixXd resVec_x = MatrixXd::Zero(NP,NP);
	MatrixXd resVec_y = MatrixXd::Zero(NP,NP);
	MatrixXd resPos_x = MatrixXd::Zero(NP,NP);
	MatrixXd resPos_y = MatrixXd::Zero(NP,NP);
	
	//Divide space with NP points..
	RowVectorXd xP = RowVectorXd::LinSpaced(NP, -xPmax, xPmax);	
	VectorXd zP = VectorXd::LinSpaced(NP, -zPmax, zPmax);

	// Creating the Mesh..
	MatrixXd xxP = xP.replicate(NP,1); 
	MatrixXd zzP = zP.replicate(1,NP);

	//Initialize B..
	MatrixXd Bx = MatrixXd::Zero(NP,NP);
	MatrixXd By = MatrixXd::Zero(NP,NP);
	MatrixXd Bz = MatrixXd::Zero(NP,NP);

	// Computation of Magnetic Field (B) using Superposition principle..
	// Compute B at each detector points due to each small cond elements & integrate them..
	for( int q=0; q<N; q++ )
	{
		MatrixXd rxm = xxP.array() - xCm(0,q); // Displacement Vector along X direction, from cond m..
		MatrixXd rxn = xxP.array() - xCn(0,q); // Displacement Vector along X direction, from cond n..
		
		double ry = yC(q,0);	// Same for m & n, no detector points on Y direction..
		
		MatrixXd rz = zzP.array() - zC(0,q); // Same for m & n..
		
		MatrixXd rm = (rxm.array()*rxm.array()+ry*ry+rz.array()*rz.array()).sqrt();	// Displacement Magnitude for an element on cond m..
		MatrixXd rn = (rxn.array()*rxn.array()+ry*ry+rz.array()*rz.array()).sqrt();	// Displacement Magnitude for an element on cond n..
		
		MatrixXd r3m = rm.array()*rm.array()*rm.array();
		MatrixXd r3n = rn.array()*rn.array()*rn.array();
		
		Bx = Bx.array() + m*Ly(0,q)*rz.array()/r3m.array() + n*Ly(0,q)*rz.array()/r3n.array();	// m & n, direction of current element..
		// By = 0;
		Bz = Bz.array() - m*Ly(0,q)*rxm.array()/r3m.array() - n*Ly(0,q)*rxn.array()/r3n.array();
	}
		
	MatrixXd Rot(2,2);
	Rot << cos(_theta), -sin(_theta),
	       sin(_theta),  cos(_theta);

	MatrixXd tmpMat(2, NP);
	for(int i=0; i<NP; i++)
	{
		//tmpMat << Bx.row(i)+xxP.row(i),
		//		Bz.row(i)+zzP.row(i);
		tmpMat << Bx.row(i),
				Bz.row(i);
		tmpMat = Rot * tmpMat;
		

		resVec_x.block(i,0,1,NP)  = tmpMat.block(0,0,1,NP).array();
		resVec_y.block(i,0,1,NP)  = tmpMat.block(1,0,1,NP).array();
		
		tmpMat << xxP.row(i),
				zzP.row(i);
		tmpMat = Rot * tmpMat;
		
		// Block Operation
		resPos_x.block(i,0,1,NP)  = (tmpMat.block(0,0,1,NP)).array()+_x;
		resPos_y.block(i,0,1,NP)  = (tmpMat.block(1,0,1,NP)).array()+_y;
	}
	///////////////////////////////////////////////////////////////////
	geometry_msgs::PoseArray poseArray;
	
    double grid_offset=grid_dim_/2.0*m_per_cell_;
    double K_rep_mag = 100.0f; 
    
	for(int i=0; i<NP; i++)
	{
		for( int j=0; j<NP; j++)
		{
			int i_ = int((resPos_x(i,j) + grid_offset - m_per_cell_/2.0) / m_per_cell_);
			int j_ = int((resPos_y(i,j) + grid_offset - m_per_cell_/2.0) / m_per_cell_);
			
			if( i_ >= 0 && j_ >= 0 && i_ < NP && j_ < NP )
			{
				PotentialMap[i_][j_].bInit = true;
				
				double vSize = sqrtf(resVec_x(i,j)*resVec_x(i,j) + resVec_y(i,j)*resVec_y(i,j));
				double vDist = sqrtf((_x-resPos_x(i,j))*(_x-resPos_x(i,j)) + (_y-resPos_y(i,j))*(_y-resPos_y(i,j)));
				if( vDist < 0.1 ) vDist = 0.1;
				PotentialMap[i_][j_].F_sum.x += K_rep_mag*resVec_x(i,j)/vSize/vDist;
				PotentialMap[i_][j_].F_sum.y += K_rep_mag*resVec_y(i,j)/vSize/vDist;
			}
		}
	}
    */
}
int main(int argc, char** argv)
{
    double start[3]={85.0, -8.0, 3.14};
    double goal[3]={77.36, -1.56, 2.21};
    double target[3]={74.852,0.0876};

    magneticVectorForce(start, goal, target);
    return 0;
}
/*
{
	Vector3d initPos(55.0, -8.0, 3.1416); 
	Vector3d goalPos(30.7491, 19.9583, 3.1416); 
	double fLookAheadDist=7.0; 
	double v_d=1.8; 
	double T=100;
	
	VectorPursuit vectorPursuit(initPos, goalPos, fLookAheadDist, v_d, T);
	vector<Edge*> edges = vectorPursuit.GetEdges();
	
	for(int i=0; i<edges.size(); i++)
	{
		cout << edges[i]->x << " "<<edges[i]->y << " " <<edges[i]->h << endl;
	}
	return 0;
}

{
	double _theta = 30.0 * 3.1415 / 180.0;
	
	double ln = 100.0; 	// Length of conductors
	double d = 2.58;		// Distance between the conductors m & n
	int m = 1; 		// Direction of current through the conductor, m(Right) 1(INTO), -1(OUT)
	int n = -1; 		// Direction of current through the conductor, n(Left) 1(INTO), -1(OUT)
	int N = 20; 		// Number sections/elements in the conductors
	int dl = ln/N; 		// Length of each element
	
	// XYZ coordinates/Location of each element from the origin (0,0,0), i.e 'd/2' is taken as origin..
	MatrixXd xCm = (d/2.0)*MatrixXd::Ones(1,N); 
	MatrixXd xCn = (-d/2.0)*MatrixXd::Ones(1,N);
	
	// Y Coordinate of each element from origin, half on +Y & other half on -Y and also sam for both conductors
	VectorXd yC = VectorXd::LinSpaced(N, -ln/2.0+dl/2.0, ln/2.0-dl/2.0);

	// zC remains 0 throughout the length, as conductors are lying on XY plane
	MatrixXd zC = MatrixXd::Zero(1,N);
	
	// Length(Projection) 7 Direction of each current element in Vector form
	MatrixXd Lx = MatrixXd::Zero(1,N);	// Length of each element is zero on X axis
	MatrixXd Ly = dl*MatrixXd::Ones(1,N);	// Length of each element is dl on Y axis
	MatrixXd Lz = MatrixXd::Zero(1,N);	// Length of each element is zero on Z axis
		
	// Points/Locations in space (here XZ plane) where B is to be computed
	int NP = 64;	//Detector points..
	int xPmax = 15;	//Dimensions of detector space.., arbitrary..
	int zPmax = 15;

	//Divide space with NP points..
	RowVectorXd xP = RowVectorXd::LinSpaced(NP, -xPmax, xPmax);	
	VectorXd zP = VectorXd::LinSpaced(NP, -zPmax, zPmax);

	// Creating the Mesh..
	MatrixXd xxP = xP.replicate(NP,1); 
	MatrixXd zzP = zP.replicate(1,NP);

	//Initialize B..
	MatrixXd Bx = MatrixXd::Zero(NP,NP);
	MatrixXd By = MatrixXd::Zero(NP,NP);
	MatrixXd Bz = MatrixXd::Zero(NP,NP);

	// Computation of Magnetic Field (B) using Superposition principle..
	// Compute B at each detector points due to each small cond elements & integrate them..
	for( int q=0; q<N; q++ )
	{
		MatrixXd rxm = xxP.array() - xCm(0,q); // Displacement Vector along X direction, from cond m..
		MatrixXd rxn = xxP.array() - xCn(0,q); // Displacement Vector along X direction, from cond n..
		
		double ry = yC(q,0);	// Same for m & n, no detector points on Y direction..
		
		MatrixXd rz = zzP.array() - zC(0,q); // Same for m & n..
		
		MatrixXd rm = (rxm.array()*rxm.array()+ry*ry+rz.array()*rz.array()).sqrt();	// Displacement Magnitude for an element on cond m..
		MatrixXd rn = (rxn.array()*rxn.array()+ry*ry+rz.array()*rz.array()).sqrt();	// Displacement Magnitude for an element on cond n..
		
		MatrixXd r3m = rm.array()*rm.array()*rm.array();
		MatrixXd r3n = rn.array()*rn.array()*rn.array();
	
		Bx = Bx.array() + m*Ly(0,q)*rz.array()/r3m.array() + n*Ly(0,q)*rz.array()/r3n.array();	// m & n, direction of current element..
		// By = 0;
		Bz = Bz.array() - m*Ly(0,q)*rxm.array()/r3m.array() - n*Ly(0,q)*rxn.array()/r3n.array();
	}
	
	MatrixXd Rot(2,2);
	Rot << cos(_theta), -sin(_theta),
	       sin(_theta),  cos(_theta);

	MatrixXd resVec = MatrixXd::Zero(NP*2,NP);
	MatrixXd resPos = MatrixXd::Zero(NP*2,NP);
	
	MatrixXd tmpMat(2, NP);
	
	for(int i=0; i<NP; i++)
	{
		tmpMat << Bx.row(i)+xxP.row(i),
				Bz.row(i)+zzP.row(i);
		tmpMat = Rot * tmpMat;
		
		// Block Operation
		resVec.block(i*2,0,2,NP)  = tmpMat;
		
		tmpMat << xxP.row(i),
				zzP.row(i);
		tmpMat = Rot * tmpMat;
		
		// Block Operation
		resPos.block(i*2,0,2,NP)  = tmpMat;
	}
	cout << resPos << endl;
	return 0;
}
*/
// Matlab --> C++ (MeshGrid)
//X = RowVectorXd::LinSpaced(1,3,3).replicate(5,1);
//Y = VectorXd::LinSpaced(10,14,5).replicate(1,3);
/*
repmat

e.g., dst = src.replicate(n,m);

see http://eigen.tuxfamily.org/dox-devel/cl ... 4e4b103032

meshgrid

you can mix LinSpaced and replicate:

X = RowVectorXd::LinSpaced(1,3,3).replicate(5,1);
Y = VectorXd::LinSpaced(10,14,5).replicate(1,3);

see: http://eigen.tuxfamily.org/dox-devel/cl ... a9c6ae34d8

reshape

Currently you can mimic it using Map:

B = Map<MatrixXd>(A.data(),m,n); 
*/
