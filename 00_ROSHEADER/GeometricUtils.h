/*
 * Copyright 2016 DYROS Dynamic Robotic Systems Lab, Seoul National University. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 * 
 *        * Redistributions in binary form must reproduce the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided
 *          with the distribution.
 * 
 *        * Neither the name of the DYROS Dynamic Robotic Systems Lab, Seoul National University.     
 *          nor the names of its contributors may be used to endorse or promote products
 *          derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/

#ifndef _GEOMETRICUTILS_H_
#define _GEOMETRICUTILS_H_

#include <Eigen/Core>
#include <MathParam.h>

using namespace Eigen;

namespace GeometricUtils
{
	///////////////////////////////////////////////////
	static void TransLocalToGlobal(double offset_x, double offset_y, double theta, double& _x, double& _y)
	{
	   double x = _x;
           double y = _y;

	    _x = cos(theta)*x - sin(theta)*y + offset_x;
	    _y = sin(theta)*x + cos(theta)*y + offset_y;
	}

	static void TransGlobalToLocal(double offset_x, double offset_y, double theta, double& _x, double& _y)
	{
	    double x_prime = _x-offset_x;
	    double y_prime = _y-offset_y;

	    _x = cos(-theta)*x_prime - sin(-theta)*y_prime;
	    _y = sin(-theta)*x_prime + cos(-theta)*y_prime;
	}

	// 기준좌표계: x, y, theta, 대상좌표: _x, _y
	static void TransRelativeCoord(double ref_x, double ref_y, double ref_h, double _x, double _y, double _h, double& ret_x, double& ret_y, double& ret_h)
	{
	    double x_prime = _x-ref_x;
	    double y_prime = _y-ref_y;

	    ret_x = cos(-ref_h)*x_prime - sin(-ref_h)*y_prime;
	    ret_y = sin(-ref_h)*x_prime + cos(-ref_h)*y_prime;
	    ret_h = _h - ref_h;
	}

	
	///////////////////////////////////////////////////
	// return value: distance between line and point
	static double CloestPointOnLine(Vector2d LinePt1, Vector2d LinePt2, Vector2d pt, Vector2d& ret )
	{
		double diffX = LinePt2[0] - LinePt1[0];
		double diffY = LinePt2[1] - LinePt1[1];
		if ((diffX == 0) && (diffY == 0))
		{
			diffX = pt[0] - LinePt1[0];
			diffY = pt[1] - LinePt1[1];
			return sqrt(diffX * diffX + diffY * diffY);
		}

		double t = ((pt[0] - LinePt1[0]) * diffX + (pt[1] - LinePt1[1]) * diffY) / (diffX * diffX + diffY * diffY);
		
		if (t < 0)
		{
			//point is nearest to the first point i.e x1 and y1
			diffX = pt[0] - LinePt1[0];
			diffY = pt[1] - LinePt1[1];
			ret = LinePt1;
		}
		else if (t > 1)
		{
			//point is nearest to the end point i.e x2 and y2
			diffX = pt[0] - LinePt2[0];
			diffY = pt[1] - LinePt2[1];
			ret = LinePt2;
		}
		else
		{
			//if perpendicular line intersect the line segment.
			Vector2d intersectPt(LinePt1[0] + t * diffX, LinePt1[1] + t * diffY);

			diffX = pt[0] - intersectPt[0];
			diffY = pt[1] - intersectPt[1];

			ret = intersectPt;
		}

		//returning shortest distance
		return sqrt(diffX * diffX + diffY * diffY);
	}

	static double CloestPointOnLine(Vector2d LineCenterPt, double theta, double lineHalfLength, Vector2d pt, Vector2d& ret )
	{
		Vector2d LinePt1( LineCenterPt[0] + lineHalfLength*cos(theta)
				, LineCenterPt[1] + lineHalfLength*sin(theta) );

		Vector2d LinePt2( LineCenterPt[0] + lineHalfLength*cos(theta-RADIANS(180.0))
				, LineCenterPt[1] + lineHalfLength*sin(theta-RADIANS(180.0)) );

		return CloestPointOnLine(LinePt1, LinePt2, pt, ret);
	}
}
#endif
