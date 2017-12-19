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

#ifndef _ANGLEUTILS_H_
#define _ANGLEUTILS_H_

#include <Eigen/Core>
#include <MathParam.h>

using namespace Eigen;

namespace AngleUtils
{
	//signed angle from (1, 0) to v
	static double angle(const Eigen::Vector2d &v) { return atan2(v[1], v[0]); }
	//signed angle from v1 to v2
	static double angle(const Eigen::Vector2d &v1, const Eigen::Vector2d &v2) { return atan2(v1[0] * v2[1] - v1[1] * v2[0], v1.dot(v2)); }

	//These functions bring an angle into the range [0,2Pi] or [rangeStart, rangeStart+2Pi]
	//They assume we're not too far on the negative side of the range
	
	// 0~360	
	static double toRange(double angle) { return fmod(angle + 8 * PI, TWOPI); }
	
	// x ~ 360+x	
	static double toRange(double angle, double rangeStart) { return fmod(angle + 16 * PI - rangeStart, TWOPI) + rangeStart; }

	// -180~180
	static double toRange_PItoPI(double angle)
	{
		double ret = angle;

	 	if( angle  > RADIANS(180.0) )
		{
			ret = angle - RADIANS(360.0);
		}

		else if( angle  < RADIANS(-180.0) )
		{
			ret = angle + RADIANS(360.0);
		}

		return ret;	
	}
}
#endif
