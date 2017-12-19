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

#ifndef _MATHPARAM_H_
#define _MATHPARAM_H_

/* Define constants that are needed in various places */
#ifndef PI
#define PI     3.141592653589793
#endif

#ifndef TWOPI
#define TWOPI  6.283185307179586
#endif

#ifndef HALFPI
#define HALFPI  1.570796326794896
#endif

#ifndef RADIANS
#define RADIANS(theta) (float(theta) / 180.0 * PI)
#endif

#ifndef DEGREES
#define DEGREES(theta) (float(theta) * 180.0 / PI)
#endif

#ifndef cot
#define cot(x) (tan(M_PI_2 - x))
#endif

#ifndef MAX
#define MAX(A, B)     ((A) > (B) ? (A) : (B))
#endif

#ifndef MIN
#define MIN(A, B)     ((A) < (B) ? (A) : (B))
#endif

#ifndef SIGN
#define SIGN(A)             ((A) <  0  ? -1  :  1)
#endif

#ifndef ABSVAL
#define ABSVAL(A)    ((A) <  0  ? -(A): (A))
#endif

#endif
