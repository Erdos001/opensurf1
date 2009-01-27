/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  contact chris.evans@irisys.co.uk for more information.  *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#ifndef INTEGRAL_H
#define INTEGRAL_H

#include "cv.h"

//! Computes the integral image of image img.  Assumes source image to be a 
//! 32-bit floating point.  Returns IplImage in 32-bit float form.
IplImage *Integral(IplImage *img);

//! Computes the sum of pixels within the rectangle specified by the top-left start
//! co-ordinate and size
float BoxIntegral(IplImage *img, int row, int col, int rows, int cols);


#endif