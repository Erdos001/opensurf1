/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  contact chris.evans@irisys.co.uk for more information.  *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#ifndef IPOINT_H
#define IPOINT_H

class Ipoint {

public:

  //! Destructor
  ~Ipoint() {};

  //! Constructor
  Ipoint() : orientation(0) {};

  //! Coordinates of the detected interest point
  float x, y;

  //! Detected scale
  float scale;

  //! Orientation measured anti-clockwise from +ve x-axis
  float orientation;

  //! Sign of laplacian for fast matching purposes
  int laplacian;

  //! Vector of descriptor components
  float descriptor[64];

  //! Placeholds for point motion (can be used for frame to frame motion analysis)
  float dx, dy;

  //! Used to store cluster index
  int clusterIndex;
};



#endif