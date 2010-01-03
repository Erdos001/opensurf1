/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  contact chris.evans@irisys.co.uk for more information.  *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#include "cv.h"
#include "integral.h"
#include "ipoint.h"
#include "utils.h"

#include <vector>

#include "fasthessian.h"

#define FH_DEBUG

using namespace std;

//-------------------------------------------------------

class ResponseLayer
{
public:

  int width, height, step, filter;
  float *responses;
  unsigned char *laplacian;

  ResponseLayer(int width, int height, int step, int filter)
  {
    assert(width > 0 && height > 0);
    
    this->width = width;
    this->height = height;
    this->step = step;
    this->filter = filter;

    responses = new float[width*height];
    laplacian = new unsigned char[width*height];

    memset(responses,0,sizeof(float)*width*height);
    memset(laplacian,0,sizeof(unsigned char)*width*height);
  }

  ~ResponseLayer()
  {
    if (responses) delete [] responses;
    if (laplacian) delete [] laplacian;
  }

  inline unsigned char getLaplacian(unsigned int row, unsigned int column)
  {
    return laplacian[row * width + column];
  }

  inline float getResponse(unsigned int row, unsigned int column)
  {
    return responses[row * width + column];
  }

#ifdef FH_DEBUG
  std::vector<std::pair<int, int>> coords;

  inline std::pair<int,int> getCoords(unsigned int row, unsigned int column)
  {
    return coords[row * width + column];
  }
#endif
};

//-------------------------------------------------------
// pre calculated lobe sizes
static const int lobe_cache [] = {3,5,7,9,13,17,25,33,49,65};
static const int lobe_map [] = {0,1,2,3, 1,3,4,5, 3,5,6,7, 5,7,8,9};

//-------------------------------------------------------

//! Constructor without image
FastHessian::FastHessian(std::vector<Ipoint> &ipts, 
                         const int octaves, const int intervals, const int init_sample, 
                         const float thresh) 
                         : ipts(ipts), i_width(0), i_height(0)
{
  // Save parameter set
  saveParameters(octaves, intervals, init_sample, thresh);
}

//-------------------------------------------------------

//! Constructor with image
FastHessian::FastHessian(IplImage *img, std::vector<Ipoint> &ipts, 
                         const int octaves, const int intervals, const int init_sample, 
                         const float thresh) 
                         : ipts(ipts), i_width(0), i_height(0)
{
  // Save parameter set
  saveParameters(octaves, intervals, init_sample, thresh);

  // Set the current image
  setIntImage(img);
}

//-------------------------------------------------------

FastHessian::~FastHessian()
{
  for (unsigned int i = 0; i < responseMap.size(); ++i)
  {
    if (responseMap[i]) free(responseMap[i]);
  }
}

//-------------------------------------------------------

//! Save the parameters
void FastHessian::saveParameters(const int octaves, const int intervals, 
                                 const int init_sample, const float thresh)
{
  // Initialise variables with bounds-checked values
  this->octaves = 
    (octaves > 0 && octaves <= 4 ? octaves : OCTAVES);
  this->intervals = 
    (intervals > 0 && intervals <= 4 ? intervals : INTERVALS);
  this->init_sample = 
    (init_sample > 0 && init_sample <= 6 ? init_sample : INIT_SAMPLE);
  this->thresh = (thresh >= 0 ? thresh : THRES);
}


//-------------------------------------------------------

//! Set or re-set the integral image source
void FastHessian::setIntImage(IplImage *img)
{
  // Change the source image
  this->img = img;

  i_height = img->height;
  i_width = img->width;
}

//-------------------------------------------------------

//! Find the image features and write into vector of features
void FastHessian::getIpoints()
{
  // Clear the vector of exisiting ipts
  ipts.clear();

  // Build the response map
  buildResponseMap();

  // Get the response layers
  ResponseLayer *bottom, *middle, *top;
  bottom = responseMap.at(0);
  middle = responseMap.at(1);
  top    = responseMap.at(2);

  // loop over middle response layer at density of the most 
  // sparse layer (always top), to find maxima across scale and space
  for (int r = 0; r < top->height; ++r)
  {
    for (int c = 0; c < top->width; ++c)
    {
      if (middle->getResponse(r,c) > thresh && isExtremum(r,c,top,middle,bottom))
      {
        Ipoint ipt;
        ipt.x = static_cast<float>(c * top->step);
        ipt.y = static_cast<float>(r * top->step);
        ipt.scale = static_cast<float>((0.1333f) * middle->filter);
        ipt.laplacian = static_cast<int>(middle->getLaplacian(r,c));
        ipts.push_back(ipt);
      }
    }
  }
}

//-------------------------------------------------------

//! Build map of DoH responses
void FastHessian::buildResponseMap()
{
  // clear any existing response layers
  responseMap.clear();

  // Get image attributes
  int w = (i_width / init_sample);
  int h = (i_height / init_sample);
  int s = (init_sample);

  // Octave 1: 9,  15, 21, 27
  // Octave 2: 15, 27, 39, 51
  // Octave 3: 27, 51, 75, 99
  // Octave 4: 51, 99, 147,195

  // Calculate approximated determinant of hessian values
  //responseMap.push_back(new ResponseLayer(w,   h,   s,   9));
  //responseMap.push_back(new ResponseLayer(w,   h,   s,   15));
  //responseMap.push_back(new ResponseLayer(w,   h,   s,   21));
  responseMap.push_back(new ResponseLayer(w,   h,   s,   27));
  //responseMap.push_back(new ResponseLayer(w/2, h/2, s*2, 39));
  responseMap.push_back(new ResponseLayer(w/2, h/2, s*2, 51));
  responseMap.push_back(new ResponseLayer(w/4, h/4, s*4, 75));
  //responseMap.push_back(new ResponseLayer(w/4, h/4, s*4, 99));
  //responseMap.push_back(new ResponseLayer(w/8, h/8, s*8, 147));
  //responseMap.push_back(new ResponseLayer(w/8, h/8, s*8, 195));
  
  for (unsigned int i = 0; i < responseMap.size(); ++i)
  {
    buildResponseLayer(responseMap[i]);
  }
}

//-------------------------------------------------------

//! Calculate DoH responses for supplied layer
void FastHessian::buildResponseLayer(ResponseLayer *rl)
{
  float *responses = rl->responses;         // response storage
  unsigned char *laplacian = rl->laplacian; // laplacian sign storage
  int step = rl->step;                      // step size for this filter
  int b = (rl->filter - 1) / 2 + 1;         // border for this filter
  int l = rl->filter / 3;                   // lobe for this filter (filter size / 3)
  int w = rl->filter;                       // filter size
  float inverse_area = 1.f/(w*w);           // normalisation factor
  float Dxx, Dyy, Dxy;

  for(int r = 0, index = 0; r < i_height - (i_height % step); r += step) 
  {
    for(int c = 0; c < i_width - (i_width % step); c += step, index++) 
    {
      Dxx = BoxIntegral(img, r - l + 1, c - b, 2*l - 1, w)
          - BoxIntegral(img, r - l + 1, c - l / 2, 2*l - 1, l)*3;
      Dyy = BoxIntegral(img, r - b, c - l + 1, w, 2*l - 1)
          - BoxIntegral(img, r - l / 2, c - l + 1, l, 2*l - 1)*3;
      Dxy = + BoxIntegral(img, r - l, c + 1, l, l)
            + BoxIntegral(img, r + 1, c - l, l, l)
            - BoxIntegral(img, r - l, c - l, l, l)
            - BoxIntegral(img, r + 1, c + 1, l, l);

      // Normalise the filter responses with respect to their size
      Dxx *= inverse_area;
      Dyy *= inverse_area;
      Dxy *= inverse_area;
     
      // Get the determinant of hessian response & laplacian sign
      responses[index] = (Dxx * Dyy - 0.81f * Dxy * Dxy);
      laplacian[index] = (Dxx + Dyy >= 0 ? 1 : 0);

#ifdef FH_DEBUG
      // check index doesn't exceed bound
      assert(index < rl->width * rl->height);

      // create list of the image coords for each response
      rl->coords.push_back(std::make_pair<int,int>(r,c));
#endif
    }
  }
}
  
//-------------------------------------------------------

//! Non Maximal Suppression function
int FastHessian::isExtremum(int r, int c, ResponseLayer *t, ResponseLayer *m, ResponseLayer *b)
{
  // bounds check
  if (r-1 < 0 || r+1 > t->height || c-1 < 0 || c+1 > t->width)
    return 0;

  float candidate = m->getResponse(r, c);
  int scaleT2M = m->width / t->width;
  int scaleT2B = b->width / t->width;

  for (int rr = -1; rr <=1; ++rr)
  {
    for (int cc = -1; cc <=1; ++cc)
    {
#ifdef FH_DEBUG
      // check the responses are being compared from the correct image coords
      std::pair<int,int> tc = t->getCoords(r+rr, c+cc);
      std::pair<int,int> mc = m->getCoords(scaleT2M*(r+rr), scaleT2M*(c+cc));
      std::pair<int,int> bc = b->getCoords(scaleT2B*(r+rr), scaleT2B*(c+cc));
      assert (tc == mc && mc == bc);
#endif

      // if any response in 3x3x3 is greater candidate not maximum
      if (
        t->getResponse(r+rr, c+cc) > candidate ||
        m->getResponse(scaleT2M*(r+rr), scaleT2M*(c+cc)) > candidate ||
        b->getResponse(scaleT2B*(r+rr), scaleT2B*(c+cc)) > candidate
        ) 
        return 0;
    }
  }

  return 1;
}

//-------------------------------------------------------
/*
//! Interpolate scale-space extrema to subpixel accuracy to form an image feature.   
void FastHessian::interpolateExtremum(int r, int c)
{
  double xi = 0, xr = 0, xc = 0;

  // Get the offsets to the actual location of the extremum
  interpolateStep( octv, intvl, r, c, &xi, &xr, &xc );

  // If point is sufficiently close to the actual extremum
  if( fabs( xi ) < 0.5f  &&  fabs( xr ) < 0.5f  &&  fabs( xc ) < 0.5f )
  {
    // Create Ipoint and push onto Ipoints vector
    Ipoint ipt;
    ipt.x = static_cast<float>(c + step*xc);
    ipt.y = static_cast<float>(r + step*xr);
    ipt.scale = static_cast<float>((1.2f/9.0f) * (3*(pow(2.0f, octv+1) * (intvl+xi+1)+1)));
    ipt.laplacian = getLaplacian(octv, intvl, c, r);
    ipts.push_back(ipt);
  }
}

//-------------------------------------------------------

//! Performs one step of extremum interpolation. 
void FastHessian::interpolateStep( int octv, int intvl, int r, int c, double* xi, double* xr, double* xc )
{
  CvMat* dD, * H, * H_inv, X;
  double x[3] = { 0 };

  dD = deriv3D( octv, intvl, r, c );
  H = hessian3D( octv, intvl, r, c );
  H_inv = cvCreateMat( 3, 3, CV_64FC1 );
  cvInvert( H, H_inv, CV_SVD );
  cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
  cvGEMM( H_inv, dD, -1, NULL, 0, &X, 0 );

  cvReleaseMat( &dD );
  cvReleaseMat( &H );
  cvReleaseMat( &H_inv );

  *xi = x[2];
  *xr = x[1];
  *xc = x[0];
}

//-------------------------------------------------------

//! Computes the partial derivatives in x, y, and scale of a pixel.
CvMat* FastHessian::deriv3D( int octv, int intvl, int r, int c )
{
  CvMat* dI;
  double dx, dy, ds;
  int step = init_sample * fRound(pow(2.0f,octv));

  dx = ( getVal(octv,intvl, c+step, r ) -
    getVal( octv,intvl, c-step, r ) ) / 2.0;
  dy = ( getVal( octv,intvl, c, r+step ) -
    getVal( octv,intvl, c, r-step ) ) / 2.0;
  ds = ( getVal( octv,intvl+1, c, r ) -
    getVal( octv,intvl-1, c, r ) ) / 2.0;

  dI = cvCreateMat( 3, 1, CV_64FC1 );
  cvmSet( dI, 0, 0, dx );
  cvmSet( dI, 1, 0, dy );
  cvmSet( dI, 2, 0, ds );

  return dI;
}

//-------------------------------------------------------

//! Computes the 3D Hessian matrix for a pixel.
CvMat* FastHessian::hessian3D(int octv, int intvl, int r, int c )
{
  CvMat* H;
  double v, dxx, dyy, dss, dxy, dxs, dys;
  int step = init_sample * fRound(pow(2.0f,octv));

  v = getVal( octv,intvl, c, r );
  dxx = ( getVal( octv,intvl, c+step, r ) + 
    getVal( octv,intvl, c-step, r ) - 2 * v );
  dyy = ( getVal( octv,intvl, c, r+step ) +
    getVal( octv,intvl, c, r-step ) - 2 * v );
  dss = ( getVal( octv,intvl+1, c, r ) +
    getVal( octv,intvl-1, c, r ) - 2 * v );
  dxy = ( getVal( octv,intvl, c+step, r+step ) -
    getVal( octv,intvl, c-step, r+step ) -
    getVal( octv,intvl, c+step, r-step ) +
    getVal( octv,intvl, c-step, r-step ) ) / 4.0;
  dxs = ( getVal( octv,intvl+1, c+step, r ) -
    getVal( octv,intvl+1, c-step, r ) -
    getVal( octv,intvl-1, c+step, r ) +
    getVal( octv,intvl-1, c-step, r ) ) / 4.0;
  dys = ( getVal( octv,intvl+1, c, r+step ) -
    getVal( octv,intvl+1, c, r-step ) -
    getVal( octv,intvl-1, c, r+step ) +
    getVal( octv,intvl-1, c, r-step ) ) / 4.0;

  H = cvCreateMat( 3, 3, CV_64FC1 );
  cvmSet( H, 0, 0, dxx );
  cvmSet( H, 0, 1, dxy );
  cvmSet( H, 0, 2, dxs );
  cvmSet( H, 1, 0, dxy );
  cvmSet( H, 1, 1, dyy );
  cvmSet( H, 1, 2, dys );
  cvmSet( H, 2, 0, dxs );
  cvmSet( H, 2, 1, dys );
  cvmSet( H, 2, 2, dss );

  return H;
}
*/
//-------------------------------------------------------