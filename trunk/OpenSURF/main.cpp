/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  contact chris.evans@irisys.co.uk for more information.  *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#include "surflib.h"
#include "kmeans.h"
#include <ctime>
#include <iostream>

typedef std::vector<std::pair<Ipoint, Ipoint>> IpPairVec;

//-------------------------------------------------------
// Define PROCEDURE as:
//  - 1 and supply image path to run on static image
//  - 2 to capture from a webcam
//  - 3 to match features between images
//  - 4 to cluster moving features
#define PROCEDURE 3

//-------------------------------------------------------

int mainImage(const char *filename);
int mainVideo(void);
int mainMatch(void);
int mainMotionCluster(void);
void getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches);
float compareIpoints(Ipoint &ip1, Ipoint &ip2);
int locatePlanarObject(IpPairVec &matches, const CvPoint src_corners[4], CvPoint dst_corners[4]);

//-------------------------------------------------------

int main() 
{
  if (PROCEDURE == 1) return mainImage("building.pgm");//"test.png");
  if (PROCEDURE == 2) return mainVideo();
  if (PROCEDURE == 3) return mainMatch();
  if (PROCEDURE == 4) return mainMotionCluster();
}

//-------------------------------------------------------

int mainImage(const char *filename)
{
  // Declare Ipoints and other stuff
  IpVec ipts;
  IplImage *img=cvLoadImage("graf.pgm");

  // Set start time
  clock_t t1 = clock();

  // Detect and describe interest points in the image
  surfDetDes(img, ipts, false, 3, 4, 2, 0.0008f);

  // Set end time
  clock_t t2 = clock();
  std::cout<< "Time: " << t2-t1 << "  Ipts: " << ipts.size();

  // Draw the detected points
  drawIpoints(img, ipts);

  // Save Ipoints
  saveSurf("Surf.txt",ipts);

  // Display the result
  showImage(img);

  return 0;
}

//-------------------------------------------------------

int mainVideo(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpVec ipts;
  IplImage *img=NULL, *int_img=NULL;
  FastHessian fh(ipts, 3, 4, 2, 0.0004f);

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);

    // Create integral-image representation of the image
    int_img = Integral(img);

    // Set integral image
    fh.setIntImage(int_img);

    // Extract interest points and store in vector ipts
    fh.getIpoints();

    // Create Surf Descriptor Object
    Surf des(int_img, ipts);

    // Extract the descriptors for the ipts
    des.getDescriptors(true);

    cvReleaseImage(&int_img);

    // Draw the detected points
    //drawWindows(img, ipts);
    drawIpoints(img, ipts);

    // Draw the FPS figure
    drawFPS(img);

    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}

//-------------------------------------------------------

int mainMatch(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpVec ipts, ref_ipts;
  IpPairVec matches;
  IplImage *img = cvLoadImage("test1.jpg");
  surfDetDes(img, ref_ipts, false, 4, 4, 2, 0.0004f);
  CvPoint src_corners[4] = {{0,0}, {img->width,0}, {img->width, img->height}, {0, img->height}};
  CvPoint dst_corners[4];

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);

    // Detect and describe interest points in the image
    //ref_ipts = ipts;
    surfDetDes(img, ipts, false, 4, 4, 2, 0.0004f);

    // Fill match vector
    getMatches(ipts,ref_ipts,matches);
    {       
      if (locatePlanarObject(matches, src_corners, dst_corners))
      {
        for(int i = 0; i < 4; i++ )
        {
          CvPoint r1 = dst_corners[i%4];
          CvPoint r2 = dst_corners[(i+1)%4];
          cvLine( img, cvPoint(r1.x, r1.y),
            cvPoint(r2.x, r2.y), cvScalar(255,255,255), 3 );
        }
      }
    }

    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  // Release the capture device
  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}

//-------------------------------------------------------

int mainMotionCluster(void)
{
  // Initialise capture device
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // Create a window 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  // Declare Ipoints and other stuff
  IpVec ipts, ref_ipts, motion;
  IpPairVec matches;
  IplImage *img;
  Kmeans km;
  bool init = true;

  // Main capture loop
  while( 1 ) 
  {
    // Grab frame from the capture source
    img = cvQueryFrame(capture);

    // Detect and describe interest points in the image
    ref_ipts = ipts;
    surfDetDes(img, ipts, true, 4, 4, 2, 0.0001f);

    // Fill match vector
    motion.clear();
    getMatches(ipts,ref_ipts,matches);
    for (int i = 0; i < matches.size(); ++i) 
    {
      Ipoint ip;
      ip = matches[i].first;
      ip.dx = matches[i].first.x - matches[i].second.x;
      ip.dy = matches[i].first.y - matches[i].second.y;
      motion.push_back(ip);
    }
    km.Run(motion, 2, init);
    if (matches.size()) init = false;
    drawPoints(img, km.ipts);

    // Display the result
    cvShowImage("OpenSURF", img);

    // If ESC key pressed exit loop
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  // Release the capture device
  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}

//-------------------------------------------------------

void getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches)
{
  float dist, d1, d2;
  Ipoint *match;

  matches.clear();

  for(unsigned int i = 0; i < ipts1.size(); i++) 
  {
    d1 = d2 = FLT_MAX;

    for(unsigned int j = 0; j < ipts2.size(); j++) 
    {
      dist = compareIpoints(ipts1[i], ipts2[j]);  

      if(dist<d1) // if this feature matches better than current best
      {
        d2 = d1;
        d1 = dist;
        match = &ipts2[j];
      }
      else if(dist<d2) // this feature matches better than second best
      {
        d2 = dist;
      }
    }

    // if closest match has a d1:d2 ratio < 0.7 ipoints are a match
    if(d1/d2 < 0.65) 
    { 
      matches.push_back(std::make_pair(ipts1[i], *match));
    }
  }
}

//-------------------------------------------------------

// Return Euclidean distance between 2 points in descriptor space
float compareIpoints(Ipoint &ip1, Ipoint &ip2)
{
  float sum=0;

  for(int i=0; i < 64; i++)
    sum += pow(ip2.descriptor[i] - ip1.descriptor[i],2);

  return sqrt(sum);
}

//-------------------------------------------------------

// a rough implementation for object location 
int locatePlanarObject(IpPairVec &matches, const CvPoint src_corners[4], CvPoint dst_corners[4])
{
  double h[9];
  CvMat _h = cvMat(3, 3, CV_64F, h);
  std::vector<CvPoint2D32f> pt1, pt2;
  CvMat _pt1, _pt2;
  int i, n;

  n = matches.size();
  if( n < 4 )
    return 0;

  pt1.resize(n);
  pt2.resize(n);
  for( i = 0; i < n; i++ )
  {
    pt1[i] = cvPoint2D32f(matches[i].second.x, matches[i].second.y);
    pt2[i] = cvPoint2D32f(matches[i].first.x, matches[i].first.y);
  }

  _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
  _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );

  if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
    return 0;

  for( i = 0; i < 4; i++ )
  {
    double x = src_corners[i].x, y = src_corners[i].y;
    double Z = 1./(h[6]*x + h[7]*y + h[8]);
    double X = (h[0]*x + h[1]*y + h[2])*Z;
    double Y = (h[3]*x + h[4]*y + h[5])*Z;
    dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
  }

  return 1;
}
