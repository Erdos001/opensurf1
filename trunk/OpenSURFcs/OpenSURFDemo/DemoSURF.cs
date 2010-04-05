using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Diagnostics;
using OpenSURFcs;

namespace OpenSURFDemo
{
  public partial class DemoSURF : Form
  {
    public DemoSURF()
    {
      InitializeComponent();
    }

    List<IPoint> ipts = new List<IPoint>();

    private void btnRunSurf_Click(object sender, EventArgs e)
    {
      OpenFileDialog openFileDialog = new OpenFileDialog();
      openFileDialog.ShowDialog();
      string pathToFile = openFileDialog.FileName;

      Stopwatch watch = new Stopwatch();
      watch.Start();

      try
      {
        // Load an Image
        Bitmap img = new Bitmap(pathToFile);
        pbMainPicture.Image = img;
        
        // Create Integral Image
        IntegralImage iimg = IntegralImage.FromImage(img);
        
        // Extract the interest points
        ipts = FastHessian.getIpoints(0.0002f, 5, 2, iimg);

        // Describe the interest points
        SurfDescriptor.DecribeInterestPoints(ipts, false, false, iimg);

        // Draw points on the image
        PaintSURF(img, ipts);
      }
      catch
      {

      }

      watch.Stop();
      this.Text = "DemoSURF - Elapsed time: " + watch.Elapsed + 
                  " for " + ipts.Count + "points";
    }

    private void PaintSURF(Bitmap img, List<IPoint> ipts)
    {
      Graphics g = Graphics.FromImage(img);
      
      Pen redPen = new Pen(Color.Red);
      Pen bluePen = new Pen(Color.Blue);
      Pen myPen;

      foreach (IPoint ip in ipts)
      {
        int S = 2 * Convert.ToInt32(2.5f * ip.scale);
        int R = Convert.ToInt32(S / 2f);

        Point pt = new Point(Convert.ToInt32(ip.x), Convert.ToInt32(ip.y));
        Point ptR = new Point(Convert.ToInt32(R * Math.Cos(ip.orientation)),
                     Convert.ToInt32(R * Math.Sin(ip.orientation)));

        myPen = (ip.laplacian > 0 ? bluePen : redPen);

        g.DrawEllipse(myPen, pt.X - R, pt.Y - R, S, S);
        g.DrawLine(new Pen(Color.FromArgb(0, 255, 0)), new Point(pt.X, pt.Y), new Point(pt.X + ptR.X, pt.Y + ptR.Y));
      }
    }

  }  // DemoApp
} // OpenSURFDemo
