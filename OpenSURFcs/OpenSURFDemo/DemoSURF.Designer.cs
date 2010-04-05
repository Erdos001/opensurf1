namespace OpenSURFDemo
{
  partial class DemoSURF
  {
    /// <summary>
    /// Required designer variable.
    /// </summary>
    private System.ComponentModel.IContainer components = null;

    /// <summary>
    /// Clean up any resources being used.
    /// </summary>
    /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
    protected override void Dispose(bool disposing)
    {
      if (disposing && (components != null))
      {
        components.Dispose();
      }
      base.Dispose(disposing);
    }

    #region Windows Form Designer generated code

    /// <summary>
    /// Required method for Designer support - do not modify
    /// the contents of this method with the code editor.
    /// </summary>
    private void InitializeComponent()
    {
      this.pbMainPicture = new System.Windows.Forms.PictureBox();
      this.btnRunSurf = new System.Windows.Forms.Button();
      ((System.ComponentModel.ISupportInitialize)(this.pbMainPicture)).BeginInit();
      this.SuspendLayout();
      // 
      // pbMainPicture
      // 
      this.pbMainPicture.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                  | System.Windows.Forms.AnchorStyles.Left)
                  | System.Windows.Forms.AnchorStyles.Right)));
      this.pbMainPicture.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
      this.pbMainPicture.Location = new System.Drawing.Point(12, 12);
      this.pbMainPicture.Name = "pbMainPicture";
      this.pbMainPicture.Size = new System.Drawing.Size(685, 458);
      this.pbMainPicture.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
      this.pbMainPicture.TabIndex = 0;
      this.pbMainPicture.TabStop = false;
      // 
      // btnRunSurf
      // 
      this.btnRunSurf.Anchor = System.Windows.Forms.AnchorStyles.Bottom;
      this.btnRunSurf.Location = new System.Drawing.Point(281, 481);
      this.btnRunSurf.Name = "btnRunSurf";
      this.btnRunSurf.Size = new System.Drawing.Size(134, 39);
      this.btnRunSurf.TabIndex = 1;
      this.btnRunSurf.Text = "&Run OpenSURF";
      this.btnRunSurf.UseVisualStyleBackColor = true;
      this.btnRunSurf.Click += new System.EventHandler(this.btnRunSurf_Click);
      // 
      // DemoSURF
      // 
      this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
      this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
      this.ClientSize = new System.Drawing.Size(708, 526);
      this.Controls.Add(this.btnRunSurf);
      this.Controls.Add(this.pbMainPicture);
      this.Name = "DemoSURF";
      this.Text = "DemoSURF";
      ((System.ComponentModel.ISupportInitialize)(this.pbMainPicture)).EndInit();
      this.ResumeLayout(false);

    }

    #endregion

    private System.Windows.Forms.PictureBox pbMainPicture;
    private System.Windows.Forms.Button btnRunSurf;
  }
}

