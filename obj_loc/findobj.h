#ifndef Find_OBJ_Head
#define Find_OBJ_Head
#include "convexhull.h"
#include "TraceBound.h"

// Warn about use of deprecated functions.
#define GNUPLOT_DEPRECATE_WARN
//#include "gnuplot-iostream.h"
#define PONIS_MINCOUNT 6400

C24BitMap  gColorImg;
C256BitMap GPic;
C24BitMap  DispPic;

void RGB2HSV(double r, double g, double b, double &h, double &s,double &v)
{
   double themin,themax,delta;
   //;
   themin = std::min( r, std::min( g, b));
   themax = std::max( r, std::max( g, b));
   delta = themax - themin;
    v = themax;
    s = 0;
   if (themax > 0)
       s = delta / themax;
    h = 0;
   if (delta > 0) {
      if (themax ==  r && themax !=  g)
          h += ( g -  b) / delta;
      if (themax ==  g && themax !=  b)
          h += (2 + ( b -  r) / delta);
      if (themax ==  b && themax !=  r)
          h += (4 + ( r -  g) / delta);
       h *= 60;
   }
   //printf("%lf, %lf\n",v,s);
   //return(h);
}

void GetGrayImage(C24BitMap&CPic,C256BitMap &GPic)
{
	int i, j;
	if(GPic.Width!= CPic.Width)
		GPic.FormatF(CPic.Width, CPic.Height);
	
	for (i = 0; i< CPic.Width; i++)
	 for (j = 0; j < CPic.Height; j++)
	 {
		 C24PixVal  Pix = get_pix_color(CPic, i, j);
		 double val, val_min, val_max, R, G, B;
		 R = *Pix.r; G = *Pix.g; B = *Pix.b;
		 double  h, s, v;
		 RGB2HSV(R, G, B, h, s, v);
 
        if( ( fabs(h - 35) < 15 ) &&  (( s * 255.0) > 60) && ( v > 150.0) )
		    val = 255;
		else
		    val = 0;
		
		*get_pix_color(GPic, i, j) = BOUND( 255 - val, 0, 255);
	 }
}

void AnalysisRegionObj(Region&Obj)
{
	int i;
	float x,y,xmin,xmax,ymin,ymax;
	float xWeight,yWeight,gWeight;
    xWeight = yWeight = gWeight = 0;
	int val;
	xmin = ymin = 9999;
	xmax = ymax = 0;
	/*-----  integrate results */
	for (i=0; i<Obj.PtVec.size();i++)
	{  
		x  = Obj.PtVec[i].x; y = Obj.PtVec[i].y;
		val= 1;//Obj.PtVec[i].pixIntensity;
		
		if (xmin > x) xmin = x; if (xmax < x) xmax = x;
		if (ymin > y) ymin = y; if (ymax < y) ymax = y;
		
		xWeight += x * val; yWeight += y *val; gWeight  +=val;
	}   
	
	/* copy some data to "obj" structure */
	//Mxx=Mxx/rv;Myy=Myy/rv;Mxy=Mxy/rv;
	
	Obj.left  = xmin; Obj.right  = xmax;
	Obj.top   = ymin; Obj.bottom = ymax;
    Obj.GeoX =  ( xmin + xmax ) / 2;
	Obj.GeoY =  ( ymin + ymax ) / 2;
	
    Obj.rwidth  = xmax - xmin +1; 
	Obj.rheight = ymax - ymin +1;
	
	Obj.x   =  (xWeight / gWeight+1.0);
	Obj.y   =  (yWeight / gWeight+1.0);
    
	double Mxx,Myy,Mxy;
    Mxx = Myy = Mxy = 0;
	
	for (i=0; i<Obj.PtVec.size();i++)
	{
		x  = float(Obj.PtVec[i].x)-Obj.x;
		y  = float(Obj.PtVec[i].y)-Obj.y;
		val= 1;//Obj.PtVec[i].Flux;
		Mxx +=  (x * x * val); // / sum (I)
		Myy +=  (y * y * val); // / sum (I)
		Mxy +=  (x * y * val); // / sum (I) 
	}
	

}

void DeleteNullRegion(vector<Region>&RVec)
{
	int i;
	vector<Region> BkVec;
	Loopi(RVec.size())
	{
		if(RVec[i].PtVec.size()==0||RVec[i].ContourPtVec.size()==0)
		{
			continue;
		}
		//if(RVec[i].PtVec.size()!=0)
		BkVec.push_back(RVec[i]);
	}
	RVec = BkVec;
}

void GetObjContourColor(vector<Region> & RVec,C24BitMap&CPic,vector<int>&LabelVec)
{
	//void CMyConvexHull::CalcuConvexhull(vector<RPoint>&ContourPtVec, vector<RPoint>&ConvexhullPt, int ptstep=3)
	CMyConvexHull Cvx;
	TraceRegion(CPic.Width,CPic.Height,RVec,LabelVec);
	DeleteNullRegion(RVec);
    //GetObjColor(CPic,RVec);
 	
	int i;

 	Loopi(RVec.size())
	{
     AnalysisRegionObj(RVec[i]);
	 //GetRegionInnerHolesAndArea( RVec[i]);
	 RVec[i].ConvexHullArea = Cvx.CalcuConvexhull(RVec[i].ContourPtVec, RVec[i].ConvexHullPtVec);
	}
	//MarkInvalidRegionByCrossValidate(RVec);
}

void GetBlackPicRegion(C256BitMap &Pic,C24BitMap&CPic,vector<Region>&RegionVec)
{
	int i,j;
	int ww,hh;
	ww = Pic.Width;
	hh = Pic.Height;
	ConnectedComponents Cp;
		
	Cp.scan_width  = Pic.LineWidth;
	Cp.scan_height = Pic.Height;
	
	Cp.left  = 0;
	Cp.right = ww-1;
	Cp.top   = 0;
	Cp.bottom= hh-1; 
	Cp.pix_threshold =128;		
	Cp.alloc_space();
	
	//Cp.label_image(Pic.Buffer,1);
	Cp.label_image(Pic.Buffer,1);
	int obnum=Cp.relabel_image(); 
	Cp.GetResultGroup(); 	 
	CPic.FormatF(Pic.Width,Pic.Height);
	CPic.CleanPic(0);
	
	RegionVec.clear();
	RegionVec.resize(Cp.ResultVec.size());
	
	for( i=0;i<Cp.ResultVec.size();i++)
	{
		 
		if(Cp.ResultVec[i].PtVec.size()< PONIS_MINCOUNT )
			continue;
		
		CPic.RandPenColor();
		Loopj(Cp.ResultVec[i].PtVec.size())
		{
			CPic.SigDot(Cp.ResultVec[i].PtVec[j].x,
				CPic.Height-1-Cp.ResultVec[i].PtVec[j].y);
			
			RPoint Pt;
			Pt.x = Cp.ResultVec[i].PtVec[j].x;
			Pt.y = CPic.Height-1-Cp.ResultVec[i].PtVec[j].y;
			RegionVec[i].PtVec.push_back(Pt);
		}
	}
}

void ProcessImg(C24BitMap&CPic, vector<Region>&RegionVec)
{

 if(DispPic.Width!= CPic.Width)
	 DispPic.FormatF( CPic.Width , CPic.Height );
 
 GetGrayImage(CPic, GPic);
 GPic.Save("grayAAA.bmp");
 printf("hello~~\n"); 

 RegionVec.clear();
 vector<int> LabelVec;
 GetBlackPicRegion( GPic, DispPic, RegionVec);
 LabelVec.clear();
 GetObjContourColor(RegionVec, CPic, LabelVec);
 
 int i;
 int count =0;
 Loopi(RegionVec.size())
 {
	//if(RegionVec[i].PtVec.size()< PONIS_MINCOUNT )
	//	continue;
	AnalysisRegionObj(RegionVec[i]);
	count++;
 }
 //if(count>4)
//	 exit(0);
}

#endif