#ifndef Find_OBJ_Head
#define Find_OBJ_Head
#include "convexhull.h"
#include "TraceBound.h"

// Warn about use of deprecated functions.
#define GNUPLOT_DEPRECATE_WARN
//#include "gnuplot-iostream.h"
#define PONIS_MINCOUNT 2000

C24BitMap  gColorImg;
C256BitMap GPic;
C24BitMap  DispPic;
void GetBlackPicRegion(C256BitMap &Pic,C24BitMap&CPic,vector<Region>&RegionVec);
void GetObjContourColor(vector<Region> & RVec,C24BitMap&CPic,vector<int>&LabelVec);
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
	int i, j,t;
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
		{
			val = 0;
			for(t=-10;t<10;t++)
			{*get_pix_color(GPic, i+t, j) =0;}
		}
		else
		    val = 255;
		
		*get_pix_color(GPic, i, j) = BOUND(  val, 0, 255);
	 }
}

void GetBallCenter(C256BitMap &GPic, double &x,double &y)
{
	C24BitMap CPic;
	CPic.FormatF(GPic.Width, GPic.Height);
	vector<Region> RegionVec;
	//GetBlackPicRegion(GPic, CPic, RegionVec);
	
	RegionVec.clear();
    vector<int> LabelVec;
    GetBlackPicRegion( GPic, CPic, RegionVec);
    LabelVec.clear();
    GetObjContourColor(RegionVec, CPic, LabelVec);
	if(RegionVec.size()>0)
	{
		x = RegionVec[0].x;
		y = RegionVec[0].y;
	}
	/*double cx,cy,num;
	 cx = cy = num =0;
	 int i,j;
	for (i = 0; i< GPic.Width; i++)
	 for (j = 0; j < GPic.Height; j++)
	 {
		if( (*get_pix_color(GPic, i, j)) <128)
        {
			 cx += i ;
			 cy += j;
			 num+=1.0;
		}			
	 }
	 x = cx/num; y= cy/num;
	 cx = cy = num =0;
	for (i = x-100; i< x+100; i++)
	 for (j = y-100; j < y+100; j++)
	 {
		if( (*get_pix_color(GPic, i, j)) <128)
        {
			 cx += i ;
			 cy += j;
			 num+=1.0;
		}			
	 }
	 x = cx/num; y= cy/num;*/
}
void GetGrayImageYellow(C24BitMap&CPic, C256BitMap &GPic)
{
	int i, j,t;
	if(GPic.Width!= CPic.Width)
		GPic.FormatF(CPic.Width, CPic.Height);
	
	for (i = 0; i< CPic.Width; i++)
	 for (j = 0; j < CPic.Height; j++)
	 {
		 
		 if(j>1500)
		 {
			 *get_pix_color(GPic, i, j) = 255;
			 continue;
		 }
			 
		 C24PixVal  Pix = get_pix_color(CPic, i, j);
		 double val, val_min, val_max, R, G, B;
		 R = *Pix.r; G = *Pix.g; B = *Pix.b;
		 double  h, s, v;
		 RGB2HSV(R, G, B, h, s, v);
        
        if( ( fabs(h - 50) < 15 ) &&  (( s * 255.0) > 140) && ( v > 120.0) )
		{
			val = 0;
			for(t=-3;t<3;t++)
			{*get_pix_color(GPic, i+t, j) =0;}
		   
		}
		else
		    val = 255;
		
		*get_pix_color(GPic, i, j) = BOUND(  val, 0, 255);
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
	
	//	double Mxx,Myy,Mxy;
    //Mxx = Myy = Mxy = 0;
	
	for (i=0; i<Obj.PtVec.size();i++)
	{
		x  = float(Obj.PtVec[i].x)-Obj.x;
		y  = float(Obj.PtVec[i].y)-Obj.y;
		val= 1;//Obj.PtVec[i].Flux;
		Mxx +=  (x * x * val); // / sum (I)
		Myy +=  (y * y * val); // / sum (I)
		Mxy +=  (x * y * val); // / sum (I) 
	}
	
	Obj.roundratio = sqrt(pow((Mxx - Myy), 2) + pow((2 * Mxy) , 2)) / (Mxx + Myy); 

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

/*void GetObjContourColor(vector<Region> & RVec,C24BitMap&CPic,vector<int>&LabelVec)
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
}*/

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


///############################################################################
//#############################################################################
//#######################################################################
int ResampleCurve(vector<RPoint>& input, vector<RPoint>& output, int nPoint)
{
	if( input.size() == 0 ) return -1;

	int i; double full_length  = 0;
	int nInPoints = int(input.size());
	for( i=1; i<nInPoints; i++ )
		full_length +=  RPointDistance(input[i-1], input[i] );

	double crtdelta = full_length / (nPoint-1);
	double cumdelta = 0; double cumlength = 0;

	output.resize(nPoint);

	int crt_idx = 0;
	output[crt_idx] = input[0];
	++crt_idx;
	cumdelta += crtdelta;

	for( i=1; i< nInPoints; i++ )
	{
		double crtlength = RPointDistance(input[i-1] , input[i]);
		crtlength = max( 1e-5, crtlength );
		cumlength += crtlength;

		while ((cumlength>=cumdelta)&& (crt_idx<(nPoint-1)))
		{
			//add intersection
			double alpha = (cumlength-cumdelta)/crtlength;
			output[crt_idx].x = alpha*input[i-1].x+(1-alpha)*input[i].x;
			output[crt_idx].y = alpha*input[i-1].y+(1-alpha)*input[i].y;
			
			++crt_idx;
			cumdelta += crtdelta;
		}
	}

	//last point
	output[crt_idx] = input[nInPoints-1];

	return 0;

}


void GetCurveCornerPoint(C24BitMap &gColorImg, 
                        const vector<RPoint> & HullPtVec,
                        vector<int>&corner_idx,
                        vector<RPoint> &corner,
                        vector<RPoint> &shapecontour)
{
   int i,j;
   vector<RPoint>  temp_pt_vec;
   temp_pt_vec = HullPtVec;
   temp_pt_vec.push_back(HullPtVec[0]);
   int resample_size = 121;
   ResampleCurve(temp_pt_vec, shapecontour, resample_size);
   
   //================================================================
   vector < int   >  bend_point;  bend_point.resize(resample_size);
   vector < double> cornerangle; cornerangle.resize(resample_size);
   int max_angle(0), max_angle_index(0);
   //================================================================
   //================================================================
   double jj_      = 0;
   int start_point = 0;
    
   for(; jj_ < double(shapecontour.size()) + (start_point); jj_++)
          {
                  j = int( jj_+ shapecontour.size())%shapecontour.size();
                  int Idx1 = (j + shapecontour.size() - 6)%shapecontour.size();
                  int Idx2 = (j + shapecontour.size() + 6)%shapecontour.size();

                  RPoint counter_Pt1, counter_Pt2, counter_Pt3, Dir1, Dir2;
                  counter_Pt1 = shapecontour[Idx1];
                  counter_Pt2 = shapecontour[j];
                  counter_Pt3 = shapecontour[Idx2];

                  Dir1.x = counter_Pt2.x - counter_Pt1.x; Dir1.y = counter_Pt2.y - counter_Pt1.y;
                  Dir2.x = counter_Pt3.x - counter_Pt2.x; Dir2.y = counter_Pt3.y - counter_Pt2.y;

                  double angle = acos( fabs( Dir1.x * Dir2.x + Dir1.y * Dir2.y )
                               / sqrt( Dir1.x * Dir1.x + Dir1.y * Dir1.y)
                                           / sqrt( Dir2.x * Dir2.x + Dir2.y * Dir2.y) ) * 180.0 / 3.1415926;

                  cornerangle[j] = angle;
                  gColorImg.PenColor.R =255; gColorImg.PenColor.G =255;

                  if( angle > 15 )
                  {
                         gColorImg.DrawCircle( shapecontour[j].x,
                                               shapecontour[j].y, 4.5 );
                         bend_point[j] = 1;
                         if( cornerangle[j] > max_angle)
                         {
                                 max_angle       = cornerangle[j] ;
                                 max_angle_index = j;
                         }
                  }
		  }
   
}

//#################################################################################################
void GetBlackPicRegion(C256BitMap &Pic, vector<Region>&RegionVec)
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
	//CPic.FormatF(Pic.Width,Pic.Height);
	//CPic.CleanPic(0);
	
	RegionVec.clear();
	RegionVec.resize(Cp.ResultVec.size());
	
	for( i=0;i<Cp.ResultVec.size();i++)
	{
		 
		if(Cp.ResultVec[i].PtVec.size()< PONIS_MINCOUNT )
			continue;
		
		//CPic.RandPenColor();
		Loopj(Cp.ResultVec[i].PtVec.size())
		{
			//CPic.SigDot(Cp.ResultVec[i].PtVec[j].x,
			//	CPic.Height-1-Cp.ResultVec[i].PtVec[j].y);
			
			RPoint Pt;
			Pt.x = Cp.ResultVec[i].PtVec[j].x;
			Pt.y = Pic.Height-1-Cp.ResultVec[i].PtVec[j].y;
			RegionVec[i].PtVec.push_back(Pt);
		}
	}
}

#endif
