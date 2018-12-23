#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "c24bitmap.h"
#include "c256bitmap.h"
//======================================================================
#include "c24bitmap.h"
#include "c256bitmap.h"
#include <vector>
#include "region.h"
#include "mregion.h"
#include "findobj.h"
#include "hzmerge.h"
#include "voronoi.h"

//0.58817209 -0.03590756
double robo_ymin = -0.03;
double robo_ymax = 0.145;
double robo_xmin = 0.588;
double robo_xmax = 0.855;

double zmin = 0.837;
double zmax = 0.845;

 

void PintAxis( double xx, double yy, int imgw, int imgh , double zz)
{
   //printf(" %.4lf, %.4lf ,  \n" , xx, yy);//, zz);
   double robox, roboy;

   double  xscale, yscale;

   xscale = (robo_xmax - robo_xmin) / imgw;
   yscale = (robo_ymax - robo_ymin) / imgh;

   robox = xx * xscale + robo_xmin;
   roboy = yy * yscale + robo_ymin;

   printf(" %.4lf, %.4lf , %.4lf, 0.0, 1.0, 0.0, -1\n" , robox, roboy, zz);
}

C24BitMap  CPic;

void GenVoronoiLink(vector<Region>&RegionVec,int PicWidth,int PicHeight, vector< vector<int> > &LinkMat)
{
   int i,j;
   
   if(LinkMat.size()!= RegionVec.size())
   {
     LinkMat.resize(RegionVec.size());
	 for(i=0;i< RegionVec.size();i++)
	   {
	   LinkMat[i].resize(RegionVec.size());
	   }
   }
   for(i=0;i<RegionVec.size();i++)
	   for(j=0;j<RegionVec.size();j++)
	       LinkMat[i][j] = 0;
	    
   
   MkVoronoi Mkv(PicWidth, PicHeight);
	  int index =0;
	  
	  Loopi(RegionVec.size())
	  {	  
		  Mkv.sites[index].coord.x = RegionVec[i].GeoX;
		  Mkv.sites[index].coord.y = RegionVec[i].GeoY;
		  Mkv.sites[index].label   = i;
		  Mkv.sites[index].sitenbr = index;
		  
		  SelectMinMax(RegionVec[i].GeoX,Mkv.xmin,Mkv.xmax);
		  SelectMinMax(RegionVec[i].GeoY,Mkv.ymin,Mkv.ymax);
		  index++;
	  }
	Mkv.nsites = index;
	qsort(Mkv.sites, Mkv.nsites, sizeof *Mkv.sites, scomp);
	  
	 Mkv.CleanSites();
	 Mkv.voronoi(PicWidth, PicHeight);
	 
	 vector<int> fdicvec;

	 Loopi(Mkv.NEIGHnbr)
	 {
		int Lab1,Lab2;
		Lab1 = Mkv.neighbor[i].lab1;
		Lab2 = Mkv.neighbor[i].lab2;
		
		LinkMat[Lab1][Lab2] =
		    LinkMat[Lab2][Lab1] = 1; 
		 int xx1,yy1,xx2,yy2;
		 xx1 = RegionVec[Mkv.neighbor[i].lab1].x; yy1 = RegionVec[Mkv.neighbor[i].lab1].y;
		 xx2 = RegionVec[Mkv.neighbor[i].lab2].x; yy2 = RegionVec[Mkv.neighbor[i].lab2].y;

		 double A1,A2;
		 A1 =  (RegionVec[Mkv.neighbor[i].lab1].PtVec.size());
		 A2 =  (RegionVec[Mkv.neighbor[i].lab2].PtVec.size());

		 fdicvec.push_back((xx1-xx2)*(xx1-xx2) + (yy1-yy2)*(yy1-yy2));
		 CPic.DrawLine(xx1,yy1,xx2,yy2);/**/
	}
}
 
int main(int argc, char *argv[]) {
 
	int i, j, t, k;
	
	C256BitMap GPic;
	GPic.Load(argv[1]);
     
	CPic.FormatF(GPic.Width, GPic.Height);
	vector<Region> RegionVec;
	//GetBlackPicRegion(GPic, CPic, RegionVec);
	
	RegionVec.clear();
    vector<int> LabelVec;
    GetBlackPicRegion( GPic, CPic, RegionVec);
    LabelVec.clear();
    GetObjContourColor(RegionVec, CPic, LabelVec);
	
	 
	int step  = 15;
 
	for(i=0;i< RegionVec.size();i++)
	{
		//int stepmax = ;
		
		 
	}
	
	vector< vector<int> >  LinkMat;
	GenVoronoiLink(RegionVec,CPic.Width, CPic.Height,  LinkMat);
	CPic.Save("temp.bmp");
    return 0;
}
