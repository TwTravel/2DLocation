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


 
int main(int argc, char *argv[]) {
 
	int i, j, t, k;
	C24BitMap  CPic;
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
		
		for(k=0;k< RegionVec[i].MContours.size();k++)
		{
		
		vector<RPoint> ContourPtVec;
		ContourPtVec = RegionVec[i].MContours[k];
		if(ContourPtVec.size()< step*2)
			continue;
			
		PintAxis(  ContourPtVec[0].x,
		           ContourPtVec[0].y, CPic.Width, CPic.Height , zmax);

		PintAxis(  ContourPtVec[0].x,
		           ContourPtVec[0].y, CPic.Width, CPic.Height , zmin);
	    
		//.push_back(contour_pts)
		for(j = 0; j< ( ContourPtVec.size() - step); )//j+=step)
		{
			t = 2;
			
			while((j + step*t)<  ContourPtVec.size())
			{
			   double  dx1, dy1, dx2, dy2;
			   dx1 =  ContourPtVec[j + step].x -  ContourPtVec[j].x;                
			   dy1 =  ContourPtVec[j + step].y -  ContourPtVec[j].y;
			   
			   dx2 =  ContourPtVec[j + step*t].x -  ContourPtVec[j].x;                
			   dy2 =  ContourPtVec[j + step*t].y -  ContourPtVec[j].y;
			   
			   double sim1 = (dx1*dx2+ dy1*dy2)/sqrt( (dx1*dx1 + dy1*dy1) *(dx2*dx2 + dy2*dy2));
			   sim1 = acos(sim1) * 180.0/3.141592653;
			   
			   dx1 =  ContourPtVec[j + step*(t-1)].x -  ContourPtVec[j + step*(t-2)].x;                
			   dy1 =  ContourPtVec[j + step*(t-1)].y -  ContourPtVec[j + step*(t-2)].y;
			   
			   dx2 =  ContourPtVec[j + step*t].x -  ContourPtVec[j+ step*(t-1)].x;                
			   dy2 =  ContourPtVec[j + step*t].y -  ContourPtVec[j+ step*(t-1)].y;
			   
			   double sim2 = (dx1*dx2+ dy1*dy2)/sqrt( (dx1*dx1 + dy1*dy1) *(dx2*dx2 + dy2*dy2));
			   sim2 = acos(sim2) * 180.0/3.141592653;
			   if( sim1 < 5.0 && sim2 < 5.0)
			   {
				   t++; 
				   //printf("hello!\n");
			   }
			   else
			   {
				   break;
			   }
			}
			t--;
			CPic.SetColor(0);
			CPic.DrawLine( ContourPtVec[j].x , 
			               ContourPtVec[j].y ,
						   ContourPtVec[j + step * t].x , 
			               ContourPtVec[j + step * t].y  );
			CPic.SetColor(1);
			CPic.DrawCircle( ContourPtVec[j].x ,
			                 ContourPtVec[j].y, 3);
			
			PintAxis(  ContourPtVec[j + step * t].x,
		               ContourPtVec[j + step * t].y, CPic.Width, CPic.Height , zmin);
			j+= step*t;
		}
		
		PintAxis(  ContourPtVec[ ContourPtVec.size()-1].x,
		           ContourPtVec[ ContourPtVec.size()-1].y,
				  CPic.Width, CPic.Height , zmax);
		}
	}
	CPic.Save("temp.bmp");
    return 0;
}
