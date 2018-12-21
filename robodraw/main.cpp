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
 
int main(int argc, char *argv[]) {
 
	int i, j, t;
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
		for(j = 0; j< (RegionVec[i].ContourPtVec.size() - step); )//j+=step)
		{
			t = 2;
			while((j + step*t)< RegionVec[i].ContourPtVec.size())
			{
			   double  dx1, dy1, dx2, dy2;
			   dx1 = RegionVec[i].ContourPtVec[j + step].x - RegionVec[i].ContourPtVec[j].x;                
			   dy1 = RegionVec[i].ContourPtVec[j + step].y - RegionVec[i].ContourPtVec[j].y;
			   
			   dx2 = RegionVec[i].ContourPtVec[j + step*t].x - RegionVec[i].ContourPtVec[j].x;                
			   dy2 = RegionVec[i].ContourPtVec[j + step*t].y - RegionVec[i].ContourPtVec[j].y;
			   
			   double sim1 = (dx1*dx2+ dy1*dy2)/sqrt( (dx1*dx1 + dy1*dy1) *(dx2*dx2 + dy2*dy2));
			   sim1 = acos(sim1) * 180.0/3.141592653;
			   
			   dx1 = RegionVec[i].ContourPtVec[j + step*(t-1)].x - RegionVec[i].ContourPtVec[j + step*(t-2)].x;                
			   dy1 = RegionVec[i].ContourPtVec[j + step*(t-1)].y - RegionVec[i].ContourPtVec[j + step*(t-2)].y;
			   
			   dx2 = RegionVec[i].ContourPtVec[j + step*t].x - RegionVec[i].ContourPtVec[j+ step*(t-1)].x;                
			   dy2 = RegionVec[i].ContourPtVec[j + step*t].y - RegionVec[i].ContourPtVec[j+ step*(t-1)].y;
			   
			   double sim2 = (dx1*dx2+ dy1*dy2)/sqrt( (dx1*dx1 + dy1*dy1) *(dx2*dx2 + dy2*dy2));
			   sim2 = acos(sim2) * 180.0/3.141592653;
			   if( sim1 < 4.0 && sim2 < 4.0)
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
			CPic.DrawLine(RegionVec[i].ContourPtVec[j].x , 
			              RegionVec[i].ContourPtVec[j].y ,
						  RegionVec[i].ContourPtVec[j + step * t].x , 
			              RegionVec[i].ContourPtVec[j + step * t].y  );
			CPic.SetColor(1);
			CPic.DrawCircle(RegionVec[i].ContourPtVec[j].x ,
			                RegionVec[i].ContourPtVec[j].y, 3);
			j+= step*t;
		}
	}
	CPic.Save("temp.bmp");
    return 0;
}
