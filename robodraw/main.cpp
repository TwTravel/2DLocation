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
 
	int i,j;
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
	
	 
	int skip  = 15;
 
	for(i=0;i< RegionVec.size();i++)
	{
		for(j = 0; j< (RegionVec[i].ContourPtVec.size() - skip); j+=skip)
		{
			CPic.SetColor(0);
			CPic.DrawLine(RegionVec[i].ContourPtVec[j].x , 
			              RegionVec[i].ContourPtVec[j].y ,
						  RegionVec[i].ContourPtVec[j + skip].x , 
			              RegionVec[i].ContourPtVec[j + skip].y  );
			CPic.SetColor(1);
			CPic.DrawCircle(RegionVec[i].ContourPtVec[j].x ,
			                RegionVec[i].ContourPtVec[j].y, 3);
		}
	}
	CPic.Save("temp.bmp");
    return 0;
}
