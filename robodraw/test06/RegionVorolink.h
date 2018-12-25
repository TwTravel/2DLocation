#ifndef Voronoi_Link_H
#define Voronoi_Link_H
#include <map>
using namespace std;
#include "voronoi.h"
#include "c24bitmap.h"

struct PixColor
{
 int R,G,B;
};

void DrawRegionLink(C24BitMap&CPic,vector<Region>&RegionVec, vector< vector<int> > &LinkMat,vector< vector<int> >& PtGroup)
{
	int i,j;
    CPic.SetPenColor(255,0,0);
	map<int,int>      PtHasColor;
    map<int,PixColor> PtColorMap;

	Loopi(PtGroup.size())
	{
		CPic.RandPenColor();
		PixColor Cr;
		Cr.R = CPic.PenColor.R;
		Cr.G = CPic.PenColor.G;
	    Cr.B = CPic.PenColor.B;
		Loopj(PtGroup[i].size())
		{
          int Idx = PtGroup[i][j];
		  PtHasColor[Idx] = 123;
          PtColorMap[Idx] = Cr;
		}
	}

	for(i=0;i<RegionVec.size();i++)
	{ 
		for(j=i+1;j<RegionVec.size();j++)
	   {   
		   if(LinkMat[i][j] != 0)
		   {
			   if( PtHasColor[i] == 123)
			      CPic.SetPenColor(PtColorMap[i].R,PtColorMap[i].G,PtColorMap[i].B);
			   if( PtHasColor[j] == 123)
				  CPic.SetPenColor(PtColorMap[j].R,PtColorMap[j].G,PtColorMap[j].B);
			   if( PtHasColor[i] == 0 && PtHasColor[j] == 0)
			   {
				  CPic.RandPenColor();
			   }
			   CPic.DrawLine( RegionVec[i].GeoX,RegionVec[i].GeoY,
			              RegionVec[j].GeoX,RegionVec[j].GeoY);
		   }
	   }
	}
     CPic.SetPenColor(0,255,0);
	 Loopi(RegionVec.size())
	 {
		 CPic.DrawCircle(RegionVec[i].GeoX,RegionVec[i].GeoY, 2);
	 }
}

void ReMoveNotSimilarLink(vector< vector<int> > &LinkMat,vector< vector<int> > &RegionSameMat)
{
  int i,j;
  int Size = LinkMat.size();
  for(i=0;i< Size; i++)
	   for(j=i+1;j< Size; j++)
	   {
         if(RegionSameMat[i][j]==0)
		 {
			 LinkMat[i][j]=0;
		 }
	   }

}


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
		/* int xx1,yy1,xx2,yy2;
		 xx1 = RegionVec[Mkv.neighbor[i].lab1].x; yy1 = RegionVec[Mkv.neighbor[i].lab1].y;
		 xx2 = RegionVec[Mkv.neighbor[i].lab2].x; yy2 = RegionVec[Mkv.neighbor[i].lab2].y;

		 double A1,A2;
		 A1 =  (RegionVec[Mkv.neighbor[i].lab1].PtVec.size());
		 A2 =  (RegionVec[Mkv.neighbor[i].lab2].PtVec.size());

		 fdicvec.push_back((xx1-xx2)*(xx1-xx2) + (yy1-yy2)*(yy1-yy2));
		 //CPic.DrawLine(xx1,yy1,xx2,yy2);*/
	}
}


#endif