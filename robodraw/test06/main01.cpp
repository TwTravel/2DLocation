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

//double 
double triangle_area(RPoint&Pt1, RPoint&Pt2, RPoint&Pt3)
{
 double a,b,c;
 a = RPointDistance(Pt1,Pt2);
 b = RPointDistance(Pt2,Pt3);
 c = RPointDistance(Pt3,Pt1);
 
 double p = (a + b + c) / 2 ;
 double S = sqrt( p * (p - a) * (p - b) * (p - c) );
 return S;
}

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
C256BitMap gGPic;


// PtVec; 
//vector<RPoint> RegionVecOut;
void AddPoint2BigArea(vector<RPoint> &RegionVec, vector< vector<int> > &LinkMat,
                      vector<RPoint> &RegionVecOut)
{
	RegionVecOut = RegionVec;
	int i,j,t;
	for(i = 0; i < RegionVec.size();i++)
	 for(j = (i+1);j < RegionVec.size();j++)
		 for(t= (j+1); t < RegionVec.size();t++)
		 {
			 if(LinkMat[i][j]&&LinkMat[j][t]&&LinkMat[t][i])
			 {  RPoint tmp;
				if(triangle_area(RegionVec[i], RegionVec[j], RegionVec[t])> 1000)
				{tmp.x = (RegionVec[i].x + RegionVec[j].x + RegionVec[t].x)/3;
			     tmp.y = (RegionVec[i].y + RegionVec[j].y + RegionVec[t].y)/3;
				 RegionVecOut.push_back(tmp);
				}					
			 }
		 }
}

void GenVoronoiLink(vector<RPoint> &RegionVec,int PicWidth,int PicHeight,
 vector< vector<int> > &LinkMat)
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
		  Mkv.sites[index].coord.x = RegionVec[i].x;
		  Mkv.sites[index].coord.y = RegionVec[i].y;
		  Mkv.sites[index].label   = i;
		  Mkv.sites[index].sitenbr = index;
		  
		  SelectMinMax(RegionVec[i].x,Mkv.xmin,Mkv.xmax);
		  SelectMinMax(RegionVec[i].y,Mkv.ymin,Mkv.ymax);
		  index++;
	  }
	Mkv.nsites = index;
	qsort(Mkv.sites, Mkv.nsites, sizeof *Mkv.sites, scomp);
	  
	 Mkv.CleanSites();
	 Mkv.voronoi(PicWidth, PicHeight);
	 
	 vector<int> fdicvec;
	 
      /*CPic.SetColor(2);
	  
	  Loopi(Mkv.LINEnbr)
	 {
		  CPic.DrawLine(Mkv.lineseg[i].xs,Mkv.lineseg[i].ys,
		                Mkv.lineseg[i].xe,Mkv.lineseg[i].ye);
	 }*/
	 
	  CPic.SetColor(1);
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

		 //double A1,A2;
		 //A1 =  (RegionVec[Mkv.neighbor[i].lab1].PtVec.size());
		// A2 =  (RegionVec[Mkv.neighbor[i].lab2].PtVec.size());

		 fdicvec.push_back((xx1-xx2)*(xx1-xx2) + (yy1-yy2)*(yy1-yy2));
		 
		 int xx,yy;
  
		 int draw = 1;
		 for(j=1;j<8;j++)
		 {
			 xx = j*xx1+(8-j)*xx2; xx/=8;
		     yy = j*yy1+(8-j)*yy2; yy/=8;
			 
		    if( *get_pix_color(gGPic,xx,yy) >128)
		     {
			  draw = 0;
		     }
		 }
		 
		 if(draw)
		    CPic.DrawTkLine(xx1,yy1,xx2,yy2,1); 
	     else
			LinkMat[Lab1][Lab2] =
		    LinkMat[Lab2][Lab1] = 0;  
		 
	}
}

void SampleContour(vector<RPoint> &RegionVecIn, vector<RPoint> &RegionVecOut,int step)
{
	int i,j,t,k;
	RegionVecOut.clear();
	for(i=0;i<RegionVecIn.size();i+= step)
	{
		RegionVecOut.push_back(RegionVecIn[i]);
	}
}
void SimplifyContour(vector<RPoint> &RegionVecIn, vector<RPoint> &RegionVecOut)
{
	int step  = 15;
	int i,j,k,t;
	double  dx1, dy1, dx2, dy2;
	RegionVecOut.clear();
	//for(i=0;i<;i++)
	RegionVecOut.push_back(RegionVecIn[0]);
	for(j = 0; j< ( RegionVecIn.size() - step); )//j+=step)
		{
			t = 2;
			
			while((j + step*t)<  RegionVecIn.size())
			{
			  
			   dx1 =  RegionVecIn[j + step].x   -  RegionVecIn[j].x;                
			   dy1 =  RegionVecIn[j + step].y   -  RegionVecIn[j].y;   
			   dx2 =  RegionVecIn[j + step*t].x -  RegionVecIn[j].x;                
			   dy2 =  RegionVecIn[j + step*t].y -  RegionVecIn[j].y;
			   
			   double sim1 = (dx1*dx2+ dy1*dy2)/sqrt( (dx1*dx1 + dy1*dy1) *(dx2*dx2 + dy2*dy2));
			   sim1 = acos(sim1) * 180.0/3.141592653;
			   
			   dx1 =  RegionVecIn[j + step*(t-1)].x -  RegionVecIn[j + step*(t-2)].x;                
			   dy1 =  RegionVecIn[j + step*(t-1)].y -  RegionVecIn[j + step*(t-2)].y;  
			   dx2 =  RegionVecIn[j + step*t].x -  RegionVecIn[j+ step*(t-1)].x;                
			   dy2 =  RegionVecIn[j + step*t].y -  RegionVecIn[j+ step*(t-1)].y;
			   
			   double sim2 = (dx1*dx2+ dy1*dy2)/sqrt( (dx1*dx1 + dy1*dy1) *(dx2*dx2 + dy2*dy2));
			   sim2 = acos(sim2) * 180.0/3.141592653;
			   if( sim1 < 5.0 && sim2 < 5.0)
			   {
				   t++;  
			   }
			   else
			   {
				   break;
			   }
			}
			t--;
			RegionVecOut.push_back(RegionVecIn[j + step * t]);
			 
			j+= step*t;
		}
		
		if(j!= (RegionVecIn.size()-1) &&RegionVecOut.size()>2)
		{
		   int last =  (RegionVecIn.size()-1);
		   dx1 =  RegionVecOut[1].x   -  RegionVecIn[last].x;                
		   dy1 =  RegionVecOut[1].y   -  RegionVecIn[last].y;   
		   dx2 =  RegionVecIn[last].x -  RegionVecIn[j].x;                
		   dy2 =  RegionVecIn[last].y -  RegionVecIn[j].y;
		
		  double sim1 = (dx1*dx2+ dy1*dy2)/sqrt( (dx1*dx1 + dy1*dy1) *(dx2*dx2 + dy2*dy2));
			   sim1 = acos(sim1) * 180.0/3.141592653;
		 if(sim1> 5.0)
		   RegionVecOut.push_back(RegionVecIn[last]);
		}
		//PintAxis(  RegionVecIn[ RegionVecIn.size()-1].x,
		//           RegionVecIn[ RegionVecIn.size()-1].y,
		//		  CPic.Width, CPic.Height , zmax);
		 CPic.SetColor(1);
		 
		 for(i=0;i< RegionVecIn.size()-10;i++)		
	     {
			 int j = (i+1);//%RegionVecIn.size();
			 CPic.DrawTkLine(RegionVecIn[i].x, RegionVecIn[i].y,
		                     RegionVecIn[j].x, RegionVecIn[j].y,1);
		 } /**/
		/*for(i=0;i<RegionVecOut.size();i++)
		{
			int j = (i+1)%RegionVecOut.size();
		    CPic.DrawTkLine(RegionVecOut[i].x, RegionVecOut[i].y,
		                 RegionVecOut[j].x, RegionVecOut[j].y,1);
		}*/
}


void ColorHSV(double c1_h, double c1_s, double c1_v,int &R,int &G,int &B)
{
	double sat_r, sat_g, sat_b;

   while (c1_h < 0)
      c1_h += 360;
   while (c1_h > 360)
      c1_h -= 360;

   if (c1_h < 120) {
      sat_r = (120 - c1_h) / 60.0;
      sat_g = c1_h / 60.0;
      sat_b = 0;
   } else if (c1_h < 240) {
      sat_r = 0;
      sat_g = (240 - c1_h) / 60.0;
      sat_b = (c1_h - 120) / 60.0;
   } else {
      sat_r = (c1_h - 240) / 60.0;
      sat_g = 0;
      sat_b = (360 - c1_h) / 60.0;
   }
   sat_r = std::min(sat_r,1.0);
   sat_g = std::min(sat_g,1.0);
   sat_b = std::min(sat_b,1.0);

   sat_r = (1 - c1_s + c1_s * sat_r) * c1_v *255.0;
   sat_g = (1 - c1_s + c1_s * sat_g) * c1_v *255.0;
   sat_b = (1 - c1_s + c1_s * sat_b) * c1_v *255.0;
   //==================================================
   R =  sat_r;
   G =  sat_g;
   B =  sat_b;
}


//===========================================================
#include "Graph.h"
#include "ChinesePostman.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

pair< Graph, vector<double> > ReadWeightedGraph(string filename)
{
	//Please see Graph.h for a description of the interface

	ifstream file;
	file.open(filename.c_str());

	string s;
	getline(file, s);
	stringstream ss(s);
	int n;
	ss >> n;
	getline(file, s);
	ss.str(s);
	ss.clear();
	int m;
	ss >> m;

	Graph G(n);
	vector<double> cost(m);
	for(int i = 0; i < m; i++)
	{
		getline(file, s);
		ss.str(s);
		ss.clear();
		int u, v;
		double c;
		ss >> u >> v >> c;

		G.AddEdge(u, v);
		cost[G.GetEdgeIndex(u, v)] = c;
	}

	file.close();
	return make_pair(G, cost);
}

pair< Graph, vector<double> > ReadWeightedGraph
(vector<RPoint> &RegionVecOut2, vector< vector<int> > & LinkMat)
{
	//Please see Graph.h for a description of the interface
    int i,j;
	Graph G(RegionVecOut2.size());
	vector<double> cost;
	for(int i = 0; i < RegionVecOut2.size(); i++)
		for(int j = i+1; j < RegionVecOut2.size(); j++)
	{
		//getline(file, s);
		//ss.str(s);
		//ss.clear();
		if(LinkMat[i][j])
		{//int u, v;
		 //double c;
		//ss >> u >> v >> c;
		G.AddEdge(i, j);
		double dist = RPointDistance(RegionVecOut2[i], RegionVecOut2[j]);
		cost.push_back(dist);
		cost[G.GetEdgeIndex(i, j)] =  dist;
		}
	}

	 
	return make_pair(G, cost);
}
//vector< vector<int> >  LinkMat;
//	GenVoronoiLink(RegionVecOut,CPic.Width, CPic.Height,  LinkMat);
//	vector<RPoint>  RegionVecOut2;
void GenPathFromDots(vector<RPoint> &RegionVecOut2, vector< vector<int> > & LinkMat)
{	   
  char* filename; 
  Graph G;
  vector<double> cost;
	
  //Read the graph
  pair< Graph, vector<double> > p = 
  ReadWeightedGraph( RegionVecOut2, LinkMat);
  G = p.first;
  cost = p.second;

  //Solve the problem
  pair< list<int> , double > sol = ChinesePostman(G, cost);

  cout << "Solution cost: " << sol.second << endl;

  list<int> s = sol.first;

  //Print edges in the solution
  cout << "Solution:" << endl;
  for(list<int>::iterator it = s.begin(); it != s.end(); it++)
  cout << *it << " ";
  cout << endl;
}
//===========================================================
//===========================================================
//===========================================================

int main(int argc, char *argv[]) {
 
	int i, j, t, k;
	
	C256BitMap GPic;
	GPic.Load(argv[1]);
    gGPic = GPic;
	
	CPic.FormatF(GPic.Width, GPic.Height);
	vector<Region> RegionVec;
	//GetBlackPicRegion(GPic, CPic, RegionVec);
	
	RegionVec.clear();
    vector<int> LabelVec;
    GetBlackPicRegion( GPic, CPic, RegionVec);
    LabelVec.clear();
    GetObjContourColor(RegionVec, CPic, LabelVec);
	
	CPic.Save("tt.bmp");
	vector<RPoint>  RegionVecOut;
	
	
	 //SampleContour(RegionVec[0].ContourPtVec,  RegionVecOut,20);
 ResampleCurve(RegionVec[0].ContourPtVec,  RegionVecOut, RegionVec[0].ContourPtVec.size()/30);
	//SimplifyContour( RegionVec[0].ContourPtVec,  RegionVecOut);
	
	CPic.SetColor( 0 );
	

	
	for(i=0;i< RegionVec.size();i++)
	{
		//int stepmax = ;
		
		for(k=0;k< RegionVec[i].MContours.size();k++)
		{
		 CPic.RandPenColor();
		 vector<RPoint> ContourPtVec;
		 ContourPtVec = RegionVec[i].MContours[k];
		 
		  for(j = 0; j< ( ContourPtVec.size() - 1); j++)//j+=step)
		  {
		    CPic.DrawTkLine( ContourPtVec[j  ].x, ContourPtVec[j  ].y,
			                 ContourPtVec[j+1].x, ContourPtVec[j+1].y,1);
		  }
		}
	}
	
	
	vector< vector<int> >  LinkMat;
	GenVoronoiLink(RegionVecOut,CPic.Width, CPic.Height,  LinkMat);
	vector<RPoint>  RegionVecOut2;
	AddPoint2BigArea(RegionVecOut,  LinkMat, RegionVecOut2);
	
	CPic.SetColor(4);
	
	Loopi( RegionVec[0].PtVec.size() )
	{
		CPic.SigDot(RegionVec[0].PtVec[i].x, RegionVec[0].PtVec[i].y);
	}
	GenVoronoiLink(RegionVecOut2,CPic.Width, CPic.Height,  LinkMat);
	
	Loopi( RegionVecOut.size()-1)
	{
		CPic.DrawTkLine(RegionVecOut[i].x, RegionVecOut[i].y,
		                RegionVecOut[i+1].x, RegionVecOut[i+1].y,1);
	}
	
	GenPathFromDots( RegionVecOut2,   LinkMat);
	CPic.Save("temp.bmp");
    return 0;
}


/*
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
      CPic.SetColor(0);
	  Loopi(Mkv.LINEnbr)
	 {
		  CPic.DrawLine(Mkv.lineseg[i].xs,Mkv.lineseg[i].ys,
		                Mkv.lineseg[i].xe,Mkv.lineseg[i].ye);
	 }
	 
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
		 //CPic.DrawLine(xx1,yy1,xx2,yy2); 
	}
}
*/