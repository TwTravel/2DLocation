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

#define PI 3.141592653
 

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



 
void AnalysisImg(C256BitMap&GPic, double&X,double&Y,double &Theta)
{
	int i,j;
	double	thresh(0),thresh2(0),t1t2(0),darea(0),
		mx(0),my(0),mx2(0),my2(0),mxy(0),rv(0),tv(0),mdx(0),mdy(0),
		xm(0),ym(0),xm2(0),ym2(0),xym(0),
		temp(0),temp2(0),theta(0),pmx2(0),pmy2(0);
		
	Loopi(GPic.Width)
	   Loopj(GPic.Height)
	   {
		  double x,y;
		  x = i;
		  y = j;
		  double cval = 255 - *get_pix_color(GPic,i, j);
		  rv  += cval ;
		  mx  += cval * x;
          my  += cval * y;
          mx2 += cval * x*x;
          my2 += cval * y*y;
          mxy += cval * x*y;
	   }
/*----- compute object's properties */
    xm = mx / rv;			/* mean x */
    ym = my / rv;			/* mean y */
    mdx /= rv;				/* mean Delta-x */
    mdy /= rv;				/* mean Delta-y */
	
	xm2 = mx2 / rv - xm * xm;	/* variance of x */
    ym2 = my2 / rv - ym * ym;	/* variance of y */
    xym = mxy / rv - xm * ym;	/* covariance */
	
	if ((fabs(temp=xm2-ym2)) > 0.0)
      theta = atan2(2.0 * xym,temp) / 2.0;
    else
      theta = PI/4.0;
  
    X = xm;
	Y = ym;
	Theta = theta;
}

void AnalysisImg(C24BitMap&CPic, double&X,double&Y,double &Theta)
{
	int i,j;
	double	thresh(0),thresh2(0),t1t2(0),darea(0),
		mx(0),my(0),mx2(0),my2(0),mxy(0),rv(0),tv(0),mdx(0),mdy(0),
		xm(0),ym(0),xm2(0),ym2(0),xym(0),
		temp(0),temp2(0),theta(0),pmx2(0),pmy2(0);
		
	Loopi(CPic.Width)
	   Loopj(CPic.Height)
	   {
		  double x,y;
		  x = i;
		  y = j;
		  C24PixVal Pix = get_pix_color(CPic,i, j);
		  double cval = 255 - *Pix.r;//*get_pix_color(GPic,i, j);
		  if(cval==0)
			    continue;
		  rv  += cval ;
		  mx  += cval * x;
          my  += cval * y;
          mx2 += cval * x*x;
          my2 += cval * y*y;
          mxy += cval * x*y;
	   }
/*----- compute object's properties */
    xm = mx / rv;			/* mean x */
    ym = my / rv;			/* mean y */
    mdx /= rv;				/* mean Delta-x */
    mdy /= rv;				/* mean Delta-y */
	
	xm2 = mx2 / rv - xm * xm;	/* variance of x */
    ym2 = my2 / rv - ym * ym;	/* variance of y */
    xym = mxy / rv - xm * ym;	/* covariance */
	
	if ((fabs(temp=xm2-ym2)) > 0.0)
      theta = atan2(2.0 * xym,temp) / 2.0;
    else
      theta = PI/4.0;
  
    X = xm;
	Y = ym;
	Theta = theta;
}


struct drawLine
{
  double x,y;
  double angle;
  double val;  
};

//double inline  SimuDrawLine(int x1,int y1,int x2,int y2);

bool operator < (const drawLine& L1, const drawLine& L2 )
{
  if(L1.val> L2.val)
	 return true;
  
  return false;
}

int main(int argc, char *argv[]) {
 
	int i, j, t, k;
	C24BitMap  CPic, Draw;
	//C256BitMap GPic;
	//GPic.Load(argv[1]);
    CPic.Load(argv[1]);
	Draw.FormatF(CPic.Width, CPic.Height);
	
	double X, Y, Theta;
	AnalysisImg( GPic,  X, Y, Theta);
	int LineLength = 100;
	CPic.PenColor.R = CPic.PenColor.G = CPic.PenColor.B = 255;
	
	vector<drawLine> drawLineVec;
	int vsize = 200;
	drawLineVec.resize(vsize);
	
	Loopi(5000)
	{
		drawLine l;
		Loopj(vsize)
		{
			//double x,y,angle;
			
			l.x     = rand()%(CPic.Width  - LineLength/4) + LineLength/4;
			l.y     = rand()%(CPic.Height - LineLength/4) + LineLength/4;
			l.angle = ( rand()%rand())%180; 
			if(l.angle ==90)
			   l.angle = 0;
			//printf("%.2lf,%.2lf,%.2lf\n", l.x, l.y, l.angle);
			l.angle = l.angle * 3.1415926/180.0;
			
			l.val  = CPic.SimuDrawLine(l.x, l.y, l.x + LineLength * cos(l.angle) , l.y + LineLength * sin(l.angle));
			l.val += CPic.SimuDrawLine(l.x, l.y, l.x - LineLength * cos(l.angle) , l.y - LineLength * sin(l.angle));
			
			//printf("%.2lf\n",l.val);
			
			drawLineVec[j] = l;
		}
		
	   sort(drawLineVec.begin(), drawLineVec.end());
	   
	   printf("%.2lf,%.2lf,%.2lf,%.2lf\n", l.x, l.y, l.angle * 180.0/ 3.1415926,l.val);
	   l = drawLineVec[0];
	   
	   CPic.DrawLine(l.x, l.y, l.x + LineLength * cos(l.angle) , l.y + LineLength * sin(l.angle));
	   CPic.DrawLine(l.x, l.y, l.x - LineLength * cos(l.angle) , l.y - LineLength * sin(l.angle));
	   
	   Draw.DrawLine(l.x, l.y, l.x + LineLength * cos(l.angle) , l.y + LineLength * sin(l.angle));
	   Draw.DrawLine(l.x, l.y, l.x - LineLength * cos(l.angle) , l.y - LineLength * sin(l.angle));
	   

	   	if(i%20==0)
	   {
		     if(LineLength >10)	   
	                  LineLength -= 5;
		   printf("%i\n", i);
		   Draw.Save("temp.bmp");
		   CPic.Save("temp1.bmp");
	   }
	   
	   /*AnalysisImg(CPic,  X, Y, Theta);
	   
	   
	   CPic.DrawLine(X, Y, X +100.0, Y + 100.0 * tan(Theta));
	   CPic.DrawLine(X, Y, X -100.0, Y - 100.0 * tan(Theta));
	   
	   Draw.DrawLine(X, Y, X +100.0, Y + 100.0 * tan(Theta));
	   Draw.DrawLine(X, Y, X -100.0, Y - 100.0 * tan(Theta));
	   
*/
	
	}
	
	
	
    return 0;
}
