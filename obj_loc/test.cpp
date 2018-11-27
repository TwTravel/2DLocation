#include <stdlib.h>
#include <stdlib.h>
#include <math.h>
#include "c24bitmap.h"
#include "c256bitmap.h"
#include <vector>
#include "region.h"
#include "mregion.h"
#include "findobj.h"


using namespace std;



void GetGrayImage_(C24BitMap&CPic,C256BitMap &GPic)
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
         /*val_min = R; val_max = R;
         if( B < val_min ) val_min = B;
		 if( B > val_max ) val_max = B;
         if( G < val_min ) val_min = G;
		 if( G > val_max ) val_max = G;
         //val = ((255 - val_min) + (val_max - val_min))/2;   */
         val = 	h;
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
}

void GetGrayImage1(C24BitMap&CPic,C256BitMap &GPic)
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
         /*val_min = R; val_max = R;
         if( B < val_min ) val_min = B;
		 if( B > val_max ) val_max = B;
         if( G < val_min ) val_min = G;
		 if( G > val_max ) val_max = G;
         //val = ((255 - val_min) + (val_max - val_min))/2;   */
         val = 	s* 255;
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
}

void GetGrayImage2(C24BitMap&CPic,C256BitMap &GPic)
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
         /*val_min = R; val_max = R;
         if( B < val_min ) val_min = B;
		 if( B > val_max ) val_max = B;
         if( G < val_min ) val_min = G;
		 if( G > val_max ) val_max = G;
         //val = ((255 - val_min) + (val_max - val_min))/2;   */
         val = 	v;
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
}

void GetGrayImage3(C24BitMap&CPic,C256BitMap &GPic)
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
         /*val_min = R; val_max = R;
         if( B < val_min ) val_min = B;
		 if( B > val_max ) val_max = B;
         if( G < val_min ) val_min = G;
		 if( G > val_max ) val_max = G;
         //val = ((255 - val_min) + (val_max - val_min))/2;   */
		 if ( ( s * 255 > 120) && (v > 70) && (h>170) &&(h<220))
           val =  255; 
	     else
		   val = 0;
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
}

void GetGrayImage4(C24BitMap&CPic,C256BitMap &GPic)
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
		
		//if( ( s * 255.0) > 100 )
		//	val = 255;
         //val = ((255 - val_min) + (val_max - val_min))/2;   */
		 //val = fabs(h - 160) + sqrt( ( R*R + (G-180)*(G-180) + (B-100)*(B-100))/3.0 ) ;
		 /*if ( ( s * 255 > 120) && (v > 70) && (h>170) &&(h<220))
           val =  255; 
	     else
		   val = 0;*/
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
}

int main(int argc, char*argv[])
{
	C24BitMap CPic;
 	vector<Region> RegionVec;
	ProcessImg( CPic, RegionVec);
	/*C24BitMap  CPic;
	C256BitMap GPic;
	CPic.Load(argv[1]);
	GetGrayImage( CPic, GPic);
	GPic.Save("dest.bmp");
	
	GetGrayImage1( CPic, GPic);
	GPic.Save("dest1.bmp");
	
	GetGrayImage2(CPic, GPic);
	GPic.Save("dest2.bmp");
	
	GetGrayImage4(CPic, GPic);
	GPic.Save("dest3.bmp");*/
	
	return 1;
}