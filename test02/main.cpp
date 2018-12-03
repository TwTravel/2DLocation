#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "c24bitmap.h"
#include "c256bitmap.h"

int main()
{
 printf("hello world!");
 C256BitMap GPic;
 GPic.FormatF(4000,2700);
 int i,j;
 int size =300; 
 for(j=0;j<2700;j++)
 for(i=0;i<4000;i++)
  {
   int a;
   int b;
   a = i/size;
   b = j/size;
   if((a%2+b%2)%2==0)
    * get_pix_color(GPic,i,j) =0;
   else
     * get_pix_color(GPic,i,j) =255;  
  }
 GPic.Save("gray2.bmp");
 return 1;
}
