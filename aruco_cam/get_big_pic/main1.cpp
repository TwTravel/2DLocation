#include<stdlib.h>
#include<stdio.h>
#include <math.h>
#include "c24bitmap.h"
#include "c256bitmap.h"

char * mkfiles[641] ={
"marker/marker_1000.bmp",
"marker/marker_1010.bmp",
"marker/marker_1016.bmp",
"marker/marker_1018.bmp",
"marker/marker_1036.bmp",
"marker/marker_1038.bmp",
"marker/marker_1043.bmp",
"marker/marker_1044.bmp",
"marker/marker_1047.bmp",
"marker/marker_1049.bmp",
"marker/marker_1052.bmp",
"marker/marker_1054.bmp",
"marker/marker_1055.bmp",
"marker/marker_1058.bmp",
"marker/marker_105.bmp",
"marker/marker_1067.bmp",
"marker/marker_106.bmp",
"marker/marker_1096.bmp",
"marker/marker_10.bmp",
"marker/marker_1119.bmp",
"marker/marker_1128.bmp",
"marker/marker_1130.bmp",
"marker/marker_1134.bmp",
"marker/marker_1137.bmp",
"marker/marker_1153.bmp",
"marker/marker_1170.bmp",
"marker/marker_1180.bmp",
"marker/marker_1189.bmp",
"marker/marker_1190.bmp",
"marker/marker_1192.bmp",
"marker/marker_1201.bmp",
"marker/marker_1205.bmp",
"marker/marker_1215.bmp",
"marker/marker_1219.bmp",
"marker/marker_1221.bmp",
"marker/marker_1223.bmp",
"marker/marker_1225.bmp",
"marker/marker_1228.bmp",
"marker/marker_1234.bmp",
"marker/marker_1239.bmp",
"marker/marker_1241.bmp",
"marker/marker_1244.bmp",
"marker/marker_1249.bmp",
"marker/marker_1253.bmp",
"marker/marker_1266.bmp",
"marker/marker_1271.bmp",
"marker/marker_1274.bmp",
"marker/marker_1275.bmp",
"marker/marker_1278.bmp",
"marker/marker_1279.bmp",
"marker/marker_1289.bmp",
"marker/marker_1290.bmp",
"marker/marker_1296.bmp",
"marker/marker_1309.bmp",
"marker/marker_1317.bmp",
"marker/marker_1323.bmp",
"marker/marker_1324.bmp",
"marker/marker_1331.bmp",
"marker/marker_1335.bmp",
"marker/marker_1352.bmp",
"marker/marker_1357.bmp",
"marker/marker_1361.bmp",
"marker/marker_1362.bmp",
"marker/marker_136.bmp",
"marker/marker_1375.bmp",
"marker/marker_1377.bmp",
"marker/marker_1382.bmp",
"marker/marker_1385.bmp",
"marker/marker_1387.bmp",
"marker/marker_1391.bmp",
"marker/marker_1399.bmp",
"marker/marker_1403.bmp",
"marker/marker_1415.bmp",
"marker/marker_141.bmp",
"marker/marker_1421.bmp",
"marker/marker_1427.bmp",
"marker/marker_1428.bmp",
"marker/marker_1431.bmp",
"marker/marker_1432.bmp",
"marker/marker_1434.bmp",
"marker/marker_1440.bmp",
"marker/marker_1441.bmp",
"marker/marker_1453.bmp",
"marker/marker_1454.bmp",
"marker/marker_1474.bmp",
"marker/marker_1489.bmp",
"marker/marker_1492.bmp",
"marker/marker_1495.bmp",
"marker/marker_1505.bmp",
"marker/marker_1508.bmp",
"marker/marker_1515.bmp",
"marker/marker_1517.bmp",
"marker/marker_1518.bmp",
"marker/marker_1541.bmp",
"marker/marker_1553.bmp",
"marker/marker_1559.bmp",
"marker/marker_1562.bmp",
"marker/marker_1569.bmp",
"marker/marker_1572.bmp",
"marker/marker_1595.bmp",
"marker/marker_1610.bmp",
"marker/marker_1611.bmp",
"marker/marker_1616.bmp",
"marker/marker_1620.bmp",
"marker/marker_1623.bmp",
"marker/marker_1624.bmp",
"marker/marker_1635.bmp",
"marker/marker_1640.bmp",
"marker/marker_1641.bmp",
"marker/marker_1648.bmp",
"marker/marker_1650.bmp",
"marker/marker_1655.bmp",
"marker/marker_1657.bmp",
"marker/marker_1660.bmp",
"marker/marker_1663.bmp",
"marker/marker_1665.bmp",
"marker/marker_1670.bmp",
"marker/marker_1672.bmp",
"marker/marker_1680.bmp",
"marker/marker_1682.bmp",
"marker/marker_1685.bmp",
"marker/marker_1692.bmp",
"marker/marker_16.bmp",
"marker/marker_1703.bmp",
"marker/marker_1709.bmp",
"marker/marker_1710.bmp",
"marker/marker_1712.bmp",
"marker/marker_1715.bmp",
"marker/marker_1719.bmp",
"marker/marker_1738.bmp",
"marker/marker_1748.bmp",
"marker/marker_175.bmp",
"marker/marker_1765.bmp",
"marker/marker_1778.bmp",
"marker/marker_1784.bmp",
"marker/marker_1796.bmp",
"marker/marker_1802.bmp",
"marker/marker_1807.bmp",
"marker/marker_1810.bmp",
"marker/marker_1812.bmp",
"marker/marker_1818.bmp",
"marker/marker_1819.bmp",
"marker/marker_181.bmp",
"marker/marker_1821.bmp",
"marker/marker_1850.bmp",
"marker/marker_1851.bmp",
"marker/marker_1858.bmp",
"marker/marker_1861.bmp",
"marker/marker_1862.bmp",
"marker/marker_1868.bmp",
"marker/marker_1870.bmp",
"marker/marker_1872.bmp",
"marker/marker_1883.bmp",
"marker/marker_1887.bmp",
"marker/marker_1891.bmp",
"marker/marker_1899.bmp",
"marker/marker_1906.bmp",
"marker/marker_1907.bmp",
"marker/marker_190.bmp",
"marker/marker_1919.bmp",
"marker/marker_191.bmp",
"marker/marker_192.bmp",
"marker/marker_1936.bmp",
"marker/marker_1941.bmp",
"marker/marker_1948.bmp",
"marker/marker_1956.bmp",
"marker/marker_1960.bmp",
"marker/marker_1961.bmp",
"marker/marker_1967.bmp",
"marker/marker_1971.bmp",
"marker/marker_1972.bmp",
"marker/marker_1974.bmp",
"marker/marker_1981.bmp",
"marker/marker_1983.bmp",
"marker/marker_1984.bmp",
"marker/marker_1985.bmp",
"marker/marker_1993.bmp",
"marker/marker_2002.bmp",
"marker/marker_2010.bmp",
"marker/marker_2011.bmp",
"marker/marker_2020.bmp",
"marker/marker_2034.bmp",
"marker/marker_2041.bmp",
"marker/marker_204.bmp",
"marker/marker_2054.bmp",
"marker/marker_2058.bmp",
"marker/marker_2059.bmp",
"marker/marker_2061.bmp",
"marker/marker_2069.bmp",
"marker/marker_2071.bmp",
"marker/marker_207.bmp",
"marker/marker_2084.bmp",
"marker/marker_2088.bmp",
"marker/marker_2090.bmp",
"marker/marker_2117.bmp",
"marker/marker_2121.bmp",
"marker/marker_2134.bmp",
"marker/marker_2138.bmp",
"marker/marker_213.bmp",
"marker/marker_2140.bmp",
"marker/marker_2143.bmp",
"marker/marker_2145.bmp",
"marker/marker_2149.bmp",
"marker/marker_214.bmp",
"marker/marker_2169.bmp",
"marker/marker_2171.bmp",
"marker/marker_2176.bmp",
"marker/marker_2179.bmp",
"marker/marker_2182.bmp",
"marker/marker_2183.bmp",
"marker/marker_2191.bmp",
"marker/marker_2192.bmp",
"marker/marker_2196.bmp",
"marker/marker_2205.bmp",
"marker/marker_2207.bmp",
"marker/marker_2212.bmp",
"marker/marker_2215.bmp",
"marker/marker_2236.bmp",
"marker/marker_2238.bmp",
"marker/marker_2244.bmp",
"marker/marker_2252.bmp",
"marker/marker_2258.bmp",
"marker/marker_2261.bmp",
"marker/marker_2265.bmp",
"marker/marker_2276.bmp",
"marker/marker_2287.bmp",
"marker/marker_2288.bmp",
"marker/marker_2290.bmp",
"marker/marker_2291.bmp",
"marker/marker_2299.bmp",
"marker/marker_2302.bmp",
"marker/marker_2305.bmp",
"marker/marker_2306.bmp",
"marker/marker_2310.bmp",
"marker/marker_2312.bmp",
"marker/marker_2317.bmp",
"marker/marker_2323.bmp",
"marker/marker_2325.bmp",
"marker/marker_2327.bmp",
"marker/marker_2334.bmp",
"marker/marker_2336.bmp",
"marker/marker_2337.bmp",
"marker/marker_2345.bmp",
"marker/marker_2346.bmp",
"marker/marker_2366.bmp",
"marker/marker_2367.bmp",
"marker/marker_2369.bmp",
"marker/marker_2371.bmp",
"marker/marker_2373.bmp",
"marker/marker_237.bmp",
"marker/marker_2391.bmp",
"marker/marker_2395.bmp",
"marker/marker_239.bmp",
"marker/marker_2406.bmp",
"marker/marker_240.bmp",
"marker/marker_2411.bmp",
"marker/marker_2415.bmp",
"marker/marker_241.bmp",
"marker/marker_2425.bmp",
"marker/marker_2426.bmp",
"marker/marker_2430.bmp",
"marker/marker_2442.bmp",
"marker/marker_2444.bmp",
"marker/marker_2460.bmp",
"marker/marker_2461.bmp",
"marker/marker_2472.bmp",
"marker/marker_2473.bmp",
"marker/marker_2491.bmp",
"marker/marker_2492.bmp",
"marker/marker_2507.bmp",
"marker/marker_2509.bmp",
"marker/marker_2521.bmp",
"marker/marker_2536.bmp",
"marker/marker_2538.bmp",
"marker/marker_2546.bmp",
"marker/marker_2550.bmp",
"marker/marker_2553.bmp",
"marker/marker_2554.bmp",
"marker/marker_2557.bmp",
"marker/marker_2561.bmp",
"marker/marker_2566.bmp",
"marker/marker_2576.bmp",
"marker/marker_2577.bmp",
"marker/marker_2591.bmp",
"marker/marker_2594.bmp",
"marker/marker_259.bmp",
"marker/marker_2600.bmp",
"marker/marker_2609.bmp",
"marker/marker_2610.bmp",
"marker/marker_2616.bmp",
"marker/marker_2632.bmp",
"marker/marker_2636.bmp",
"marker/marker_263.bmp",
"marker/marker_264.bmp",
"marker/marker_2660.bmp",
"marker/marker_2668.bmp",
"marker/marker_2674.bmp",
"marker/marker_2687.bmp",
"marker/marker_2696.bmp",
"marker/marker_2703.bmp",
"marker/marker_2710.bmp",
"marker/marker_2715.bmp",
"marker/marker_2724.bmp",
"marker/marker_2726.bmp",
"marker/marker_2746.bmp",
"marker/marker_2749.bmp",
"marker/marker_2755.bmp",
"marker/marker_2760.bmp",
"marker/marker_2762.bmp",
"marker/marker_2782.bmp",
"marker/marker_2787.bmp",
"marker/marker_2789.bmp",
"marker/marker_278.bmp",
"marker/marker_2792.bmp",
"marker/marker_2797.bmp",
"marker/marker_2799.bmp",
"marker/marker_27.bmp",
"marker/marker_2806.bmp",
"marker/marker_2807.bmp",
"marker/marker_2832.bmp",
"marker/marker_2834.bmp",
"marker/marker_2838.bmp",
"marker/marker_2844.bmp",
"marker/marker_2852.bmp",
"marker/marker_2854.bmp",
"marker/marker_2855.bmp",
"marker/marker_2857.bmp",
"marker/marker_2864.bmp",
"marker/marker_2871.bmp",
"marker/marker_2872.bmp",
"marker/marker_2881.bmp",
"marker/marker_2883.bmp",
"marker/marker_2889.bmp",
"marker/marker_2892.bmp",
"marker/marker_2895.bmp",
"marker/marker_2899.bmp",
"marker/marker_2902.bmp",
"marker/marker_2903.bmp",
"marker/marker_2909.bmp",
"marker/marker_2910.bmp",
"marker/marker_2915.bmp",
"marker/marker_2916.bmp",
"marker/marker_2925.bmp",
"marker/marker_2935.bmp",
"marker/marker_2936.bmp",
"marker/marker_2950.bmp",
"marker/marker_2958.bmp",
"marker/marker_2961.bmp",
"marker/marker_2962.bmp",
"marker/marker_2966.bmp",
"marker/marker_2971.bmp",
"marker/marker_2972.bmp",
"marker/marker_2978.bmp",
"marker/marker_2986.bmp",
"marker/marker_2990.bmp",
"marker/marker_3008.bmp",
"marker/marker_300.bmp",
"marker/marker_302.bmp",
"marker/marker_3031.bmp",
"marker/marker_3033.bmp",
"marker/marker_3034.bmp",
"marker/marker_303.bmp",
"marker/marker_3043.bmp",
"marker/marker_3054.bmp",
"marker/marker_3055.bmp",
"marker/marker_305.bmp",
"marker/marker_3064.bmp",
"marker/marker_3075.bmp",
"marker/marker_3082.bmp",
"marker/marker_3089.bmp",
"marker/marker_3096.bmp",
"marker/marker_3102.bmp",
"marker/marker_3105.bmp",
"marker/marker_310.bmp",
"marker/marker_3119.bmp",
"marker/marker_3123.bmp",
"marker/marker_3131.bmp",
"marker/marker_3155.bmp",
"marker/marker_3172.bmp",
"marker/marker_3177.bmp",
"marker/marker_3184.bmp",
"marker/marker_3185.bmp",
"marker/marker_3187.bmp",
"marker/marker_3193.bmp",
"marker/marker_3196.bmp",
"marker/marker_3198.bmp",
"marker/marker_3202.bmp",
"marker/marker_3206.bmp",
"marker/marker_3207.bmp",
"marker/marker_320.bmp",
"marker/marker_3218.bmp",
"marker/marker_322.bmp",
"marker/marker_3235.bmp",
"marker/marker_3247.bmp",
"marker/marker_3254.bmp",
"marker/marker_3258.bmp",
"marker/marker_326.bmp",
"marker/marker_3270.bmp",
"marker/marker_3275.bmp",
"marker/marker_3276.bmp",
"marker/marker_327.bmp",
"marker/marker_3281.bmp",
"marker/marker_3283.bmp",
"marker/marker_3306.bmp",
"marker/marker_3308.bmp",
"marker/marker_3311.bmp",
"marker/marker_3322.bmp",
"marker/marker_3331.bmp",
"marker/marker_3337.bmp",
"marker/marker_3343.bmp",
"marker/marker_3348.bmp",
"marker/marker_3349.bmp",
"marker/marker_3356.bmp",
"marker/marker_3358.bmp",
"marker/marker_3360.bmp",
"marker/marker_336.bmp",
"marker/marker_3371.bmp",
"marker/marker_3372.bmp",
"marker/marker_3373.bmp",
"marker/marker_3385.bmp",
"marker/marker_3386.bmp",
"marker/marker_3390.bmp",
"marker/marker_3391.bmp",
"marker/marker_3396.bmp",
"marker/marker_33.bmp",
"marker/marker_3411.bmp",
"marker/marker_3416.bmp",
"marker/marker_341.bmp",
"marker/marker_3421.bmp",
"marker/marker_3428.bmp",
"marker/marker_3430.bmp",
"marker/marker_3432.bmp",
"marker/marker_3442.bmp",
"marker/marker_3443.bmp",
"marker/marker_3446.bmp",
"marker/marker_3449.bmp",
"marker/marker_344.bmp",
"marker/marker_3400.bmp",
"marker/marker_3455.bmp",
"marker/marker_3456.bmp",
"marker/marker_3461.bmp",
"marker/marker_3467.bmp",
"marker/marker_3473.bmp",
"marker/marker_3478.bmp",
"marker/marker_3489.bmp",
"marker/marker_3492.bmp",
"marker/marker_3498.bmp",
"marker/marker_349.bmp",
"marker/marker_3500.bmp",
"marker/marker_3503.bmp",
"marker/marker_3510.bmp",
"marker/marker_3519.bmp",
"marker/marker_351.bmp",
"marker/marker_3521.bmp",
"marker/marker_3523.bmp",
"marker/marker_3524.bmp",
"marker/marker_3529.bmp",
"marker/marker_3531.bmp",
"marker/marker_3535.bmp",
"marker/marker_3538.bmp",
"marker/marker_3543.bmp",
"marker/marker_3548.bmp",
"marker/marker_3550.bmp",
"marker/marker_3553.bmp",
"marker/marker_3556.bmp",
"marker/marker_3558.bmp",
"marker/marker_3561.bmp",
"marker/marker_3562.bmp",
"marker/marker_3563.bmp",
"marker/marker_3564.bmp",
"marker/marker_3568.bmp",
"marker/marker_3570.bmp",
"marker/marker_3575.bmp",
"marker/marker_3587.bmp",
"marker/marker_3594.bmp",
"marker/marker_35.bmp",
"marker/marker_3609.bmp",
"marker/marker_3610.bmp",
"marker/marker_3612.bmp",
"marker/marker_3614.bmp",
"marker/marker_3620.bmp",
"marker/marker_3628.bmp",
"marker/marker_3632.bmp",
"marker/marker_3634.bmp",
"marker/marker_3637.bmp",
"marker/marker_3641.bmp",
"marker/marker_3657.bmp",
"marker/marker_3659.bmp",
"marker/marker_3671.bmp",
"marker/marker_3672.bmp",
"marker/marker_3673.bmp",
"marker/marker_3675.bmp",
"marker/marker_3681.bmp",
"marker/marker_3683.bmp",
"marker/marker_3684.bmp",
"marker/marker_3688.bmp",
"marker/marker_3696.bmp",
"marker/marker_3709.bmp",
"marker/marker_370.bmp",
"marker/marker_3719.bmp",
"marker/marker_3724.bmp",
"marker/marker_3730.bmp",
"marker/marker_3732.bmp",
"marker/marker_3738.bmp",
"marker/marker_3744.bmp",
"marker/marker_3745.bmp",
"marker/marker_3752.bmp",
"marker/marker_3755.bmp",
"marker/marker_3756.bmp",
"marker/marker_3764.bmp",
"marker/marker_3767.bmp",
"marker/marker_3770.bmp",
"marker/marker_3786.bmp",
"marker/marker_3787.bmp",
"marker/marker_3789.bmp",
"marker/marker_378.bmp",
"marker/marker_3793.bmp",
"marker/marker_3807.bmp",
"marker/marker_3818.bmp",
"marker/marker_3819.bmp",
"marker/marker_3820.bmp",
"marker/marker_3821.bmp",
"marker/marker_3825.bmp",
"marker/marker_3837.bmp",
"marker/marker_3841.bmp",
"marker/marker_389.bmp",
"marker/marker_38.bmp",
"marker/marker_391.bmp",
"marker/marker_402.bmp",
"marker/marker_404.bmp",
"marker/marker_406.bmp",
"marker/marker_408.bmp",
"marker/marker_411.bmp",
"marker/marker_418.bmp",
"marker/marker_425.bmp",
"marker/marker_428.bmp",
"marker/marker_442.bmp",
"marker/marker_447.bmp",
"marker/marker_448.bmp",
"marker/marker_452.bmp",
"marker/marker_453.bmp",
"marker/marker_454.bmp",
"marker/marker_45.bmp",
"marker/marker_460.bmp",
"marker/marker_462.bmp",
"marker/marker_465.bmp",
"marker/marker_476.bmp",
"marker/marker_482.bmp",
"marker/marker_496.bmp",
"marker/marker_507.bmp",
"marker/marker_50.bmp",
"marker/marker_513.bmp",
"marker/marker_514.bmp",
"marker/marker_519.bmp",
"marker/marker_522.bmp",
"marker/marker_523.bmp",
"marker/marker_529.bmp",
"marker/marker_535.bmp",
"marker/marker_542.bmp",
"marker/marker_543.bmp",
"marker/marker_546.bmp",
"marker/marker_552.bmp",
"marker/marker_556.bmp",
"marker/marker_559.bmp",
"marker/marker_564.bmp",
"marker/marker_56.bmp",
"marker/marker_573.bmp",
"marker/marker_579.bmp",
"marker/marker_587.bmp",
"marker/marker_589.bmp",
"marker/marker_601.bmp",
"marker/marker_605.bmp",
"marker/marker_606.bmp",
"marker/marker_625.bmp",
"marker/marker_627.bmp",
"marker/marker_62.bmp",
"marker/marker_631.bmp",
"marker/marker_63.bmp",
"marker/marker_649.bmp",
"marker/marker_652.bmp",
"marker/marker_659.bmp",
"marker/marker_674.bmp",
"marker/marker_676.bmp",
"marker/marker_67.bmp",
"marker/marker_688.bmp",
"marker/marker_689.bmp",
"marker/marker_69.bmp",
"marker/marker_708.bmp",
"marker/marker_712.bmp",
"marker/marker_71.bmp",
"marker/marker_720.bmp",
"marker/marker_722.bmp",
"marker/marker_733.bmp",
"marker/marker_735.bmp",
"marker/marker_747.bmp",
"marker/marker_74.bmp",
"marker/marker_753.bmp",
"marker/marker_772.bmp",
"marker/marker_776.bmp",
"marker/marker_778.bmp",
"marker/marker_780.bmp",
"marker/marker_792.bmp",
"marker/marker_796.bmp",
"marker/marker_799.bmp",
"marker/marker_808.bmp",
"marker/marker_810.bmp",
"marker/marker_818.bmp",
"marker/marker_823.bmp",
"marker/marker_82.bmp",
"marker/marker_842.bmp",
"marker/marker_851.bmp",
"marker/marker_855.bmp",
"marker/marker_859.bmp",
"marker/marker_862.bmp",
"marker/marker_863.bmp",
"marker/marker_866.bmp",
"marker/marker_884.bmp",
"marker/marker_887.bmp",
"marker/marker_897.bmp",
"marker/marker_901.bmp",
"marker/marker_905.bmp",
"marker/marker_906.bmp",
"marker/marker_918.bmp",
"marker/marker_928.bmp",
"marker/marker_931.bmp",
"marker/marker_932.bmp",
"marker/marker_937.bmp",
"marker/marker_939.bmp",
"marker/marker_941.bmp",
"marker/marker_943.bmp",
"marker/marker_949.bmp",
"marker/marker_954.bmp",
"marker/marker_95.bmp",
"marker/marker_964.bmp",
"marker/marker_968.bmp",
"marker/marker_976.bmp",
"marker/marker_97.bmp",
"marker/marker_984.bmp",
"marker/marker_989.bmp",
"marker/marker_997.bmp",
"marker/marker_99.bmp"};
	
int main(int argc,char*argv[])
{
  C256BitMap GPic,tmpPic;
  GPic.FormatF(3800, 2700*2);
  int i,j,t,k; 
  
  GPic.Clear();
  for(i=0;i<9;i++)
   for(j=0;j<18;j++) 
  {
	  int idx = rand()%641;
	  tmpPic.Load(mkfiles[idx]);
	  for(t=0 ;t<350;t++)
		  for(k=0;k<350;k++)
		  {
			    *get_pix_color(GPic,i*400+t+20,j*400+k)=
			    *get_pix_color(tmpPic,  t, k);
		  }
   }	  
  GPic.Save("gen5.bmp");   
  
  /*  GPic.Clear();
  for(i=0;i<9;i++)
   for(j=0;j<7;j++) 
  {
	  int idx = rand()%641;
	  tmpPic.Load(mkfiles[idx]);
	  for(t=0 ;t<350;t++)
		  for(k=0;k<350;k++)
		  {
			    *get_pix_color(GPic,i*400+t+20,j*400+k)=
			    *get_pix_color(tmpPic,  t, k);
		  }
   }	  
  GPic.Save("gen2.bmp"); 
  
    GPic.Clear();
  for(i=0;i<9;i++)
   for(j=0;j<7;j++) 
  {
	  int idx = rand()%641;
	  tmpPic.Load(mkfiles[idx]);
	  for(t=0 ;t<350;t++)
		  for(k=0;k<350;k++)
		  {
			    *get_pix_color(GPic,i*400+t+20,j*400+k)=
			    *get_pix_color(tmpPic,  t, k);
		  }
   }	  
  GPic.Save("gen3.bmp"); 
  
    GPic.Clear();
  for(i=0;i<9;i++)
   for(j=0;j<7;j++) 
  {
	  int idx = rand()%641;
	  tmpPic.Load(mkfiles[idx]);
	  for(t=0 ;t<350;t++)
		  for(k=0;k<350;k++)
		  {
			    *get_pix_color(GPic,i*400+t+20,j*400+k)=
			    *get_pix_color(tmpPic,  t, k);
		  }
   }	  
  GPic.Save("gen4.bmp"); */
  
  
  printf("hello world\n");
}