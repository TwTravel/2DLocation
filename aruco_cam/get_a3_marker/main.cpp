#include<stdlib.h>
#include<stdio.h>
#include <math.h>
#include "c24bitmap.h"
#include "c256bitmap.h"

char * mkfiles[999] ={
"gen/out001.bmp",
"gen/out002.bmp",
"gen/out003.bmp",
"gen/out004.bmp",
"gen/out005.bmp",
"gen/out006.bmp",
"gen/out007.bmp",
"gen/out008.bmp",
"gen/out009.bmp",
"gen/out010.bmp",
"gen/out011.bmp",
"gen/out012.bmp",
"gen/out013.bmp",
"gen/out014.bmp",
"gen/out015.bmp",
"gen/out016.bmp",
"gen/out017.bmp",
"gen/out018.bmp",
"gen/out019.bmp",
"gen/out020.bmp",
"gen/out021.bmp",
"gen/out022.bmp",
"gen/out023.bmp",
"gen/out024.bmp",
"gen/out025.bmp",
"gen/out026.bmp",
"gen/out027.bmp",
"gen/out028.bmp",
"gen/out029.bmp",
"gen/out030.bmp",
"gen/out031.bmp",
"gen/out032.bmp",
"gen/out033.bmp",
"gen/out034.bmp",
"gen/out035.bmp",
"gen/out036.bmp",
"gen/out037.bmp",
"gen/out038.bmp",
"gen/out039.bmp",
"gen/out040.bmp",
"gen/out041.bmp",
"gen/out042.bmp",
"gen/out043.bmp",
"gen/out044.bmp",
"gen/out045.bmp",
"gen/out046.bmp",
"gen/out047.bmp",
"gen/out048.bmp",
"gen/out049.bmp",
"gen/out050.bmp",
"gen/out051.bmp",
"gen/out052.bmp",
"gen/out053.bmp",
"gen/out054.bmp",
"gen/out055.bmp",
"gen/out056.bmp",
"gen/out057.bmp",
"gen/out058.bmp",
"gen/out059.bmp",
"gen/out060.bmp",
"gen/out061.bmp",
"gen/out062.bmp",
"gen/out063.bmp",
"gen/out064.bmp",
"gen/out065.bmp",
"gen/out066.bmp",
"gen/out067.bmp",
"gen/out068.bmp",
"gen/out069.bmp",
"gen/out070.bmp",
"gen/out071.bmp",
"gen/out072.bmp",
"gen/out073.bmp",
"gen/out074.bmp",
"gen/out075.bmp",
"gen/out076.bmp",
"gen/out077.bmp",
"gen/out078.bmp",
"gen/out079.bmp",
"gen/out080.bmp",
"gen/out081.bmp",
"gen/out082.bmp",
"gen/out083.bmp",
"gen/out084.bmp",
"gen/out085.bmp",
"gen/out086.bmp",
"gen/out087.bmp",
"gen/out088.bmp",
"gen/out089.bmp",
"gen/out090.bmp",
"gen/out091.bmp",
"gen/out092.bmp",
"gen/out093.bmp",
"gen/out094.bmp",
"gen/out095.bmp",
"gen/out096.bmp",
"gen/out097.bmp",
"gen/out098.bmp",
"gen/out099.bmp",
"gen/out100.bmp",
"gen/out101.bmp",
"gen/out102.bmp",
"gen/out103.bmp",
"gen/out104.bmp",
"gen/out105.bmp",
"gen/out106.bmp",
"gen/out107.bmp",
"gen/out108.bmp",
"gen/out109.bmp",
"gen/out110.bmp",
"gen/out111.bmp",
"gen/out112.bmp",
"gen/out113.bmp",
"gen/out114.bmp",
"gen/out115.bmp",
"gen/out116.bmp",
"gen/out117.bmp",
"gen/out118.bmp",
"gen/out119.bmp",
"gen/out120.bmp",
"gen/out121.bmp",
"gen/out122.bmp",
"gen/out123.bmp",
"gen/out124.bmp",
"gen/out125.bmp",
"gen/out126.bmp",
"gen/out127.bmp",
"gen/out128.bmp",
"gen/out129.bmp",
"gen/out130.bmp",
"gen/out131.bmp",
"gen/out132.bmp",
"gen/out133.bmp",
"gen/out134.bmp",
"gen/out135.bmp",
"gen/out136.bmp",
"gen/out137.bmp",
"gen/out138.bmp",
"gen/out139.bmp",
"gen/out140.bmp",
"gen/out141.bmp",
"gen/out142.bmp",
"gen/out143.bmp",
"gen/out144.bmp",
"gen/out145.bmp",
"gen/out146.bmp",
"gen/out147.bmp",
"gen/out148.bmp",
"gen/out149.bmp",
"gen/out150.bmp",
"gen/out151.bmp",
"gen/out152.bmp",
"gen/out153.bmp",
"gen/out154.bmp",
"gen/out155.bmp",
"gen/out156.bmp",
"gen/out157.bmp",
"gen/out158.bmp",
"gen/out159.bmp",
"gen/out160.bmp",
"gen/out161.bmp",
"gen/out162.bmp",
"gen/out163.bmp",
"gen/out164.bmp",
"gen/out165.bmp",
"gen/out166.bmp",
"gen/out167.bmp",
"gen/out168.bmp",
"gen/out169.bmp",
"gen/out170.bmp",
"gen/out171.bmp",
"gen/out172.bmp",
"gen/out173.bmp",
"gen/out174.bmp",
"gen/out175.bmp",
"gen/out176.bmp",
"gen/out177.bmp",
"gen/out178.bmp",
"gen/out179.bmp",
"gen/out180.bmp",
"gen/out181.bmp",
"gen/out182.bmp",
"gen/out183.bmp",
"gen/out184.bmp",
"gen/out185.bmp",
"gen/out186.bmp",
"gen/out187.bmp",
"gen/out188.bmp",
"gen/out189.bmp",
"gen/out190.bmp",
"gen/out191.bmp",
"gen/out192.bmp",
"gen/out193.bmp",
"gen/out194.bmp",
"gen/out195.bmp",
"gen/out196.bmp",
"gen/out197.bmp",
"gen/out198.bmp",
"gen/out199.bmp",
"gen/out200.bmp",
"gen/out201.bmp",
"gen/out202.bmp",
"gen/out203.bmp",
"gen/out204.bmp",
"gen/out205.bmp",
"gen/out206.bmp",
"gen/out207.bmp",
"gen/out208.bmp",
"gen/out209.bmp",
"gen/out210.bmp",
"gen/out211.bmp",
"gen/out212.bmp",
"gen/out213.bmp",
"gen/out214.bmp",
"gen/out215.bmp",
"gen/out216.bmp",
"gen/out217.bmp",
"gen/out218.bmp",
"gen/out219.bmp",
"gen/out220.bmp",
"gen/out221.bmp",
"gen/out222.bmp",
"gen/out223.bmp",
"gen/out224.bmp",
"gen/out225.bmp",
"gen/out226.bmp",
"gen/out227.bmp",
"gen/out228.bmp",
"gen/out229.bmp",
"gen/out230.bmp",
"gen/out231.bmp",
"gen/out232.bmp",
"gen/out233.bmp",
"gen/out234.bmp",
"gen/out235.bmp",
"gen/out236.bmp",
"gen/out237.bmp",
"gen/out238.bmp",
"gen/out239.bmp",
"gen/out240.bmp",
"gen/out241.bmp",
"gen/out242.bmp",
"gen/out243.bmp",
"gen/out244.bmp",
"gen/out245.bmp",
"gen/out246.bmp",
"gen/out247.bmp",
"gen/out248.bmp",
"gen/out249.bmp",
"gen/out250.bmp",
"gen/out251.bmp",
"gen/out252.bmp",
"gen/out253.bmp",
"gen/out254.bmp",
"gen/out255.bmp",
"gen/out256.bmp",
"gen/out257.bmp",
"gen/out258.bmp",
"gen/out259.bmp",
"gen/out260.bmp",
"gen/out261.bmp",
"gen/out262.bmp",
"gen/out263.bmp",
"gen/out264.bmp",
"gen/out265.bmp",
"gen/out266.bmp",
"gen/out267.bmp",
"gen/out268.bmp",
"gen/out269.bmp",
"gen/out270.bmp",
"gen/out271.bmp",
"gen/out272.bmp",
"gen/out273.bmp",
"gen/out274.bmp",
"gen/out275.bmp",
"gen/out276.bmp",
"gen/out277.bmp",
"gen/out278.bmp",
"gen/out279.bmp",
"gen/out280.bmp",
"gen/out281.bmp",
"gen/out282.bmp",
"gen/out283.bmp",
"gen/out284.bmp",
"gen/out285.bmp",
"gen/out286.bmp",
"gen/out287.bmp",
"gen/out288.bmp",
"gen/out289.bmp",
"gen/out290.bmp",
"gen/out291.bmp",
"gen/out292.bmp",
"gen/out293.bmp",
"gen/out294.bmp",
"gen/out295.bmp",
"gen/out296.bmp",
"gen/out297.bmp",
"gen/out298.bmp",
"gen/out299.bmp",
"gen/out300.bmp",
"gen/out301.bmp",
"gen/out302.bmp",
"gen/out303.bmp",
"gen/out304.bmp",
"gen/out305.bmp",
"gen/out306.bmp",
"gen/out307.bmp",
"gen/out308.bmp",
"gen/out309.bmp",
"gen/out310.bmp",
"gen/out311.bmp",
"gen/out312.bmp",
"gen/out313.bmp",
"gen/out314.bmp",
"gen/out315.bmp",
"gen/out316.bmp",
"gen/out317.bmp",
"gen/out318.bmp",
"gen/out319.bmp",
"gen/out320.bmp",
"gen/out321.bmp",
"gen/out322.bmp",
"gen/out323.bmp",
"gen/out324.bmp",
"gen/out325.bmp",
"gen/out326.bmp",
"gen/out327.bmp",
"gen/out328.bmp",
"gen/out329.bmp",
"gen/out330.bmp",
"gen/out331.bmp",
"gen/out332.bmp",
"gen/out333.bmp",
"gen/out334.bmp",
"gen/out335.bmp",
"gen/out336.bmp",
"gen/out337.bmp",
"gen/out338.bmp",
"gen/out339.bmp",
"gen/out340.bmp",
"gen/out341.bmp",
"gen/out342.bmp",
"gen/out343.bmp",
"gen/out344.bmp",
"gen/out345.bmp",
"gen/out346.bmp",
"gen/out347.bmp",
"gen/out348.bmp",
"gen/out349.bmp",
"gen/out200.bmp",
"gen/out351.bmp",
"gen/out352.bmp",
"gen/out353.bmp",
"gen/out354.bmp",
"gen/out355.bmp",
"gen/out356.bmp",
"gen/out357.bmp",
"gen/out358.bmp",
"gen/out359.bmp",
"gen/out360.bmp",
"gen/out361.bmp",
"gen/out362.bmp",
"gen/out363.bmp",
"gen/out364.bmp",
"gen/out365.bmp",
"gen/out366.bmp",
"gen/out367.bmp",
"gen/out368.bmp",
"gen/out369.bmp",
"gen/out370.bmp",
"gen/out371.bmp",
"gen/out372.bmp",
"gen/out373.bmp",
"gen/out374.bmp",
"gen/out375.bmp",
"gen/out376.bmp",
"gen/out377.bmp",
"gen/out378.bmp",
"gen/out379.bmp",
"gen/out380.bmp",
"gen/out381.bmp",
"gen/out382.bmp",
"gen/out383.bmp",
"gen/out384.bmp",
"gen/out385.bmp",
"gen/out386.bmp",
"gen/out387.bmp",
"gen/out388.bmp",
"gen/out389.bmp",
"gen/out390.bmp",
"gen/out391.bmp",
"gen/out392.bmp",
"gen/out393.bmp",
"gen/out394.bmp",
"gen/out395.bmp",
"gen/out396.bmp",
"gen/out397.bmp",
"gen/out398.bmp",
"gen/out399.bmp",
"gen/out400.bmp",
"gen/out401.bmp",
"gen/out402.bmp",
"gen/out403.bmp",
"gen/out404.bmp",
"gen/out405.bmp",
"gen/out406.bmp",
"gen/out407.bmp",
"gen/out408.bmp",
"gen/out409.bmp",
"gen/out410.bmp",
"gen/out411.bmp",
"gen/out412.bmp",
"gen/out413.bmp",
"gen/out414.bmp",
"gen/out415.bmp",
"gen/out416.bmp",
"gen/out417.bmp",
"gen/out418.bmp",
"gen/out419.bmp",
"gen/out420.bmp",
"gen/out421.bmp",
"gen/out422.bmp",
"gen/out423.bmp",
"gen/out424.bmp",
"gen/out425.bmp",
"gen/out426.bmp",
"gen/out427.bmp",
"gen/out428.bmp",
"gen/out429.bmp",
"gen/out430.bmp",
"gen/out431.bmp",
"gen/out432.bmp",
"gen/out433.bmp",
"gen/out434.bmp",
"gen/out435.bmp",
"gen/out436.bmp",
"gen/out437.bmp",
"gen/out438.bmp",
"gen/out439.bmp",
"gen/out440.bmp",
"gen/out441.bmp",
"gen/out442.bmp",
"gen/out443.bmp",
"gen/out444.bmp",
"gen/out445.bmp",
"gen/out446.bmp",
"gen/out447.bmp",
"gen/out448.bmp",
"gen/out449.bmp",
"gen/out250.bmp",
"gen/out451.bmp",
"gen/out452.bmp",
"gen/out453.bmp",
"gen/out454.bmp",
"gen/out455.bmp",
"gen/out456.bmp",
"gen/out457.bmp",
"gen/out458.bmp",
"gen/out459.bmp",
"gen/out460.bmp",
"gen/out461.bmp",
"gen/out462.bmp",
"gen/out463.bmp",
"gen/out464.bmp",
"gen/out465.bmp",
"gen/out466.bmp",
"gen/out467.bmp",
"gen/out468.bmp",
"gen/out469.bmp",
"gen/out470.bmp",
"gen/out471.bmp",
"gen/out472.bmp",
"gen/out473.bmp",
"gen/out474.bmp",
"gen/out475.bmp",
"gen/out476.bmp",
"gen/out477.bmp",
"gen/out478.bmp",
"gen/out479.bmp",
"gen/out480.bmp",
"gen/out481.bmp",
"gen/out482.bmp",
"gen/out483.bmp",
"gen/out484.bmp",
"gen/out485.bmp",
"gen/out486.bmp",
"gen/out487.bmp",
"gen/out488.bmp",
"gen/out489.bmp",
"gen/out490.bmp",
"gen/out491.bmp",
"gen/out492.bmp",
"gen/out493.bmp",
"gen/out494.bmp",
"gen/out495.bmp",
"gen/out496.bmp",
"gen/out497.bmp",
"gen/out498.bmp",
"gen/out499.bmp",
"gen/out500.bmp",
"gen/out501.bmp",
"gen/out502.bmp",
"gen/out503.bmp",
"gen/out504.bmp",
"gen/out505.bmp",
"gen/out506.bmp",
"gen/out507.bmp",
"gen/out508.bmp",
"gen/out509.bmp",
"gen/out510.bmp",
"gen/out511.bmp",
"gen/out512.bmp",
"gen/out513.bmp",
"gen/out514.bmp",
"gen/out515.bmp",
"gen/out516.bmp",
"gen/out517.bmp",
"gen/out518.bmp",
"gen/out519.bmp",
"gen/out520.bmp",
"gen/out521.bmp",
"gen/out522.bmp",
"gen/out523.bmp",
"gen/out524.bmp",
"gen/out525.bmp",
"gen/out526.bmp",
"gen/out527.bmp",
"gen/out528.bmp",
"gen/out529.bmp",
"gen/out530.bmp",
"gen/out531.bmp",
"gen/out532.bmp",
"gen/out533.bmp",
"gen/out534.bmp",
"gen/out535.bmp",
"gen/out536.bmp",
"gen/out537.bmp",
"gen/out538.bmp",
"gen/out539.bmp",
"gen/out540.bmp",
"gen/out541.bmp",
"gen/out542.bmp",
"gen/out543.bmp",
"gen/out544.bmp",
"gen/out545.bmp",
"gen/out546.bmp",
"gen/out547.bmp",
"gen/out548.bmp",
"gen/out549.bmp",
"gen/out550.bmp",
"gen/out551.bmp",
"gen/out552.bmp",
"gen/out553.bmp",
"gen/out554.bmp",
"gen/out555.bmp",
"gen/out556.bmp",
"gen/out557.bmp",
"gen/out558.bmp",
"gen/out559.bmp",
"gen/out560.bmp",
"gen/out561.bmp",
"gen/out562.bmp",
"gen/out563.bmp",
"gen/out564.bmp",
"gen/out565.bmp",
"gen/out566.bmp",
"gen/out567.bmp",
"gen/out568.bmp",
"gen/out569.bmp",
"gen/out570.bmp",
"gen/out571.bmp",
"gen/out572.bmp",
"gen/out573.bmp",
"gen/out574.bmp",
"gen/out575.bmp",
"gen/out576.bmp",
"gen/out577.bmp",
"gen/out578.bmp",
"gen/out579.bmp",
"gen/out580.bmp",
"gen/out581.bmp",
"gen/out582.bmp",
"gen/out583.bmp",
"gen/out584.bmp",
"gen/out585.bmp",
"gen/out586.bmp",
"gen/out587.bmp",
"gen/out588.bmp",
"gen/out589.bmp",
"gen/out590.bmp",
"gen/out591.bmp",
"gen/out592.bmp",
"gen/out593.bmp",
"gen/out594.bmp",
"gen/out595.bmp",
"gen/out596.bmp",
"gen/out597.bmp",
"gen/out598.bmp",
"gen/out599.bmp",
"gen/out600.bmp",
"gen/out601.bmp",
"gen/out602.bmp",
"gen/out603.bmp",
"gen/out604.bmp",
"gen/out605.bmp",
"gen/out606.bmp",
"gen/out607.bmp",
"gen/out608.bmp",
"gen/out609.bmp",
"gen/out610.bmp",
"gen/out611.bmp",
"gen/out612.bmp",
"gen/out613.bmp",
"gen/out614.bmp",
"gen/out615.bmp",
"gen/out616.bmp",
"gen/out617.bmp",
"gen/out618.bmp",
"gen/out619.bmp",
"gen/out620.bmp",
"gen/out621.bmp",
"gen/out622.bmp",
"gen/out623.bmp",
"gen/out624.bmp",
"gen/out625.bmp",
"gen/out626.bmp",
"gen/out627.bmp",
"gen/out628.bmp",
"gen/out629.bmp",
"gen/out630.bmp",
"gen/out631.bmp",
"gen/out632.bmp",
"gen/out633.bmp",
"gen/out634.bmp",
"gen/out635.bmp",
"gen/out636.bmp",
"gen/out637.bmp",
"gen/out638.bmp",
"gen/out639.bmp",
"gen/out640.bmp",
"gen/out641.bmp",
"gen/out642.bmp",
"gen/out643.bmp",
"gen/out644.bmp",
"gen/out645.bmp",
"gen/out646.bmp",
"gen/out647.bmp",
"gen/out648.bmp",
"gen/out649.bmp",
"gen/out650.bmp",
"gen/out651.bmp",
"gen/out652.bmp",
"gen/out653.bmp",
"gen/out654.bmp",
"gen/out655.bmp",
"gen/out656.bmp",
"gen/out657.bmp",
"gen/out658.bmp",
"gen/out659.bmp",
"gen/out660.bmp",
"gen/out661.bmp",
"gen/out662.bmp",
"gen/out663.bmp",
"gen/out664.bmp",
"gen/out665.bmp",
"gen/out666.bmp",
"gen/out667.bmp",
"gen/out668.bmp",
"gen/out669.bmp",
"gen/out670.bmp",
"gen/out671.bmp",
"gen/out672.bmp",
"gen/out673.bmp",
"gen/out674.bmp",
"gen/out675.bmp",
"gen/out676.bmp",
"gen/out677.bmp",
"gen/out678.bmp",
"gen/out679.bmp",
"gen/out680.bmp",
"gen/out681.bmp",
"gen/out682.bmp",
"gen/out683.bmp",
"gen/out684.bmp",
"gen/out685.bmp",
"gen/out686.bmp",
"gen/out687.bmp",
"gen/out688.bmp",
"gen/out689.bmp",
"gen/out690.bmp",
"gen/out691.bmp",
"gen/out692.bmp",
"gen/out693.bmp",
"gen/out694.bmp",
"gen/out695.bmp",
"gen/out696.bmp",
"gen/out697.bmp",
"gen/out698.bmp",
"gen/out699.bmp",
"gen/out700.bmp",
"gen/out701.bmp",
"gen/out702.bmp",
"gen/out703.bmp",
"gen/out704.bmp",
"gen/out705.bmp",
"gen/out706.bmp",
"gen/out707.bmp",
"gen/out708.bmp",
"gen/out709.bmp",
"gen/out710.bmp",
"gen/out711.bmp",
"gen/out712.bmp",
"gen/out713.bmp",
"gen/out714.bmp",
"gen/out715.bmp",
"gen/out716.bmp",
"gen/out717.bmp",
"gen/out718.bmp",
"gen/out719.bmp",
"gen/out720.bmp",
"gen/out721.bmp",
"gen/out722.bmp",
"gen/out723.bmp",
"gen/out724.bmp",
"gen/out725.bmp",
"gen/out726.bmp",
"gen/out727.bmp",
"gen/out728.bmp",
"gen/out729.bmp",
"gen/out730.bmp",
"gen/out731.bmp",
"gen/out732.bmp",
"gen/out733.bmp",
"gen/out734.bmp",
"gen/out735.bmp",
"gen/out736.bmp",
"gen/out737.bmp",
"gen/out738.bmp",
"gen/out739.bmp",
"gen/out740.bmp",
"gen/out741.bmp",
"gen/out742.bmp",
"gen/out743.bmp",
"gen/out744.bmp",
"gen/out745.bmp",
"gen/out746.bmp",
"gen/out747.bmp",
"gen/out748.bmp",
"gen/out749.bmp",
"gen/out750.bmp",
"gen/out751.bmp",
"gen/out752.bmp",
"gen/out753.bmp",
"gen/out754.bmp",
"gen/out755.bmp",
"gen/out756.bmp",
"gen/out757.bmp",
"gen/out758.bmp",
"gen/out759.bmp",
"gen/out760.bmp",
"gen/out761.bmp",
"gen/out762.bmp",
"gen/out763.bmp",
"gen/out764.bmp",
"gen/out765.bmp",
"gen/out766.bmp",
"gen/out767.bmp",
"gen/out768.bmp",
"gen/out769.bmp",
"gen/out770.bmp",
"gen/out771.bmp",
"gen/out772.bmp",
"gen/out773.bmp",
"gen/out774.bmp",
"gen/out775.bmp",
"gen/out776.bmp",
"gen/out777.bmp",
"gen/out778.bmp",
"gen/out779.bmp",
"gen/out780.bmp",
"gen/out781.bmp",
"gen/out782.bmp",
"gen/out783.bmp",
"gen/out784.bmp",
"gen/out785.bmp",
"gen/out786.bmp",
"gen/out787.bmp",
"gen/out788.bmp",
"gen/out789.bmp",
"gen/out790.bmp",
"gen/out791.bmp",
"gen/out792.bmp",
"gen/out793.bmp",
"gen/out794.bmp",
"gen/out795.bmp",
"gen/out796.bmp",
"gen/out797.bmp",
"gen/out798.bmp",
"gen/out799.bmp",
"gen/out800.bmp",
"gen/out801.bmp",
"gen/out802.bmp",
"gen/out803.bmp",
"gen/out804.bmp",
"gen/out805.bmp",
"gen/out806.bmp",
"gen/out807.bmp",
"gen/out808.bmp",
"gen/out809.bmp",
"gen/out810.bmp",
"gen/out811.bmp",
"gen/out812.bmp",
"gen/out813.bmp",
"gen/out814.bmp",
"gen/out815.bmp",
"gen/out816.bmp",
"gen/out817.bmp",
"gen/out818.bmp",
"gen/out819.bmp",
"gen/out820.bmp",
"gen/out821.bmp",
"gen/out822.bmp",
"gen/out823.bmp",
"gen/out824.bmp",
"gen/out825.bmp",
"gen/out826.bmp",
"gen/out827.bmp",
"gen/out828.bmp",
"gen/out829.bmp",
"gen/out830.bmp",
"gen/out831.bmp",
"gen/out832.bmp",
"gen/out833.bmp",
"gen/out834.bmp",
"gen/out835.bmp",
"gen/out836.bmp",
"gen/out837.bmp",
"gen/out838.bmp",
"gen/out839.bmp",
"gen/out840.bmp",
"gen/out841.bmp",
"gen/out842.bmp",
"gen/out843.bmp",
"gen/out844.bmp",
"gen/out845.bmp",
"gen/out846.bmp",
"gen/out847.bmp",
"gen/out848.bmp",
"gen/out849.bmp",
"gen/out850.bmp",
"gen/out851.bmp",
"gen/out852.bmp",
"gen/out853.bmp",
"gen/out854.bmp",
"gen/out855.bmp",
"gen/out856.bmp",
"gen/out857.bmp",
"gen/out858.bmp",
"gen/out859.bmp",
"gen/out860.bmp",
"gen/out861.bmp",
"gen/out862.bmp",
"gen/out863.bmp",
"gen/out864.bmp",
"gen/out865.bmp",
"gen/out866.bmp",
"gen/out867.bmp",
"gen/out868.bmp",
"gen/out869.bmp",
"gen/out870.bmp",
"gen/out871.bmp",
"gen/out872.bmp",
"gen/out873.bmp",
"gen/out874.bmp",
"gen/out875.bmp",
"gen/out876.bmp",
"gen/out877.bmp",
"gen/out878.bmp",
"gen/out879.bmp",
"gen/out880.bmp",
"gen/out881.bmp",
"gen/out882.bmp",
"gen/out883.bmp",
"gen/out884.bmp",
"gen/out885.bmp",
"gen/out886.bmp",
"gen/out887.bmp",
"gen/out888.bmp",
"gen/out889.bmp",
"gen/out890.bmp",
"gen/out891.bmp",
"gen/out892.bmp",
"gen/out893.bmp",
"gen/out894.bmp",
"gen/out895.bmp",
"gen/out896.bmp",
"gen/out897.bmp",
"gen/out898.bmp",
"gen/out899.bmp",
"gen/out900.bmp",
"gen/out901.bmp",
"gen/out902.bmp",
"gen/out903.bmp",
"gen/out904.bmp",
"gen/out905.bmp",
"gen/out906.bmp",
"gen/out907.bmp",
"gen/out908.bmp",
"gen/out909.bmp",
"gen/out910.bmp",
"gen/out911.bmp",
"gen/out912.bmp",
"gen/out913.bmp",
"gen/out914.bmp",
"gen/out915.bmp",
"gen/out916.bmp",
"gen/out917.bmp",
"gen/out918.bmp",
"gen/out919.bmp",
"gen/out920.bmp",
"gen/out921.bmp",
"gen/out922.bmp",
"gen/out923.bmp",
"gen/out924.bmp",
"gen/out925.bmp",
"gen/out926.bmp",
"gen/out927.bmp",
"gen/out928.bmp",
"gen/out929.bmp",
"gen/out930.bmp",
"gen/out931.bmp",
"gen/out932.bmp",
"gen/out933.bmp",
"gen/out934.bmp",
"gen/out935.bmp",
"gen/out936.bmp",
"gen/out937.bmp",
"gen/out938.bmp",
"gen/out939.bmp",
"gen/out940.bmp",
"gen/out941.bmp",
"gen/out942.bmp",
"gen/out943.bmp",
"gen/out944.bmp",
"gen/out945.bmp",
"gen/out946.bmp",
"gen/out947.bmp",
"gen/out948.bmp",
"gen/out949.bmp",
"gen/out950.bmp",
"gen/out951.bmp",
"gen/out952.bmp",
"gen/out953.bmp",
"gen/out954.bmp",
"gen/out955.bmp",
"gen/out956.bmp",
"gen/out957.bmp",
"gen/out958.bmp",
"gen/out959.bmp",
"gen/out960.bmp",
"gen/out961.bmp",
"gen/out962.bmp",
"gen/out963.bmp",
"gen/out964.bmp",
"gen/out965.bmp",
"gen/out966.bmp",
"gen/out967.bmp",
"gen/out968.bmp",
"gen/out969.bmp",
"gen/out970.bmp",
"gen/out971.bmp",
"gen/out972.bmp",
"gen/out973.bmp",
"gen/out974.bmp",
"gen/out975.bmp",
"gen/out976.bmp",
"gen/out977.bmp",
"gen/out978.bmp",
"gen/out979.bmp",
"gen/out980.bmp",
"gen/out981.bmp",
"gen/out982.bmp",
"gen/out983.bmp",
"gen/out984.bmp",
"gen/out985.bmp",
"gen/out986.bmp",
"gen/out987.bmp",
"gen/out988.bmp",
"gen/out989.bmp",
"gen/out990.bmp",
"gen/out991.bmp",
"gen/out992.bmp",
"gen/out993.bmp",
"gen/out994.bmp",
"gen/out995.bmp",
"gen/out996.bmp",
"gen/out997.bmp",
"gen/out998.bmp",
"gen/out999.bmp"};
	
int main(int argc,char*argv[])
{
  C256BitMap GPic,tmpPic;
  GPic.FormatF(2000, 1350);
  int i,j,t,k; 
  
  GPic.Clear();
  for(i=0;i<9;i++)
   for(j=0;j<7;j++) 
  {
	  int idx = rand()%641;
	  tmpPic.Load(mkfiles[idx]);
	  for(t=0 ;t<200;t++)
		  for(k=0;k<200;k++)
		  {
			    *get_pix_color(GPic,i*250+t+20,j*250+k+40)=
			    *get_pix_color(tmpPic,  t, k);
		  }
   }	  
  GPic.Save("gen1.bmp");   
  
    GPic.Clear();
  for(i=0;i<9;i++)
   for(j=0;j<7;j++) 
  {
	  int idx = rand()%641;
	  tmpPic.Load(mkfiles[idx]);
	  for(t=0 ;t<200;t++)
		  for(k=0;k<200;k++)
		  {
			    *get_pix_color(GPic,i*250+t+20,j*250+k+40)=
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
	  for(t=0 ;t<200;t++)
		  for(k=0;k<200;k++)
		  {
			    *get_pix_color(GPic,i*250+t+20,j*250+k+40)=
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
	  for(t=0 ;t<200;t++)
		  for(k=0;k<200;k++)
		  {
			    *get_pix_color(GPic,i*250+t+20,j*250+k+40)=
			    *get_pix_color(tmpPic,  t, k);
		  }
   }	  
  GPic.Save("gen4.bmp"); 
  
  
  printf("hello world\n");
}