fh = open('pos.txt')
#for line in fh.readlines():
line = fh.readlines()
str  = line[0].split(',');
xx = float(str[1])
yy = float(str[2])
print(xx,yy)
   #yy = float(str[3])
   #print(xx,yy)