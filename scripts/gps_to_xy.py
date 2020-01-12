import utm,math

a=float(input("lat1:"))
b=float(input("lon1:"))
c=float(input("lat2:"))
d=float(input("lon2:"))
q,w,e,r= utm.from_latlon(a,b)
t,y,u,i= utm.from_latlon(c,d)

print(q,w ,"to" ,t,y)
print(t-q,y-w)
print(math.sqrt((t-q)**2+(y-w)**2))