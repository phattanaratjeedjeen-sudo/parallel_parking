import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

Ex=10
Ey=9.5
Einit_x=15
Einit_y=12
a=2.9
steer_angle_deg = 30
Remin =math.tan(a/math.radians(steer_angle_deg)) 
clx=Ex
cly=Ey+Remin
init_yaw = 0
dclEinit=math.sqrt((clx-Einit_x)**2+(cly-Einit_y)**2)
alpha = math.atan2(Einit_y-cly,Einit_x-clx)+math.pi/2+init_yaw
Reinitr = (dclEinit**2-Remin**2)/(2*Remin+2*(dclEinit*math.cos(alpha)))
dcrcl = Remin+Reinitr
Crx = Einit_x+Reinitr*math.sin(init_yaw)
Cry = Einit_y-Reinitr*math.cos(init_yaw)
z0= cly-Cry
z1 = -clx+Crx
thetasteer =math.pi- math.atan2(a,Reinitr)
print("steer angle (degrees): ", math.degrees(thetasteer))
theta = math.pi-math.atan2(z1,z0)
print(math.degrees(theta))