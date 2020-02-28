
import matplotlib.pyplot as plt
import numpy as np

kp=3e-5
kn=2e-5
np=[]
nn=[]
tp=[]
tn=[]

for i in range(1000):
    np.append(i)
    nn.append(-i)
    tp.append(kp*i*i)
    tn.append(kn*i*i)

plt.figure(1, figsize=(8, 6))
plt.plot(np, tp, lw=2)
plt.plot(nn, tn, lw=2)
plt.ylabel('thrust (N)')
plt.xlabel('n (rpm)')
plt.savefig('C:\\Users\\scar1et\\Desktop\\thruster.png', dpi=300)
plt.show()
