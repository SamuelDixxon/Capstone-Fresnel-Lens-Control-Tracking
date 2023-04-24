import os 
import matplotlib.pyplot as plt

sup = []
out = []
cur_dr = []
power = []

with open("Downloads/tst.csv", "r") as rF:
    a = rF.readline()
    b = rF.readlines()
    for eachLine in b:
        parts = eachLine.strip("\n").split(",")
        for i, a in enumerate(parts):
            if i == 0:
                sup.append(a)
            elif i == 1:
                out.append(a)
            elif i == 2:
                cur_dr.append(a)
            else:
                power.append(a)

plt.style.use("seaborn")
plt.title("Supply vs. Output Voltage")
plt.xlabel("Supply (V)")
plt.ylabel("Regulated Voltage (V)")
plt.plot(sup, out)       
    
    
    
