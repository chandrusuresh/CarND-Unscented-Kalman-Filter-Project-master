import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt

nis_file = ["../build/nis_stdAcc_0.200000_stdYawDD_0.100000_P_init_Ones.csv",
            "../build/nis_stdAcc_0.200000_stdYawDD_0.100000.csv",
            "../build/nis_stdAcc_0.200000_stdYawDD_0.200000.csv",
            "../build/nis_stdAcc_0.500000_stdYawDD_0.200000.csv"]
#radar_file = ["../build/RADAR_nis.csv",
#              "../build/RADAR_nis_stdAcc_0.200000_stdYawDD_0.200000.csv",
#              "../build/RADAR_nis_stdAcc_0.200000_stdYawDD_0.392699.csv",
#              "../build/RADAR_nis_stdAcc_0.200000_stdYawDD_0.100000.csv"]

data_len = 0
for file in nis_file:
    if os.path.exists(file):
        df = pd.read_csv(file, header=None, names = ["NIS"])
        if (len(df["NIS"]) > data_len):
            data_len = len(df["NIS"])
        plt.plot(df["NIS"])

plt.plot([7.815]*data_len)
plt.plot([5.991]*data_len)
plt.ylim((0,10))
plt.show()

#print(os.path.exists(laser_file))

