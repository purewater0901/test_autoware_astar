import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import matplotlib.patches as pat
from scipy import stats
import platform
import os

if __name__ == '__main__':
    os.chdir('../')
    path = os.getcwd()
    data = pd.read_csv(path+"/result/output.csv")

    fig = plt.figure(figsize=(14, 10))
    plt.ylim([-1.5, 1.0])
    #plt.plot(data['qp_time'], data['qp_velocity'], label="qp_velocity", color="b)
    plt.plot(data['qp_time'], data['qp_acceleration'], label="qp_acceleration", color="red")
    plt.plot(data['qp_time'], data['qp_jerk'], label="qp_jerk", color="orange")
    plt.legend()
    plt.show()
