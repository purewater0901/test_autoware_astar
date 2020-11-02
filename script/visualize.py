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
    astar_data = pd.read_csv(path+"/result/astar_result.csv")
    qp_data = pd.read_csv(path+"/result/qp_result.csv")

    fig = plt.figure(figsize=(14, 10))
    plt.ylim([-1.5, 13.5])
    plt.plot(qp_data['qp_time'], qp_data['qp_velocity'], label="qp_velocity", color="b")
    plt.plot(qp_data['qp_time'], qp_data['qp_acceleration'], label="qp_acceleration", color="red")
    plt.plot(qp_data['qp_time'], qp_data['qp_jerk'], label="qp_jerk", color="orange")
    plt.plot(astar_data['astar_time'], astar_data['astar_acceleration'], label="astar_acceleration", color="black")
    plt.plot(astar_data['astar_time'], astar_data['astar_velocity'], label="astar_velocity", color="pink")
    plt.legend()
    plt.show()
