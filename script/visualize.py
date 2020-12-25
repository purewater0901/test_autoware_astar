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
    ref_vel_data = pd.read_csv(path+"/result/reference_velocity.csv");
    astar_data = pd.read_csv(path+"/result/astar_result.csv")
    qp_data = pd.read_csv(path+"/result/qp_result.csv")

    fig = plt.figure(figsize=(14, 10))
    plt.ylim([-7.5, 17.5])
    plt.plot(qp_data['qp_position'], qp_data['qp_velocity'], label="qp_velocity", color="b")
    plt.plot(qp_data['qp_position'], qp_data['qp_acceleration'], label="qp_acceleration", color="red")
    #plt.plot(qp_data['qp_time'], qp_data['qp_jerk'], label="qp_jerk", color="orange")
    plt.plot(ref_vel_data["position"], ref_vel_data["original_velocity"],label="Original Velocity Limit")
    plt.plot(ref_vel_data["position"], ref_vel_data["filtered_velocity"],label="Filtered Velocity Limit")
    plt.plot(astar_data['astar_position'], astar_data['astar_acceleration'], label="astar_acceleration", color="black")
    plt.plot(astar_data['astar_position'], astar_data['astar_velocity'], label="astar_velocity", color="purple")
    plt.xlabel("s [m]", fontsize=15)
    plt.ylabel("velocity [m/s]", fontsize=15)
    plt.tick_params(labelsize=18)
    plt.legend(fontsize=18)
    plt.show()
