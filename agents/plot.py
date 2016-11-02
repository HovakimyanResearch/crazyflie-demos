#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    data1 = np.genfromtxt('agent1.csv', delimiter=',') # Load CSV files
    data2 = np.genfromtxt('agent2.csv', delimiter=',')
    data3 = np.genfromtxt('agent3.csv', delimiter=',')

    plt.plot(data1[1, :], data1[2, :], label='agent1', color='blue')
    plt.plot(data2[1, :], data2[2, :], label='agent2', color='green')
    plt.plot(data3[1, :], data3[2, :], label='agent3', color='red')
    plt.legend(prop={'size': 10})
    ax = plt.gca()
    ax.set_aspect('equal')
    plt.savefig('agents_plot.pdf', bbox_inches='tight')
    #plt.show()
