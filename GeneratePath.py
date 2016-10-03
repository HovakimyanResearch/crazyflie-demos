#!/usr/bin/env python
# GeneratePath.py by Kristian Sloth Lauszus

from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
from scipy.misc import comb
from math import sqrt, floor
import os
import time
import sys
import getopt


def bernstein_polynomial(n, t, k):
    """Return the Bernstein polynomial of n, k as a function of t."""
    return comb(n, k)*(1-t)**(n-k)*t**k


def bernstein_matrix(n, t):
    """Return a matrix containing the Bernstein base polynomials evaluated at elements of t."""
    return np.array([[bernstein_polynomial(n, i, k) for k in range(n + 1)] for i in t])


def path_length(x, y):
    """
    Return the length of a path given by its x,y-coordinates.
    This is only a good assumptions when the path has a lot of samples.
    """
    return sum([sqrt((x[i] - x[i - 1]) ** 2 + (y[i] - y[i - 1]) ** 2) for i in range(1, len(x))])


def print_usage(status=None):
    print('Usage:\n  GeneratePath.py -i <inputfile> -o <outputfile> [-p | --plot] [-w | --wait]')
    sys.exit(status)


def main(argv):
    # Pass input arguments
    input_file = ''
    output_file = ''
    draw_plot = 0
    wait = 0

    try:
        opts, args = getopt.getopt(argv, 'hi:o:pw', ['--plot', '--wait'])
    except getopt.GetoptError as err:
        print('Unknown option: -%s' % err.opt)
        print_usage(2)

    for opt, arg in opts:
        if opt == '-h':
            print_usage()
        if opt == '-i':
            input_file = arg
        elif opt == '-o':
            output_file = arg
        elif opt in ('-p', '--plot'):
            draw_plot = 1
        elif opt in ('-w', '--wait'):
            wait = 1

    # Make sure that input file exist and the output filename is not empty
    if not os.path.isfile(input_file):
        print('Input file not found')
        print_usage(2)
    elif not output_file:
        print('Not a valid output file name')
        print_usage(2)

    if wait:
        print('Waiting for "%s" to change' % input_file, end='')
        stamp = os.stat(input_file).st_mtime
        while stamp == os.stat(input_file).st_mtime:
            sys.stdout.flush()
            time.sleep(1)
            print('.', end='')
        print('') # New line
    print('Calculating Bezier path')

    data = np.genfromtxt(input_file, delimiter=',') # Load CSV file

    #t = data[:, 0]
    x = 2*(data[:, 1])
    y = 2*data[:, 2]
    if draw_plot:
        plt.plot(x, y, linestyle='--')

    # Bezier paths
    max_pts = 500
    bez_step = 0.001
    intervals = int(floor(len(x)/max_pts)) + 1
    time_length = int(round(1 / (bez_step * intervals)))

    bezier_x = np.empty([intervals*time_length])
    bezier_y = np.empty([intervals*time_length])

    for i in range(intervals):
        index = i * max_pts
        x_tmp = x[index:index + max_pts]
        y_tmp = y[index:index + max_pts]
        n = len(x_tmp)

        if i == 0:
            t = np.linspace(0, 1, time_length)
        else:
            t = np.linspace(bez_step * intervals, 1, time_length)

        B = bernstein_matrix(n - 1, t)

        out_index = i * time_length
        bezier_x[out_index:out_index + B.shape[0]] = np.dot(B, x_tmp)
        bezier_y[out_index:out_index + B.shape[0]] = np.dot(B, y_tmp)

    # Generate time-stamps
    v_max = 0.1
    t = np.empty(bezier_x.shape)
    for i in range(len(t)):
        length = path_length(bezier_x[:i+1], bezier_y[:i+1])
        t[i] = length / v_max

    np.savetxt(output_file, np.transpose([t, bezier_x, bezier_y]), delimiter=',', fmt='%.8f') # Save CSV file

    print('Bezier path written to: "%s"' % output_file)

    if draw_plot:
        plt.plot(bezier_x, bezier_y, 'r')
        ax = plt.gca()
        ax.set_aspect('equal')
        plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])
