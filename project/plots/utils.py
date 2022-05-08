#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

# To read text files and plot them

# Get data from txt file, and return array
def getData(dataDir):
    data = np.loadtxt(dataDir, skiprows=0)
    time = np.arange(np.shape(data)[0])
    time = time/100
    return time, data

def autoscale_axes(ax,margin=0.1):

    def getYMinMax(line):
        yd = line.get_ydata()
        h = np.abs(np.max(yd) - np.min(yd))
        bot = np.min(yd)
        top = np.max(yd)
        return bot,top,h

    def getXMinMax(line):
        xd = line.get_xdata()
        w = np.abs(np.max(xd) - np.min(xd))
        left = np.min(xd)
        right = np.max(xd)
        return left,right,w

    lines = ax.get_lines()
    bot, top, h = np.inf, -np.inf, 0
    l, r, w = np.inf, -np.inf, 0

    for line in lines:
        new_bot, new_top, new_h = getYMinMax(line)
        new_l, new_r, new_w = getXMinMax(line)
        if new_bot < bot: bot = new_bot
        if new_top > top: top = new_top
        if new_h > h: h = new_h
        if new_l < l: l = new_l
        if new_r > r: r = new_r
        if new_w > w: w = new_w

    ax.set_ylim(bot-margin*h,top+margin*h)
    ax.set_xlim(l-margin/4*w,r+margin/4*w)

# Plot x, y, z, 
def plotXYZ(namePlots, data, desiredData, time, data_label, data_des_label):
    
    namePlots[0].plot(time, data[:,0], 'r', label=data_label[0])
    if type(desiredData) is np.ndarray:
        namePlots[0].plot(time, desiredData[:,0], 'r--', label=data_des_label[0])
    autoscale_axes(namePlots[0])
    # namePlots[0].autoscale(enable=True, axis='both', tight=False)
    lgd = namePlots[0].legend(loc=(0.8,0.5),frameon=True)

    namePlots[1].plot(time, data[:,1], 'g', label=data_label[1])
    if type(desiredData) is np.ndarray:
        namePlots[1].plot(time, desiredData[:,1], 'g--', label=data_des_label[1])
    autoscale_axes(namePlots[1])
    lgd = namePlots[1].legend(loc=(0.8,0.5),frameon=True)

    namePlots[2].plot(time, data[:,2], 'b', label=data_label[2])
    if type(desiredData) is np.ndarray:
        namePlots[2].plot(time, desiredData[:,2], 'b--', label=data_des_label[2])
    autoscale_axes(namePlots[2])
    lgd = namePlots[2].legend(loc=(0.8,0.5),frameon=True)


def simpleSubplot(namePlot, data, time, data_label, lineType, legendLoc=(0.8,0.5)):

    numC = np.asarray(data).ndim
    if numC != 1:
        numC = data.shape[1]

    for i in range(numC):
        if numC == 1: namePlot.plot(time, data, lineType[i], label=data_label[i])
        else: namePlot.plot(time, data[:,i], lineType[i], label=data_label[i])

    autoscale_axes(namePlot)
    lgd = namePlot.legend(loc=legendLoc,frameon=True)
