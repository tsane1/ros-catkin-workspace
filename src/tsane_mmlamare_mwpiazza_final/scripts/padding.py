#!/usr/bin/env python

""" 
File for expanding obstacles in occupancy grid map

Authors: 
    - Matthew Lamare
    - Matthew Piazza
    - Tanuj Sane

Updated: 4/26/17
"""

# Imports
from nav_msgs.msg import OccupancyGrid

"""
pads the obstacles in an occupancy grid with more obstacles cells up to a given depth in meters
"""
def dilateByK(gridMap, padding):

    paddingSquares = int(padding/gridMap.info.resolution)
    for i in range(0, paddingSquares):
        gridMap = dilate(gridMap)
    return gridMap

"""
surrounds each obstacle cell in the map with another 8-connected layer of obstacles
"""
def dilate(gridMap):
    # create deep copy to edit
    newGridMap = OccupancyGrid()
    newGridMap.header = gridMap.header
    newGridMap.info = gridMap.info 
    newGridMap.data = [_ for _ in gridMap.data]

    # dilate grid
    for row in range(gridMap.info.height):
        for col in range(gridMap.info.width):
            index = (row*gridMap.info.width) + col
            isNotTopRow = row > 0
            isNotBottomRow = row+1 < gridMap.info.height
            isNotLeftColumn = col > 0
            isNotRightColumn = col+1 < gridMap.info.width            
            if gridMap.data[index] == 100:
                newGridMap.data[index] = 100 # set unknown cells to obstacles
                #Four connect 
                if isNotTopRow: 
                    newGridMap.data[index-gridMap.info.width] = 100
                if isNotBottomRow:
                    newGridMap.data[index+gridMap.info.width] = 100
                if isNotLeftColumn:
                    newGridMap.data[index-1] = 100
                if isNotRightColumn:
                    newGridMap.data[index+1] = 100
                #Eight Connect
                if isNotTopRow and isNotLeftColumn: 
                    newGridMap.data[index-gridMap.info.width-1] = 100  
                if isNotTopRow and isNotRightColumn: 
                    newGridMap.data[index-gridMap.info.width+1] = 100 
                if isNotBottomRow and isNotLeftColumn:
                    newGridMap.data[index+gridMap.info.width-1] = 100 
                if isNotBottomRow and isNotRightColumn: 
                    newGridMap.data[index+gridMap.info.width+1] = 100
    return newGridMap
