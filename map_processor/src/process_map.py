import rosbag2_py
import sys
import os
import pathlib as Path
import sklearn.cluster._dbscan

'''
Purpose of this script is to process the map data from an arbitrary ros2 bag and output a cleaned map and provide the cleaned map to pure pursuit node

Input:
    /map_refined_points
Ouput:
    /map_cleaned
    or 
    image of the map

Reading the bag file and extracting the map data

'''
