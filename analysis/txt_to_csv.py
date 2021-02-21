import numpy as np
import re
import csv

import argparse
import sys

#print all the sys argument passed from cmd line including the program name.
print(sys.argv)

#print the second argument passed from cmd line; Note it starts from ZERO
print(sys.argv[1])

file = open(sys.argv[1],"r")
lines = file.readlines()
file.close()
init=False
csvname=sys.argv[1]
csvname=csvname[:-3]+'csv'
print(csvname)
with open(csvname, 'w', newline='') as csvfile:
    posewriter = csv.writer(csvfile, delimiter=',')
    posewriter.writerow(['timestamp','x','y','z','theta'])
    for line in lines:
        line = line.strip()
        if(line.find("At time"))!=-1:
            time=re.findall("\d+\.\d+",line)
            if not init: 
                init_time=time
                init=True
        if line.find("- Translation: ") != -1 :
            robot1pose=[float(time[-1])-float(init_time[-1])]+(re.findall("\d+\.\d+", line))
        if line.find("in RPY (degree)")!=-1:
            robot1pose=robot1pose+[(re.findall("\d+\.\d+", line))[-1]]
            posewriter.writerow(robot1pose)