#!/usr/bin/python

# Author: Robert Bennett

import time
import re
import os

date = time.strftime("\"%m/%d/%Y\"")
time = time.strftime("\"%I:%M:%S%p\"")

iFile = open("include/version.h", 'r')
oFile = open("include/version.tmp", 'w')

for line in iFile:
    replaced = False
	
    bNum = re.search('(#define.*BUILDNUMBER\s+)(\d+)(.*)', line)
    bDate = re.search('(#define.*BUILDDATE\s+)(\".*\")(.*)', line)
    bTime = re.search('(#define.*BUILDTIME\s+)(\".*\")(.*)', line)
    
    if bNum is not None:
        oFile.write(bNum.group(1) + str(int(bNum.group(2))+1) + bNum.group(3) + '\n')
        replaced = True
        bNum = None

    if bDate is not None:
        oFile.write(bDate.group(1) + date + bDate.group(3) + '\n')
        replaced = True
        bDate = None

    if bTime is not None:
        oFile.write(bTime.group(1) + time  + bTime.group(3) + '\n')
        replaced = True
        bTime = None
        
    if replaced is False:
    	oFile.write(line)

iFile.close()
oFile.close()

os.remove("include/version.h")
os.rename("include/version.tmp", "include/version.h")
