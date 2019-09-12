import datetime
import sys
import subprocess
now = datetime.datetime.now()
timestamp = int(datetime.datetime.timestamp(now)) 
date = str(datetime.datetime.date(now))

f = open('Hours.txt',"r")

lines = f.readlines()
lastLine = lines[len(lines)-1].split(',')
checkout = lastLine[7]
f.close()



lastLine[7] = timestamp
dt = int(lastLine[7] - int(lastLine[5]))
lastLine[-1] = dt
string = ""
for i in range(0,len(lastLine)):
	
	if i == len(lastLine) - 1:
		string = string + str(lastLine[i])
		break
	string = string + str(lastLine[i]) + ","
string = string + "\n"
lines[-1] = string
print("Current time registered: "+ str(int(dt/3600)) + " : " + str(int(dt/60) - int(dt/3600)*60 )) 
print("Checkin time was       : " + str(datetime.datetime.fromtimestamp(int(lastLine[5])).time()))