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


if (int(checkout) == 0) and (1):
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

	f = open('Hours.txt',"w")
	f.writelines(lines)	
	print("\nChecked OUT: Time registered: "+ str(int(dt/3600)) + " : " + str(int(dt/60) - int(dt/3600)*60 )) 
	f.close()
	date = str(datetime.datetime.fromtimestamp(int(lastLine[5])).day)+'.'+str(datetime.datetime.fromtimestamp(int(lastLine[5])).month)

	print("Commiting to git with message: " + "\""+date+"\"")
	print("\n===========GIT COMMANDS============")
	subprocess.call('git add .', shell=False)
	subprocess.call('git commit -m '+"\""+date+"\"", shell=True)
	subprocess.call('git push origin master', shell=True)
	subprocess.call('git status', shell=True)
	print("=============GIT FINISHED============\n")
	print("Checkout complete")
else:
	string = "month,"+date.split('-')[1]+",day,"+date.split('-')[2]
	string = string + ",timestampIn," + str(timestamp) + ",timestampOut," + str(0) 
	string = string + ",time,"+str(0)+ "\n"
	f = open('Hours.txt',"a+")
	f.write(string)	
	print("Checked IN")

