#!/bin/bash
#@Author: Xincheng Han
#
#Template Declaration
#Adding code before the final 'fi'
#echo "6.xxxxxx"
#elif [ $number -eq 6 ];then  #Do not loss the blank!
##your command;               #Do not forget ';' at the end!
#

usingTarget='Develop'     #Modify it if the other target is used!

echo "******** FUNCTION LIST ***********"
echo "******** 1.MAKE        ***********"
echo "******** 2.COPYFILES   ***********"
echo "88888888 22.COPYFILES  88888888888"
echo "******** 3.LOGIN       ***********"
echo "88888888 33.LOGIN      88888888888"
echo "******** 4.SIMROBOT    ***********"
echo "******** 5.GameControl ***********"
echo "******** 6.TeamPosition***********"
echo "******** 7.Bush        ***********"


read number

if [ $number -eq 1 ];then
cd Make/Linux
make CONFIG=$usingTarget
cd;

elif [ $number -eq 2 ];then
cd Make/Linux
echo "Input your IP:192.168.20."
read ip
./copyfiles $usingTarget 192.168.20.$ip -r
cd

elif [ $number -eq 22 ];then
cd Make/Linux
echo "Input your IP:10.0.20."
read ip
./copyfiles $usingTarget 10.0.20.$ip -r
cd

elif [ $number -eq 3 ];then
cd Make/Linux
echo "Input your IP:192.168.20."
read ip
./login 192.168.20.$ip 
cd

elif [ $number -eq 33 ];then
cd Make/Linux
echo "Input your IP:10.0.20."
read ip
./login 10.0.20.$ip 
cd

elif [ $number -eq 4 ];then
cd Build/Linux/SimRobot/Develop
./SimRobot;

elif [ $number -eq 5 ];then
cd GameController2016
java -jar GameController.jar

elif [ $number -eq 6 ];then
cd GameController2016
java -jar TeamCommunicationMonitor.jar

elif [ $number -eq 7 ];then
cd Build/Linux/bush/Develop
./bush;

fi






