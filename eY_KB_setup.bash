#!/bin/bash  

# Set the color variable
green='\033[0;32m'
# Clear the color after that
clear='\033[0m'

printf "${green}"
echo "--------------------------"
echo "      e-Yantra KB22       "
echo "--------------------------"
printf "${clear}"

echo -e "\e[3m(Use ^C only once to terminate the bash.)\e[0m"
echo "Make sure you have added the both lines in bashrc:"
echo "export ROS_IPV6=on"
echo "export ROS_MASTER_URI=http://master:11311"

trap ctrl-c
trap ctrl_c INT
function ctrl_c() {
echo "** Trapped CTRL-C"
sudo cp hoststemp /etc/hosts
sudo cp hostnametemp /etc/hostname
sudo systemctl stop husarnet
wait
hostnameval=`cat hostnametemp`
sudo hostnamectl set-hostname $hostnameval
printf "${green}"
echo "--------------------------"
echo "      BYE $hostnameval    "
echo "--------------------------"
printf "${clear}"
echo "Make sure you have commented the both lines in bashrc (to use ROS in simulation):"
echo "export ROS_IPV6=on"
echo "export ROS_MASTER_URI=http://master:11311"
kill $$ 
}

echo "You will be using this machine for:"
echo "1. For ROS connection"
echo "2. To watch camera feed"
echo -n "Enter the option (1 or 2): "
read machine

if [ "$machine" -gt "2" ]
then
    echo "Invalid option use ^C to kill, continuing as ROS machine."
fi

echo -n "Please enter your team id (for ex: enter 1234 for KB#1234): "
read team_id
echo "Team KB#$team_id, kindly enter the join code :"
echo -n "> "
read join_code

# restarting husarnet and waiting for 5 secs
sudo systemctl restart husarnet
wait
sleep 5

# Creating and copying hosts and hostname file for backup
touch hoststemp
touch hostnametemp
sudo cp /etc/hosts hoststemp
sudo cp /etc/hostname hostnametemp
hostnameval=`cat hostnametemp`
echo "Your hostname: $hostnameval"

#Join husarnet command
husarnet join $join_code team-$team_id-$machine
wait
sleep 2

# If option is 2 setup camera feed on default browser
if [ "$machine" -eq "2" ]
then
	ip_add=`dig eyantra-greenhouse AAAA +short`
	wait
	link="http://[$ip_add]:8083/pages/multiview/full?controls#"
	echo "Do open this link below:"
	echo "$link"
fi

# Show elapsed time
declare -i sec=0

while true
	do
		echo -ne "Elapsed Time: $sec s"\\r
		sleep 1
		((sec++))

done
