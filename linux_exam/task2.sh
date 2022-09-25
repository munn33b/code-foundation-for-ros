#!/usr/bin/env bash


#read -p "Enter Option: " USER_OPTION

if [[ "$1" == "small_square" ]]; then
rosrun linux_exam small_square.py
elif [[ "$1" == "medium_square" ]]; then
rosrun linux_exam medium_square.py
elif [[ "$1" == "big_square" ]]; then
rosrun linux_exam big_square.py
else
	echo "Please Provide Valid Parameter"
fi
