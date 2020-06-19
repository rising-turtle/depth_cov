#!/bin/bash

#
# June. 19, 2020, He Zhang, hzhang8@vcu.edu 
# 
# run solve_lambda_folder to estimate lambda  
# ./run_solve_lambda_folder.sh 
#

echo "usage: ./run_solve_lambda_folder.sh"
cur_dir=`pwd`
user_name=`whoami`

exe_dir="/home/davidz/work/git/depth_cov/build/devel/lib/depth_cov"

folder_name="./tmp"
num="30"
cd $exe_dir

echo "./run_solve_lambda_folder $folder_name $num"
# roslaunch $ros_launch_file "error_type:=$err_type" "noise_level:=$noise_level" >/dev/null 2>&1
./solve_lambda_folder $folder_name $num 

cd $cur_dir

exit 0
