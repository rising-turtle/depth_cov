#!/bin/bash

#
# June. 13, 2020, He Zhang, hzhang8@vcu.edu 
# 
# run depth_distribute to compute gt_image 
# ./run_depth_cov.sh 
#

echo "usage: ./run_depth_cov.sh"
cur_dir=`pwd`
user_name=`whoami`

bag_dir="/home/davidz/work/data/gt_table"
# bag_list_folder="pitch roll yaw" #"x_t z_t"
bag_list_folder="x z" #"x_t z_t"
# bag_list_id="0 3 6 9 12 15"
bag_list_id="00 10 20 30 40 50"
exe_dir="/home/davidz/work/git/depth_cov/build/devel/lib/depth_cov"

cd $exe_dir

i=19
# mkdir "tmp"
for bagfolder in $bag_list_folder
do
	for bag_id in $bag_list_id
	do
		num="$i"
		bag_file="$bag_dir/${bagfolder}_t/${bagfolder}_$bag_id.bag"
		output_gt_exr="./tmp/$((i)).exr"
		output_gt_img="./tmp/$((i)).png"
		echo "./depth_distribute $bag_file $output_gt_exr $output_gt_img"
		# roslaunch $ros_launch_file "error_type:=$err_type" "noise_level:=$noise_level" >/dev/null 2>&1
		./depth_distribute $bag_file $output_gt_exr $output_gt_img
		echo "finish $bag_file"
		i=$((i+1))
		sleep 1
	done
done
cd $cur_dir

exit 0
