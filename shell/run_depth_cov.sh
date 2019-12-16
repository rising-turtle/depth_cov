#!/bin/bash

#
# Dec. 12, 2019, He Zhang, hzhang8@vcu.edu 
# 
# run depth cov
# ./run_depth_cov.sh 
#

echo "usage: ./run_depth_cov.sh"
cur_dir=`pwd`
user_name=`whoami`

# bag_dir="/home/$user_name/work/data/struct_core_cal_v2/cov_test"
bag_dir="/home/$user_name/work/data/struct_core_cal_v2/cov_testRoom"
# bags="20cm.bag 40cm.bag 60cm"

cd "$cur_dir/../../../devel/lib/depth_cov"

max_i=700 #60 
i=100  #540 # 700
while [ $i -le $max_i ]; do
	num="$i"
	bag_file="$bag_dir/$((i))cm.bag"
	img_file="$((i))cm.png"
	echo "./depth_cov $bag_file $err_type $img_file"
	# roslaunch $ros_launch_file "error_type:=$err_type" "noise_level:=$noise_level" >/dev/null 2>&1
	./depth_cov $bag_file $err_type $img_file
	echo "finish $i"
	i=$((i+20))
	sleep 1
done

cd $cur_dir

exit 0

