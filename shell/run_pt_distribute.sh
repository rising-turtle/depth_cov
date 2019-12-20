#!/bin/bash

#
# Dec. 19, 2019, He Zhang, hzhang8@vcu.edu 
# 
# run pt distribute
# ./run_pt_distribute.sh 
#

echo "usage: ./run_depth_cov.sh"
cur_dir=`pwd`
user_name=`whoami`

# bag_dir="/home/$user_name/work/data/struct_core_cal_v2/cov_test"
bag_dir="/home/$user_name/work/data/struct_core_cal_v2/cov_room"
# bags="20cm.bag 40cm.bag 60cm"

cd "$cur_dir/../../../devel/lib/depth_cov"
inv_dep="true"
max_i=700 #60 
i=60  #540 # 700
while [ $i -le $max_i ]; do
# for appro_std in $approximate_std
#do
	num="$i"
	bag_file="$bag_dir/$((i))cm.bag"
	# img_file="$((i))cm.png"
	output_file="$((i))cm.log"
	echo "./pt_distribute $bag_file $output_file $inv_dep"
	./pt_distribute $bag_file $output_file $inv_dep
	echo "finish $i"
	i=$((i+20))
	sleep 1
done

cd $cur_dir

exit 0

