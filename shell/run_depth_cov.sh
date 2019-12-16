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

approximate_std="0.0005	0.000715452	0.00117601	0.00166713	0.00221071	0.00171963	0.00289148	0.0043574	0.00439971	0.00620145	0.00542744	0.0125936	0.0104017	0.00840924	0.0105343	0.011435	0.0148287	0.0164929	0.0174127	0.0140677	0.0172327	0.033716	0.02063	0.0268778	0.0336935	0.0279987	0.0407693	0.0360158	0.0388805	0.043303	0.0465069	0.0757872	0.054274"

max_i=700 #60 
i=60  #540 # 700
#while [ $i -le $max_i ]; do
for appro_std in $approximate_std
do
	num="$i"
	bag_file="$bag_dir/$((i))cm.bag"
	# img_file="$((i))cm.png"
	img_file="$((i))cm"
	echo "./depth_cov $bag_file $img_file $appro_std"
	# roslaunch $ros_launch_file "error_type:=$err_type" "noise_level:=$noise_level" >/dev/null 2>&1
	./depth_cov $bag_file $img_file $appro_std
	echo "finish $i"
	i=$((i+20))
	sleep 1
done

cd $cur_dir

exit 0

