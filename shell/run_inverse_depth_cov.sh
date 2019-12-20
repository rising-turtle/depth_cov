#!/bin/bash

#
# Dec. 18, 2019, He Zhang, hzhang8@vcu.edu 
# 
# run inverse depth cov
# ./run_inverse_depth_cov.sh 
#

echo "usage: ./run_depth_cov.sh"
cur_dir=`pwd`
user_name=`whoami`

# bag_dir="/home/$user_name/work/data/struct_core_cal_v2/cov_test"
bag_dir="/home/$user_name/work/data/struct_core_cal_v2/cov_room"
# bags="20cm.bag 40cm.bag 60cm"

cd "$cur_dir/../../../devel/lib/depth_cov"

# approximate_std="0.0005	0.000715452	0.00117601	0.00166713	0.00221071	0.00171963	0.00289148	0.0043574	0.00439971	0.00620145	0.00542744	0.0125936	0.0104017	0.00840924	0.0105343	0.011435	0.0148287	0.0164929	0.0174127	0.0140677	0.0172327	0.033716	0.02063	0.0268778	0.0336935	0.0279987	0.0407693	0.0360158	0.0388805	0.043303	0.0465069	0.0757872	0.054274"

approximate_inv_std="0.00110133 0.00117764 0.00119786 0.00113151 0.000672004 0.000889915 0.00109681 0.000884261
 0.00106168 0.000805887 0.00160474 0.0011474 0.000828119 0.000912799 0.000887894 0.00103853 0.00104677 0.00100398
 0.000747116 0.00082744 0.00138401 0.000795861 0.000947222 0.0011119 0.000848025 0.0011653 0.000963962 0.000980091
 0.00100063 0.00103235 0.00156291 0.00106813"

run_inv_depth="true"

max_i=700 #60 
i=60  #540 # 700
#while [ $i -le $max_i ]; do
for appro_std in $approximate_inv_std
do
	num="$i"
	bag_file="$bag_dir/$((i))cm.bag"
	# img_file="$((i))cm.png"
	img_file="$((i))cm"
	echo "./depth_cov $bag_file $img_file $appro_std $run_inv_depth"
	# roslaunch $ros_launch_file "error_type:=$err_type" "noise_level:=$noise_level" >/dev/null 2>&1
	./depth_cov $bag_file $img_file $appro_std $run_inv_depth
	echo "finish $i"
	i=$((i+20))
	sleep 1
done

cd $cur_dir

exit 0

