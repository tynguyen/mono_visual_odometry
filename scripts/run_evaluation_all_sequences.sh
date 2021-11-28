echo "Running evaluation ..."
echo "Results: " > log.txt
for i in {1..8}
do
	echo "--> Seq $i"
	echo "* Seq $i" >> log.txt
	./bin/evaluate_pred_traj ./results/pred_poses_0$i.txt /media/tynguyen/docker_folder/kitti/dataset/sequences/0$i/pose.txt >> log.txt
done