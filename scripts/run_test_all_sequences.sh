echo "Running Test ALL Sequences"
for i in {1..8}
do
	echo "--> Seq $i"
	./bin/monoVO_KLT /media/tynguyen/docker_folder/kitti/dataset/sequences/0$i
done