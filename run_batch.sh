#path="/data/abobu/data_dso/00488b40-0a8cf1a0"

for path in /data/abobu/data_orbslam/*;
do
    echo $path
    video=$path"/*.mov"

    bash video_to_images.sh $video  $path"/images"
    
./Examples/Monocular/mono_nexar \
Vocabulary/ORBvoc.txt \
Examples/Monocular/nexar.yaml \
$path"/images" \
0 \
none

    mv CameraTrajectory.txt $path
    mv KeyFrameTrajectory.txt  $path
done
