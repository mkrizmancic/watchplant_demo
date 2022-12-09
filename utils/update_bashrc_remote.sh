ip=$(ip -o route get to 8.8.8.8 | sed -n 's/.*src \([0-9.]\+\).*/\1/p')

nodes="20 21 22 23 27"

for val in $nodes; do
    echo "Modifying rpi$val."
    ssh pi@192.168.0."$val" "sed -ie 's/export ROS_MASTER_URI.*/export ROS_MASTER_URI=http:\/\/$ip:11311/g' ~/.bashrc"
done