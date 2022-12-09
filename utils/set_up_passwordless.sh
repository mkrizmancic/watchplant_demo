echo "This script will generate a key for passwordless ssh onto Raspberries."
echo "On each prompt just press enter key to confirm the defaults."
sleep 2

ssh-keygen -t rsa

nodes="20 21 22 23 27"

for val in $nodes; do
    echo "Modifying rpi$val."
    sshpass -p watchplant ssh-copy-id -o StrictHostKeyChecking=no pi@192.168.0."$val"
done