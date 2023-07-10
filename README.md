# moveit-servo-bug-replication
Repository to replicate a bug in moveit2-servo


sudo docker build ./ --pull -t servoerrorimage

xhost +

sudo docker run --rm -it --env="DISPLAY" --privileged --network host --cpus="3.0" --name servoerrorcontainer servoerrorimage

sudo docker exec -it servoerrorcontainer /bin/bash
