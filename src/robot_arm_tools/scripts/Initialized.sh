#! /bin/bash

until rostopic pub /initialized std_msgs/Bool "data: true"; do
  echo "Next initialization test in 1s"
  sleep 1
done