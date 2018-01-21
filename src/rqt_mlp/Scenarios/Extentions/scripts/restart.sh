#!/bin/bash
kill -9 `ps aux | grep ros | grep -v grep | grep -v "rosbag reindex" | awk '{print $2}'`
kill -9 `ps aux | grep rqt | grep -v restart | grep -v grep | grep -v "rosbag reindex" | awk '{print $2}'`
kill -9 `ps aux | grep rviz | grep -v restart | grep -v grep | grep -v "rosbag reindex" | awk '{print $2}'`
sleep 1
rosbag reindex -f $1".bag"
rm $1".orig.bag"
sleep 2
roscore &
sleep 1
rosrun rqt_mlp rqt_mlp
