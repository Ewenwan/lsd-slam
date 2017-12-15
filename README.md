LSD_SLAM 机器人开发
===================================
1.lsd_slam_core与lsd_slam_viewer之间的ROS交互
-----------------------------------
### (1).core发布消息，viewer接受消息
core发布消息的函数都在/lsd_slam_core/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp中，
</br>
    virtual void publishKeyframeGraph(KeyFrameGraph* graph);

	// publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
	virtual void publishKeyframe(Frame* f);

	// published a tracked frame that did not become a keyframe (i.e. has no depth data)
	virtual void publishTrackedFrame(Frame* f);

	// publishes graph and all constraints, as well as updated KF poses.
