LSD_SLAM 机器人开发
每个人都应该为自己热爱的事业奋斗终生，但是绝大多数人都会因为主动或者被动的原因而失去斗志。我跟赛格找到了共鸣， 希望在我临死的时候，我能够说：我的整个生命和全部精力，都已经献给世界上最壮丽的事业——为让机器人能够更好的理解世界，改变世界从而为千家万户服务的伟大事业而斗争。
===================================
1.lsd_slam_core与lsd_slam_viewer之间的ROS交互
-----------------------------------
### (1).core发布消息，viewer接受消息
core发布消息的函数都在/lsd_slam_core/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp::70中，
```    	virtual void publishKeyframeGraph(KeyFrameGraph* graph);
	// publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
	virtual void publishKeyframe(Frame* f);
	// published a tracked frame that did not become a keyframe (i.e. has no depth data)
	virtual void publishTrackedFrame(Frame* f);
	// publishes graph and all constraints, as well as updated KF poses.
```
publishKeyframe()发布关键帧,填写id，时间戳，isKeyframe信息等等。
在SlamSystem::updateKeyframe()的最后对publishKeyframe()调用，就是说每更新一次关键帧，都会发布一次消息
