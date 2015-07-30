#robotControl.thrift
service robotControl {
	bool setHomePose();
	bool goToHomePose();
	bool setStartingPose();
	bool setEndPose();
	bool goToStartingPose();
	bool goToEndPose();
	bool explore(1: bool onOff);
	bool updateHomePose();
	bool updateContactPose();
  	bool approach();
	bool contact();
	bool quit();
	
}
