#robotControl.thrift
service robotControl {
	bool setHomePose();
	bool goToHomePose();
	bool setStartingPose();
	bool goToStartingPose();
	bool setEndPose();
	bool goToEndPose();
	bool exploreObject(1: bool onOff);
	bool quit();
	
}
