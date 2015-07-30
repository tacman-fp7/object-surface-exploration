#robotControl.thrift
service robotControl {
	bool setHomePose();
	bool goToHomePose();
	bool setStartingPose();
	bool setEndPose();
	bool goToStartingPose();
	bool goToEndPose();
	bool explore();
	bool updateHomePose();
	bool updateContactPose();
  	bool approach();
	bool contact();
	bool quit();
	
}
