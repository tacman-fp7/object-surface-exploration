#robotControl.thrift
service robotControl {
	bool goToHomePose();
	bool updateHomePose();
	bool updateContactPose();
  	bool approach();
	bool contact();
	bool explore();
	bool quit();
}
