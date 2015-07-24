#robotControl.thrift
service robotControl {
  	bool approach();
	bool contact();
	bool explore();
	bool quit();
}
