#robotControl.thrift
service robotControl {
	bool setStartingPose();
	bool goToStartingPose();
	bool setEndPose();
	bool goToEndPose();
	bool startExploring(1: string type 2: string objectName);
	bool stopExploring();
	bool fingerSetAngle(1: double angle);
	bool prepHand();
	bool openHand();
	bool calibrateHand();
        bool enableSurfaceSampling();
        bool disableSurfaceSampling();
        bool refineModelEnable();
        bool refineModelDisable();
        bool nRepeatsSet(1: i32 nRepeats);
        bool validatePositionsEnable();
        bool validatePositionsDisable();
	bool quit();
	
}
