#robotControl.thrift
service robotControl {
	bool setHomePose();
	bool goToHomePose();
	bool setStartingPose();
	bool goToStartingPose();
	bool setEndPose();
	bool goToEndPose();
	bool startExploring();
	bool stopExploring();
	bool fingerSetAngle(1: double angle);
	bool prepHand();
	bool openHand();
	bool calibrateHand();
	bool startExploringGP();
	bool exploreGPSurface(1: string objectName);
        bool enableSurfaceSampling();
        bool disableSurfaceSampling();
        bool refineModelEnable();
        bool refineModelDisable();
        bool nRepeatsSet(1: i32 nRepeats);
        bool validatePositionsEnable();
        bool validatePositionsDisable();
	bool quit();
	
}
