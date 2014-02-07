
const int xBlurSize=3;
const int yBlursize=3;

const double cannyLowThreshold=50;
const double cannyLargeThreshold=200;
const int cannyApertureSize=3;

const int houghThreshold=50;
const double houghMinLineLength=50;
const double houghMaxLineGap=10;


const int kptDet_maxCorners=400;
const double kptDet_qualityLevel = 0.01;
const double kptDet_minDistance = 8;
const int kptDet_blockSize = 3;
const bool kptDet_useHarrisDetector = false;
const double kptDet_k = 0.04;

const int trajectoryL=7;

const int kptTrack_iter=20;
const double kptTrack_epsin=0.03;
const int kptTrack_winsize=21;
//const kptTrack_
const int kptTrack_maxlevel=3;