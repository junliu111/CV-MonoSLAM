/** @file
* @author Jun Liu
* @date 2014-1-1
* @version v1.0.0
*/
/*=========================================================================
*
* This is the main part for SLAM, including initialization, prediction, 
* and updating.
*
*=========================================================================*/

#include "StdAfx.h"
#include "SLAM.h"

/**
* @brief Constructor
* @param[in] void
* @param[out] void
*/
CSLAM::CSLAM (void)
:FLAG_4_VIDEO(0), 
FLAG_4_IMAGE(1),
FLAG_4_WEIGHT1(0),
FLAG_4_WEIGHT2(1),
FLAG_4_WEIGHT3(2),
FLAG_4_NOISE1(0),
FLAG_4_NOISE2(1),
FLAG_4_NOISE3(2),
FLAG_4_NOISE4(3),
FLAG_4_UPDATING(0),
FLAG_4_DOWNDATING(1),
FLAG_4_NORMAL_UPDATE(0),
FLAG_4_LOW_INLIER(1),
FLAG_4_HIGH_INLIER(2),
FLAG_4_NEED_REORDER(0),
FLAG_4_NEEDNOT_REORDER(1),
FLAG_4_UNSELECT_MODEL(-1),
FLAG_4_DEBUG_MODEL(0),
FLAG_4_RELEASE_MODEL(1),
HP_INIT_W(10),
HP_INIT_H(10),
HP_MATCH_W(8),
HP_MATCH_H(8),
MIN_STEP_X(0.01),
MIN_STEP_Y(0.01),
MIN_STEP_THETA(45),
DIST_2_BORDER(20.0),
HFOV(180),
m_model(0),
LINEARITY_INDEX_THRESHOLD(0.1),
EPSILON(1e-13),    // (DBL_EPSILON),
// EPSINON(std::numeric_limits<double>::epsilon()),
CHI2INV_TABLE(Matx33d(0.95, 2, 5.99146454710798, 0.95, 3, 7.81472790325116, 0.95, 6, 12.59158724374398))
{
	initializeParameters();
}

/**
* @brief Deconstructor
* @param[in] none.
* @param[out] none.
*/
CSLAM::~CSLAM (void)
{
	cvReleaseImage(&m_srcImage);
	cvReleaseImage(&m_gryImage);

	if (TRUE == isRecordRobotInfo)
	{
		fclose(m_robotFile);
	}

	if (TRUE == isRecordFeaturesInfo)
	{
		fclose(m_featuresFile);
	}
}

/**
* @brief Processing of SLAM
* @param[in] none.
* @param[out] none.
* @note
*  We can use timer to count the each processing part time.\n
*/
void CSLAM::SLAM (void)
{
	//m_timer.reset(); m_timer.start();
	startTimer();
	predictMotion();            
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	predictMeasurement();       
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	loadPictures();             
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	dataAssociation();          
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	KalmanUpdate();                
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	updateFeaturesInformation();
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	updateRobotInformation();   
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	display2DFeatureModel();    
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	recordData();               
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	addFeatures();              
	//m_timer.stop(); cout<<m_timer.getTimeMilli()<<endl; m_timer.reset(); m_timer.start();
	stopTimer();
}

/**
* @brief Start timer.
* @param[in] none.
* @param[out] none.
* @note
*  This part is used to starting the timer for\n
*  total processing time counting.
*/
void CSLAM::startTimer (void)
{
	m_showCounter++; 

	if (1 == m_frame.counter)
	{
		m_totalTimer.reset(); m_totalTimer.start();
	}

	m_frameTimer.reset(); m_frameTimer.start();
}

/**
* @brief Stop timer.
* @param[in] none.
* @param[out] none.
* @note
*  This part is used for counting each frame processing time\n
*  and the total processing time.
*/
void CSLAM::stopTimer (void)
{
	m_frame.counter++;

	m_frameTimer.stop();
	m_totalTimer.stop();

	m_frameTime  = m_frameTimer.getTimeSec();
	m_totalTime += m_totalTimer.getTimeSec();
}

/**
* @brief Initialize all parameters
* @param[in] none.
* @param[out] none.
*/
void CSLAM::initializeParameters (void)
{
	//rng((unsigned)getTickCount()); 

	// If choose debug model, load the default parameters,
	// else use the parameters selected from MFC windows.
	if (FLAG_4_DEBUG_MODEL == m_model)
	{
		// =========Parameters for 3D show in main frame======== //
		isShowRobotPath  = FALSE;
		isShowOdoInfo    = FALSE;

		// ======Parameters for debug setting in the  paraemters dialog======== //
		// Parameters about finding keypoints.
		m_deep           = 3.0;
		m_rho            = 1.0/m_deep;
		isUseHarris      = FALSE; //=====需要做类型转换BOOL到bool，暂时未用上????????
		m_blockSize      = 3;
		m_qualityLevel   = 0.1;
		m_nInitialRaws   = 8;
		m_nProcessRaws   = 8;
		m_minNUM         = 5;
		m_minDist        = 15.0;
		m_minDist2       = m_minDist*m_minDist;

		// Parameters for threshold.
		THRESHOLD_MATCH_PATCH = 0.8;
		isUseRANSAC      = FALSE;
		THRESHOLD_RANSAC = 8.0;

		// Parameters for noise value.
		m_sigmaMeasure   = 3.0;
		m_sigmaRHO       = m_rho/2.0;
		//a1               = 0.0004;
		//a2               = 0.0004;
		//a3               = 0.0006;
		//a4               = 0.0006;
		a1               = 8;
		a2               = 8;
		a3               = 8;
		a4               = 8;
	
		// Parameters for wether recording the output data.
		isRecordRobotInfo    = TRUE;
		isRecordFeaturesInfo = FALSE;

		// Default output data restore path for file.
		m_videoDir           = "E:\\SLAM\\sequences\\2013.3.9.22.00\\1\\1_Xvid.avi";
		m_imageDir           = "E:\\SLAM\\sequences\\6\\%04d.jpg";
		m_odometryDir        = "E:\\SLAM\\sequences\\data\\6.txt";
		m_recordRobotDir     = "E:\\RobotPath.txt";
		m_recordFeaturesDir  = "E:\\FeaturesInfo.txt";

		// ======Parameters for debug model out of the dialog======== //
		m_frame.start        = 600;//150;//1020;//600;
		m_playType		     = FLAG_4_IMAGE;
	}

	//====Parameters initialization for both debug and release model======//
	//m_sigmaX        = 0.008;
	//m_sigmaY        = 0.008;
	//m_sigmaZ        = 0.0005;
	//m_sigmaTheta    = 0.001;
	m_sigmaX        = 0.02;
	m_sigmaY        = 0.02;
	m_sigmaZ        = 0.005;
	m_sigmaTheta    = 0.02;

	m_X_k           = Mat::zeros(4, 1, CV_64F);
	m_S_k           = Mat::zeros(4, 4, CV_64F);
	m_S_k.ptr<double>(0)[0] = m_sigmaX;
	m_S_k.ptr<double>(1)[1] = m_sigmaY;
	m_S_k.ptr<double>(2)[2] = m_sigmaZ;
	m_S_k.ptr<double>(3)[3] = m_sigmaTheta;

	m_initOdo       = Mat::zeros(2, 1, CV_64F);
	m_initPos       = Mat::zeros(2, 1, CV_64F);
	m_odoTheta      = Mat::zeros(3, CAPACITY+1, CV_64F);

	Mt              = Mat::zeros(3, 3, CV_64F);
	Qt              = Mat::eye(2, 2, CV_64F)*m_sigmaMeasure;
	Ut              = Mat::zeros(3, 1, CV_64F);

	m_weightType    = FLAG_4_WEIGHT1;
	m_noiseType     = FLAG_4_NOISE1;

	m_frame.stop    = m_frame.start + CAPACITY;
	m_frame.index	= m_frame.start;
	m_frame.counter = 1;

	ID			    = 1;
	m_odoCounter    = 0;
	m_showCounter   = 1;
	m_nFilters		= 0;
	m_nDeletes      = 0;
	m_nStores       = 0;
	m_nMapFeatures	= 0;
	m_nPredicts		= 0;
	m_nMatches		= 0;
	m_nStoreMap     = 0;
	m_nStorePredicts= 0;
	m_nStoreMatches = 0;
	m_nShowMap      = 0;
	m_nShowPredicts = 0;
	m_nShowMatches  = 0;
	m_sample.Alpha  = 1e-3;
	m_sample.Beta   = 2;
	m_frameTime     = 0;
	m_totalTime     = 0;

	isAdding = FALSE;
	isCompensate = TRUE;
	isShowOdoCtrlList = FALSE;

	// ======Parameters for open and store files========== //
	if (TRUE == isRecordRobotInfo)
	{
		strcpy_s(record_robot_dir, 100, m_recordRobotDir);
	}

	if (TRUE == isRecordFeaturesInfo)
	{
		strcpy_s(record_features_dir, 100, m_recordFeaturesDir);
	}
	
	if (!m_frame.stop)
	{
		m_frame.stop = INT_MAX;  // initilize the stoping frame to the maximum int value
	}

	// Choose to use video or image sequence
	if (FLAG_4_VIDEO == m_playType)
	{
		strcpy_s(video_dir,100, m_videoDir);
		m_capture = cvCreateFileCapture(video_dir);
		m_frame.rate = static_cast<int>(cvGetCaptureProperty(m_capture, CV_CAP_PROP_FPS));

		if (!m_capture)
		{
			cout<<"Warning: error occured in getting video!"<<endl;
			system("pause");
		}
		cvSetCaptureProperty(m_capture, CV_CAP_PROP_POS_FRAMES, m_frame.start);
		m_srcImage = cvQueryFrame(m_capture);
	} 
	else
	{
		strcpy_s(image_dir,100, m_imageDir);
		char image_name[100];
		sprintf_s(image_name, image_dir, m_frame.index);
		m_srcImage = cvLoadImage(image_name);
	}

	// Initialize the image structure
	imageWidth  = m_srcImage->width;
	imageHeight = m_srcImage->height;
	m_imageSize = Size(imageWidth, imageHeight);
	m_gryImage  = cvCreateImage(cvSize(imageWidth, imageHeight),IPL_DEPTH_8U,1);
	cvCvtColor(m_srcImage, m_gryImage, CV_RGB2GRAY);

	// Load odometry data set
	strcpy_s(odometry_dir, 100, m_odometryDir);
	int err_open = fopen_s(&m_odometryFile, odometry_dir, "r");

	if (0 != err_open) 
	{
		cout<<"Error occured in loading odometry data!"<<endl;
		system("pause");
	}

	// ===========Set the camera parameters========== //
	cam_dx	   = 0.0028;
	cam_dy	   = 0.0028;
	cam_cx	   = 310.1129;
	cam_cy     = 236.7526;
	cam_k1     = 0.0001;
	cam_k2     = 0.0000;
	cam_f      = 2.1735;
	cam_f1     = cam_f/cam_dx;
	cam_f2     = cam_f/cam_dy;
	cam_K	   = 0;
	cam_K(0,0) = cam_f1;
	cam_K(1,1) = cam_f2;
	cam_K(0,2) = cam_cx;
	cam_K(1,2) = cam_cy; 
	cam_K(2,2) = 1.0;

	map = NULL;
	//blackList = NULL; // =========后期加入?????????

	loadOdometryData();
	addFeatures();

	m_nStore4Show  = m_nMapFeatures;
	m_nMapFeatures = 0;
}

/**
* @brief Load overall odometry data.
* @param[in] none.
* @param[out] none.
* @note
*  The overall process of getting odometry information from .txt file.\n
*  And the data should satisfy some constraints described in the coding line.
*/
void CSLAM::loadOdometryData (void)
{
	//=============Read the odometry data from .txt file==========//
	int id;
	int dim = m_X_k.rows;
	int index=1;
	double x0, x1, y0, y1, theta0, theta1;
	double diff_x, diff_y, diff_theta;

	if (1 == m_frame.start)
	{
		getOneMomentData(id, m_odoCounter);
	}
	else
	{
		while(!feof(m_odometryFile))
		{
			getOneMomentData(id, m_odoCounter);
			index++;

			if (index == m_frame.start)
			{
				getOneMomentData(id, m_odoCounter);

				index = 2*m_odoCounter;

				x0 = *(m_odoXY+index+0);
				y0 = *(m_odoXY+index+1);

				break;
			}
		}
	}
	m_odoCounter++;
	m_X_k.ptr<double>(dim-1)[0] = m_odoTheta.ptr<double>(1)[0];
	m_odoTheta.ptr<double>(2)[0] = 0;
	
	for (int i = 0; i < m_frame.stop - m_frame.start; i++)
	{
		if (feof(m_odometryFile))
		{
			break;
		}

		getOneMomentData(id, m_odoCounter);

		index = 2*m_odoCounter;

		x0 = *(m_odoXY+index-2);
		x1 = *(m_odoXY+index+0);
		y0 = *(m_odoXY+index-1);
		y1 = *(m_odoXY+index+1);

		diff_x = abs(x1 - x0);
		diff_y = abs(y1 - y0);

		while ((diff_x < MIN_STEP_X) && (diff_y < MIN_STEP_Y))
		{
			getOneMomentData(id, m_odoCounter);

			index = 2*m_odoCounter;

			x0 = *(m_odoXY+index-2);
			x1 = *(m_odoXY+index+0);
			y0 = *(m_odoXY+index-1);
			y1 = *(m_odoXY+index+1);

			diff_x = abs(x1 - x0);
			diff_y = abs(y1 - y0);
		}

		theta0 = m_odoTheta.ptr<double>(1)[m_odoCounter-1];
		theta1 = m_odoTheta.ptr<double>(1)[m_odoCounter];
		diff_theta = abs(wrapAngle(theta1 - theta0));

		if (diff_theta > MIN_STEP_THETA*CV_PI/180)
		{
			m_odoTheta.ptr<double>(2)[m_odoCounter] = 1;
		} 
		else
		{
			m_odoTheta.ptr<double>(2)[m_odoCounter] = 0;
		}

		m_odoCounter++;
	}
	fclose(m_odometryFile);
}

/**
* @brief Load each line odometry data.
* @param[in] 
*  id the id of qualified features.
* @param[in]
*  counter the counter of odometry data line.
* @param[out] none.
* @note
*  Processing each line of odometry data.\n
*/
void CSLAM::getOneMomentData (int &id, const int &counter)
{
	//int id = 0;
	int index = 2*counter;
	double x = 0, y = 0, theta = 0;
	char line[500]="";

	if (feof(m_odometryFile))
	{
		return;
	}

	fgets(line, sizeof(line), m_odometryFile);
	sscanf_s(line, "%d : %*lf %lf %lf %lf", &id, &x, &y, &theta);

	m_odoTheta.ptr<double>(0)[counter] = id;
	m_odoTheta.ptr<double>(1)[counter] = theta;

	if (0 == counter)
	{
		m_initOdo.ptr<double>(0)[0] = x;        // for initilization
		m_initOdo.ptr<double>(1)[0] = y;

		m_initPos.ptr<double>(0)[0] = m_X_k.ptr<double>(0)[0];
		m_initPos.ptr<double>(1)[0] = m_X_k.ptr<double>(1)[0];

		*(m_odoXY+index+0) = m_initPos.ptr<double>(0)[0];
		*(m_odoXY+index+1) = m_initPos.ptr<double>(1)[0];
	} 
	else
	{
		*(m_odoXY+index+0) = m_initPos.ptr<double>(0)[0] + (x - m_initOdo.ptr<double>(0)[0]); 
		*(m_odoXY+index+1) = m_initPos.ptr<double>(1)[0] + (y - m_initOdo.ptr<double>(1)[0]);
	}
}

/**
* @brief Angle wrap
* @param[in]
*  angle the angle of robot pose.
* @param[out]
*  angle the angle of robot pose.
* @note
*  Limit the angle in -PI to PI.
*/
double CSLAM::wrapAngle (double angle) const
{
	if (angle > CV_PI)
	{
		angle -= 2.0*CV_PI;
	}
	else if (angle < -CV_PI)
	{
		angle += 2.0*CV_PI;
	}

	return angle;
}

/**
* @brief Image loading.
* @param[in] none.
* @param[out] none.
* @note
*  Load video or image sequence, but the format should\n
*  the requirement of OpenCV constraint.
*/
void CSLAM::loadPictures (void)
{
	if (FLAG_4_VIDEO == m_playType)
	{
		m_srcImage = cvQueryFrame(m_capture);
	} 
	else
	{
		char image_name[100];
		sprintf_s(image_name, image_dir, m_frame.index);
		m_srcImage = cvLoadImage(image_name);
	}

	cvCvtColor(m_srcImage, m_gryImage, CV_RGB2GRAY);
}

/**
* @brief Adding features.
* @param[in] none.
* @param[out] none.
* @note
*  Add features into state vector and map.\n
*/
void CSLAM::addFeatures (void)
{
	m_nAddings = 0;

	if ((m_nMatches < m_minNUM) || (TRUE == isAdding))             // only add features when matched number below a threshold
	{
		detectAndfilteringFeatures();
		insureEnoughFeatures();
		integrateFeaturesInformation();
	}
} 

/**
* @brief Features filtering.
* @param[in] none.
* @param[out] none.
* @note
*  Filtering the obtained features according to some constraints.\n
*  e.g. 1. The features should not too close to each others.
*       2. The new features should not too close to the former features\n
*          (including features in the map & ceiling image).
*/
void CSLAM::detectAndfilteringFeatures (void)
{
	vector<KeyPoint> keyPoints;
	Mat image(m_gryImage);

	Point2d uvu, uvd;
	Point3d xyz;
	int minNum;
	int counter = 0;
	int dim = m_X_k.rows;
	static bool flag = true;
	double px, py, mx, my, kx, ky;
	double dx, dy, dmx, dmy, dpx, dpy, dist;
	Mat Hlr, Hlw, state, position, Rwc, Rcw;
	FeatureInfo featuresInfo;

	if (1 == m_frame.counter || TRUE == isAdding)
	{
		minNum = m_nInitialRaws;
	}
	else
	{
		minNum = m_nProcessRaws;
	}

	GoodFeaturesToTrackDetector detector(minNum, m_qualityLevel, m_minDist, m_blockSize, false);
	detector.detect(m_gryImage, keyPoints);

	// =========暂时未用上?????????
	//SurfFeatureDetector detector;			// 特征提取
	//detector.detect(image,keyPoints);
	//FAST(image,keyPoints,60);

	bool flag4Adding;  // assume the selected keypoint satisfy the default assumption
	bool isThereNoZero;

	int id = 0;
	int index = 0;
	int nCols = m_featuresAllInfo.size();
	m_loopPointCounter = 0;
	Mat pixelPos = Mat::zeros(2, nCols, CV_64F);
	Mat error    = Mat::zeros(2, 1, CV_64F);
	Mat restoreID = Mat::zeros(m_featuresAllInfo.size(), 1, CV_64F);
	
	if (TRUE == isAdding)
	{
		m_loopPointID = Mat::zeros(m_featuresAllInfo.size(), 1, CV_8U);

		for (vector<FeatureInfo>::size_type j=0; j<m_featuresAllInfo.size(); j++)
		{
			position = m_X_k.rowRange(dim-4,dim-1);
			getTransferMatrix(Rwc, m_X_k.ptr<double>(dim-1)[0]);
			Rcw = Rwc.inv();

			state = (Mat)m_featuresAllInfo[j].state;
			coordinatesState2World(Hlw, state, position);
			coordinatesWorld2Camera(Hlr, Hlw, Rcw);
			coordinatesCamera2Image(uvu, Hlr, error);
			distortOnePointRW(uvd, uvu);

			pixelPos.ptr<double>(0)[id] = uvd.x;
			pixelPos.ptr<double>(1)[id] = uvd.y;

			id++;
		}
	}

	for (vector<KeyPoint>::size_type i=0; i<keyPoints.size(); i++)
	{
		flag4Adding = true;

		KeyPoint kp = keyPoints[i];

		kx = kp.pt.x;
		ky = kp.pt.y;

		if (kx >= DIST_2_BORDER && kx <= (imageWidth - DIST_2_BORDER)   // features shouldn't too close to the edge
			&& ky >= DIST_2_BORDER && ky <= (imageHeight - DIST_2_BORDER))
		{
			if ((1 == m_frame.counter) || (true == flag))
			{
				m_keyPoints.push_back(kp);
				counter++;
			}
			else
			{
				if (0 != m_nMatches)
				{
					// features shouldn't be too close to the predicted and matched features
					PointsMap *map_p = map;
					while (NULL != map_p)		   
					{
						mx = map_p->matchLocation.x;
						my = map_p->matchLocation.y;
						px = map_p->predictLocation.x;
						py = map_p->predictLocation.y;

						isThereNoZero = (0 != mx) && (0 != my) && (0 != px) && (0 != py);

						if (true == isThereNoZero)
						{
							dmx = kx - mx;
							dmy = ky - my;
							dpx = kx - px;
							dpy = ky - py;

							if ((m_minDist2 > dmx*dmx + dmy*dmy) || (m_minDist2 > dpx*dpx + dpy*dpy))
							{
								flag4Adding = false;

								//cout<<"Warn: too close to the map features!"<<endl<<endl;
								//system("pause");

								break;
							}
						}
						else
						{
							flag4Adding = false;
						}

						map_p = map_p->next;
					}
				}

				if ((true == flag4Adding) && (0 != m_featuresAllInfo.size()))
				{
					for (vector<FeatureInfo>::size_type j=0; j<m_featuresAllInfo.size(); j++)
					{
						featuresInfo = m_featuresAllInfo[j];

						dx = kx - pixelPos.ptr<double>(0)[j];
						dy = ky - pixelPos.ptr<double>(1)[j];
						dist = dx*dx + dy*dy;

						if (dist < m_minDist2)
						{
							flag4Adding = false;

							//// store all the features' info for loop closure
							//m_formerFeatures.push_back(m_featuresAllInfo[j]);

							m_loopPointID.ptr<int>(m_loopPointCounter)[0] = m_featuresAllInfo[j].ID;

							m_loopPointCounter++;
							counter++;

							//cout<<"dx = "<<dx<<",  dy = "<<dy<<endl;
							//cout<<"Warn: already observed the feature!  dist = "<<dist<<endl<<endl;
							cout<<"Warn: already observed the feature!"<<endl<<endl;
							//system("pause");

							continue;
						}
					}
				}

				// Restrict the distance between two features
				if (true == flag4Adding)
				{
					for (vector<KeyPoint>::size_type j=0; j<m_keyPoints.size(); j++)
					{
						dx = kx-m_keyPoints[j].pt.x;
						dy = ky-m_keyPoints[j].pt.y;

						if (m_minDist2 > dx*dx+dy*dy)	
						{
							flag4Adding = false;
							break;
						}
					}
					if (true == flag4Adding)
					{
						m_keyPoints.push_back(kp);
						counter++;
					}
				}
			}
		}
	}
	flag = false;

	if (TRUE == isAdding)
	{
		m_nFilters     = counter;
		m_nAddings     = counter;
		m_nMapFeatures = counter;
	}
	else
	{
		m_nFilters     = counter;
		m_nAddings     = counter;
		m_nMapFeatures = m_nMapFeatures + m_nFilters;
	}
}

/**
* @brief Constraint checking.
* @param[in] none.
* @param[out] none.
* @note
*  To check whether have obtained enough features.
*/
void CSLAM::insureEnoughFeatures (void)
{
	int numberStore = m_nInitialRaws;
	double qualityLevelStore = m_qualityLevel;
	
	//int tmp = 1;

	while (m_nMapFeatures < m_minNUM)
	{
		m_nInitialRaws += m_minNUM;
		//m_qualityLevel -= 0.005;

		//cout<<tmp<<endl<<endl;
		//tmp++;
		//cout<<"/////////////////////"<<endl;
		//cout<<"m_nInitialRaws = "<<m_nInitialRaws<<endl;
		//cout<<"m_qualityLevel = "<<m_qualityLevel<<endl;
		//cout<<"m_nMapFeatures = "<<m_nMapFeatures<<endl<<endl;

		//if (m_qualityLevel <= 0.0)
		if (m_nInitialRaws > 30)
		{
			break;
		}

		vector<KeyPoint>().swap(m_keyPoints);
		detectAndfilteringFeatures();
	}

	m_nInitialRaws = numberStore;
	m_qualityLevel = qualityLevelStore;
}

/**
* @brief Integrating newly features' info.
* @param[in] none.
* @param[out] none.
* @note
*  Integrate the new feature's info into state and\n
*  square root of covariance.
*/
void CSLAM::integrateFeaturesInformation (void)
{
	if (0 == m_nAddings)
	{
		return;
	}

	Mat image(m_gryImage);
	int dim = m_X_k.rows;
	m_sample.num = dim + 3*m_nFilters;
	int Na = m_sample.num;
	int index = 0;
	int counter = 0;

	Mat mu2(3*m_nFilters, 1, CV_64F);
	Mat sr2 = Mat::zeros(3*m_nFilters, 3*m_nFilters, CV_64F);
	Mat cam_position = m_X_k.rowRange(dim-4, dim-1);
	Mat Rwc;
	getTransferMatrix(Rwc, m_X_k.ptr<double>(dim-1)[0]);

	if (TRUE == isAdding)
	{
		counter = m_nFilters - m_loopPointCounter;
	} 
	else
	{
		counter = m_nFilters;
	}

	for (int i = 0; i < counter; i++)
	{
		index = 3*i;

		mu2.ptr<double>(index+0)[0] = m_keyPoints[i].pt.x;
		mu2.ptr<double>(index+1)[0] = m_keyPoints[i].pt.y;
		mu2.ptr<double>(index+2)[0] = m_rho;

		sr2.ptr<double>(index+0)[index+0] = m_sigmaMeasure;
		sr2.ptr<double>(index+1)[index+1] = m_sigmaMeasure;
		sr2.ptr<double>(index+2)[index+2] = m_sigmaRHO;
	}

	Mat mu(Na, 1, CV_64F);
	Mat sr = Mat::zeros(Na, Na, CV_64F);
	Mat sigma_in(dim+3*m_nFilters, 2*Na+1, CV_64F);
	Mat sigma_out(dim+6*m_nFilters, 2*Na+1, CV_64F);
	Mat mu_Hlw(3*m_nFilters, 1, CV_64F);
	Mat mu_angle(3*m_nFilters, 1, CV_64F);

	calculateSampleParameter(Na);
	expandMatrix(mu, sr, m_X_k, m_S_k, mu2, sr2);
	generateSigmaPoints(sigma_in, mu, sr);			// sigma_in is disordered
	passSigmaThroughMapingFunction(sigma_out, mu_Hlw, mu_angle, sigma_in);
	QrAndCholeskyForInitilization(sigma_out);

	// ==========Integrate feature's information into features map=========== //
	Point2d uvd;
	double Hlw_x, Hlw_y, Hlw_z;
	
	m_filterID = Mat::zeros(m_nFilters, 1, CV_64F); // CV_16U:  0--65535

	Mat initInformation(6, 1, CV_64F);

	if (TRUE == isAdding)
	{
		while (NULL != map)
		{
			PointsMap* map_p = map->next;
			delete map;
			map = map_p;
		}
	}

	for (int i=0; i<counter; i++)
	{
		index = 3*i;

		uvd = m_keyPoints[i].pt;

		// uvd.x特征点在横轴（即图像的宽度范围）上的位置
		// uvd.y特征点在纵轴（即图像的高度范围）上的位置

		Hlw_x = mu_Hlw.ptr<double>(index+0)[0];
		Hlw_y = mu_Hlw.ptr<double>(index+1)[0];
		Hlw_z = mu_Hlw.ptr<double>(index+2)[0];

		m_filterID.ptr<double>(i)[0] = ID;

		cam_position.copyTo(initInformation.rowRange(0,3));
		mu_angle.rowRange(index, index+3).copyTo(initInformation.rowRange(3,6));
		
		// =============initialize parameters=============//
		PointsMap* map_new = new PointsMap();

		map_new->ID              = ID++;    
		map_new->isVisible       = false;
		map_new->isMatching      = false;
		map_new->isLoop          = false;
		map_new->nPredictTimes   = 0;
		map_new->nMatchTimes     = 0;
		map_new->xyz.x           = Hlw_x + cam_position.ptr<double>(0)[0]; 
		map_new->xyz.y           = Hlw_y + cam_position.ptr<double>(1)[0];
		map_new->xyz.z           = Hlw_z + cam_position.ptr<double>(2)[0];
		map_new->initPixel       = uvd;
		map_new->initTrans		 = cam_position;
		map_new->initRotation    = Rwc;
		map_new->initPatch       = 1*image(Rect(cvRound(uvd.x)-HP_INIT_W, cvRound(uvd.y)-HP_INIT_H,
								   2*HP_INIT_W+1, 2*HP_INIT_H+1));
		map_new->matchPatch      = Mat::zeros(2*HP_MATCH_W+1, 2*HP_MATCH_H+1, CV_8U);

		map_new->next = NULL;

		if ((0 == i) && ((1 == m_frame.counter) || (TRUE == isAdding))) // ======判断头结点????????
		{
			map =  map_new;
		} 
		else
		{
			PointsMap* map_p = map; //=========此法效率较低????????

			while(map->next)
			{
				map = map->next;
			}
			map->next = map_new;

			map = map_p;
		}
	}

	if ((TRUE == isAdding) && (0 != m_loopPointCounter))
	{
		Mat x_store, s_store;
		Mat state = Mat::zeros(6, 1, CV_64F);
		Mat sr    = Mat::zeros(6, 6, CV_64F);

		for (int i=0; i<m_loopPointCounter; i++)
		{
			dim = m_X_k.rows;

			vector<FeatureInfo>::iterator iter = m_featuresAllInfo.begin();

			int j = 0;
			while (m_featuresAllInfo[j].ID != m_loopPointID.ptr<int>(i)[0])
			{
				j++;
				iter++;
			}

			x_store = Mat::zeros(dim+6, 1, CV_64F);
			s_store = Mat::zeros(dim+6, dim+6, CV_64F);

			// Add the observed features into state and covariance
			state = m_featuresAllInfo[i].state;
			sr    = m_featuresAllInfo[i].sr;

			m_X_k.rowRange(0,dim-4).copyTo(x_store.rowRange(0,dim-4));
			state.copyTo(x_store.rowRange(dim-4,dim+2));
			m_X_k.rowRange(dim-4,dim).copyTo(x_store.rowRange(dim+2,dim+6));
			x_store.copyTo(m_X_k);
			
			m_S_k(Range(0,dim-4),Range(0,dim-4)).copyTo(s_store(Range(0,dim-4),Range(0,dim-4)));
			m_S_k(Range(0,dim-4),Range(dim-4,dim)).copyTo(s_store(Range(0,dim-4),Range(dim+2,dim+6)));
			m_S_k(Range(dim-4,dim),Range(0,dim-4)).copyTo(s_store(Range(dim+2,dim+6),Range(0,dim-4)));
			m_S_k(Range(dim-4,dim),Range(dim-4,dim)).copyTo(s_store(Range(dim+2,dim+6),Range(dim+2,dim+6)));
			sr.copyTo(s_store(Range(dim-4,dim+2),Range(dim-4,dim+2)));
			s_store.copyTo(m_S_k);

			// update map
			PointsMap* map_new = new PointsMap();

			map_new->ID              = m_featuresAllInfo[j].ID;   
			map_new->isVisible       = false;
			map_new->isMatching      = false;
			map_new->isLoop          = true;
			map_new->nPredictTimes   = 0;
			map_new->nMatchTimes     = 0;
			map_new->xyz             = m_featuresAllInfo[j].initXYZ;
			map_new->initPixel       = m_featuresAllInfo[j].initPixel;
			map_new->initTrans		 = m_featuresAllInfo[j].initTrans;
			map_new->initRotation    = m_featuresAllInfo[j].initRotation;
			map_new->initPatch       = m_featuresAllInfo[j].initPatch;
			map_new->matchPatch      = Mat::zeros(2*HP_MATCH_W+1, 2*HP_MATCH_H+1, CV_8U);
			map_new->next            = NULL;

			iter = m_featuresAllInfo.erase(iter);

			PointsMap* map_p = map;
			while (map->next)
			{
				map = map->next;
			}
			map->next = map_new;
			map = map_p;
		}

		getPermutationMatrix();
	}

	vector<KeyPoint>().swap(m_keyPoints);
}

/**
* @brief Rotation matrix computation.
* @param[in]
*  theta the angle of robot pose.
* @param[out]
*  Rwc the rotation of robot.
* @note
*  Get transformation matrix.\n
*  This is only contains two demisional rotation info due to\n
*  the robot is only move in a plat environment.
*/
void CSLAM::getTransferMatrix(Mat &Rwc, const double &theta) const
{
	Rwc = (Mat_<double>(3,3) <<
		cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1);
}

/**
* @brief Calculate sample parameter.
* @param[in]
*  Na the dimension of augment state vector.
* @param[out] none.
* @note
*  This are three different sets of parameters according to\n
*  1. SRUKF (author: Murray, et al.),
*  2. UKF (edition: 2000),
*  3. UKF (edition: 2004).
*/
void CSLAM::calculateSampleParameter(const int & Na)
{
	m_sample.Kappa  = 0;//pow(1.0/3.0, Na);
	m_sample.Lammda = pow(m_sample.Alpha, 2)*(Na + m_sample.Kappa) - Na;
	m_sample.Gamma  = sqrt(Na+m_sample.Lammda);
	m_sample.wm0    = m_sample.Lammda/(Na + m_sample.Lammda);
	m_sample.wc0    = m_sample.wm0 + (1 - pow(m_sample.Alpha,2) + m_sample.Beta);
	m_sample.wi     = 1.0/(2*(Na+m_sample.Lammda));
	//m_sample.wm0    = m_sample.Lammda/3.0;
	//m_sample.wc0    = m_sample.Lammda/3.0;
	//m_sample.wi     = 1.0/6.0;

	switch (m_weightType)
	{
	case 0: // From: SRUKF (author: Murray, et al.)
		{
			wm0    = 1.0 - Na/3.0;
			wm0_sr = sqrt(abs(wm0));
			wc0    = 1.0 - Na/3.0;
			wc0_sr = sqrt(abs(wm0));
			wi     = (1.0 - wc0)/(2*Na);
			wi_sr  = sqrt(wi);
			gamma  = sqrt(Na/(1.0 - wm0));
			
			break;
		}

	case 1: // From: UKF (edition: 2000)
		{
			gamma  = m_sample.Gamma;
			wm0    = m_sample.wm0;
			wm0_sr = sqrt(abs(m_sample.wm0));
			wc0    = m_sample.wc0;
			wc0_sr = sqrt(abs(m_sample.wc0));
			wi     = m_sample.wi;
			wi_sr  = sqrt(abs(m_sample.wi));

			break;
		}

	case 2: // From: UKF (edition: 2004)
		{
			gamma  = sqrt(3.0*Na/2.0);
			wm0    = 1.0/3.0;
			wm0_sr = sqrt(wm0);
			wc0    = 1.0/3.0;
			wc0_sr = sqrt(wc0);
			wi     = 1.0/(3.0*Na);
			wi_sr  = sqrt(wi);
			
			break;
		}
	}
}

/**
* @brief Matrix expanding.
* @param[in]
*  mu1 the mean of 1th state vector.
* @param[in]
*  sr1 the covariance of 1th state vector.
* @param[in]
*  mu2 the mean of 2th state vector.
* @param[in]
*  sr1 the covariance of 2th state vector.
* @param[out]
*  mu the mean of augmented state vector,
* @param[out]
*  sr the covariance of augmented state vector.
* @note
*  Integrating two matrix with the same dimensions\n
*  to generate another matrix.
*/
void CSLAM::expandMatrix (Mat &mu, Mat &sr, const Mat &mu1, const Mat &sr1, const Mat &mu2,
	const Mat &sr2) const
{
	int dim1 = mu1.rows;
	int dim2 = mu2.rows;
	int dim  = dim1 + dim2;

	mu1.copyTo(mu.rowRange(Range(0,dim1)));
	mu2.copyTo(mu.rowRange(Range(dim1,dim)));

	sr1.copyTo(sr(Range(0,dim1), Range(0,dim1)));
	sr2.copyTo(sr(Range(dim1,dim), Range(dim1,dim)));
}

/**
* @brief Generating sigma points.
* @param[in]
*  mu the mean of augmented state vector.
* @param[in]
*  sr the covariance of augmented state vector.
* @param[out]
*  sigma sigma points.
* @note
*  The process is according to general UKF process.
*/
void CSLAM::generateSigmaPoints (Mat &sigma, const Mat &mu, const Mat &sr)
{
	int Na    = mu.rows;

	mu.copyTo(sigma.col(0));
	Mat element(Na, 1, CV_64F);

	for (int i = 0; i < Na; i++)
	{
		element = sr.row(i).t() + 0;

		addWeighted(mu, 1, element, gamma, 0, sigma.col(i+1));
		addWeighted(mu, 1, element, (-1)*gamma, 0, sigma.col(Na+i+1)); 
	}	
}

/**
* @brief Features' info transformation.
* @param[in]
*  mu_Hlw the vector of all features' mean wrt world coordinate.
* @param[in]
*  mu_angle the vector of all features' mean.
* @param[in]
*  sigma_in the sigma points before transforming.
* @param[out]
*  sigma_out the sigma points after transforming.
* @note
*  Passing the generated sigma points through map extending function.
*/
void CSLAM::passSigmaThroughMapingFunction (Mat &sigma_out, Mat &mu_Hlw, Mat &mu_angle, const Mat &sigma_in)
{ 
	int dim   = m_X_k.rows;     // normal order of state vector before augmented
	int Na    = m_sample.num;
	int nCols = 2*Na + 1;

	//===========Generate augmented sigma-points=================//
	Mat cam_position = m_X_k.rowRange(dim-4,dim-1) + 0; 
	sigma_out.rowRange(Range(0,dim)) = sigma_in.rowRange(Range(0,dim)) + 0;
	//sigma_in.rowRange(Range(0,dim)).copyTo(sigma_out.rowRange(Range(0,dim))); 
	//repeat(cam_position, 1, nCols, sigma_out.rowRange(Range(N-3, N)));

	Point2d uvu, uvd;
	int index_in, index_out1, index_out2, index_Hlw;
	double rho;

	Mat Hlr, Hlw, state, position, Rwc;
	Mat mu(3*m_nFilters, 1, CV_64F);
	Mat x_new(dim+6*m_nFilters, 1, CV_64F);
	Mat sigma_Hlw(3*m_nFilters, nCols, CV_64F);
	Mat sigma_angle(3*m_nFilters, nCols, CV_64F);
	Mat element_Hlw(3*m_nFilters, 1, CV_64F);
	Mat element_angle(3*m_nFilters, 1, CV_64F);

	for (int i=0; i<nCols; i++)
	{
		position = sigma_in(Range(dim-4,dim-1), Range(i,i+1));
		getTransferMatrix(Rwc, sigma_in.ptr<double>(dim-1)[i]);

		for (int id=0; id<m_nFilters; id++)
		{
			index_in    = dim + 3*id;                             // index of sigma_in points
			index_out1  = index_in;						          // index of angle in sigma_out points
			index_out2  = index_in + 3*m_nFilters;                // index of location in sigma_out points
			index_Hlw   = 3*id;									  // index of Hlw set

			uvd.x = sigma_in.ptr<double>(index_in+0)[i];
			uvd.y = sigma_in.ptr<double>(index_in+1)[i];
			rho   = sigma_in.ptr<double>(index_in+2)[i];

			undistortOnePointRW(uvu, uvd);
			coordinatesImage2Camera(Hlr, uvu);
			coordinatesCamera2World(Hlw, Hlr, Rwc); 
			coordinatesWorld2State(state, Hlw, rho);

			state.copyTo(sigma_out(Range(index_out1,index_out1+3), Range(i,i+1)));
			position.copyTo(sigma_out(Range(index_out2,index_out2+3), Range(i,i+1)));

			sigma_Hlw(Range(index_Hlw, index_Hlw+3), Range(i,i+1))   = Hlw   + 0;
			sigma_angle(Range(index_Hlw, index_Hlw+3), Range(i,i+1)) = state + 0;
		}

		element_Hlw   = sigma_Hlw.col(i);
		element_angle = sigma_angle.col(i);

		if (!i)
		{
			addWeighted(element_Hlw, wm0, mu_Hlw, 0, 0, mu_Hlw);
			addWeighted(element_angle, wm0, mu_angle, 0, 0, mu_angle);
		}
		else
		{
			addWeighted(element_Hlw, wi, mu_Hlw, 1, 0, mu_Hlw);
			addWeighted(element_angle, wi, mu_angle, 1, 0, mu_angle);
		}
	}

	//========Update the state vector=============//
	m_X_k.copyTo(x_new.rowRange(Range(0,dim)));
	mu_angle.copyTo(x_new.rowRange(Range(dim,Na)));  // state vector of disordered //======是否只需计算一部分???????
	repeat(cam_position, m_nFilters, 1, x_new.rowRange(Range(Na,dim+6*m_nFilters)));

	x_new.copyTo(m_X_k);
}

/**
* @brief QR & Cholesky processing.
* @param[in]
*  the sigma points after transforming.
* @param[out] none.
* @note
*  QR decomposition and Cholesky updating for initialization.
*/
void CSLAM::QrAndCholeskyForInitilization (const Mat &sigma)
{
	int dim = m_X_k.rows;       // disordered 
	int Na  = m_sample.num; 

	// ========== prepare matrix for QR decomposition ==============//
	int dimx = 2*Na;  //==========用公式加速??????????
	int dimy = dim;  

	Mat QR(dimx, dimy, CV_64F);      // =========验证用，可删除???????????
	Mat sigma0 = sigma.col(0) + 0;

	for (int i = 0; i < dimx; i++)
	{
		QR.row(i) = wi_sr*(sigma.col(i+1) - sigma0).t();
	}
	GSLQrDecomposition(m_S_k, QR);
	//GSLModifiedQRDecomposition(m_S_k,QR);

	//================== get the permutation matrix ===============//
	if (1 == m_frame.counter)
	{
		getPermutationMatrix();
	}
	else 
	{
		if (0 != m_nAddings)
		{
			getPermutationMatrix();
		}
	}

	//============Reorder the state vector ============//
	m_X_k = m_permutation*m_X_k; // normal order
	GSLQrDecomposition(m_S_k, m_permutation*m_S_k*m_permutation.t());
	//m_S_k = m_permutation*m_S_k*m_permutation.t();

	//Mat sr_store(dim, dim, CV_64F);
	//GSLQrDecomposition(sr_store, m_S_k);
	//sr_store.copyTo(m_S_k);
}

// Get the permutation matrix to make the disordered state vector into normal order
void CSLAM::getPermutationMatrix (void)
{   
	int dim = m_X_k.rows;  //========可删除z、rho?????????

	m_permutation = Mat::zeros(dim, dim, CV_64F);

	int dimOld = dim - 6*m_nFilters;  // Dimension of state vector before adding features
	int M      = m_nFilters;

	if (dimOld != 4)														    // for unchanged part
	{
		Mat I = Mat::eye(dimOld-4, dimOld-4, CV_64F);
		I.copyTo(m_permutation(Range(0,dimOld-4), Range(0,dimOld-4)));
	}

	// For robot's pose
	m_permutation.ptr<double>(dim-4)[dimOld-4] = 1;	// x						 
	m_permutation.ptr<double>(dim-3)[dimOld-3] = 1; // y
	m_permutation.ptr<double>(dim-2)[dimOld-2] = 1; // z
	m_permutation.ptr<double>(dim-1)[dimOld-1] = 1; // theta

	for (int id = 0; id < M; id++)
	{
		// For features' parameters
		m_permutation.ptr<double>(dimOld-4+6*id+0)[dimOld+3*M+3*id+0] = 1; // x
		m_permutation.ptr<double>(dimOld-4+6*id+1)[dimOld+3*M+3*id+1] = 1; // y
		m_permutation.ptr<double>(dimOld-4+6*id+2)[dimOld+3*M+3*id+2] = 1; // z
		m_permutation.ptr<double>(dimOld-4+6*id+3)[dimOld+3*id+0]     = 1; // theta
		m_permutation.ptr<double>(dimOld-4+6*id+4)[dimOld+3*id+1]     = 1; // phi
		m_permutation.ptr<double>(dimOld-4+6*id+5)[dimOld+3*id+2]     = 1; // tho
	}
}

/**
* @brief Motion prediction.
* @param[in] none.
* @param[out] none.
* @note
*  The dynamic motion model is odometry model.
*/
void CSLAM::predictMotion (void)
{
	if (1 == m_frame.counter)
	{
		m_nMapFeatures = m_nStore4Show;
		m_nShowMap = m_nMapFeatures;
	}

	m_frame.index = static_cast<int>(m_odoTheta.ptr<double>(0)[m_frame.counter]);

	// ============ For robot's redirection ========= //
	if (1 == m_odoTheta.ptr<double>(2)[m_frame.counter])
	{
		// Adding the redirection features into the vector
		int id = 0;
		FeatureInfo featuresInfo;
		PointsMap* map_p = map;
		while (NULL != map_p)
		{
			featuresInfo.ID            = map_p->ID;
			featuresInfo.isLoop        = map_p->isLoop;
			featuresInfo.nPredictTimes = map_p->nPredictTimes;
			featuresInfo.nMatchTimes   = map_p->nMatchTimes;
			featuresInfo.initXYZ       = map_p->xyz;
			featuresInfo.initPixel     = map_p->initPixel;
			featuresInfo.initTrans     = map_p->initTrans;
			featuresInfo.initRotation  = map_p->initRotation;
			featuresInfo.initPatch     = map_p->initPatch;
			featuresInfo.state         = m_X_k.rowRange(6*id, 6*id+6) + 0;
			getFeatureCartesianInformation(featuresInfo.position, featuresInfo.sr, featuresInfo.cov, id);
			get3DdisplayInformation(featuresInfo.axis, featuresInfo.sigma, featuresInfo.cov);
			m_featuresAllInfo.push_back(featuresInfo);

			id++;
			map_p = map_p->next;
		}

		// Loading new iamge and start a new SLAM process
		if (FLAG_4_VIDEO == m_playType)
		{
			m_srcImage = cvQueryFrame(m_capture);
		} 
		else
		{
			char image_name[100];
			sprintf_s(image_name, image_dir, m_frame.index);
			m_srcImage = cvLoadImage(image_name);
		}

		cvCvtColor(m_srcImage, m_gryImage, CV_RGB2GRAY);

		// Initilize all parameters for the new cycle
		m_initPos = m_X_k.rowRange(m_X_k.rows-4, m_X_k.rows-2) + 0;

		m_X_k = Mat::zeros(4, 1, CV_64F);
		m_initPos.copyTo(m_X_k.rowRange(0,2));
		m_X_k.ptr<double>(3)[0] = m_odoTheta.ptr<double>(1)[m_frame.counter];

		m_S_k = Mat::zeros(4, 4, CV_64F);
		m_S_k.ptr<double>(0)[0] = m_sigmaX;
		m_S_k.ptr<double>(1)[1] = m_sigmaY;
		m_S_k.ptr<double>(2)[2] = m_sigmaZ;
		m_S_k.ptr<double>(3)[3] = m_sigmaTheta;

		m_nStoreMap      = m_nMapFeatures;
		m_nStorePredicts = m_nPredicts;
		m_nStoreMatches  = m_nMatches;

		m_nFilters	   = 0;
		m_nDeletes     = 0;
		m_nMapFeatures = 0;
		m_nPredicts	   = 0;
		m_nMatches	   = 0;

		isAdding = TRUE;
		addFeatures();
		isAdding = FALSE;

		m_nShowMap = m_nMapFeatures + m_nStoreMap;

		m_frame.counter++;
		m_frame.index = static_cast<int>(m_odoTheta.ptr<double>(0)[m_frame.counter]);

		int dim = m_X_k.rows;
		m_X_k.ptr<double>(dim-1)[0] = m_odoTheta.ptr<double>(1)[m_frame.counter];
	}

	// ============Motion prediction========== //
	int dim = m_X_k.rows;
	m_sample.num = dim + 3 + 2;
	int Na  = m_sample.num;

	Mat Z2 = Mat::zeros(2, 1, CV_64F);
	Mat Z3 = Mat::zeros(3, 1, CV_64F);
	Mat mu1(dim+3, 1, CV_64F);
	Mat mu(Na, 1, CV_64F);
	Mat sr1 = Mat::zeros(dim+3, dim+3, CV_64F);
	Mat sr  = Mat::zeros(Na, Na, CV_64F);

	m_sigma = Mat::zeros(Na, 2*Na+1, CV_64F);

	int index = 2*m_frame.counter;

	double dx    = *(m_odoXY+index+0) - *(m_odoXY+index-2);
	double dy    = *(m_odoXY+index+1) - *(m_odoXY+index-1);
	double rot1  = atan2(dy, dx) - m_odoTheta.ptr<double>(1)[m_frame.counter-1];
	double trans = sqrt(dy*dy + dx*dx);
	double rot2  = m_odoTheta.ptr<double>(1)[m_frame.counter] - m_odoTheta.ptr<double>(1)[m_frame.counter-1] - rot1;

	Ut.ptr<double>(0)[0] = rot1;
	Ut.ptr<double>(1)[0] = trans;
	Ut.ptr<double>(2)[0] = rot2;

	Mt.ptr<double>(0)[0] = a1*rot1*rot1   + a2*trans*trans;
	Mt.ptr<double>(1)[1] = a3*trans*trans + a4*rot1*rot1 + a4*rot2*rot2;
	Mt.ptr<double>(2)[2] = a1*rot2*rot2   + a2*trans*trans;

	calculateSampleParameter(Na);
	expandMatrix(mu1, sr1, m_X_k, m_S_k, Z3, Mt);
	expandMatrix(mu, sr, mu1, sr1, Z2, Qt);
	generateSigmaPoints(m_sigma, mu, sr);
	passSigmaThroughMotionFunction(Ut);
	QrAndCholeskyForMotion(); 
}

/**
* @brief Calculating means.
* @param[in]
*  Ut control input.
* @param[out] none.
* @note
*  Passing sigma-points through motion function.
*/
void CSLAM::passSigmaThroughMotionFunction (const Mat &Ut)
{
	int dim   = m_X_k.rows;
	int Na    = m_sample.num;
	int nCols = 2*Na + 1; 

	// ==========Generate sigma-points=========== //
	double rot1, trans, rot2;
	Mat element_update(4, 1, CV_64F);
	Mat element(4, 1, CV_64F);
	Mat mu(4, 1, CV_64F);
	
	for (int i=0; i<nCols; i++)
	{
		if (FLAG_4_NOISE1 == m_noiseType)
		{
			rot1  = Ut.ptr<double>(0)[0] - m_sigma.ptr<double>(dim+0)[i];//*m_sigma.ptr<double>(dim+0)[i];
			trans = Ut.ptr<double>(1)[0] - m_sigma.ptr<double>(dim+1)[i];//*m_sigma.ptr<double>(dim+1)[i];
			rot2  = Ut.ptr<double>(2)[0] - m_sigma.ptr<double>(dim+2)[i];//*m_sigma.ptr<double>(dim+2)[i];
			//rot1  = Ut.ptr<double>(0)[0] - Mt.ptr<double>(0)[0];//*m_sigma.ptr<double>(dim+0)[i];
			//trans = Ut.ptr<double>(1)[0] - Mt.ptr<double>(1)[1];//*m_sigma.ptr<double>(dim+1)[i];
			//rot2  = Ut.ptr<double>(2)[0] - Mt.ptr<double>(2)[2];//*m_sigma.ptr<double>(dim+2)[i];
			//rot1  = Ut.ptr<double>(0)[0] - Gaussian(m_sigma.ptr<double>(dim+0)[i]);//*m_sigma.ptr<double>(dim+0)[i];
			//trans = Ut.ptr<double>(1)[0] - Gaussian(m_sigma.ptr<double>(dim+1)[i]);//*m_sigma.ptr<double>(dim+1)[i];
			//rot2  = Ut.ptr<double>(2)[0] - Gaussian(m_sigma.ptr<double>(dim+2)[i]);//*m_sigma.ptr<double>(dim+2)[i];
			//rot1  = Ut.ptr<double>(0)[0] - Gaussian(Mt.ptr<double>(0)[0]);//*m_sigma.ptr<double>(dim+0)[i];
			//trans = Ut.ptr<double>(1)[0] - Gaussian(Mt.ptr<double>(1)[1]);//*m_sigma.ptr<double>(dim+1)[i];
			//rot2  = Ut.ptr<double>(2)[0] - Gaussian(Mt.ptr<double>(2)[2]);//*m_sigma.ptr<double>(dim+2)[i];
		} 
		else if (FLAG_4_NOISE2 == m_noiseType)
		{
			rot1  = Ut.ptr<double>(0)[0] - Gauss(m_sigma.ptr<double>(dim+0)[i]);//*m_sigma.ptr<double>(dim+0)[i]);
			trans = Ut.ptr<double>(1)[0] - Gauss(m_sigma.ptr<double>(dim+1)[i]);//*m_sigma.ptr<double>(dim+1)[i]);
			rot2  = Ut.ptr<double>(2)[0] - Gauss(m_sigma.ptr<double>(dim+2)[i]);//*m_sigma.ptr<double>(dim+2)[i]);
		}
		else if (FLAG_4_NOISE3 == m_noiseType)
		{
			rot1  = Ut.ptr<double>(0)[0] - rng.gaussian(m_sigma.ptr<double>(dim+0)[i]);//*m_sigma.ptr<double>(dim+0)[i]);
			trans = Ut.ptr<double>(1)[0] - rng.gaussian(m_sigma.ptr<double>(dim+1)[i]);//*m_sigma.ptr<double>(dim+1)[i]);
			rot2  = Ut.ptr<double>(2)[0] - rng.gaussian(m_sigma.ptr<double>(dim+2)[i]);//*m_sigma.ptr<double>(dim+2)[i]);
		}

		element_update.ptr<double>(0)[0] = trans*cos(m_sigma.ptr<double>(dim-1)[i] + rot1);
		element_update.ptr<double>(1)[0] = trans*sin(m_sigma.ptr<double>(dim-1)[i] + rot1);
		element_update.ptr<double>(2)[0] = 0;
		element_update.ptr<double>(3)[0] = rot1 + rot2;

		m_sigma(Range(dim-4,dim),Range(i,i+1)) += element_update; 
		element = m_sigma(Range(dim-4,dim), Range(i,i+1)) + 0;

		if (!i)
			addWeighted(element, wm0, mu, 0, 0, mu);
		else
			addWeighted(element, wi, mu, 1, 0, mu);
	}
	mu.copyTo(m_X_k.rowRange(Range(dim-4,dim)));
}

/**
* @brief Calculating covariance.
* @param[in] none.
* @param[out] none.
*/
void CSLAM::QrAndCholeskyForMotion (void)
{
	int dim  = m_X_k.rows;       
	int Na   = m_sample.num; // augmented dim of sigma points before passing through func 
	int dimx = 2*Na;
	int dimy = dim;

	// ================================== //
	Mat QR(dimx, dimy, CV_64F); 
	Mat sigma0 = m_sigma(Range(0,dim), Range(0,1)) + 0;

	for (int i = 0; i < dimx; i++)
	{
		QR.row(i) = wi_sr*(m_sigma(Range(0,dim), Range(i+1,i+2)) - sigma0).t();
	}
	//============此处可加速????????????
	GSLQrDecomposition(m_S_k, QR);
	//GSLModifiedQRDecomposition(m_S_k, QR);
	// ================================== //

	// ================================== //
	//Mat QR(dimx, dimy, CV_64F);      // =========验证用，可删除???????????
	//for (int i = 0; i < dimx; i++)
	//{
	//	QR.row(i) = wi_sr*(m_sigma(Range(0,dim), Range(i+1,i+2)) - m_X_k).t();
	//}
	//Mat e0 = wc0_sr*(m_sigma(Range(0,dim),Range(0,1)) - m_X_k); 

	//Mat Cov;
	//if (wc0 > 0.0)
	//{
	//	GSLQrDecomposition(m_S_k, QR);
	//	Cov = m_S_k.t()*m_S_k + e0*e0.t();
	//	Cov = QR.t()*QR + e0*e0.t();  
	//	m_covRank = dim - 3*m_nMapFeatures;
	//} 
	//else
	//{
	//	GSLQrDecomposition(m_S_k, QR);
	//	Cov = m_S_k.t()*m_S_k - e0*e0.t();
	//	Cov = QR.t()*QR - e0*e0.t();
	//	m_covRank = dim - 3*m_nFilters;
	//}

	//getPermutationMatrix();
	//Cov = m_permutation.t()*Cov*m_permutation;  // normal to normal order

	//Mat S_disordered = Mat::zeros(dim, dim, CV_64F);
	//CholeskyDecompositionWithPivoting(S_disordered, Cov);

	//m_S_k = m_permutation*S_disordered*m_permutation.t(); // disorder to normal order

	//Mat s_store = Mat::zeros(dim, dim, CV_64F);
	//GSLQrDecomposition(s_store, m_S_k);
	//s_store.copyTo(m_S_k);
	// ================================== //
}

/**
* @brief Measurement prediction. 
* @param[in] none.
* @param[out] none.
* @note 
*  Consist two processes: mean & covariances calculating process.
*/
void CSLAM::predictMeasurement (void)
{
	passSigmaThroughMesaurementFunction(); 
	QrAndCholeskyForMeasurement();
}

/**
* @brief Calculating means.
* @param[in] none.
* @param[out] none.
*/
void CSLAM::passSigmaThroughMesaurementFunction (void)
{
	int dim   = m_X_k.rows;
	int Na    = m_sample.num;
	int nCols = 2*Na + 1;

	m_sigma_allPixel.create(2*m_nMapFeatures, nCols, CV_64F);

	// ==========Passing sigma-points through measurement function============ //
	PointsMap* map_p;
	Point2d uvu, uvd;
	int id, index_in, index_out;

	Mat position, Rwc, Rcw, state, Hlw, Hlr;
	Mat error(2, 1, CV_64F);
	Mat element(2*m_nMapFeatures, 1, CV_64F);

	m_allPredictSet.create(2*m_nMapFeatures, 1, CV_64F);

	for (int i = 0; i < nCols; i++)
	{
		
		error = m_sigma(Range(dim+3, dim+5), Range(i, i+1)) + 0;
		//error = Mat::zeros(2, 1, CV_64F);

		position = m_sigma(Range(dim-4,dim-1), Range(i,i+1)) + 0;

		getTransferMatrix(Rwc, m_sigma.ptr<double>(dim-1)[i]);
		Rcw = Rwc.inv();

		id = 0;
		map_p = map;
		while (NULL != map_p)
		{
			//if (true == map_p->isDelay)
			//{
			//	id++;
			//	map_p = map_p->next;
			//	continue;
			//}

			//error.ptr<double>(0)[0] = Gaussian(m_sigma.ptr<double>(dim+3)[i]);
			//error.ptr<double>(1)[0] = Gaussian(m_sigma.ptr<double>(dim+4)[i]); 

			index_in  = 6*id;
			index_out = 2*id;

			state = m_sigma(Range(index_in,index_in+6), Range(i,i+1));

			coordinatesState2World(Hlw, state, position);
			coordinatesWorld2Camera(Hlr, Hlw, Rcw);
			coordinatesCamera2Image(uvu, Hlr, error);
			distortOnePointRW(uvd, uvu);

			m_sigma_allPixel.ptr<double>(index_out+0)[i] = uvd.x;
			m_sigma_allPixel.ptr<double>(index_out+1)[i] = uvd.y;

			id++;
			map_p = map_p->next;
		}

		element = m_sigma_allPixel.col(i);

		if (!i)
			addWeighted(element, wm0, m_allPredictSet, 0, 0, m_allPredictSet);
		else
			addWeighted(element,  wi, m_allPredictSet, 1, 0, m_allPredictSet);
	}

	//PointsMap* tmp_map = map;
	//while (NULL != tmp_map)
	//{
	//	cout<<tmp_map->initPixel.x<<"  "<<tmp_map->initPixel.y<<endl<<endl;
	//	tmp_map = tmp_map->next;
	//}

}

/**
* @brief Calculating all features' covariance.
* @param[in] none.
* @param[out] none.
* @note
*  This part is used for calculating all feature' means.
*/
void CSLAM::QrAndCholeskyForMeasurement (void)
{
	double px, py;

	m_nPredicts = 0;

	int index_visible = 0;

	Mat si(2, 2, CV_64F);

	int id = 0;
	PointsMap *map_p = map;
	while (NULL != map_p)
	{
		//if (true == map_p->isDelay)
		//{
		//	map_p->nDelay++;

		//	id++;
		//	map_p = map_p->next;

		//	continue;
		//}

		px = m_allPredictSet.ptr<double>(2*id+0)[0];
		py = m_allPredictSet.ptr<double>(2*id+1)[0];

		if (px != 0 && py != 0)
		{
			m_nPredicts++;
			map_p->isVisible  = true; //=======需修改距离边界的条件????????
			map_p->isMatching = false;
			map_p->nPredictTimes++;
			map_p->predictLocation.x = px;
			map_p->predictLocation.y = py;

			calculateOneFeatureCovariance(si, id);  // =========若不需画2D图，则不需要计算????????

			si.copyTo(map_p->Si);

			index_visible += 2;
		}

		id++;
		map_p = map_p->next;
	}

	m_nShowPredicts = m_nPredicts + m_nStorePredicts;
}

/**
* @brief Calculating each feature's covariance.
* @param[in]
*  id the id of features.
* @param[out]
*  si the covariance of each feature.
* @note
*  This part is used for calculating each feature's covariance.
*/
void CSLAM::calculateOneFeatureCovariance (Mat &si, const int &id)
{
	int dim	  = m_X_k.rows;
	int Na    = m_sample.num;
	int index = 2*id;
	int dimx  = 2*Na;
	int dimy  = 2;
	Mat QR(dimx, dimy, CV_64F);

	//===============================================//
	Mat sigma0 = m_sigma_allPixel(Range(index, index+2), Range(0,1)) + 0;

	for (int i=0; i < 2*Na; i++)
	{
		QR.row(i) = wi_sr*(m_sigma_allPixel(Range(index,index+2), Range(i+1,i+2)) - sigma0).t();
	}
	GSLQrDecomposition(si, QR);
	//===============================================//

 	//=================实验验证=====================//
 	//Mat ei(dimy, 1, CV_64F);
 
 	//for (int i=0; i < 2*Na; i++)
 	//{
 	//	ei = wi_sr*(m_sigma_allPixel(Range(index,index+2), Range(i,i+1)) - m_allPredictSet.rowRange(index,index+2)).t();
 	//	ei.copyTo(QR.rowRange(i,i+1));
 	//}
 
 	//GSLQrDecomposition(si, QR);
 
 	//Mat e0(dimy, 1, CV_64F);
 	//subtract(m_sigma_allPixel(Range(index, index+2), Range(0,1)), m_allPredictSet.rowRange(index,index+2), e0);
 	//Mat tmp = si.t()*si - e0*e0.t();
 	////GSLCholeskyDecomposition(si, tmp);
 	//modifiedCholeskyDecomposition(si, tmp);
 	//===================实验验证===================//
}

/**
* @brief Patch warp.
* @param[in] none.
* @param[out] none.
* @note
*  Get the patch around the predict feature.
*/
void CSLAM::wrapPatch (void)
{
	int dim = m_X_k.rows;                         
	Mat Rwc;
	getTransferMatrix(Rwc, m_X_k.ptr<double>(dim-1)[0]);

	Mat cam_position = m_X_k.rowRange(dim-4,dim-1);
	Mat K = Mat(cam_K);

	PointsMap* map_p = map;
	while (NULL != map_p)
	{
		//if (true == map_p->isDelay)
		//{
		//	map_p = map_p->next;
		//	continue;
		//}

		Mat Rotate_C0_W = Mat::eye(4,4,CV_64F);
		Mat Rotate_C1_W = Mat::eye(4,4,CV_64F);

		Rotate_C0_W(Range(0,3),Range(0,3)) = map_p->initRotation + 0;
		Rotate_C0_W(Range(0,3),Range(3,4)) = (map_p->initRotation)*(map_p->initTrans) + 0;
		Rotate_C1_W(Range(0,3),Range(0,3)) = Rwc + 0;
		Rotate_C1_W(Range(0,3),Range(3,4)) = Rwc*cam_position + 0;

		Mat Rotate_C1_C0 = (Rotate_C0_W.inv())*Rotate_C1_W;
		Mat R_C1_C0      = Rotate_C1_C0(Range(0,3),Range(0,3));
		Mat r_C1_C0      = Rotate_C1_C0(Range(0,3),Range(3,4));

		Mat n0 = (Mat_<double>(3,1) << map_p->initPixel.x-cam_cx, map_p->initPixel.y-cam_cy, -cam_f1); 
		Mat n1_tmp0 = (Mat_<double>(4,1) <<
			map_p->predictLocation.x-cam_cx, map_p->predictLocation.y-cam_cy, -cam_f1, 1);
		Mat n1_tmp1 = Rotate_C1_C0*n1_tmp0;
		n1_tmp1 /= (n1_tmp1.at<double>(3));
		Mat n1_n0 = n1_tmp1(Range(0,3),Range::all());
		n0 /= (::norm(n0));
		n1_n0 /= (::norm(n1_n0));
		Mat n = n0+n1_n0;
		n /= (::norm(n));

		Mat XYZ_W = (Mat_<double>(4,1) <<map_p->xyz.x, map_p->xyz.y, map_p->xyz.z, 1);
		Mat XYZ_C0 = (Rotate_C0_W.inv())*XYZ_W;
		XYZ_C0 /= (XYZ_C0.at<double>(3));
		Mat XYZ = XYZ_C0(Range(0,3),Range::all()) + 0;
		Mat tmp = (-1)*n.t()*XYZ;
		double d = tmp.at<double>(0);

		Point2d uv_C0,uv_C1;
		undistortOnePointRW(uv_C1,map_p->initPixel);

		Mat uv_c1_tmp = (Mat_<double>(3,1) << uv_C1.x, uv_C1.y, 1);
		Mat uv_c1 = K*(R_C1_C0-r_C1_C0*(n.t())/d)*(K.inv());
		Mat uv_c1_und = (uv_c1.inv())*uv_c1_tmp;
		uv_c1_und /= (uv_c1_und.at<double>(2));

		uv_C0.x = uv_c1_und.at<double>(0);
		uv_C0.y = uv_c1_und.at<double>(1);
		distortOnePointRW(uv_C1,uv_C0);

		Point2d uv = uv_C1;
		for (int i=0; i<2*HP_MATCH_W+1; i++)
		{
			for (int j=0; j<2*HP_MATCH_H+1; j++)
			{
				uv_C0.x = uv.x-HP_MATCH_W + i;
				uv_C0.y = uv.y-HP_MATCH_H + j;
				undistortOnePointRW(uv_C1,uv_C0);

				Mat uv_c0_tmp = (Mat_<double>(3,1) << uv_C1.x, uv_C1.y, 1);
				Mat uv_c0_und = K*(R_C1_C0-r_C1_C0*(n.t())/d)*(K.inv())*uv_c0_tmp;
				uv_c0_und /= (uv_c0_und.at<double>(2));

				uv_C0.x = uv_c0_und.at<double>(0);
				uv_C0.y = uv_c0_und.at<double>(1);
				distortOnePointRW(uv_C1,uv_C0);

				uv_C1.x -= (map_p->initPixel.x-HP_INIT_W - 1);
				uv_C1.y -= (map_p->initPixel.y-HP_INIT_H - 1);

				int left_x = cvFloor(uv_C1.x);
				int left_y = cvFloor(uv_C1.y);
				int right_x = cvCeil(uv_C1.x);
				int right_y = cvCeil(uv_C1.y);

				if (left_x >= 0 && right_x < 2*HP_INIT_W && left_y >= 0 && right_y < 2*HP_INIT_H)
				{
					double rate_lx = right_x-uv_C1.x;
					double rate_ly = right_y-uv_C1.y;
					double rate_rx = 1.0-rate_lx;
					double rate_ry = 1.0-rate_ly;
					uchar left_left = map_p->initPatch.at<uchar>(left_x,left_y);
					uchar left_right = map_p->initPatch.at<uchar>(left_x,right_y);
					uchar right_left = map_p->initPatch.at<uchar>(right_x,left_y);
					uchar right_right = map_p->initPatch.at<uchar>(right_x,right_y);
					map_p->matchPatch.at<uchar>(i,j) = uchar(left_left*rate_lx*rate_ly
						+left_right*rate_lx*rate_ry+right_left*rate_rx*rate_ly+right_right*rate_rx*rate_ry);
				}
			}
		}
		map_p = map_p->next;
	}
}

/**
* @brief Data association
* @param[in] none.
* @param[out] none.
* @note
*  This process is according to image patch matching.
*/
void CSLAM::dataAssociation (void)
{
	Mat image(m_gryImage);
	
	int dim = m_X_k.rows;
	int Na  = m_sample.num; // dim of augmented matrix                   // index of matching sigma points set

	Point maxLoc;
	int id, index_set, half_x, half_y;
	double px, py, mx, my, maxVal;
	Mat Rwc, pi, kp_descriptors, descriptor, roi;
	vector<KeyPoint> keyPoints;

	getTransferMatrix(Rwc, m_X_k.ptr<double>(dim-1)[0]);
	wrapPatch();

	m_nMatches = 0;
	m_matchID.create(m_nPredicts, 1, CV_64F);

	id = 0;
	index_set = 0;
	PointsMap* map_p = map;
	while (NULL != map_p)
	{
		//if (true == map_p->isDelay)
		//{
		//	id++;
		//	map_p = map_p->next;
		//	continue;
		//}
		
		if (true == map_p->isVisible)
		{
			px = map_p->predictLocation.x;
			py = map_p->predictLocation.y;
			pi = (map_p->Si.t())*(map_p->Si);

			half_x = cvCeil(2*map_p->Si.ptr<double>(0)[0]);
			half_y = cvCeil(2*map_p->Si.ptr<double>(1)[1]);
			half_x = min(HP_INIT_W, max(HP_MATCH_W, half_x));
			half_y = min(HP_INIT_H, max(HP_MATCH_H, half_y));

			Mat correlation = Mat::zeros(2*half_y+1, 2*half_x+1, CV_64F);

			// find ROI patch with high similarity according to the predict feature's covariance
			for (int i=(int)px-half_x; i<=(int)px+half_x; i++)
			{
				if (i<HP_MATCH_W || i> imageWidth-HP_MATCH_W-1)	
				{
					continue;
				}

				for (int j=(int)py-half_y; j<=(int)py+half_y; j++)
				{
					if (j<HP_MATCH_H || j> imageHeight-HP_MATCH_H-1)	
					{
						continue;
					}

					Mat err = (Mat_<double>(1,2) << i-px, j-py); 
					Mat pii = err*(pi.inv())*(err.t());          // Mahalanobis diatance

					if (pii.ptr<double>(0)[0] < CHI2INV_TABLE(0,2))
					{
						roi = image(Rect(i-HP_MATCH_W,j-HP_MATCH_H, 2*HP_MATCH_W+1, 2*HP_MATCH_H+1)) + 0;
						calculateCrossCorrelation(correlation.ptr<double>(j-(int)py+half_y)[i-(int)px+half_x],
							roi, map_p->matchPatch);
					}
				}
			}

			maxVal = 0.0;
			minMaxLoc(correlation, NULL, &maxVal, NULL, &maxLoc);

			if (maxVal >THRESHOLD_MATCH_PATCH)
			{
				mx = maxLoc.x - half_x + px;
				my = maxLoc.y - half_y + py;

				m_matchID.ptr<double>(m_nMatches)[0] = map_p->ID;

				m_nMatches++;
				map_p->nMatchTimes++;
				map_p->isMatching = true;
				map_p->matchLocation.x = mx;
				map_p->matchLocation.y = my;
			}
		}
		id++;
		map_p = map_p->next;
	}

	m_matchID = m_matchID.rowRange(0, m_nMatches) + 0;
	m_nShowMatches = m_nMatches + m_nStoreMatches;
}

/**
* @brief Calculating means & covariance.
* @param[in]
*  id the id of each feature.
* @param[in]
*  hi the prediction of each feature.
* @param[out]
*  Pxy the cross covariance of each feature.
*/
void CSLAM::calculateOneFeatureCrossCovariance (Mat &Pxy, const int &id, const Mat &hi) const
{
	int dim   = m_X_k.rows;
	int Na    = m_sample.num;
	int index = 2*id;
	Mat sub1(dim, 1, CV_64F);
	Mat sub2(2, 1, CV_64F);

	for (int i = 0; i <= 2*Na; i++)
	{
		sub1 = m_sigma(Range(0,dim), Range(i,i+1)) - m_X_k;
		sub2 = m_sigma_allPixel(Range(index,index+2), Range(i,i+1)) - hi;

		if (!i)
			Pxy = wc0*sub1*sub2.t() + 0;
		else
			Pxy += wi*sub1*sub2.t();
	}
}

/**
* @brief Kalman Updating.
* @param[in] none.
* @param[out] none.
* @note
*  The switches can be used to decide whether use 1-point RANSAC,\n
*  update at one step method or not.
*/
void CSLAM::KalmanUpdate (void)
{
	if (0 == m_nMatches)
		return;

	int dim = m_X_k.rows;
	int index = 0;

	if (FALSE == isUseRANSAC)
	{
		Mat zi, hi, si, sii;
		Mat Pxy(dim, 2, CV_64F);
		Mat Ki(dim, 2, CV_64F);
		Mat U(dim, 2, CV_64F);

		int id = 0;
		PointsMap *map_p = map;

		while (NULL != map_p)
		{
			if ((true == map_p->isMatching))// && (false == map_p->isDelay))
			{
				zi = (Mat_<double>(2,1) << map_p->matchLocation.x,   map_p->matchLocation.y); 
				hi = (Mat_<double>(2,1) << map_p->predictLocation.x, map_p->predictLocation.y);

				map_p->Si.copyTo(si);

				calculateOneFeatureCrossCovariance(Pxy, id, hi);

				sii    = si.inv();
				Ki     = Pxy*sii*sii.t();
				m_X_k += Ki*(zi - hi);
				U      = Ki*si.t();
				

				if (0 != m_nAddings)
				{
					GSLCholeskyUpdate(U, FLAG_4_DOWNDATING, FLAG_4_NEED_REORDER);
				}
				else
				{
					GSLCholeskyUpdate(U, FLAG_4_DOWNDATING, FLAG_4_NEEDNOT_REORDER);
				}
				
			}
			id++;
			map_p = map_p->next;
		}
	}
	else if (TRUE == isUseRANSAC)
	{
		//onePointRansacHypotheses();
		//updateLowInnovationInliers();
		//rescueHighInnovationInliers();
		//updateHighInnovationInliers();
	}
}

void CSLAM::GSLCholeskyUpdate (const Mat &u, const int &flag4UpOrDown, const int &flag4Order)
{
	int dim   = m_X_k.rows;
	int nCols = u.cols;

	Mat u_Col(dim, 1, CV_64F);         // each column of input matrix u
	Mat	src1(dim, dim, CV_64F);
	Mat src2(dim, dim, CV_64F);
	Mat dst(dim, dim, CV_64F);

	for (int i = 0; i < nCols; i++)
	{
		src1  = m_S_k.t()*m_S_k;
		u_Col = u.col(i) + 0;
		src2  = u_Col*u_Col.t();

		if (flag4Order == FLAG_4_NEED_REORDER)
		{
			if (flag4UpOrDown == FLAG_4_UPDATING)
			{
				m_covRank = dim - 3*m_nMapFeatures;
				dst = m_permutation.t()*(src1 + src2)*m_permutation + 0;
			} 
			else
			{
				m_covRank = dim - 3*m_nFilters;
				dst = m_permutation.t()*(src1 - src2)*m_permutation + 0;
			}

			Mat S_disordered(dim, dim, CV_64F);
			CholeskyDecompositionWithPivoting(S_disordered, dst);
			GSLQrDecomposition(m_S_k, m_permutation*S_disordered*m_permutation.t());
		} 
		else
		{
			if (flag4UpOrDown == FLAG_4_UPDATING)
			{
				m_covRank = dim - 3*m_nMapFeatures;
				dst = src1 + src2;
			} 
			else
			{
				m_covRank = dim - 3*m_nFilters;
				dst = src1 - src2;
			}

			modifiedCholeskyDecomposition(m_S_k, dst);
		}
	}
}

// Modified Cholesky decomposition
void CSLAM::CholeskyDecompositionWithPivoting (Mat &sr, Mat &Cov)
{
	int dim = m_X_k.rows;
	sr = Mat::zeros(dim, dim, CV_64F);

	if (dim == m_covRank)
	{
		modifiedCholeskyDecomposition(sr, Cov);
	} 
	else
	{
		Mat R11 = Mat::zeros(m_covRank, m_covRank, CV_64F);
		Mat Cov11 = Cov(Range(0,m_covRank), Range(0,m_covRank));
		Mat Cov12 = Cov(Range(0,m_covRank), Range(m_covRank,dim));

		modifiedCholeskyDecomposition(R11, Cov11); 

		Mat R12 = R11.inv().t()*Cov12;
		R11.copyTo(sr(Range(0,m_covRank), Range(0,m_covRank)));
		R12.copyTo(sr(Range(0,m_covRank), Range(m_covRank,dim)));
	}
}

// Modified Cholesky decomposition
//
//  [L,D,E,pneg]=mchol1(G)
//
//  Given a symmetric matrix G, find a matrix E of "small" norm and c
//  L, and D such that  G+E is Positive Definite, and 
//      G+E = L*D*L'
//
//  Also, calculate a direction pneg, such that if G is not PD, then
//
//      pneg'*G*pneg < 0
//
//  Note that if G is PD, then the routine will return pneg=[]. 
//
//  Reference: Gill, Murray, and Wright, "Practical Optimization".

void CSLAM::modifiedCholeskyDecomposition(Mat &S, const Mat &input)
{
	int dim = input.rows;
	double gamma, zi, nu, beta2, num;

	Mat G = input;

	minMaxLoc(G.diag(), NULL, &gamma, NULL, NULL);
	minMaxLoc(G-Mat::diag(G.diag()), NULL, &zi, NULL, NULL);

	Mat mat1 = (Mat_<double>(2,1)<< 1, sqrt(dim*dim - 1.0));
	minMaxLoc(mat1, NULL, &nu, NULL, NULL);

	Mat mat2 = (Mat_<double>(3,1) << gamma, zi/nu, 1e-15);
	minMaxLoc(mat2, NULL, &beta2, NULL, NULL);

	Mat mat3(3, 1, CV_64F);

	Mat L = Mat::zeros(dim, dim, CV_64F);
	Mat D = Mat::zeros(dim, dim, CV_64F);
	Mat C = Mat::diag(G.diag());
	Mat E = Mat::zeros(dim, dim, CV_64F);
	Mat theta = Mat::zeros(dim, 1, CV_64F);

	for (int j = 0; j <= dim-1; j++)
	{
		// ==========Calculate the jth row of L==============//
		if ( j > 0)
		{
			if (1 == j)
			{
				L.ptr<double>(1)[0] = C.ptr<double>(1)[0]/D.ptr<double>(0)[0]; 
			}
			else
			{
				divide(C(Range(j,j+1),Range(0,j)), D(Range(0,j),Range(0,j)).diag().t(), L(Range(j,j+1), Range(0,j)));
			}
		}

		// ==============Update the jth columnn of C==============//
		if (j >= 1)
		{
			if (j<dim-1)
			{
				if (1 == j)
				{
					C(Range(2,dim), Range(1,2)) = G(Range(2,dim), Range(1,2)) - 
						(L(Range(1,2), Range(0,1))*(C(Range(2,dim), Range(0,1)).t())).t();
				}
				else if (dim-2 == j)
				{
					C(Range(dim-1,dim),Range(dim-2,dim-1)) = G(Range(dim-1,dim), Range(dim-2,dim-1)) - 
						(L(Range(dim-2,dim-1),Range(0,dim-2))*(C(Range(dim-1,dim),Range(0,dim-2))).t());
				}
				else
				{
					C(Range(j+1,dim), Range(j,j+1)) = G(Range(j+1,dim), Range(j,j+1)) -
						(L(Range(j,j+1), Range(0,j))*(C(Range(j+1,dim), Range(0,j)).t())).t();
				}
			}
		}
		else
		{
			C(Range(1, dim), Range(0,1)) = G(Range(1, dim), Range(0,1)) + 0;
		}

		// ===============Update theta===================//
		if (dim-1 == j)
		{
			theta.ptr<double>(dim-1)[0] = 0;
		}
		else if (dim-2 == j)
		{
			theta.ptr<double>(dim-2)[0] = abs(C.ptr<double>(dim-1)[dim-2]);
		}
		else
		{
			minMaxLoc(abs(C(Range(j+1,dim), Range(j,j+1))), NULL, &num, NULL, NULL);
			theta.ptr<double>(j)[0] = num;
		}

		// =========Update D================//
		mat3 = (Mat_<double>(3,1) << 
			EPSILON, 
			abs(C.ptr<double>(j)[j]), 
			theta.ptr<double>(j)[0]*theta.ptr<double>(j)[0]/beta2);

		minMaxLoc(mat3, NULL, &num, NULL, NULL);
		D.ptr<double>(j)[j] = num;

		// =========Update E================//
		E.ptr<double>(j)[j] = D.ptr<double>(j)[j] - C.ptr<double>(j)[j];

		// ==========Update C again===========//
		for (int i = j+1; i < dim; i++)
		{
			C.ptr<double>(i)[i] = C.ptr<double>(i)[i] - 
				C.ptr<double>(i)[j]*C.ptr<double>(i)[j]/D.ptr<double>(j)[j];
		}
	}

	// ============Put 1's on the diagonal of L===========//
	for (int i = 0; i < dim*dim; i=i+dim+1)
	{
		L.ptr<double>(i/dim)[i%dim] = 1;
	}

	// ==========if needed, find a descent direction===========//
	Point minLoc;
	Mat rhs, pneg;
	minMaxLoc(C.diag(), &num, NULL, &minLoc, NULL);
	if (num < 0.0)
	{
		rhs = Mat::zeros(dim, 1, CV_64F);
		rhs.ptr<double>(minLoc.x)[0] = 1;
		pneg = L.t().inv()*rhs;
	}
	else
	{
		pneg = NULL;
	}

	Mat D_sr;
	sqrt(D, D_sr);
	S = D_sr*L.t();

	//if (pneg.rows != 0)
	//{
	//	cout<<pneg.t()*G*pneg<<endl<<endl; 
	//}	
}

// GSL QR decomposition.
void CSLAM::GSLQrDecomposition (Mat &cv_s, const Mat &cv_qr) const
{
	int dimx = cv_qr.rows;
	int dimy = cv_qr.cols;

	gsl_matrix * gsl_qr  = gsl_matrix_alloc(dimx, dimy);
	gsl_vector * gsl_tau = gsl_vector_alloc(dimy);

	dataTypeCVMat2GSLMat(gsl_qr, cv_qr);
	gsl_linalg_QR_decomp(gsl_qr, gsl_tau);

	cv_s = Mat::zeros(dimy, dimy, CV_64F);

	for (int i=0; i<dimy; i++)
	{
		for (int j=i; j<dimy; j++)
		{
			cv_s.ptr<double>(i)[j] = gsl_matrix_get(gsl_qr, i, j);
		}
	}

	gsl_vector_free(gsl_tau);
	gsl_matrix_free(gsl_qr);
}

// QR decomposition with column pivoting
void CSLAM::GSLModifiedQRDecomposition(Mat &sr, const Mat &QR)
{
	// ===================== Cholesky update =======================// 
	Mat cov  = QR;

	int dimx = cov.rows;
	int dimy = cov.cols;
	int signum;
	Mat	R(dimx, dimy, CV_64F);
	Mat P = Mat::zeros(dimy, dimy, CV_64F);

	gsl_matrix* gsl_cov    = gsl_matrix_alloc(dimx, dimy);
	gsl_matrix* gsl_q      = gsl_matrix_alloc(dimx, dimx);
	gsl_matrix* gsl_r      = gsl_matrix_alloc(dimx, dimy);
	gsl_vector* gsl_tau    = gsl_vector_alloc(dimy);
	gsl_vector* gsl_norm   = gsl_vector_alloc(dimy);
	gsl_permutation* gsl_p = gsl_permutation_alloc(dimy);

	dataTypeCVMat2GSLMat(gsl_cov, cov);
	gsl_linalg_QRPT_decomp2(gsl_cov, gsl_q, gsl_r, gsl_tau, gsl_p, &signum, gsl_norm);
	dataTypeGSLMat2CVMat(R, gsl_r);
	dataTypeGSLPermutation2CVMat(P, gsl_p);

	gsl_matrix_free(gsl_cov);
	gsl_matrix_free(gsl_q);
	gsl_vector_free(gsl_tau);
	gsl_vector_free(gsl_norm);

	Mat R_new = R*P.t();
	Mat S_new = R_new.rowRange(Range(0,dimy));

	GSLQrDecomposition(sr, S_new);
}

/**
* @brief Updating features' info.
* @param[in] none.
* @param[out] none.
* @note
*  Delete all unstable features.
*/
void CSLAM::updateFeaturesInformation (void)
{
	if (!m_nMapFeatures)
	{
		return;
	}

	m_P_k = m_S_k.t()*m_S_k;

	int dim;
	int id = 0;
	double xi, yi, zi, theta, phi, rho, Hlr_z;
	double px, py, mx, my, dpx, dpy, dmx, dmy;
	bool isDelete, isNeedStore;

	Mat sr(6, 6, CV_64F);
	Mat keyPointCov(3, 3, CV_64F);
	Mat keyPointInfo(6, 1, CV_64F);

	CString showFeatureInfo;
	FeatureInfo featuresInfo;

	m_nDeletes = 0;
	m_nStores  = 0;

	m_deleteID.create(m_nMapFeatures, 1, CV_64F);

	PointsMap* map_p = map;
	while (NULL != map_p)
	{
		xi	  = m_X_k.ptr<double>(6*id+0)[0];
		yi	  = m_X_k.ptr<double>(6*id+1)[0];
		zi	  = m_X_k.ptr<double>(6*id+2)[0];
		theta = m_X_k.ptr<double>(6*id+3)[0];
		phi   = m_X_k.ptr<double>(6*id+4)[0];
		rho   = m_X_k.ptr<double>(6*id+5)[0];

		dim = m_X_k.rows;
		Hlr_z = rho*(zi - m_X_k.ptr<double>(dim-2)[0]) + cos(phi)*cos(theta);

		px  = map_p->predictLocation.x;
		py  = map_p->predictLocation.y;

		dpx = imageWidth  - px;
		dpy = imageHeight - py;

		isDelete = (map_p->nPredictTimes > 2*map_p->nMatchTimes && map_p->nPredictTimes >= 10) ||
			rho < 0.01 || Hlr_z < 0.0 ||
			px  < DIST_2_BORDER || py  < DIST_2_BORDER ||
			dpx < DIST_2_BORDER || dpy < DIST_2_BORDER;

		if (true == map_p->isMatching)
		{
			mx  = map_p->matchLocation.x;
			my  = map_p->matchLocation.y;

			dmx = imageWidth  - mx;
			dmy = imageHeight - my;

			isDelete = isDelete ||
				mx  < DIST_2_BORDER || my  < DIST_2_BORDER || 
				dmx < DIST_2_BORDER || dmy < DIST_2_BORDER;
		}

		if ( true == isDelete)  // Nedd to delete one node
		{
			isNeedStore = false;

			if ((map_p->nPredictTimes > 2*map_p->nMatchTimes && map_p->nPredictTimes >= 10))
			{
				m_nPredicts--;
				m_nFilters--;
				cout<<"删除特征点:ID="<<map_p->ID<<"，原因：特征点多次未匹配上"<<endl<<endl;
			} 
			else if (rho < 0)
			{
				m_nPredicts--;
				m_nFilters--;
				cout<<"删除特征点:ID="<<map_p->ID<<"，原因：特征点深度值为负"<<endl<<endl;
			}
			else if ((rho >= 0) && (rho < 0.01))
			{
				m_nPredicts--;
				m_nFilters--;
				cout<<"删除特征点:ID="<<map_p->ID<<"，原因：特征点深度值过大"<<endl<<endl;
			}
			else if (Hlr_z < 0.0)
			{
				m_nPredicts--;
				m_nFilters--;
				cout<<"删除特征点:ID="<<map_p->ID<<"，原因：相机高度值为负"<<endl<<endl;
			}
			else if (px < DIST_2_BORDER || py < DIST_2_BORDER || dpx < DIST_2_BORDER || dpy <DIST_2_BORDER)
			{
				m_nPredicts--;
				m_nFilters--;

				if (true == map_p->isMatching)
				{
					isNeedStore = true;
					m_nStores++;
					m_nMatches--;
				}

				cout<<"删除特征点:ID="<<map_p->ID<<"，原因：特征点的预测位置靠近边界"<<endl<<endl;
			}
			else if (mx < DIST_2_BORDER || my < DIST_2_BORDER || dmx < DIST_2_BORDER || dmy <DIST_2_BORDER)
			{
				isNeedStore = true;
				m_nFilters--; 
				m_nStores++; 
				m_nPredicts--; 
				m_nMatches--;

				cout<<"删除特征点:ID="<<map_p->ID<<"，原因：特征点的匹配位置靠近边界"<<endl<<endl;
			}

			m_deleteID.ptr<double>(m_nDeletes)[0] = map_p->ID;

			if (true == isNeedStore)
			{
				featuresInfo.ID            = map_p->ID;
				featuresInfo.isLoop        = map_p->isLoop;
				featuresInfo.nPredictTimes = map_p->nPredictTimes;
				featuresInfo.nMatchTimes   = map_p->nMatchTimes;
				featuresInfo.initXYZ       = map_p->xyz;
				featuresInfo.initPixel     = map_p->initPixel;
				featuresInfo.initTrans     = map_p->initTrans;
				featuresInfo.initRotation  = map_p->initRotation;
				featuresInfo.initPatch     = map_p->initPatch;
				featuresInfo.state         = m_X_k.rowRange(6*id, 6*id+6) + 0;
				getFeatureCartesianInformation(featuresInfo.position, featuresInfo.sr, featuresInfo.cov, id);
				get3DdisplayInformation(featuresInfo.axis, featuresInfo.sigma, featuresInfo.cov);

				m_featuresAllInfo.push_back(featuresInfo); // Store the information of all validate keypoints
			}

			//for (vector<FeatureInfo>::size_type i=0; i<m_featuresAllInfo.size(); i++)
			//{
			//	cout<<m_featuresAllInfo[i].ID<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].nPredictTimes<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].nMatchTimes<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].initXYZ.x<<"  "<<m_featuresAllInfo[i].initXYZ.x<<"  "<<m_featuresAllInfo[i].initXYZ.z<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].initPixel.x<<"  "<<m_featuresAllInfo[i].initPixel.y<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].initTrans<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].initRotation<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].initPatch<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].state<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].Position.x<<"  "<<m_featuresAllInfo[i].Position.y<<"  "<<m_featuresAllInfo[i].Position.z <<endl<<endl;
			//	cout<<m_featuresAllInfo[i].SR<<endl<<endl;
			//	cout<<m_featuresAllInfo[i].Cov<<endl<<endl;
			//	//cout<<m_featuresAllInfo[i].Axis<<endl<<endl;
			//	//cout<<m_featuresAllInfo[i].Sigma<<endl<<endl;
			//}



			map = deleteOneFeature(id, map_p->ID, map);
			map_p = map;

			m_P_k = m_S_k.t()*m_S_k;

			int index = 0;

			while (NULL != map_p)
			{
				if (index == id) 
				{
					break; 
				}

				index++;
				map_p = map_p->next;
			}
		}
		else // Need not to delete node
		{
			getFeatureCartesianInformation(map_p->xyz, sr, keyPointCov, id);
			get3DdisplayInformation(map_p->axis, map_p->sigma, keyPointCov);

			if (true == map_p->isMatching)
			{
				if (TRUE == isCompensate)
				{
					showFeatureInfo.Format(showFeatureInfo + "%02d: %.1f %.1f %.1f; \r\n", 
						map_p->ID,
						map_p->xyz.x,
						map_p->xyz.y,
						(map_p->xyz.z + m_deep)/2.0);
				} 
				else
				{
					showFeatureInfo.Format(showFeatureInfo + "%02d: %.1f %.1f %.1f; \r\n", 
						map_p->ID,
						map_p->xyz.x,
						map_p->xyz.y,
						map_p->xyz.z);
				}
			}

			map_p->isVisible = false;

			if (TRUE == isUseRANSAC)
			{
				//map_p->inliner_L = false;
				//map_p->inliner_H = false;
			}
		}

		if (id == m_nMapFeatures)
		{
			break;
		} 
		else
		{
			id++;
			map_p = map_p->next;
		}
	}

	m_deleteID = m_deleteID.rowRange(0,m_nDeletes);

	m_showFeatureInfo.SetSel(0,-1);
	m_showFeatureInfo.ReplaceSel(showFeatureInfo,true);
}

/**
* @brief Updating features' info.
* @param[in]
*  id the normal id of feature.
* @param[in]
*  mapID the ith feature's id in the map.
* @param[in]
*  p_head the overall map.
* @param[out] none.
* @note
*  Delete one feature and return the pointer.\n
*  The deleted feature will be inserted into blacklist,\n
*  and the state, covariance, map will also be updated.
*/
PointsMap* CSLAM::deleteOneFeature (const int &id, const int &mapID, PointsMap* p_head)
{
	int dim = m_X_k.rows;
	Mat V; 

	// ========update state vector & covariance============//
	if (!id)
	{
		m_X_k.rowRange(6,dim).copyTo(m_X_k);

		m_S_k(Range(0,6), Range(6,dim)).copyTo(V);
		m_S_k(Range(6,dim), Range(6,dim)).copyTo(m_S_k);
	}
	else
	{
		m_X_k.rowRange(Range(6*(id+1),dim)).copyTo(m_X_k.rowRange(Range(6*id,dim-6)));
		m_X_k.rowRange(Range(0,dim-6)).copyTo(m_X_k);

		m_S_k.rowRange(Range(6*id,6*(id+1))).copyTo(V);
		V.colRange(Range(6*(id+1), dim)).copyTo(V.colRange(Range(6*id,dim-6)));
		V = V.colRange(Range(0,dim-6));
		m_S_k(Range(0,6*id), Range(6*(id+1),dim)).copyTo(m_S_k(Range(0,6*id), Range(6*id,dim-6))); // S01
		m_S_k(Range(6*(id+1),dim), Range(0,6*id)).copyTo(m_S_k(Range(6*id,dim-6), Range(0,6*id))); // S10
		m_S_k(Range(6*(id+1),dim), Range(6*(id+1),dim)).copyTo(m_S_k(Range(6*id,dim-6), Range(6*id,dim-6))); // S11
		m_S_k(Range(0,dim-6), Range(0,dim-6)).copyTo(m_S_k);
	}
	m_nDeletes++;
	m_nMapFeatures--;

	Mat VT = V.t();
	GSLCholeskyUpdate(VT, FLAG_4_UPDATING, FLAG_4_NEEDNOT_REORDER); 

	// ================update the map================== //
	// Notes: delete current feature's information, and update the map
	PointsMap *map_p1,*map_p2;
	map_p1 = p_head;
	map_p2 = p_head;

	while (map_p2->next && mapID !=map_p2->ID)
	{
		map_p1 = map_p2;
		map_p2 = map_p2->next;
	}
	if (mapID == map_p2->ID)
	{
		if (map_p1 == map_p2)	   
		{
			// Delete the first node
			// In this case, make the map_p1 pointer start from the second node of map_p2
			map_p1 = map_p2->next; 
			delete map_p2;
			return map_p1;
		}
		else                   
		{
			// Delete the node other than first
			// In this case, make the map_p1 pointer start from the next node of map_p2
			map_p1->next = map_p2->next;
			delete map_p2;
			return p_head;
			//return map_p1;
		}
	}
	else
	{
		cout<<"Error occured in this program!"<<endl;
		system("pause");
		return p_head;
	}
}

/**
* @brief Get 3D info.
* @param[in]
*  sr the square root of covarance of feature.
* @param[in]
*  cov the covariance of feature.
* @param[in]
*  id the id of features.
* @param[out]
*  xyz the 3D position of feature.
* @note
*  Get feature's real world position and covariance.
*/
void CSLAM::getFeatureCartesianInformation(Point3d &xyz, Mat &sr, Mat &cov, const int &id) const
{
	//============Calculate the feature's 3D position=========//
	Mat state = m_X_k.rowRange(Range(6*id,6*id+6)) + 0;

	double xi	 = state.ptr<double>(0)[0];
	double yi	 = state.ptr<double>(1)[0];
	double zi	 = state.ptr<double>(2)[0];
	double theta = state.ptr<double>(3)[0];
	double phi   = state.ptr<double>(4)[0];
	double rho   = state.ptr<double>(5)[0];
	//double rho   = (state.ptr<double>(5)[0] + m_rho)/2.0;

	//xyz.x = xi + cos(phi)*sin(theta)*exp(rho);  // ========避免负的逆深度?????????
	//xyz.y = yi - sin(phi)*exp(rho);
	//xyz.z = zi + cos(phi)*cos(theta)*exp(rho);
	xyz.x = xi + cos(phi)*sin(theta)/rho;
	xyz.y = yi - sin(phi)/rho;
	xyz.z = zi + cos(phi)*cos(theta)/rho;

	Mat dFw_dYi = Mat::eye(3, 6, CV_64F);
	Mat dFw_dthetaphirho = (Mat_<double>(3,3) <<
		 cos(phi)*cos(theta)/rho,   -sin(phi)*sin(theta)/rho,   -cos(phi)*sin(theta)/(rho*rho),
		 0,                         -cos(phi)/rho,               sin(phi)/(rho*rho),
		-cos(phi)*sin(theta)/rho,   -sin(phi)*cos(theta)/rho,   -cos(phi)*cos(theta)/(rho*rho));
	dFw_dthetaphirho.copyTo(dFw_dYi(Range::all(),Range(3,6)));

	cov = dFw_dYi*m_P_k(Range(6*id,6*id+6),Range(6*id,6*id+6))*(dFw_dYi.t());

	sr  = m_S_k(Range(6*id,6*id+6), Range(Range(6*id,6*id+6))) + 0;
}

/**
* @brief Inverse depth to Cartesian.
* @param[in]
*  position the camera's position.
* @param[in]
*  state the collective vector of two angles & inverse depth.
* @param[out]
*  xyz the 3D position of feature.
* @note
*  Only for means.\n
*  May be duplicate comprision to above.
*/
void CSLAM::coordinatesInverseDepth2Cartesian(Point3d &xyz, const Mat &position, const Mat &state) const
{
	double xi	 = position.ptr<double>(0)[0];
	double yi	 = position.ptr<double>(1)[0];
	double zi	 = position.ptr<double>(2)[0];
	double theta = state.ptr<double>(0)[0];
	double phi   = state.ptr<double>(1)[0];
	double rho   = state.ptr<double>(2)[0];
	//double rho   = (state.ptr<double>(2)[0] + m_rho)/2.0;

	xyz.x = xi + cos(phi)*sin(theta)/rho;
	xyz.y = yi - sin(phi)/rho;
	xyz.z = zi + cos(phi)*cos(theta)/rho;
}

/**
* @brief Getting 3D info.
* @param[in]
*  sigma the distance of each axis.
* @param[in]
*  matrix the 3D covariance of each feature.
* @param[out]
*  axis 
* @note
*  Calculate the information need to perform the features'information
*/
void CSLAM::get3DdisplayInformation (Quaternion &axis, Point3d &sigma, const Mat &matrix) const
{
	Mat eigenvalues  = Mat::zeros(3,3,CV_32FC1);  //the eigenvalues are stored in the diag
	Mat	eigenvectors = Mat::zeros(3,3,CV_32FC1);

	calculateEigenvaluesAndEigenvectors(matrix, eigenvalues, eigenvectors);
	matrix2Quaternion(axis, eigenvectors);

	sigma.x =  sqrt(eigenvalues.ptr<double>(0)[0]);
	sigma.y =  sqrt(eigenvalues.ptr<double>(1)[1]);
	sigma.z =  sqrt(eigenvalues.ptr<double>(2)[2]);   //calculate the std-covariance
}

/**
* @brief Eigen value calculating.
* @param[in]
*  src input matrix.
* @param[out]
*  eigenvalues Eigen values of matrix.
* @param[out]
*  eigenvectors Eigen vectors of matrix.
* @note
*  Calculating the eigen value of given matrix.
*/
bool CSLAM::calculateEigenvaluesAndEigenvectors(Mat src, Mat &eigenvalues, Mat &eigenvectors) const
{
	int i, j, p, q, l, n, jt;
	double fm,cn,sn,omega,x,y,d;
	n = src.rows;
	l = 1;
	jt = n*n*30;
	src.copyTo(eigenvalues);

	eigenvectors = Mat::eye(eigenvalues.rows, eigenvalues.cols, CV_64F);

	while (1)
	{ 
		fm = 0.0;
		for (i = 1; i <= n-1; i++)
		{
			for (j = 0; j <= i-1; j++)
			{ 
				d = fabs(eigenvalues.at<double>(i,j));
				if ((i!=j)&&(d>fm))
				{
					fm=d;
					p=i;
					q=j;
				}
			}
		}

		if (fm < EPSILON)	
			return(true);

		if (l > jt)	
			return(false);

		l = l + 1;
		x = -eigenvalues.at<double>(p,q); 
		y = (eigenvalues.at<double>(q,q) - eigenvalues.at<double>(p,p))/2.0;
		omega = x/sqrt(x*x + y*y);

		if (y<0.0)	
			omega = -omega;

		sn = 1.0+sqrt(1.0 - omega*omega);
		sn = omega/sqrt(2.0*sn);
		cn = sqrt(1.0 - sn*sn);
		fm = eigenvalues.at<double>(p,p);
		eigenvalues.at<double>(p,p) = fm*cn*cn + eigenvalues.at<double>(q,q)*sn*sn + eigenvalues.at<double>(p,q)*omega;
		eigenvalues.at<double>(q,q) = fm*sn*sn + eigenvalues.at<double>(q,q)*cn*cn - eigenvalues.at<double>(p,q)*omega;
		eigenvalues.at<double>(p,q) = 0.0; 
		eigenvalues.at<double>(q,p) = 0.0;

		for (j=0; j <= n-1; j++)
		{
			if ((j!=p) && (j!=q))
			{ 
				fm=eigenvalues.at<double>(p,j);
				eigenvalues.at<double>(p,j) = fm*cn  + eigenvalues.at<double>(q,j)*sn;
				eigenvalues.at<double>(q,j) = -fm*sn + eigenvalues.at<double>(q,j)*cn;
			}
		}
		for (i=0; i<=n-1; i++)
		{
			if ((i!=p) && (i!=q))
			{
				fm = eigenvalues.at<double>(i,p);
				eigenvalues.at<double>(i,p) = fm*cn  + eigenvalues.at<double>(i,q)*sn;
				eigenvalues.at<double>(i,q) = -fm*sn + eigenvalues.at<double>(i,q)*cn;
			}
		}
		for (i=0; i <= n-1; i++)
		{ 
			fm = eigenvectors.at<double>(i,p);
			eigenvectors.at<double>(i,p) = fm*cn  + eigenvectors.at<double>(i,q)*sn;
			eigenvectors.at<double>(i,q) = -fm*sn + eigenvectors.at<double>(i,q)*cn;
		}
	}
	return true;
}

/**
* @brief Matrix to quaternion.
* @param[in]
*  matrix input matrix.
* @param[out]
*  quaternion output quaternion.
* @note
*  Convert matrix to quaternion.
*/
void CSLAM::matrix2Quaternion(Quaternion &quaternion, const Mat &matrix) const
{
	double m11, m12, m13, m21, m22, m23, m31, m32, m33, tmp;

	m11 = matrix.ptr<double>(0)[0];  m12 = matrix.ptr<double>(0)[1];  m13 = matrix.ptr<double>(0)[2];
	m21 = matrix.ptr<double>(1)[0];  m22 = matrix.ptr<double>(1)[1];  m23 = matrix.ptr<double>(1)[2];
	m31 = matrix.ptr<double>(2)[0];  m32 = matrix.ptr<double>(2)[1];  m33 = matrix.ptr<double>(2)[2];

	double tr = m11 + m22 +m33;

	if(tr > 0.0)
	{
		tmp = 0.5 / sqrt(tr + 1);
		quaternion.r = 0.25 / tmp; 
		quaternion.x = (m23 - m32) * tmp;
		quaternion.y = (m31 - m13) * tmp;
		quaternion.z = (m12 - m21) * tmp;
	}
	else
	{
		if(m11 > m22 && m11 > m33)
		{
			tmp = 2.0 * sqrt(1.0 + m11 - m22 - m33);
			quaternion.r = (m32 - m23) / tmp;
			quaternion.x = 0.25 * tmp;
			quaternion.y = (m12 + m21) / tmp;
			quaternion.z = (m13 + m31) / tmp;
		}
		else if( m22 > m33)
		{
			tmp = 2.0 * sqrt(1.0 + m22 - m11 - m33);
			quaternion.r = (m13 - m31) / tmp;
			quaternion.x = (m12 + m21) / tmp;
			quaternion.y = 0.25 * tmp;
			quaternion.z = (m23 + m32) / tmp;
		}
		else
		{
			tmp = 2.0 * sqrt(1.0 + m33 - m11 - m22);
			quaternion.r = (m21 - m12) / tmp;
			quaternion.x = (m13 + m31) / tmp;
			quaternion.y = (m23 + m32) / tmp;
			quaternion.z = 0.25 * tmp;
		}
	}
}

/**
* @brief Robot info updating.
* @param[in] none.
* @param[out] none.
* @note
*  This version do not store the covariance.
*/
void CSLAM::updateRobotInformation (void)
{
	int dim   = m_X_k.rows;
	int index = 2*m_frame.counter;

	double x_now, y_now, theta_now;
	x_now = m_X_k.ptr<double>(dim-4)[0];
	y_now = m_X_k.ptr<double>(dim-3)[0];
	theta_now = m_X_k.ptr<double>(dim-1)[0];

	*(m_path+index+0) = x_now;
	*(m_path+index+1) = y_now;

	//Mat cov = m_S_k(Rect(dim-4,dim-4,3,3)).t()*m_S_k(Rect(dim-4,dim-4,3,3));
	//cov.copyTo(m_robotInfo.Cov);
	//get3DdisplayInformation(m_robotInfo.Axis, m_robotInfo.Sigma, cov);
	//m_robotAllInfo.push_back(robotInfo);  //==========可把机器人位置转成二维?????????

	// Show robot's information in list control
	char id[10];
	char x[50], y[50], theta[50];

	sprintf_s(id, "%d", m_showCounter);
	sprintf_s(x, "%f", x_now);
	sprintf_s(y, "%f", y_now);
	sprintf_s(theta, "%f", theta_now*180.0/3.141593);

	m_listCtrl.SetItemText(0, 1, id);
	m_listCtrl.SetItemText(0, 2, x);
	m_listCtrl.SetItemText(0, 3, y);
	m_listCtrl.SetItemText(0, 4, theta);

	if (TRUE == isShowOdoCtrlList)
	{
		sprintf_s(x, "%f", *(m_odoXY+index+0));
		sprintf_s(y, "%f", *(m_odoXY+index+1));
		sprintf_s(theta, "%f", m_odoTheta.ptr<double>(1)[m_frame.counter]*180.0/3.141593);

		m_listCtrl.SetItemText(1, 1, id);
		m_listCtrl.SetItemText(1, 2, x);
		m_listCtrl.SetItemText(1, 3, y);
		m_listCtrl.SetItemText(1, 4, theta);
	}
}

/**
* @brief Features' 2D showing.
* @param[in] none.
* @param[out] none.
* @note
*  show the result in the picture
*/
void CSLAM::display2DFeatureModel (void)
{
	char num[10];	// 用于显示ID号
	Mat Pi(2, 2, CV_64F);

	CvFont font;
	CvPoint pt_p, pt_m;

	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 3, 8);

	if (0 != m_nMatches)
	{
		PointsMap* map_p = map;
		while (NULL != map_p)
		{
			if ((true == map_p->isMatching))// && (false == map_p->isDelay))
			{
				_itoa_s(map_p->ID, num, 10);  // 类型转换

				pt_p = map_p->predictLocation;
				pt_m = map_p->matchLocation;

				Pi = map_p->Si.t()*map_p->Si;

				//=========For all predicted keypoints, draw on blue color==============//
				cvPutText(m_srcImage, num, pt_p, &font, CV_RGB(0,0,255));
				cvLine(m_srcImage, cvPoint(pt_p.x-10, pt_p.y), cvPoint(pt_p.x+10, pt_p.y), CV_RGB(0,0,255),2,8,0);
				cvLine(m_srcImage, cvPoint(pt_p.x, pt_p.y-10), cvPoint(pt_p.x, pt_p.y+10), CV_RGB(0,0,255),2,8,0);

				//=========For all matched keypoints, draw on red color==============//
				cvPutText(m_srcImage, num, pt_m, &font, CV_RGB(255,0,0));
				cvLine(m_srcImage, cvPoint(pt_m.x-10, pt_m.y), cvPoint(pt_m.x+10, pt_m.y), CV_RGB(255,0,0),2,8,0);
				cvLine(m_srcImage, cvPoint(pt_m.x, pt_m.y-10), cvPoint(pt_m.x, pt_m.y+10), CV_RGB(255,0,0),2,8,0);
				draw2DEllipse(m_srcImage, pt_m, Pi, CV_RGB(255,0,0), 2);
			}
			map_p = map_p->next; 
		}
	}
	
	CCvImage img;
	img.CopyOf(m_srcImage,1);
	img.DrawToHDC(m_pDc->GetSafeHdc(),&m_rect);
}

/**
* @brief Draw 2D ellipse.
* @param[in]
*  displayImage the image to be show.
* @param[in]
*  center the center of image.
* @param[in]
*  covMatrix the covariance of matrix.
* @param[in]
*  color the color of ellipse.
* @param[in]
*  thickness the thickness of 2D ellipse.
* @param[out] none.
*/
void CSLAM::draw2DEllipse (IplImage* displayImage, const CvPoint &center, const Mat &covMatrix,
	const CvScalar color, int thickness) const
{
	Mat eigenvalues = Mat::zeros(2,1,CV_64F);
	Mat	eigenvectors = Mat::zeros(2,2,CV_64F);

	if(!eigen(covMatrix,eigenvalues,eigenvectors))	return ;

	CvSize size;
	size.width =  int(sqrt(eigenvalues.ptr<double>(0)[0])*sqrt(CHI2INV_TABLE(0,2)));
	size.height = int(sqrt(eigenvalues.ptr<double>(1)[0])*sqrt(CHI2INV_TABLE(0,2)));
	size.width = max(1, size.width);
	size.height = max(1, size.height);

	double angle = (double)(atan2(eigenvectors.ptr<double>(0)[1],eigenvectors.ptr<double>(0)[0])*180.0/CV_PI);
	cvEllipse(displayImage, center, size, angle, 0, 360, color, thickness);
}

/**
* @brief Reset parameters.
* @param[in] none.
* @param[out] none.
*/
void CSLAM::resetAllParameters (void)
{
	vector<FeatureInfo>().swap(m_featuresAllInfo);

	while (NULL != map)
	{
		PointsMap* map_p = map->next;
		delete map;
		map = map_p;
	}

	fclose(m_odometryFile);
	initializeParameters();

	// Reset information for features' show
	m_showFeatureInfo.SetSel(0,-1);
	m_showFeatureInfo.Clear();

	// Show information for robot's location show
	char theta[50];
	sprintf_s(theta, "%f", m_odoTheta.ptr<double>(1)[0]*180.0/3.141593);

	m_listCtrl.SetItemText(0, 1, "1");
	m_listCtrl.SetItemText(0, 4, theta);
	for (int i = 0; i < 2; i++)
	{
		m_listCtrl.SetItemText(0, i+2, "0");
	}

	if (TRUE == isShowOdoCtrlList)
	{
		m_listCtrl.SetItemText(1, 1, "1");
		m_listCtrl.SetItemText(1, 4, theta);
		for (int i = 0; i < 2; i++)
		{
			m_listCtrl.SetItemText(1, i+2, "0");
		}
	}
}

/**
* @brief Matrix correlation.
* @param[in]
*  src1 the input matrix1.
* @param[in]
*  src2 the input matrix2.
* @param[out]
*  corr the output matrix.
* @note
*  Calculate the correlation of two matrix.
*/
void CSLAM::calculateCrossCorrelation (double &corr,const Mat &src1,const Mat &src2)const
{
	Scalar avg1,avg2;
	Scalar std1, std2;

	Mat num1(src1.rows, src1.cols, CV_64F);
	Mat num2(src2.rows, src2.cols, CV_64F);

	src1.convertTo(num1,CV_64F);
	src2.convertTo(num2,CV_64F);

	avg1 = cv::mean(num1);
	avg2 = cv::mean(num2);

	num1-=avg1(0);
	num2-=avg2(0);

	std1 = cv::norm(num1);
	std2 = cv::norm(num2);
	corr = num1.dot(num2);

	if(std1(0)==0 || std2(0)==0)
		corr=0;
	else
		corr=corr/std1(0)/std2(0);
}

/**
* @brief Feature distortion.
* @param[in]
*  uvu the pixel of undistorted feature.
* @param[out]
*  uvd the pixel of feature after distortion.
* @note
*  Distort a feature point using RW method.
*/
void CSLAM::distortOnePointRW (Point2d &uvd, const Point2d &uvu) const
{
	double f, ff;

	double xu = (uvu.x-cam_cx)*cam_dx;	// similar to the model in book
	double yu = (uvu.y-cam_cy)*cam_dy;
	double ru = sqrt(xu*xu+yu*yu);
	double rd = ru/( 1+cam_k1*ru*ru+cam_k2*pow(ru,4));

	int iterationNum = 100;

	for(int i=0; i<iterationNum; i++)
	{
		f  = rd  + cam_k1*pow(rd,3)+cam_k2*pow(rd,5)-ru;
		ff = 1.0 + 3.0*cam_k1*rd*rd+5.0*cam_k2*pow(rd,4);
		rd = rd  - f/ff;
	}

	double d  = 1+cam_k1*rd*rd+cam_k2*pow(rd,4);

	if (d == 0)	
		d = EPSILON;

	double xd = xu/d;
	double yd = yu/d;

	uvd.x = cam_cx+xd/cam_dx;					/*图像坐标系下加扭曲后的点坐标*/
	uvd.y = cam_cy+yd/cam_dy;					/*参考matlab: distor_a_point*/

	bool isVisible = (uvd.x>=0) && (uvd.x <= imageWidth) && (uvd.y>=0) && (uvd.y<=imageHeight);

	if (false == isVisible)   // whether is visible or not
	{
		uvd.x = 0;
		uvd.y = 0;
	}
}

/**
* @brief Feature undistortion.
* @param[in]
*  uvd the pixel of feature after distortion.
* @param[out]
*  uvu the pixel of undistorted feature.
* @note
*  Distort a feature point according to 1-point RANSAC paper.
*/
void CSLAM::undistortOnePointRW (Point2d &uvu, const Point2d &uvd) const
{
	double xd = (uvd.x-cam_cx)*cam_dx;  // point点为像素坐标（扭曲的）
	double yd = (uvd.y-cam_cy)*cam_dy;

	double rd = sqrt(xd*xd+yd*yd);
	double d = 1+cam_k1*pow(rd,2)+cam_k2*pow(rd,4);
	double xu = xd*d;
	double yu = yd*d;

	uvu.x = cam_cx+xu/cam_dx;	
	uvu.y = cam_cy+yu/cam_dy;	
}

/**
* @brief coordinate transformation.
* @param[in]
*  state the inverse depth parameterization.
* @param[in]
*  cam_position camera's position.
* @param[out]
*  Hlw feature's position wrt world.
* @note
*  Transform the feature's coordinates in\n
*  state vector into world frame (6*1 or 3*1-->3*1).
*/
void CSLAM::coordinatesState2World (Mat &Hlw, const Mat &state, const Mat &cam_position) const
{
	double xi	 = state.ptr<double>(0)[0];
	double yi	 = state.ptr<double>(1)[0];
	double zi	 = state.ptr<double>(2)[0];
	double theta = state.ptr<double>(3)[0];
	double phi   = state.ptr<double>(4)[0];
	double rho   = state.ptr<double>(5)[0];

	//if (rho == 0.0)
	//{
	//	rho = EPSINON;  
	//	cout<<"There is a bug in the program!"<<endl<<endl;
	//	system("pause");
	//}

	//Hlw = (Mat_<double>(3,1) <<     
	//	rho*(xi - cam_position.ptr<double>(0)[0]) + cos(phi)*sin(theta), 
	//	rho*(yi - cam_position.ptr<double>(1)[0]) - sin(phi), 
	//	rho*(zi - cam_position.ptr<double>(2)[0]) + cos(phi)*cos(theta));          


	Hlw = (Mat_<double>(3,1) <<           
		xi + 1/rho*cos(phi)*sin(theta) - cam_position.ptr<double>(0)[0], 
		yi - 1/rho*sin(phi) - cam_position.ptr<double>(1)[0],
		zi + 1/rho*cos(phi)*cos(theta) - cam_position.ptr<double>(2)[0]);         
}

/**
* @brief coordinate transformation.
* @param[in]
*  Hlw feature's position wrt world coordinate.
* @param[in]
*  Rcw the ratation matrix of camera.
* @param[out]
*  Hlc feature's position wrt camera coordinate.
* @note
*  Calculating features' coordinates wrt camera from world frame.
*/
void CSLAM::coordinatesWorld2Camera (Mat &Hlr, const Mat &Hlw, const Mat &Rcw) const
{
	//Hlr = Rcw*(Hlw - T);
	Hlr = Rcw*Hlw;
	//multiply(Rcw, Hlw, Hlr); 

	//if (Hlr.ptr<double>(2)[0] == 0)		
	//{
	//	Hlr.ptr<double>(2)[0] = EPSINON;
	//}

	//double Hlr_x = Hlr.ptr<double>(0)[0];
	//double Hlr_y = Hlr.ptr<double>(1)[0];
	//double Hlr_z = Hlr.ptr<double>(2)[0];

	//double camera_theta = atan2(Hlr_x,Hlr_z)*180/CV_PI;
	//double camera_phi   = atan2(Hlr_y,Hlr_z)*180/CV_PI;

	//bool isInHFOV = camera_theta > (-1)*HFOV && camera_theta < HFOV && camera_phi > (-1)*HFOV && camera_phi < HFOV;

	//if (false == isInHFOV) // whether the feature is in front of the camera or not
	//	Hlr = Mat::zeros(3, 1, CV_64F);  
}

/**
* @brief coordinate transformation.
* @param[in]
*  Hlr feature's position wrt camera coordinate.
* @param[in]
*  error the noise of distortion process.
* @param[out]
*  uvu the pixel of undistorted feature.
* @note
*  Calculating the feature point's coordinates wrt image frome camera frame.
*/
void CSLAM::coordinatesCamera2Image (Point2d &uvu, const Mat &Hlr, const Mat &error) const
{
	double Hlr_x = Hlr.ptr<double>(0)[0];
	double Hlr_y = Hlr.ptr<double>(1)[0];
	double Hlr_z = Hlr.ptr<double>(2)[0];

	// whether the feature is in HFOV or not
	if (Hlr_z==0)   // ========需改进判断形式???????????
	{
		uvu.x = 0;
		uvu.y = 0;
	} 
	else
	{
		uvu.y = cam_cx + cam_f1*Hlr_x/Hlr_z + error.ptr<double>(0)[0];
		uvu.x = cam_cy + cam_f2*Hlr_y/Hlr_z + error.ptr<double>(1)[0];

		if (uvu.x < 10 || uvu.x >imageWidth-10 || uvu.y < 10 || uvu.y > imageHeight-10)
		{
			uvu.x = 0;
			uvu.y = 0;
		}
	}
}

/**
* @brief coordinate transformation.
* @param[in]
*  uvu the pixel of undistorted feature.
* @param[out]
*  Hlr feature's position wrt camera coordinate.
* @note
*  Calculating the feature point's coordiante wrt camera from image frame.
*/
void CSLAM::coordinatesImage2Camera (Mat &Hlr, const Point2d & uvu) const
{
	Hlr = (Mat_<double>(3,1) <<
		(uvu.y - cam_cx)/cam_f1,
		(uvu.x - cam_cy)/cam_f2,
		1);

	//Hlr = (Mat_<double>(3,1) <<
	//	(cam_cx - uvu.x)/cam_f1,
	//	(cam_cy - uvu.y)/cam_f2,
	//	1);
}

/**
* @brief coordinate transformation.
* @param[in]
*  Hlr feature's position wrt camera coordinate.
* @param[in]
*  Rwc the ratation matrix of camera.
* @param[out]
*  Hlw feature's position wrt world coordinate.
* @note
*  Calculating the feature point's coordiante wrt world from camera frame.
*/
void CSLAM::coordinatesCamera2World (Mat &Hlw, const Mat &Hlr, const Mat &Rwc) const
{
	//multiply(Rwc, Hlr, Hlw);
	//Hlw = Rwc*Hlr + T;  
	Hlw = Rwc*Hlr;
}

/**
* @brief coordinate transformation.
* @param[in]
*  Hlw feature's position wrt world coordinate.
* @param[in]
*  rho the inverse depth of feature.
* @param[out]
*  state the collective vector set of two angles & inverse depth.
* @note
*  get the state-expression of the feature wrt world from\n
*  the position between feature and camera wrt world
*/
void CSLAM::coordinatesWorld2State (Mat &state, const Mat &Hlw, const double &rho) const
{
	state.create(3, 1, CV_64F);

	double Hlw_x = Hlw.ptr<double>(0)[0];
	double Hlw_y = Hlw.ptr<double>(1)[0];
	double Hlw_z = Hlw.ptr<double>(2)[0];

	//double theta = atan2(Hlw_x,Hlw_z);
	//double phi   = atan2(-Hlw_y,sqrt( Hlw_x*Hlw_x + Hlw_z*Hlw_z));
	double theta = atan2(Hlw_x, Hlw_z);
	double phi   = atan2(-Hlw_y,sqrt( Hlw_x*Hlw_x + Hlw_z*Hlw_z));

	//wrapAngle(theta);
	//wrapAngle(phi);

	state.ptr<double>(0)[0] = theta;
	state.ptr<double>(1)[0] = phi;
	state.ptr<double>(2)[0] = rho;
}

// Datatype convert: gsl_matrix to Mat.
/**
* @brief Datatype: CVMAT to GSLMat.
* @param[in]
*  cv_mat matrix of OpenCV format.
* @param[out]
*  gsl_mat matrix of GSL format.
* @note
*  Datatype convert: gsl_matrix to Mat.
*/
void CSLAM::dataTypeCVMat2GSLMat (gsl_matrix *gsl_mat, const Mat &cv_mat) const
{
	int dimx = cv_mat.rows;
	int dimy = cv_mat.cols;

	for (int m=0; m<dimx; m++)
	{
		for (int n=0; n<dimy; n++)
		{
			gsl_matrix_set(gsl_mat, m, n, cv_mat.ptr<double>(m)[n]);
		}
	}
}

/**
* @brief Datatype: gsl_matrix to Mat.
* @param[in]
*  gsl_mat matrix of GSL format.
* @param[out]
* @note
*  cv_mat matrix of OpenCV format.
*/
void CSLAM::dataTypeGSLMat2CVMat (Mat &cv_mat, gsl_matrix *gsl_mat) const
{
	int dimx = gsl_mat->size1;
	int dimy = gsl_mat->size2;

	for (int m=0; m<dimx; m++)
	{
		for (int n=0; n<dimy; n++)
		{
			cv_mat.ptr<double>(m)[n] = gsl_matrix_get(gsl_mat, m, n);
		}
	}
	gsl_matrix_free(gsl_mat); // Note: release memory
}

/**
* @brief Affiliation checking.
* @param[in]
*  id the id of feature,
*  set the matrix to be checked.
* @param[out] none.
* @note
*  To see whether the selected id belongs to the set or not.
*/
bool CSLAM::isIdBelongsToSet (const double &id, const Mat &set) const
{
	int num = set.rows;

	for (int i = 0; i < num; i++)
	{
		if (id == set.ptr<double>(i)[0])
		{
			return true;
		}
	}
	return false;
}

/**
* @brief Info recording.
* @param[in] none.
* @param[out] none.
* @note
*  Record robot's and features' information
*/
void CSLAM::recordData (void)
{
	recordRobotInformation();
	recordFeaturesInformation();
}

/**
* @brief Robot info recording.
* @param[in] none.
* @param[out] none.
* @note
*  Store all the robot's trajectory
*/
void CSLAM::recordRobotInformation (void)
{
	if (FALSE == isRecordRobotInfo)
	{
		return;
	}

	int dim = m_X_k.rows;

	if (1 == m_frame.counter)
	{
		int err_open = fopen_s(&m_robotFile, record_robot_dir, "a");
		if (0 != err_open)
		{
			cout<<"Error occured in storing robot information!"<<endl;
			system("pause");
		}
		
		fprintf(m_robotFile,"%d\t",1);  // index
		fprintf(m_robotFile,"%f\t",0);  // odometry x
		fprintf(m_robotFile,"%f\t",0);  // odometry y
		fprintf(m_robotFile,"%f\t",0);  // x
		fprintf(m_robotFile,"%f\t",0);  // y
		//fprintf(m_robotFile,"%f\t",0);  // P00
		//fprintf(m_robotFile,"%f\t",0);  // P01
		//fprintf(m_robotFile,"%f\t",0);  // P10
		//fprintf(m_robotFile,"%f\t",0);  // P11
		fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-4)[dim-4]);
		fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-4)[dim-3]);
		fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-3)[dim-4]);
		fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-3)[dim-3]);
		fprintf(m_robotFile,"\n");
	}

	fprintf(m_robotFile,"%d\t",m_showCounter);
	fprintf(m_robotFile,"%f\t",*(m_odoXY+2*m_frame.counter+0));
	fprintf(m_robotFile,"%f\t",*(m_odoXY+2*m_frame.counter+1));
	fprintf(m_robotFile,"%f\t",m_X_k.ptr<double>(dim-4)[0]);
	fprintf(m_robotFile,"%f\t",m_X_k.ptr<double>(dim-3)[0]);
	//fprintf(ff,"%f\t",m_X_k.at<double>(dim-1,0));
	fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-4)[dim-4]);
	fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-4)[dim-3]);
	//fprintf(ff,"%f\t",m_S_k.at<double>(dim-4,dim-1));
	fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-3)[dim-4]);
	fprintf(m_robotFile,"%f\t",m_P_k.ptr<double>(dim-3)[dim-3]);
	//fprintf(ff,"%f\t",m_S_k.at<double>(dim-3,dim-1));
	//fprintf(ff,"%f\t",m_S_k.at<double>(dim-1,dim-4));
	//fprintf(ff,"%f\t",m_S_k.at<double>(dim-1,dim-3));
	//fprintf(ff,"%f\t",m_S_k.at<double>(dim-1,dim-1));
	fprintf(m_robotFile,"\n");
}

/**
* @brief Fearures info recording.
* @param[in] none.
* @param[out] none.
* @note
*  Store all the features' trajectory
*/
void CSLAM::recordFeaturesInformation (void)
{
	if (FALSE == isRecordFeaturesInfo)
	{
		return;
	}

	//vector<Mat>::size_type i = m_featuresInfo.size();
	if (0 == m_featuresAllInfo.size())
	{
		return;
	}
	
	if (1 == m_frame.counter)
	{
		int err_open = fopen_s(&m_featuresFile, record_features_dir, "a");
		if (0 != err_open)
		{
			cout<<"Error occured store features' information!"<<endl;
			system("pause");
		}
	}

	vector<Mat>::size_type start = m_featuresAllInfo.size() - m_nStores;

	for (vector<Mat>::size_type i = start; i<m_featuresAllInfo.size(); i++)
	{
		fprintf(m_featuresFile,"%f\t", m_featuresAllInfo[i].position.x);
		fprintf(m_featuresFile,"%f\t", m_featuresAllInfo[i].position.y);
		fprintf(m_featuresFile,"%f\t", m_featuresAllInfo[i].position.z);
		fprintf(m_robotFile,"\n");
	}
}

/**
* @brief Fearures info recording.
* @param[in]
*  gsl_permutate gsl matrix.
* @param[out]
*  cv_mat OpenCV matrix.
* @note
*  Datatype convert: gsl_permutation to Mat
*/
void CSLAM::dataTypeGSLPermutation2CVMat(Mat &cv_mat, gsl_permutation *gsl_permutate)
{
	size_t  size = gsl_permutation_size(gsl_permutate);

	for (size_t i=0; i<size; i++)
	{
		int num = gsl_permutation_get(gsl_permutate, i);
		cv_mat.ptr<double>(num)[i] = 1;
	}
	gsl_permutation_free(gsl_permutate);
}

/**
* @brief Showing keypoints.
* @param[in] none.
* @param[out] none.
* @note
*  Only for debug: showing keypoints' 2D position.
*/
void CSLAM::keyPointShow (void)
{
	for (vector<KeyPoint>::size_type i=0; i<m_keyPoints.size(); i++)
	{
		CvPoint pt_p = m_keyPoints[i].pt;

		cvLine(m_srcImage, cvPoint(pt_p.x-8, pt_p.y), cvPoint(pt_p.x+8, pt_p.y), CV_RGB(255,0,0),2,8,0);
		cvLine(m_srcImage, cvPoint(pt_p.x, pt_p.y-8), cvPoint(pt_p.x, pt_p.y+8), CV_RGB(255,0,0),2,8,0);
	}

	Mat outImage;
	drawKeypoints(m_srcImage,m_keyPoints,outImage);
	cvWaitKey(10);
	//char windowNum[1] = {m_frame};
	//_itoa_s(m_frame, windowNum, 10);  // 类型转换
	imshow("1",outImage);
	cvWaitKey(10);
	cvMoveWindow("1", 20, 50);
}

/**
* @brief Showing keypoints.
* @param[in]
*  V input matrix.
* @param[out] none.
* @note
*  Only for debug: Generating Gaussian.
*/
double CSLAM::Gauss(const double &V)
{
	//srand((unsigned)getTickCount());
	static double V1, V2, S;
	static int phase = 0;
	double X;

	if ( phase == 0 ) 
	{
		do 
		{
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while(S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	} 
	else
	{
		X = V2 * sqrt(-2 * log(S) / S);
	}
	phase = 1 - phase;
	X = X * V;

	return X;
}

/**
* @brief Gaussian generator.
* @param[in]
*  V input matrix.
* @param[out] none.
* @note
*  Only for debug: Gaussian random number generator using opencv library.
*/
double CSLAM::Gaussian(const double &sigma)
{
	double time;
	time = getTickCount();
	RNG rng(time);
	return rng.gaussian(sigma);
}

/**
* @brief Robot mean printer.
* @param[in] none.
* @param[out] none.
* @note
*  Only for debug: Print robot's mean.
*/
void CSLAM::printLocationMean (void) const
{
	int dim  = m_X_k.rows;

	double a = m_X_k.ptr<double>(dim-4)[0];
	double b = m_X_k.ptr<double>(dim-3)[0];
	double c = m_X_k.ptr<double>(dim-2)[0];

	//cout<<a<<"  "<<b<<"  "<<c<<endl<<endl;
	cout<<m_X_k.rowRange(dim-4,dim)<<endl<<endl;

}

/**
* @brief Robot covariance printer.
* @param[in] none.
* @param[out] none.
* @note
*  Only for debug: Print robot's covariance.
*/
void CSLAM::printLocationCovariance (void) const
{
	Mat pp = m_S_k.t()*m_S_k;
	int dim = pp.rows;
	cout<<pp(Range(dim-4,dim-1), Range(dim-4,dim-1))<<endl<<endl;
}

/**
* @brief Features' mean printer.
* @param[in] none.
* @param[out] none.
* @note
*  Only for debug: Print Features' mean.
*/
void CSLAM::print2DFeatureMean (void)
{
	int dim = m_X_k.rows;
	int index = 0;

	for (int i = 0; i < (dim-4)/6; i++)
	{
		index = 6*i;
		cout<<m_X_k.rowRange(index, index+6)<<endl<<endl;
	}
}

/**
* @brief Features' covariance printer.
* @param[in] none.
* @param[out] none.
* @note
*  Only for debug: Print feature's covariance
*/
void CSLAM::print2DFeatureCovariance(void)
{
	m_P_k = m_S_k.t()*m_S_k;
	int dim = m_X_k.rows;
	for (int id = 0; id < m_nMapFeatures; id++)
	{
		cout<<m_P_k(Range(6*id,6*id+6), Range(6*id,6*id+6))<<endl<<endl;
	}
}	

/**
* @brief Features' 3D mean printer.
* @param[in] none.
* @param[out] none.
* @note
*  Only for debug: Print feature's 3D location mean.
*/
void CSLAM::print3DFeatureMean (void)
{
	Point3d xyz;
	Mat cov = Mat::zeros(3, 3, CV_64F);
	Mat sr  = Mat::zeros(6, 6, CV_64F);

	m_P_k = m_S_k.t()*m_S_k;

	for (int id = 0; id < m_nMapFeatures; id++)
	{
		getFeatureCartesianInformation(xyz, sr, cov, id);
		cout<<xyz.x<<"  "<<xyz.y<<"  "<<xyz.z<<"  "<<endl<<endl;
	}
}

/**
* @brief Features' 3D covariance printer.
* @param[in] none.
* @param[out] none.
* @note
*  Only for debug: Print feature's 3d location covariance.
*/
void CSLAM::print3DFeatureCovariance (void)
{
	Point3d xyz;
	Mat cov = Mat::zeros(3, 3, CV_64F);
	Mat sr  = Mat::zeros(6, 6, CV_64F);

	m_P_k = m_S_k.t()*m_S_k;

	for (int id = 0; id < m_nMapFeatures; id++)
	{
		getFeatureCartesianInformation(xyz, sr, cov, id);
		cout<<cov<<endl<<endl;
	}
}

/**
* @brief Matrix symmetriclization.
* @param[in]
*  input input matrix.
* @param[out] none.
* @note
*  Only for debug: Make the input matrix symmetrical.
*/
void CSLAM::makeMatrixSymmetric (Mat &input) const
{
	Mat store;
	store = (input + input.t())*0.5;
	store.copyTo(input);
}

/**
* @brief Cholesky decomposition.
* @param[in]
*  input input matrix.
* @param[out]
*  chol Cholesky factor.
* @note
*  Only for debug: Cholesky decomposition using GNU Scientific Library.
*/
void CSLAM::GSLCholeskyDecomposition(Mat &chol, const Mat &input)
{
	int dim = input.rows;
	Mat output = Mat::zeros(dim, dim, CV_64F);
	gsl_matrix * gsl_mat = gsl_matrix_alloc(dim, dim);

	dataTypeCVMat2GSLMat(gsl_mat, input);	
	gsl_linalg_cholesky_decomp(gsl_mat);

	// if the matrix is not positive-definite, 
	// then the decomposition will fail and return error code GSL_EDOM(1)
	if(GSL_EDOM==gsl_linalg_cholesky_decomp(gsl_mat))
	{
		cout<<"GSL Cholesky decomposition fail!"<<endl;
	}
	else
	{
		dataTypeGSLMat2CVMat(output, gsl_mat);
		GSLGetCholeskyUpperTriangularMatrix(chol, output);
	}
}

/**
* @brief Cholesky decomposition.
* @param[in]
*  input input matrix.
* @param[out]
*  output Cholesky upper triangular matrix.
* @note
*  Only for debug: get the lower triangular matrix or uper\n
*                  triangular matrix of the decompositioned matrix.
*/
void CSLAM::GSLGetCholeskyUpperTriangularMatrix(Mat &output, const Mat &input)
{
	int dim = input.rows;

	for (int i=0; i<dim; i++)   // get lower triangular matrix
	{
		for (int j=i; j<dim; j++)
		{
			output.ptr<double>(i)[j] = input.ptr<double>(i)[j];
		}
	}
}

/**
* @brief Matching features' covariance.
* @param[in]
*  flag flag for updating or downdating.
* @param[out]
*  SI square root covariance.
* @note
*  Only for debug: Calculate all features' covariance.
*/
void CSLAM::calculateMatchFeatureCovariance (Mat &SI, const int &flag)
{
	int dim	 = m_X_k.rows;
	int Na   = m_sample.num; 
	int dimx = 2*Na;
	int dimy = 2;
	int index = 0;
	Mat QR(dimx, dimy, CV_64F);
	Mat si(2, 2, CV_64F);
	int N = 0;

	if (FLAG_4_NORMAL_UPDATE == flag)
	{
		N = m_nMatches;
	}
	else if (FLAG_4_LOW_INLIER == flag)
	{
		N = m_nLowInliers;
	}
	else if (FLAG_4_HIGH_INLIER == flag)
	{
		N = m_nHighInliers;
	}

	for (int j = 0; j < N; j++)
	{
		index = 2*j;

		Mat sigma0 = m_matchingSet.rowRange(Range(index,index+2)) + 0;

		for (int i=0; i < 2*Na; i++)
		{
			QR.row(i) = wi_sr*(m_sigma_matchPixel(Range(index,index+2),Range(i+1,i+2)) - sigma0).t();
		}
		GSLQrDecomposition(si, QR);

		si.copyTo(SI(Range(index,index+2), Range(index,index+2)));
	}

	//QR.create(2*Na, 2*m_nMatches, CV_64F);
	//for (int i=0; i < 2*Na; i++)
	//{
	//	QR.row(i) = wi_sr*(m_sigma_matchPixel.col(i+1) - m_matchingSet).t();
	//}
	//GSLQrDecomposition(si, QR);
}

/**
* @brief Matching features' cross covariance.
* @param[in]
*  flag flag for updating or downdating.
* @param[out]
*  Pxy cross covariance.
* @note
*  Only for debug: Calculate all features' cross covariance.
*/
void CSLAM::calculateMatchFeatureCrossCovariance (Mat &Pxy, const int &flag)
{
	int dim = m_X_k.rows;
	int Na  = m_sample.num;
	int N   = 0;  

	if (FLAG_4_NORMAL_UPDATE == flag)
	{
		N = m_nMatches;
	}
	else if (FLAG_4_LOW_INLIER == flag)
	{
		N = m_nLowInliers;
	}
	else if (FLAG_4_HIGH_INLIER == flag)
	{
		N = m_nHighInliers;
	}

	Mat sub1(dim, 1, CV_64F);
	Mat sub2(2*N, 1, CV_64F);

	for (int i = 0; i <= 2*Na; i++)
	{
		sub1 = m_sigma(Range(0,dim), Range(i,i+1)) - m_X_k;
		sub2 = m_sigma_matchPixel.col(i) - m_updatePredictSet;

		if (!i)
			Pxy = wc0*sub1*sub2.t() + 0;
		else
			Pxy += wi*sub1*sub2.t();
	}
}
