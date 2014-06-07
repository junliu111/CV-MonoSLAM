/** @file
* @author Jun Liu
* @date 2014-1-1
* @version v1.0.0
*/
#pragma once

#include <iostream>
#include <string>
#include <stdlib.h>
#include <MATH.H>

#include <opencv.hpp>
#include <opencv2/legacy/legacy.hpp>  
#include <opencv2/nonfree/nonfree.hpp>  

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_permutation.h>

#include "CvImage.h"

//#include "StdAfx.h"

//#include <Eigen/Eigen>
//#include <Eigen/Dense>
//#include <Eigen/src/LU/FullPivLU.h>

// Sift/SurfFeatureDetector, Sift/SurfDescriptorExtractor包含在
// D:\Program Files\OpenCV 2.4.2\include\opencv2\nonfree

#define GSL_DLL

using namespace std;
using namespace cv;
//using namespace Eigen;

struct Quaternion
{
	double r;
	double x;
	double y;
	double z;
};

struct PointsMap  // ============注意去除不需要的变量????
{
	int     ID;					    // ID
	bool    isVisible;				// whether the predicted points is visiable or not
	bool    isMatching;				// whether the feature is matching or not
	bool    isLoop;                 // whether to delay for deleting features or not
	int     nPredictTimes;			// the predicted times of feature
	int     nMatchTimes;			// the matched times of feature
	Point2d predictLocation;		// predicted position =========== 改为整型以减少所需空间???????
	Point2d matchLocation;			// matched position=========== 改为整型以减少所需空间???????
	Mat     Si;	                    // the feature's covariance

	Point2d initPixel;     
	Mat     initTrans;				// the feature's translation vector 
	Mat     initRotation;		    // the feature's rotation matrix
	Mat     initPatch;			    // the block which center on the observed feature, will be used to cal the corr with match block
	Mat     matchPatch;

	Point3d    xyz;					// feature's coordinate wrt world
	Quaternion axis;                // rotation quaternion
	Point3d    sigma;               // intercept

	PointsMap *next;
};

struct SampleParameter
{
	int    num;
	double Alpha;
	double Beta;
	double Gamma;
	double Kappa;
	double Lammda;
	double wm0;
	double wc0;
	double wi;
};

struct FrameInfo
{
	int rate;						    // frame rate
	int start;						    // which frame to start
	int stop;						    // which frame to stop
	int index;						    // index of frame
	int counter;						// counter for passed frames
};

struct FeatureInfo
{
	bool       isLoop;
	int        ID;
	int        nPredictTimes;
	int        nMatchTimes;
	Point3d    initXYZ;
	Point2d    initPixel;     
	Mat        initTrans;			
	Mat        initRotation;		  
	Mat        initPatch;			 

	Mat        state;
	Point3d    position;                 // 3D position
	Mat        sr;                       // square of convariance
	Mat        cov;                      // covariance
	Quaternion axis;                     // rotation quaternion
	Point3d    sigma;                    // intercept
};

/**
* @class
* @brief Header file of SLAM.cpp
*/
class CSLAM
{
public:

	//========Constructor & Destructor=============//
	CSLAM(void);
	~CSLAM(void);

	//================Constant value===============//
	static const int CAPACITY           = 3000;
	const int HFOV;							// horizontal field of view
	const int FLAG_4_VIDEO;                 // flag for video
	const int FLAG_4_IMAGE;                 // flag for image consequence
	const int FLAG_4_WEIGHT1;               // flag for configuration of different weight
	const int FLAG_4_WEIGHT2;               // flag for configuration of different weight
	const int FLAG_4_WEIGHT3;               // flag for configuration of different weight
	const int FLAG_4_NOISE1;
	const int FLAG_4_NOISE2;
	const int FLAG_4_NOISE3;
	const int FLAG_4_NOISE4;
	const int FLAG_4_UPDATING;              // flag for updating in cholesky update
	const int FLAG_4_DOWNDATING;            // flag for downdating in cholesky update
	const int FLAG_4_NORMAL_UPDATE;         // 
	const int FLAG_4_LOW_INLIER;
	const int FLAG_4_HIGH_INLIER;
	const int FLAG_4_NEED_REORDER;
	const int FLAG_4_NEEDNOT_REORDER;
	const int FLAG_4_DEBUG_MODEL;
	const int FLAG_4_RELEASE_MODEL;
	const int FLAG_4_UNSELECT_MODEL;
	const double EPSILON;                   // numerical accuracy	
	const double DIST_2_BORDER;
	const Matx33d CHI2INV_TABLE;			// Chi-square table


	//==================Variables==================//
	PointsMap*       map;					// features map
	PointsMap*       blackList;				// blacklist of map  
	IplImage*        m_srcImage;			// variables for store the original image or video
	IplImage*        m_gryImage;			// variables for store the gray image
	IplImage*        m_tmpImage;
	CvCapture*       m_capture;				// get video interface structure
	Size             m_imageSize;
	SampleParameter  m_sample;
	FrameInfo		 m_frame;   
	TickMeter        m_frameTimer;           // timer for per frame elapsed time
	TickMeter        m_totalTimer;          // timer for total elapsed time
	CEdit            m_showFeatureInfo;     // 特征位置显示
	CListCtrl        m_listCtrl;            // for show robot's information
	CDC*             m_pDc;	                // 用来存显示图片的DC
	CRect            m_rect;			    // 用来显示图片的矩形框
	vector<KeyPoint>          m_keyPoints;  // vector for filtered keypoints // =======需转换精度????????
	vector<FeatureInfo>	m_featuresAllInfo;  // vector for all features' information
	//SurfDescriptorExtractor     extractor;	// Surf-descriptor
	//BruteForceMatcher<L2<double>> matcher;	// Matcher
	FILE                  *m_odometryFile;
	FILE					 *m_robotFile;
	FILE				  *m_featuresFile;
	CString                    m_videoDir;
	CString                    m_imageDir;
	CString                 m_odometryDir;
	CString              m_recordRobotDir;
	CString           m_recordFeaturesDir;

	char				   video_dir[100];
	char                   image_dir[100];  // directory of image
	char                odometry_dir[100];  // directory of odometry data set
	char            record_robot_dir[100];
	char         record_features_dir[100];

	double        m_odoXY[2*(CAPACITY+1)];
	double         m_path[2*(CAPACITY+1)];

	bool isCheckHFOV;
	BOOL isAdding;
	BOOL isUseRANSAC;                       // switch: whether use 1-point ransac or not
	BOOL isUseHarris;
	BOOL isRecordRobotInfo;
	BOOL isRecordFeaturesInfo;
	BOOL isShowRobotPath;
	BOOL isShowOdoInfo;
	BOOL isCompensate;
	BOOL isShowOdoCtrlList;

	int ID; 								// ID
	int imageWidth;						    // image width
	int imageHeight;						// image height
	int m_playType;					        // use video or image sequence
	int m_weightType;                       // type of weight
	int m_noiseType;                        // type of noise
	int m_odoCounter;                       // 
	int m_showCounter;                      // 
	int m_covRank;                          // rank of covariance
	int m_nInitialRaws;                     // number of raw features in initialization
	int m_nProcessRaws;			            // number of raw features in process
	int m_nFilters;				            // number of filtered features
	int m_nAddings;                         // number of new adding features
	int m_nDeletes;                         // number of deleted features
	int m_nStores;                          // number of features have to be stored in current frame, mainly for .txt recording
	int m_nPredicts;						// number of prditct features
	int m_nMatches;							// number of matching features
	int m_nMapFeatures;						// the total NO. of features in the map
	int m_nStoreMap;                        // comtemporary store the number of map features before redirection
	int m_nStorePredicts;					// comtemporary store the number of predict features before redirection
	int m_nStoreMatches;					// comtemporary store the number of match features before redirection
	int m_nShowMap;
	int m_nShowPredicts;
	int m_nShowMatches;
	int m_nStore4Show;                      // to store the total map features for initial show
	int m_nLowInliers;                      // number of low-innovation inliers
	int m_nHighInliers;                     // number of high-innovation inliers
	int HP_INIT_W;					        // half of the width of initial patch 
	int HP_INIT_H;					        // half of the height of initial patch 
	int HP_MATCH_W;					        // half of the width of match patch 
	int HP_MATCH_H;					        // half of the height of match patch 
	int m_minNUM;						    // the minimum number of feature points in the map
	int m_blockSize;                        // parameter for finding features using goodfeaturestotrack
	int m_model;                            // flag for select whether the debug model or release model
	int m_loopPointCounter;               // counter for former point which satisfy the limitation

	double m_minDist;						// the minimum distance between two feature point
	double m_minDist2;                      // square of minimun distance between two feature point
	double m_sigmaMeasure;				    // standard variance of measurement
	double m_deep;
	double m_rho;						    // inverse depth of feature point
	double m_sigmaRHO;					    // standard variance of inversedepth
	double m_sigmaX;                        // covariance of the state vector in x axis 
	double m_sigmaY;                        // covariance of the state vector in y axis
	double m_sigmaZ;                        // covariance of the state vector in z axis
	double m_sigmaTheta;                    // covariance of the state vector in angle
	double THRESHOLD_MATCH_PATCH;		    // threshold for patch matching
	double THRESHOLD_RANSAC;                // the Euclidean distance between measurement and prediction in 1-Point method 
	double LINEARITY_INDEX_THRESHOLD;       // threshold for transform inverse depth into deepth
	double gamma;
	double wm0;                             
	double wm0_sr;
	double wc0;
	double wc0_sr;
	double wi;
	double wi_sr;
	double fps;
	double fourcc;
	double m_qualityLevel;
	double a1, a2, a3, a4;
	double MIN_STEP_X;
	double MIN_STEP_Y;
	double MIN_STEP_THETA;
	double m_frameTime;
	double m_totalTime;

	Mat m_initOdo;                          // initial odometry value
	Mat m_initPos;                          // initial position value
	Mat m_odoTheta;                         // initial index, theta, and flag for redirection
	Mat m_X_k;								// state vector
	Mat m_S_k;								// square root of covariance(upper triangular matrix)
	Mat m_P_k;                              // covariance
	Mat m_zt;								// actual position of feature point
	Mat m_ht;								// predict position of feature point
	Mat m_St;								// the squre root of covariance of feature point
	Mat m_filterID;                         // set of new added feature's ID
	Mat m_deleteID;                         // set of deleted feature's ID
	Mat m_matchID;                          // set of matched feature's ID
	Mat m_permutation;                      // permutation for Cholesky decomposition
	Mat m_allPredictSet;					// predict location of all predict features
	Mat m_updatePredictSet;					// feature set that used in Kalman update 
	Mat m_matchingSet;						// matching features of all predict features
	Mat m_sigma;							// set of sigma points to store state vector before measurement update
	Mat m_sigma_allPixel;				    // set of sigma points for measurement only
	Mat m_sigma_matchPixel;					// set of sigma points for matched features only, for calculate cross-covariance
	Mat Mt;                                 // control noise
	Mat Qt;                                 // measurement noise
	Mat Ut;                                 // control input
	Mat m_loopPointID;                      // ID for loop closure

	//================Camera's parameters=====================//
	double  cam_dx;							// 单个像素在x方向的物理尺寸(mm/pixel)
	double  cam_dy;							// 单个像素在y方向的物理尺寸(mm/pixel)
	double  cam_cx;							// 图像坐标系中物理坐标中心在像素坐标中心上x方向的像素(pixel)
	double  cam_cy;							// 图像坐标系中物理坐标中心在像素坐标中心上y方向的像素(pixel)
	double  cam_k1;							// 相机畸变参数
	double  cam_k2;							// 相机畸变参数
	double  cam_f;							// 焦距(mm)
	double  cam_f1;							// 相机内参alpha_x(pixel)
	double  cam_f2;							// 相机内参alpha_y(pixel)
	Matx33d cam_K;							// 相机内参矩阵

	RNG rng;

	//============Function declaration===================//
	PointsMap* deleteOneFeature (const int &id, const int &mapID, PointsMap* p_head); 

	bool calculateEigenvaluesAndEigenvectors(Mat src, Mat& eigenvalues, Mat& eigenvectors) const;
	bool isIdBelongsToSet (const double &id, const Mat &set) const;
	bool isKeyPointInDeletedSet(const int &id) const;

	double Gauss(const double &V);
	double wrapAngle (double angle) const;
	
	void addFeatures (void);
	void calculateCrossCorrelation (double &corr, const Mat &src1, const Mat &src2) const;
	void calculateMatchFeatureCovariance (Mat &SI, const int &flag);
	void calculateMatchFeatureCrossCovariance (Mat &Pxy, const int &flag);
	void calculateOneFeatureCovariance (Mat &si, const int &id);
	void calculateOneFeatureCrossCovariance (Mat &Pxy, const int &id, const Mat &hi) const;
	void calculateSampleParameter(const int & Na);
	void CholeskyDecomposition (Mat &output, Mat &input);
	void CholeskyDecompositionWithPivoting (Mat &sr, Mat &Cov);
	void coordinatesCamera2Image (Point2d &uvu, const Mat &Hlr, const Mat &error) const;
	void coordinatesCamera2World (Mat &Hlw, const Mat &Hlr, const Mat &Rwc) const;
	void coordinatesImage2Camera (Mat &Hlr, const Point2d & uvu) const;
	void coordinatesInverseDepth2Cartesian(Point3d &xyz, const Mat &position, const Mat &state) const;
	void coordinatesState2World (Mat &Hlw, const Mat &state, const Mat &cam_position) const;
	void coordinatesWorld2Camera (Mat &Hlr, const Mat &Hlw, const Mat &Rcw) const;
	void coordinatesWorld2State (Mat &state, const Mat &Hlw, const double &rho) const;
	void dataAssociation (void);
	void dataTypeCVMat2GSLMat (gsl_matrix *gsl_mat, const Mat &cv_mat) const;
	void dataTypeGSLMat2CVMat (Mat &cv_mat, gsl_matrix *gsl_mat) const;
	void detectAndfilteringFeatures (void);
	void display2DFeatureModel (void);
	void display3DRobot(Point3f &position, Quaternion &axis, Point3f& sigma, int &framCounter) const;
	void distortOnePointRW (Point2d &uvd, const Point2d &uvu) const;
	void draw2DEllipse (IplImage* displayImage,const CvPoint &center,const Mat &covMatrix, const CvScalar color, int thickness) const;
	void expandMatrix (Mat &mu, Mat &sr, const Mat &mu1, const Mat &sr1, const Mat &mu2, const Mat &sr2) const;
	void generateSigmaPoints (Mat &sigma, const Mat &Mu, const Mat &SR);
	void get3DdisplayInformation (Quaternion &axis, Point3d &sigma, const Mat &matrix) const;
	void getFeatureCartesianInformation(Point3d &xyz, Mat &sr, Mat &cov, const int &id) const;
	void getOneMomentData (int &id, const int &counter);
	void getPermutationMatrix (void);
	void getTransferMatrix(Mat &Rwc, const double &theta) const;
	void GSLCholeskyUpdate (const Mat &u, const int &flag4UpOrDown, const int &flag4Order);
	void GSLQrDecomposition (Mat &cv_s, const Mat &cv_qr) const;
	void initializeParameters (void);
	void insureEnoughFeatures (void);
	void integrateFeaturesInformation (void);
	void loadOdometryData (void);
	void loadPictures (void);
	void matrix2Quaternion(Quaternion &quaternion, const Mat &matrix) const;
	void modifiedCholeskyDecomposition(Mat &S, const Mat &input);
	void passSigmaThroughMapingFunction (Mat &sigma_out, Mat &mu_Hlw, Mat &mu_angle, const Mat &sigma_in);
	void passSigmaThroughMesaurementFunction (void);
	void passSigmaThroughMotionFunction (const Mat &Ut);
	void predictMeasurement (void);
	void predictMotion (void);
	void QrAndCholeskyForInitilization (const Mat &sigma);
	void QrAndCholeskyForMeasurement (void);
	void QrAndCholeskyForMotion (void);
	void recordData (void);
	void recordFeaturesInformation (void);
	void recordRobotInformation (void);
	void resetAllParameters (void);
	void startTimer (void);
	void stopTimer (void);
	void SLAM (void);
	void undistortOnePointRW (Point2d &uv, const Point2d &point) const;
	void KalmanUpdate (void);
	void updateFeaturesInformation (void);
	void updatePermutationMatrix (void);
	void updateRobotInformation (void);
	void wrapPatch (void);
	
	



	// 暂时未用上
	bool checkMatrixProperties (const Mat &input, const bool &isCheckProperties, const int &flag4Process) const;
	void CSLAM::printLocationCovariance (void) const;
	void CSLAM::printLocationMean (void) const;
	void CSLAM::dataAssociationSimulation (void);
	double CSLAM::Gaussian(const double &sigma);
	void CSLAM::GSLModifiedQRDecomposition(Mat &sr, const Mat &QR);
	void CSLAM::dataTypeGSLPermutation2CVMat(Mat &cv_mat, gsl_permutation *gsl_permutate);
	void CSLAM::GSLCholeskyDecomposition(Mat &chol, const Mat &input);
	void CSLAM::GSLGetCholeskyUpperTriangularMatrix(Mat &output, const Mat &input);
	void CSLAM::print2DFeatureMean (void);
	void CSLAM::print2DFeatureCovariance(void);
	void CSLAM::print3DFeatureMean (void);
	void CSLAM::print3DFeatureCovariance (void);
	void makeMatrixSymmetric (Mat &input) const; //使矩阵对称化
	void CSLAM::keyPointShow (void);
};