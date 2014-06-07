#pragma once
#include <gl\gl.h> 
#include <gl\glu.h> 
#include <gl\glaux.h> 
#include <gl\glut.h>
#include <stdarg.h>
#include "SLAM.h"

struct viewStruct
{
	GLdouble eyex;
	GLdouble eyey;
	GLdouble eyez;
	GLdouble centerx;
	GLdouble centery;
	GLdouble centerz;
	GLdouble upx;
	GLdouble upy;
	GLdouble upz;
};

class COpenGlDisplay
{
public:
	COpenGlDisplay(void);
	~COpenGlDisplay(void);
	
public:
	
	HDC hrenderDC;
	HGLRC hrenderRC;
	int PixelFormat; 
	viewStruct vs;     /**< the view values of the glLookat */

	bool shiftKeyStatus;  /**< shift Key */
	double xRot, yRot, zRot, xTrans, yTrans, zTrans;    /**< 3D Roaming variable */
	double robotScale, robotHeight;

	GLuint nFontList;
	GLuint robotModelList;
	GLuint worldFrameList;
	GLuint groundList;

	bool myInit(CWnd *wnd);

	BOOL setWindowPixelFormat(HDC hDC);   /**< Setting pixel format */
	BOOL createViewGLContext(HDC hDC);    /**< Create View GL Context */

	void buildGroundList (void);
	void buildRobotList (void);
	void buildWorldFrameList (void);
	void createFont(void);
	void drawCurrentFeatures(CSLAM* SLAM);
	void drawHistoryFeatures (CSLAM* SLAM);
	void drawText(const char *fmt, GLfloat scale);
	void drawPathAndOdometry (CSLAM* SLAM);
	void drawRobot(CSLAM* SLAM); 
	void drawWorldFrame(void);
	bool KeyProcessing(UINT nChar);
	void showModel(CSLAM* SLAM);    /**< Draw model */
};
