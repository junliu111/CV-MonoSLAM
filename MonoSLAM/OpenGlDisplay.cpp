/** @file
* @author Jun Liu
* @date 2014-1-1
* @version v1.0.0
*/
/*=========================================================================
*
* This part is used for 3D modeling.
*
*=========================================================================*/

#include "StdAfx.h"
#include "OpenGlDisplay.h"

COpenGlDisplay::COpenGlDisplay(void)
{
}

COpenGlDisplay::~COpenGlDisplay(void)
{
}

// OpenGL display initilization
bool COpenGlDisplay::myInit(CWnd *wnd)
{
	// 初始化转换值
	shiftKeyStatus = FALSE;

	robotScale  = 0.5;
	robotHeight = 0.2;

	xRot = -90.f;
	yRot = 10.f;
	zRot = yRot;

	xTrans = 0.f;
	yTrans = -2.f;
	zTrans = -8.f;

	vs.eyex = 1.0;
	vs.eyey = 3.5;
	vs.eyez = -6;
	vs.centerx = xTrans;
	vs.centery = yTrans;
	vs.centerz = zTrans;
	vs.upx = 0;
	vs.upy = 1;
	vs.upz = 0;
	// ============需要按实际情况设定======= //

	CRect mRect;

	hrenderDC=::GetDC(wnd->m_hWnd); 
	if(setWindowPixelFormat(hrenderDC)==FALSE)  //设置像素格式
		return 0; 

	if(createViewGLContext(hrenderDC)==FALSE)   // 取得着色描述表
		return 0; 

	wnd->GetClientRect(&mRect);
	glViewport(0,0,mRect.Width(),mRect.Height()); 
	glMatrixMode(GL_PROJECTION);				// 选择投影矩阵
	glLoadIdentity();							// 重置投影矩阵
	gluPerspective(90,1.0,0.01,100.0); 	        // 设置视口的大小
	glMatrixMode(GL_MODELVIEW);				    // 选择模型观察矩阵
	glLoadIdentity();							// 重置模型观察矩阵
	// ============需要按实际情况设定======= //
	gluLookAt(vs.eyex, vs.eyey, vs.eyez,
			  vs.centerx, vs.centery, vs.centerz,
			  vs.upx, vs.upy, vs.upz);          // 调整观察，
	// ============需要按实际情况设定======= //
	glShadeModel(GL_SMOOTH);					// Enable Smooth Shading 
	glClearColor(200.0/255.0, 200.0/255.0, 200.0/255.0, 0);	// Black Background 
	glClearDepth(1.0f);							// Depth Buffer Setup 
	glEnable(GL_DEPTH_TEST);					// Enables Depth Testing 
	glDepthFunc(GL_LEQUAL);						// The Type Of Depth Testing To Do 
	glEnableClientState(GL_VERTEX_ARRAY); 
	glEnableClientState(GL_TEXTURE_COORD_ARRAY); 
	createFont();

	GLfloat lightPos[] ={0, 0, 0};
	GLfloat ambientLight[] = {0.3f, 0.3f, 0.3f, 1.0f};
	GLfloat diffuseLight[] = {0.5f, 0.5f, 0.5f, 1.0f};

	//glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);    //设置并启动光照
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE); //设置材料属性，与glColor值对应
	glEnable(GL_COLOR_MATERIAL);                       //启动颜色跟踪

	this->buildRobotList();     // construct the show list
	this->buildWorldFrameList();
	this->buildGroundList();

	return true;
}

// Set window pixel format
BOOL COpenGlDisplay::setWindowPixelFormat(HDC hDC) 
{/*
 下面的代码描述象素格式。我们选择了通过RGBA(红、绿、蓝、alpha通道)支持OpenGL和双缓存的格式。
 我们试图找到匹配我们选定的色彩深度(16位、24位、32位)的象素格式。最后设置0位Z-缓存。
 其余的参数要么未使用要么不重要(stencil buffer模板缓存和accumulation buffer聚集缓存除外)。
 */
	PIXELFORMATDESCRIPTOR pixelDesc; 
	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR); 
	pixelDesc.nVersion = 1; 
	pixelDesc.dwFlags = 
		PFD_DRAW_TO_WINDOW |   // support window
		PFD_SUPPORT_OPENGL |   // support OpenGL
		PFD_DOUBLEBUFFER |     // double buffered
		PFD_TYPE_RGBA;         // RGBA type
	pixelDesc.iPixelType = PFD_TYPE_RGBA; 
	pixelDesc.cColorBits = 32; 
	pixelDesc.cRedBits = 0; 
	pixelDesc.cRedShift = 0; 
	pixelDesc.cGreenBits = 0; 
	pixelDesc.cGreenShift = 0; 
	pixelDesc.cBlueBits = 0; 
	pixelDesc.cBlueShift = 0; 
	pixelDesc.cAlphaBits = 0; 
	pixelDesc.cAlphaShift = 0; 
	pixelDesc.cAccumBits = 0; 
	pixelDesc.cAccumRedBits = 0; 
	pixelDesc.cAccumGreenBits = 0; 
	pixelDesc.cAccumBlueBits = 0; 
	pixelDesc.cAccumAlphaBits = 0; 
	pixelDesc.cDepthBits = 0; 
	pixelDesc.cStencilBits = 1; 
	pixelDesc.cAuxBuffers = 0; 
	pixelDesc.iLayerType = PFD_MAIN_PLANE; 
	pixelDesc.bReserved = 0; 
	pixelDesc.dwLayerMask = 0; 
	pixelDesc.dwVisibleMask = 0; 
	pixelDesc.dwDamageMask = 0; 
	PixelFormat = ChoosePixelFormat(hDC,&pixelDesc);// Windows 找到相应的象素格式了吗? 
	if(PixelFormat==0) // Choose default 
	{ 
		PixelFormat = 1; 
		if(DescribePixelFormat(hDC,PixelFormat, 
			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc)==0){ 
				return FALSE; 
		} 

	} 
	if(SetPixelFormat(hDC,PixelFormat,&pixelDesc)==FALSE){ 
		return FALSE; 
	} // 能够设置象素格式么?

	return TRUE; 
} 

// Create view GL context
BOOL COpenGlDisplay::createViewGLContext(HDC hDC) 
{ 
	hrenderRC = wglCreateContext(hDC);  // 能否取得着色描述表?

	if(hrenderRC==NULL) 
	{
		return FALSE; 
	}
	if(wglMakeCurrent(hDC,hrenderRC)==FALSE) // 尝试激活着色描述表
	{
		return FALSE; 
	}

	return TRUE; 
} 

// Create font
void COpenGlDisplay::createFont(void)
{
	HFONT hFont;
	GLYPHMETRICSFLOAT agmf[128];
	LOGFONT logfont;

	logfont.lfHeight = -10;
	logfont.lfWidth = 0;
	logfont.lfEscapement = 0;
	logfont.lfOrientation = 0;
	logfont.lfWeight = FW_BOLD;
	logfont.lfItalic = FALSE;
	logfont.lfUnderline = FALSE;
	logfont.lfStrikeOut = FALSE;
	logfont.lfCharSet = ANSI_CHARSET;
	logfont.lfOutPrecision = OUT_DEFAULT_PRECIS;
	logfont.lfClipPrecision = CLIP_DEFAULT_PRECIS;
	logfont.lfQuality = DEFAULT_QUALITY;
	logfont.lfPitchAndFamily = DEFAULT_PITCH;
	strcpy_s(logfont.lfFaceName, _T("Arial"));

	//创建字体和显示列表
	hFont = CreateFontIndirect(&logfont);
	SelectObject(hrenderDC, hFont);

	nFontList = glGenLists(128);
	wglUseFontOutlines(hrenderDC, 0, 128, nFontList, 0.0f, 0.1f, WGL_FONT_POLYGONS, agmf);

	DeleteObject(hFont);
}

// Show 3D model
void COpenGlDisplay::showModel(CSLAM* SLAM)   
{ 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	glPushMatrix();
	glTranslatef (xTrans, yTrans, zTrans); // OpenGL施加操作的平移、旋转顺序与程序的先后顺序相反
	glRotatef(yRot, 0, 1, 0);
	glCallList(groundList);
	glPopMatrix();

	glPushMatrix();
	glTranslatef (xTrans, yTrans, zTrans);
	glRotatef(xRot, 1, 0, 0);
	glRotatef(zRot, 0, 0, 1);
	glCallList(worldFrameList);
	drawRobot(SLAM);
	drawPathAndOdometry(SLAM);
	drawCurrentFeatures(SLAM);
	drawHistoryFeatures(SLAM);
	glPopMatrix();

	SwapBuffers(hrenderDC);
} 

// Build world frame list
void COpenGlDisplay::buildWorldFrameList(void)
{
	worldFrameList = glGenLists(1);

	if (0 == worldFrameList)
	{
		return;
	}

	float axis_length = 2.0f;
	float axis_width  = 1.5f;
	float text_scale  = 0.6;

	glNewList(worldFrameList, GL_COMPILE);   // Generate show list

	glPushMatrix();
	glColor3f(205./255., 186./255., 250./255.);
	glLineWidth(axis_width);

	// Draw x axis
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(axis_length,0,0);
	glEnd();

	glPushMatrix();
	glTranslatef(axis_length, 0.0f, 0.0f);
	drawText("X", text_scale);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(axis_length, 0.0f, 0.0f);
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	glutSolidCone(axis_length*0.02, axis_length*0.05, 100,100);   // Draw coordinate arrow
	glPopMatrix();
	
	// Draw y axis
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(0,axis_length,0);
	glEnd();

	glPushMatrix();
	glTranslatef(0.0f, axis_length, 0.0f);
	drawText("Y", text_scale);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(0.0f, axis_length, 0.0f);
	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	glutSolidCone(axis_length*0.02, axis_length*0.05, 100,100);  // Draw coordinate arrow
	glPopMatrix();
	
	// Draw z axis
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(0,0,axis_length);   
	glEnd();

	glPushMatrix();
	glTranslatef(0.0f, 0.0f, axis_length);
	glRotatef(90, 1, 0, 0);
	drawText("Z", text_scale);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(0.0f, 0.0f, axis_length);
	glutSolidCone(axis_length*0.02, axis_length*0.05, 100,100);  // Draw coordinate arrow
	glPopMatrix();

	glPopMatrix();
	glEndList();
}

// Build ground list
void COpenGlDisplay::buildGroundList (void)
{
	GLfloat lineWidth = 0.1;
	GLfloat fExtent = 6.0f;
	GLfloat fStep = 0.5f;
	GLfloat y = yTrans;
	GLfloat iLine;

	groundList = glGenLists(1);

	if (0 == groundList)
	{
		return;
	}

	glNewList(groundList, GL_COMPILE);   // Generate show list

	glColor3f( 218./255, 165./255., 32./255.);
	glLineWidth(lineWidth);
	glBegin(GL_LINES);

	for(iLine = -fExtent; iLine <= fExtent; iLine += fStep)
	{
		glVertex3f(fExtent, y, iLine);   // x-axis
		glVertex3f(-fExtent, y, iLine);
		glVertex3f(iLine, y, fExtent);   // z-axis
		glVertex3f(iLine, y, -fExtent);
	}

	glEnd();
	glEndList();
}

// Draw robot's show list (robot model)
void COpenGlDisplay::buildRobotList (void)
{
	robotModelList = glGenLists(1);

	if (0 == robotModelList)
	{
		return;
	}

	glNewList(robotModelList, GL_COMPILE);   // Generate show list

	glColor3f(126./255., 192./255., 238./255.);
	glBegin(GL_POLYGON);
	glVertex3f(0, robotScale, 0);  
	glVertex3f(-0.57*robotScale, -0.5*robotScale, 0);  
	glVertex3f(0, 0, 0);
	glVertex3f(0.57*robotScale, -0.5*robotScale, 0);
	glEnd(); 

	glBegin(GL_POLYGON); 
	glVertex3f(0, robotScale, robotHeight);
	glVertex3f(-0.57*robotScale, -0.5*robotScale, robotHeight);  
	glVertex3f(0, 0, robotHeight);
	glVertex3f(0.57*robotScale, -0.5*robotScale, robotHeight);
	glEnd(); 

	glColor3f(205./255., 150./255., 205./255.);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(0, robotScale, robotHeight);
	glVertex3f(0, robotScale, 0);
	glVertex3f(-0.57*robotScale, -0.5*robotScale, robotHeight);
	glVertex3f(-0.57*robotScale, -0.5*robotScale, 0);
	glVertex3f(0, 0, robotHeight);
	glVertex3f(0, 0, 0);
	glVertex3f(0.57*robotScale, -0.5*robotScale, robotHeight);
	glVertex3f(0.57*robotScale, -0.5*robotScale, 0);
	glVertex3f(0, robotScale, robotHeight);
	glVertex3f(0, robotScale, 0);
	glEnd();

	glEndList();
}

// Draw robot state (history)
void COpenGlDisplay::drawRobot(CSLAM* SLAM)
{
	int dim = SLAM->m_X_k.rows;
	double angle, x, y;

	x     = SLAM->m_X_k.ptr<double>(dim-4)[0];
	y     = SLAM->m_X_k.ptr<double>(dim-3)[0];
	angle = SLAM->m_X_k.ptr<double>(dim-1)[0];
	angle = angle*180/3.141593f - 90;    // 注：opengl中的夹角为与y轴正方向的夹角

	// Draw robot
	glPushMatrix();
	glTranslatef(x, y, 0);
	glRotatef(angle, 0, 0, 1);
	glCallList(robotModelList);
	glPopMatrix();

	// Draw the connection between robot and features
	glColor3f(255./255., 0./255., 255./255.);
	PointsMap* map_p = SLAM->map;
	while (NULL != map_p)
	{
		if ((true == map_p->isMatching))// && (false == map_p->isDelay))
		{
			glLineWidth(1.0);
			glBegin(GL_LINES);
			glVertex3d(x, y, robotHeight);

			if (TRUE == SLAM->isCompensate)
			{
				glVertex3d(map_p->xyz.x, map_p->xyz.y, (map_p->xyz.z + SLAM->m_deep)/2.0);
			} 
			else
			{
				glVertex3d(map_p->xyz.x, map_p->xyz.y, map_p->xyz.z);
			}

			
			glEnd();
		}
		map_p = map_p->next;
	}
}

// Draw robot path and odometry information using vertex array
void COpenGlDisplay::drawPathAndOdometry (CSLAM* SLAM)
{
	float line_width = 2;

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_DOUBLE, 0, SLAM->m_path);
	glColor3f(0, 1, 0);
	glLineWidth(line_width);
	glDrawArrays(GL_LINE_STRIP, 0, SLAM->m_frame.counter);
	glDisableClientState(GL_VERTEX_ARRAY); 

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_DOUBLE, 0, SLAM->m_odoXY);
	glColor3f(1, 0, 0);
	glLineWidth(line_width);
	glDrawArrays(GL_LINE_STRIP, 0, SLAM->m_frame.counter);
	glDisableClientState(GL_VERTEX_ARRAY); 
}

// Draw feature with respect to the world frame
void COpenGlDisplay::drawCurrentFeatures(CSLAM* SLAM)
{
	if (0 == SLAM->m_nMatches)
	{
		return;
	}

	Point3f location, sigma;
	Quaternion axis;
	char text[10];

	PointsMap* map_p = SLAM->map; 
	while (NULL != map_p)
	{
		if (true == map_p->isMatching)
		{
			glPushMatrix();

			location.x = (float)map_p->xyz.x;
			location.y = (float)map_p->xyz.y;

			if (TRUE == SLAM->isCompensate)
			{
				location.z = (float)(map_p->xyz.z + SLAM->m_deep)/2.0;
			} 
			else
			{
				location.z = (float)map_p->xyz.z;
			}

			glTranslatef(location.x,location.y,location.z);      // Move Left 1.5 Units And Into The Screen 6.0 
			glColor3f( 1.0f, 0.0f, 0.0f );
			_itoa_s(map_p->ID, text, 10);
			drawText(text, 0.3);
			
			sigma.x = (float)map_p->sigma.x;
			sigma.y = (float)map_p->sigma.y;
			sigma.z = (float)map_p->sigma.z;

			axis = map_p->axis;
			float angle = 2*atan2(sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z), axis.r);
			angle = angle*180/3.141593f;

			//glTranslatef(location.x,location.y,location.z);
			glRotated(angle, -axis.x, -axis.y, -axis.z); 
			glScaled(min(4.0f,sigma.x),min(4.0f,sigma.y),min(4.0f,sigma.z));
			glEnable(GL_LIGHTING);

			if (true == map_p->isLoop)
			{
				glColor3f( 0.0f, 1.0f, 0.0f );
			} 
			else
			{
				glColor3f( 1.0f, 0.0f, 0.0f );
			}

			glutWireSphere(3.0f, 10,10);
			glPopMatrix();
		}
		map_p = map_p->next;
	}
}

// Draw all history features
void COpenGlDisplay::drawHistoryFeatures (CSLAM* SLAM)
{ 
	int number = SLAM->m_featuresAllInfo.size();

	if (0 == number)
	{
		return;
	}

	Point3f location, sigma;
	Quaternion axis;
	char text[10];
	int id;

	for(int i = 0; i < number; i++)
	{
		glPushMatrix();
		location.x = (float)SLAM->m_featuresAllInfo[i].position.x;
		location.y = (float)SLAM->m_featuresAllInfo[i].position.y;

		if (TRUE == SLAM->isCompensate)
		{
			location.z = (float)(SLAM->m_featuresAllInfo[i].position.z + SLAM->m_deep)/2.0;
		} 
		else
		{
			location.z = (float)SLAM->m_featuresAllInfo[i].position.z;
		}

		id = SLAM->m_featuresAllInfo[i].ID;

		glTranslatef(location.x,location.y,location.z);
		//glColor3f( 0.0f, 0.0f, 1.0f );
		//_itoa_s(id, text, 10);
		//drawText(text, 0.3);

		sigma.x = (float)SLAM->m_featuresAllInfo[i].sigma.x;
		sigma.y = (float)SLAM->m_featuresAllInfo[i].sigma.y;
		sigma.z = (float)SLAM->m_featuresAllInfo[i].sigma.z;

		axis = SLAM->m_featuresAllInfo[i].axis;

		float angle = 2*atan2(sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z), axis.r);
		angle = angle*180/3.141593f;

		glRotated(angle, -axis.x, -axis.y, -axis.z); 
		glScaled(min(4.0f,sigma.x),min(4.0f,sigma.y),min(4.0f,sigma.z));
		glEnable(GL_LIGHTING);
		//glColor3f( 1.0f, 0.0f, 0.0f );

		if (true == SLAM->m_featuresAllInfo[i].isLoop)
		{
			glColor3f( 0.0f, 1.0f, 0.0f);
		}
		else
		{
			glColor3f( 0.0f, 0.0f, 250./255.);
		}
		//glColor3f(54./255., 100./255., 139./255.);
		glutWireSphere(3.0f, 10,10);
		glPopMatrix();

		//glColor3f(192./255., 255./255., 62./255.);
		//glLineWidth(1.0);
		//glBegin(GL_LINES);
		//glVertex3d(location.x, location.y, 0);
		//glVertex3d(location.x, location.y, location.z);
		//glEnd();
	}
}

// Draw text
void COpenGlDisplay::drawText(const char *fmt, GLfloat scale)
{
	glPushMatrix();
	glScalef(scale, scale, scale);
	glListBase(nFontList);
	glCallLists(strlen(fmt), GL_UNSIGNED_BYTE, fmt);
	glPopMatrix();
}

// Process the key input
bool COpenGlDisplay::KeyProcessing(UINT nChar)
{
	bool needRedraw = false;

	switch (nChar) 
	{
	case 'w':
	case 'W':
		if (shiftKeyStatus)
			yTrans += .15f;
		else
			xRot += 3.0f;
		needRedraw = true;
		break;

	case 's':
	case 'S':
		if (shiftKeyStatus)
			yTrans -= .15f;
		else
			xRot -= 3.0f;
		needRedraw = true;
		break;

	case 'a':
	case 'A':
		if (shiftKeyStatus)
			xTrans -= .15f;
		else
			zRot += 3.f;
		needRedraw = true;
		break;
	
	case 'd':
	case 'D':
		if (shiftKeyStatus)
			xTrans += .15f;
		else
			zRot -= 3.f;
		needRedraw = true;
		break;

	case VK_SHIFT:
		shiftKeyStatus = shiftKeyStatus^1;
		needRedraw = false;
	}

	return needRedraw;
}
