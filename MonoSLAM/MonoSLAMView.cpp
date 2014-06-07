/** @file
* @author Jun Liu
* @date 2014-1-1
* @version v1.0.0
*/
/*=========================================================================
*
* This part is used for MFC setting.
*
*=========================================================================*/

#include "stdafx.h"
#include "MonoSLAM.h"
#include "atlImage.h"

#include "MonoSLAMDoc.h"
#include "MonoSLAMView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CMonoSLAMView

IMPLEMENT_DYNCREATE(CMonoSLAMView, CFormView)

BEGIN_MESSAGE_MAP(CMonoSLAMView, CFormView)
	ON_BN_CLICKED(IDC_OPEN_CMD, &CMonoSLAMView::OnBnClickedOpenCmd)
	ON_BN_CLICKED(IDC_RADIO_DEBUG, &CMonoSLAMView::OnBnClickedRadioDebug)
	ON_BN_CLICKED(IDC_RADIO_RELEASE, &CMonoSLAMView::OnBnClickedRadioRelease)
	ON_BN_CLICKED(IDC_BTN_SOURCE_FILE, &CMonoSLAMView::OnBnClickedSourceFile)
	ON_BN_CLICKED(IDC_BTN_ODOMETRY_FILE, &CMonoSLAMView::OnBnClickedOdometryFile)
	ON_BN_CLICKED(IDC_BTN_SET_PARAMETERS, &CMonoSLAMView::OnBnClickedSetParameters)
	ON_BN_CLICKED(IDC_UKF_SLAM, &CMonoSLAMView::OnBnClickedSLAM)
	ON_BN_CLICKED(IDC_BTN_AUTO, &CMonoSLAMView::OnBnClickedAuto)
	ON_BN_CLICKED(IDC_Restart, &CMonoSLAMView::OnBnClickedRestart)
	ON_BN_CLICKED(IDC_Exit, &CMonoSLAMView::OnBnClickedExit)
	ON_BN_CLICKED(IDC_CHECK_SHOW_PATH, &CMonoSLAMView::OnBnClickedCheckShowPath)
	ON_BN_CLICKED(IDC_CHECK_SHOW_ODOMETRY, &CMonoSLAMView::OnBnClickedCheckShowOdometry)
	ON_WM_PAINT()
	ON_WM_MOUSEWHEEL()
	ON_WM_MOUSEMOVE()
END_MESSAGE_MAP()

// CMonoSLAMView 构造/析构

CMonoSLAMView::CMonoSLAMView()
	: CFormView(CMonoSLAMView::IDD)
{
	// TODO: 在此处添加构造代码
	m_model = 0;
	m_nSteps = 1;
	isOpenCMD = TRUE;
	m_model = 0;

	isSelectSource = false;
	isSelectOdometry = false;

	//=======保存为视频?????????
	//writer = cvCreateVideoWriter("Video.avi",CV_FOURCC_PROMPT,8.0,cvSize(1100,600),3);
}

CMonoSLAMView::~CMonoSLAMView()
{
}

void CMonoSLAMView::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);

	// Set the default progressive step
	DDX_Text(pDX, IDC_STEP_NUM, m_nSteps);
	DDV_MinMaxInt(pDX, m_nSteps, 1, INT_MAX);

	// Show the real time data
	DDX_Text(pDX, IDC_RAW_FRAME, SLAM.m_frame.index);

	// Show features' information
	DDX_Text(pDX, IDC_FEATURES_NUM, SLAM.m_nMapFeatures);
	DDX_Text(pDX, IDC_PREDICTED_NUM, SLAM.m_nPredicts);
	DDX_Text(pDX, IDC_MATCHED_NUM, SLAM.m_nMatches);

	// Set running model
	DDX_Radio(pDX, IDC_RADIO_DEBUG, SLAM.m_model);
	DDX_Check(pDX, IDC_OPEN_CMD, isOpenCMD);

	// For show complete information
	DDX_Check(pDX, IDC_CHECK_SHOW_PATH, SLAM.isShowRobotPath);
	DDX_Check(pDX, IDC_CHECK_SHOW_ODOMETRY, SLAM.isShowOdoInfo);

	// Show elapsed time for per frame and total
	DDX_Text(pDX, IDC_FRAME_TIME, SLAM.m_frameTime);
	DDX_Text(pDX, IDC_TOTAL_TIME, SLAM.m_totalTime);

	// Set the robot's and features' show windows
	DDX_Control(pDX, IDC_LIST_CTRL, SLAM.m_listCtrl);
	DDX_Control(pDX, IDC_FEATURES_INFO, SLAM.m_showFeatureInfo);
}

BOOL CMonoSLAMView::PreCreateWindow(CREATESTRUCT& cs)
{
	//  CREATESTRUCT cs 来修改窗口类或样式
	return CFormView::PreCreateWindow(cs);
}

void CMonoSLAMView::OnInitialUpdate()
{
	CFormView::OnInitialUpdate();
	GetParentFrame()->RecalcLayout();
	ResizeParentToFit();
	glDisplay.myInit(GetDlgItem(IDC_FEATURE_SHOW));

	OnBnClickedOpenCmd(); // ==========后期可以不需要?????????

	((CButton*)GetDlgItem(IDC_BTN_SOURCE_FILE))->EnableWindow(FALSE);
	((CButton*)GetDlgItem(IDC_BTN_ODOMETRY_FILE))->EnableWindow(FALSE);

	// Add shortcut
	m_hAcc=LoadAccelerators(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDR_ACCELERATOR));

	// ============Setting for list control=========== //
	// Set list constrol's extended style
	// LVS_EX_GRIDLINES: Displays gridlines around items and subitems.
	// LVS_EX_FULLROWSELECT: When an item is selected, the item and all its subitems are highlighted.
	SLAM.m_listCtrl.SetExtendedStyle(LVS_EX_GRIDLINES|LVS_EX_FULLROWSELECT);

	// Initialize the list control
	SLAM.m_listCtrl.InsertColumn(0, "Location", LVCFMT_CENTER);
	SLAM.m_listCtrl.InsertColumn(1, "Index", LVCFMT_CENTER);
	SLAM.m_listCtrl.InsertColumn(2, "X", LVCFMT_CENTER);
	SLAM.m_listCtrl.InsertColumn(3, "Y", LVCFMT_CENTER);
	SLAM.m_listCtrl.InsertColumn(4, "Theta", LVCFMT_CENTER);

	CRect rect;
	SLAM.m_listCtrl.GetClientRect(rect); // Get current client information.

	int nCols = 5;
	int width = rect.Width()/nCols;

	// Change the width of list control
	for (int i = 0; i < nCols; i++)
	{
		SLAM.m_listCtrl.SetColumnWidth(i, width); 
	}

	char theta[50];
	sprintf_s(theta, "%f", SLAM.m_odoTheta.ptr<double>(1)[0]*180.0/3.141593);

	SLAM.m_listCtrl.InsertItem(0, "SLAM:");
	SLAM.m_listCtrl.SetItemText(0, 1, "1");
	SLAM.m_listCtrl.SetItemText(0, 4, theta);

	if (TRUE == SLAM.isShowOdoCtrlList)
	{
		SLAM.m_listCtrl.InsertItem(1, "Odometry:");
		SLAM.m_listCtrl.SetItemText(1, 1, "1");
		SLAM.m_listCtrl.SetItemText(1, 4, theta);
	}

	for (int i = 0; i < 2; i++)
	{
		SLAM.m_listCtrl.SetItemText(0, i+2, "0");
		SLAM.m_listCtrl.SetItemText(1, i+2, "0");
	}
}


// CMonoSLAMView 诊断

#ifdef _DEBUG
void CMonoSLAMView::AssertValid() const
{
	CFormView::AssertValid();
}

void CMonoSLAMView::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}

CMonoSLAMDoc* CMonoSLAMView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CMonoSLAMDoc)));
	return (CMonoSLAMDoc*)m_pDocument;
}
#endif //_DEBUG


// CMonoSLAMView 消息处理程序
void CMonoSLAMView::OnPaint()
{
	GetDlgItem(IDC_PICTURE_SHOW)->GetClientRect(&(SLAM.m_rect));	// 用来显示图片的矩形框
	SLAM.m_pDc = GetDlgItem(IDC_PICTURE_SHOW)->GetDC();			// 用来存显示图片的DC
	SLAM.display2DFeatureModel();

	glDisplay.showModel(&SLAM);
	CFormView::OnPaint();
}

BOOL CMonoSLAMView::PreTranslateMessage(MSG* pMsg)
{
	bool needRedraw = false;

	if (WM_KEYFIRST <= pMsg->message && pMsg->message <= WM_KEYLAST)
	{
		if (m_hAcc && ::TranslateAccelerator(m_hWnd, m_hAcc, pMsg)) 
			return TRUE;

		if ((pMsg->message == WM_KEYDOWN) && (pMsg->lParam & 0x80000000)==0)
		{
			needRedraw = glDisplay.KeyProcessing(pMsg->wParam);

			if (true == needRedraw)	
			{
				GetDlgItem(IDC_PICTURE_SHOW)->GetClientRect(&(SLAM.m_rect));	// 用来显示图片的矩形框
				SLAM.m_pDc = GetDlgItem(IDC_PICTURE_SHOW)->GetDC();			// 用来存显示图片的DC
				glDisplay.showModel(&SLAM);
			}
		}
	}
	return CFormView::PreTranslateMessage(pMsg);
}

BOOL CMonoSLAMView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	if(zDelta>0)	
	{
		glDisplay.zTrans+=0.5;
	}
	else	
	{
		glDisplay.zTrans-=0.5;
	}

	OnPaint();

	return CFormView::OnMouseWheel(nFlags, zDelta, pt);
}

void CMonoSLAMView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CFormView::OnMouseMove(nFlags, point);
}

void CMonoSLAMView::OnBnClickedOpenCmd()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	if (isOpenCMD)
	{
		FILE* stream;
		AllocConsole();                     // 打开控制台资源
		freopen_s(&stream, "CONOUT$","w+t", stdout );// 申请写
		freopen_s(&stream, "CONIN$", "r+t", stdin );  // 申请读
	}
	else
	{
		FreeConsole();                      // 释放控制台资源*/
	}
}

void CMonoSLAMView::OnBnClickedRadioDebug()
{
	((CButton*)GetDlgItem(IDC_BTN_SOURCE_FILE))->EnableWindow(FALSE);
	((CButton*)GetDlgItem(IDC_BTN_ODOMETRY_FILE))->EnableWindow(FALSE);

	SLAM.m_model = SLAM.FLAG_4_DEBUG_MODEL;
}

void CMonoSLAMView::OnBnClickedRadioRelease()
{
	((CButton*)GetDlgItem(IDC_BTN_SOURCE_FILE))->EnableWindow(TRUE);
	((CButton*)GetDlgItem(IDC_BTN_ODOMETRY_FILE))->EnableWindow(TRUE);

	SLAM.m_model = SLAM.FLAG_4_RELEASE_MODEL;
}

void CMonoSLAMView::OnBnClickedSourceFile()
{
	UpdateData(FALSE);

	if (BST_UNCHECKED == IsDlgButtonChecked(IDC_RADIO_RELEASE))
	{
		return;
	}

	SLAM.m_videoDir = "";
	SLAM.m_imageDir = "";

	CString strFilter = _T("所有文件(*.*)|*.*||");

	CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY | 
		OFN_OVERWRITEPROMPT, strFilter);
	dlg.m_ofn.lpstrTitle = _T("Choose video or image sequences!");

	if(dlg.DoModal() == IDOK)
	{
		srcPathName = dlg.GetPathName();
		srcExtName  = dlg.GetFileExt();
	}

	if ("" == srcPathName) //=========这种判断方式有问题?????????
	{
		MessageBox("Please reselect the correct source file!");
		return;
	}

	if ("avi" == srcExtName)
	{
		flag4PlayType = SLAM.FLAG_4_VIDEO;
		SLAM.m_videoDir = srcPathName;
	} 
	else
	{
		flag4PlayType = SLAM.FLAG_4_IMAGE;
		SLAM.m_imageDir = srcPathName;
	}

	getSourceStartIndex();

	isSelectSource = true;
}

void CMonoSLAMView::getSourceStartIndex (void)
{
	if (SLAM.FLAG_4_VIDEO == SLAM.m_playType)
	{
		SLAM.m_frame.start = 1;
		return;
	}

	int index = SLAM.m_imageDir.ReverseFind('\\');
	srcPathName = SLAM.m_imageDir.Left(index-4) + "%04d." + srcExtName;
	SLAM.m_imageDir = srcPathName;
}

void CMonoSLAMView::OnBnClickedOdometryFile()
{
	UpdateData(FALSE);

	if (BST_UNCHECKED == IsDlgButtonChecked(IDC_RADIO_RELEASE))
	{
		MessageBox("Please select release running model first!");
		return;
	}

	if (false == isSelectSource)
	{
		MessageBox("Please choose source file first!");
		return;
	}

	SLAM.m_odometryDir = "";

	CString strFilter = _T("文本文件(*.txt)|*.txt|所有文件(*.*)|*.*||");

	CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY | 
		OFN_OVERWRITEPROMPT, strFilter);
	dlg.m_ofn.lpstrTitle = _T("Choose video or image sequences!");

	if(dlg.DoModal() == IDOK)
	{
		odoPathName = dlg.GetPathName();
	}

	if ("" == odoPathName)
	{
		MessageBox("Please reselect the correct odometry file!");
		return;
	}

	SLAM.m_odometryDir = odoPathName;

	isSelectOdometry = true;

	SLAM.initializeParameters();
	OnPaint();
}

void CMonoSLAMView::OnBnClickedSetParameters()
{
	CSetParameters setDlg;

	// Setting finding keypoints parameters
	setDlg.isUseHarris = SLAM.isUseHarris;
	setDlg.deep = SLAM.m_deep;
	setDlg.blockSize = SLAM.m_blockSize;
	setDlg.qualityLevel = SLAM.m_qualityLevel;
	setDlg.nInitialRaws = SLAM.m_nInitialRaws;
	setDlg.nProcessRaws = SLAM.m_nProcessRaws;
	setDlg.minNUM = SLAM.m_minNUM; 
	setDlg.minDist = SLAM.m_minDist; 

	// Setting threshold
	setDlg.THRESHOLD_MATCH_PATCH = SLAM.THRESHOLD_MATCH_PATCH;
	setDlg.isUseRANSAC = SLAM.isUseRANSAC;
	setDlg.THRESHOLD_RANSAC = SLAM.THRESHOLD_RANSAC;

	// Setting noise value
	setDlg.sigmaMeasure = SLAM.m_sigmaMeasure;
	setDlg.sigmaRHO = SLAM.m_sigmaRHO;
	setDlg.a1 = SLAM.a1;
	setDlg.a2 = SLAM.a2;
	setDlg.a3 = SLAM.a3;
	setDlg.a4 = SLAM.a4;

	// Setting recording path
	setDlg.isRecordRobotInfo = SLAM.isRecordRobotInfo;
	setDlg.isRecordFeaturesInfo = SLAM.isRecordFeaturesInfo;
	setDlg.recordRobotDir = SLAM.m_recordRobotDir;
	setDlg.recordFeaturesDir = SLAM.m_recordFeaturesDir;

	if(setDlg.DoModal() == IDOK)
	{
		// Setting finding keypoints parameters
		SLAM.isUseHarris = setDlg.isUseHarris;
		SLAM.m_deep = setDlg.deep;
		SLAM.m_blockSize = setDlg.blockSize;
		SLAM.m_qualityLevel = setDlg.qualityLevel;
		SLAM.m_nInitialRaws = setDlg.nInitialRaws;
		SLAM.m_nProcessRaws = setDlg.nProcessRaws;
		SLAM.m_minNUM = setDlg.minNUM;
		SLAM.m_minDist = setDlg.minDist;

		// Setting threshold
		SLAM.THRESHOLD_MATCH_PATCH = setDlg.THRESHOLD_MATCH_PATCH;
		SLAM.isUseRANSAC = setDlg.isUseRANSAC;
		SLAM.THRESHOLD_RANSAC = setDlg.THRESHOLD_RANSAC;

		// Setting noise value
		SLAM.m_sigmaMeasure = setDlg.sigmaMeasure;
		SLAM.m_sigmaRHO = setDlg.sigmaRHO;
		SLAM.a1 = setDlg.a1;
		SLAM.a2 = setDlg.a2;
		SLAM.a3 = setDlg.a3;
		SLAM.a4 = setDlg.a4;

		// Setting recording path
		SLAM.isRecordRobotInfo = setDlg.isRecordRobotInfo;
		SLAM.isRecordFeaturesInfo = setDlg.isRecordFeaturesInfo;
		SLAM.m_recordRobotDir = setDlg.recordRobotDir;
		SLAM.m_recordFeaturesDir = setDlg.recordFeaturesDir;
	}	
}

void CMonoSLAMView::OnBnClickedCheckShowPath()
{

}

void CMonoSLAMView::OnBnClickedCheckShowOdometry()
{

}

void CMonoSLAMView::informBeforProcess (void)
{
	if (1 != SLAM.m_frame.counter)
	{
		return;
	}

	// Confirm select the right running model.
	if (-1 == SLAM.m_model)
	{
		MessageBox("Please choose running model!");
		return;
	}

	// If select the release model, src and odometry files should be selected.
	if ((SLAM.FLAG_4_RELEASE_MODEL == SLAM.m_model) && (1 == SLAM.m_frame.counter))
	{
		if (false == isSelectSource)
		{
			MessageBox("Please select source and odometry file first!");
			return;
		}

		bool informFilter = ((SLAM.FLAG_4_VIDEO == SLAM.m_playType) && (SLAM.m_videoDir == "")) || 
			((SLAM.FLAG_4_IMAGE == SLAM.m_playType) && (SLAM.m_imageDir == ""));

		if (true == informFilter)
		{
			MessageBox("Please choose the correct source file!");
			return;
		} 

		informFilter = SLAM.m_odometryDir == "";

		if (true == informFilter)
		{
			MessageBox("Please choose the correct odometry file!");
			return;
		}
	}
}

void CMonoSLAMView::OnBnClickedSLAM()
{
	informBeforProcess();
	m_nSteps = GetDlgItemInt(IDC_STEP_NUM);

	CString tempStr;

	tempStr.Format(_T("Waiting"));
	((CButton*)GetDlgItem(IDC_UKF_SLAM))->SetWindowText(tempStr);

	for (int i=0; i<m_nSteps; i++)
	{
		//clock_t start,finish;
		//start = clock();
		SLAM.SLAM();
		//finish = clock();
		//printf("%f seconds\n\n",(double)(finish-start)/CLOCKS_PER_SEC);
		cout<<SLAM.m_frameTimer.getTimeSec()<<"s"<<endl<<endl;
		UpdateData(FALSE);
		UpdateWindow();
		OnPaint();
	}

	tempStr.Format(_T("UKF_SLAM"));
	GetDlgItem(IDC_UKF_SLAM)->SetWindowText(tempStr);
}

void CMonoSLAMView::OnBnClickedAuto()
{
	informBeforProcess();
	UpdateData();

	m_nSteps = GetDlgItemInt(IDC_STEP_NUM);
	
	CString displayStr;
	displayStr.Format(_T("Waiting"));
	((CButton*)GetDlgItem(IDC_BTN_AUTO))->SetWindowText(displayStr);
	((CEdit*)GetDlgItem(IDC_STEP_NUM))->EnableWindow(FALSE);
	((CButton*)GetDlgItem(IDC_BTN_SLAM))->EnableWindow(FALSE);
	((CButton*)GetDlgItem(IDC_BTN_RESET))->EnableWindow(FALSE);

	UpdateWindow();  // Controls need to be refreshed, otherwise it will not display normally
	
	int counter = 0;

	CString tempStr;
	for (int i = 0; i < INT_MAX; i++)
	{
		if (counter == SLAM.m_odoCounter)
		{
			break;
		}

		clock_t start,finish;
		start = clock();
		SLAM.SLAM();
		finish = clock();
		printf("%f seconds\n\n",(double)(finish-start)/CLOCKS_PER_SEC);
		UpdateData(FALSE);
		OnPaint();
		UpdateWindow();

		counter++;
	}
	
	displayStr.Format(_T("AUTO"));
	((CButton*)GetDlgItem(IDC_BTN_AUTO))->SetWindowText(displayStr);
	((CEdit*)GetDlgItem(IDC_STEP_NUM))->EnableWindow(TRUE);
	((CButton*)GetDlgItem(IDC_BTN_SLAM))->EnableWindow(TRUE);
	((CButton*)GetDlgItem(IDC_BTN_RESET))->EnableWindow(TRUE);
	SetDlgItemInt(IDC_STEP_NUM, m_nSteps); // Make the steps to normal display

	UpdateWindow();
}

void CMonoSLAMView::OnBnClickedRestart()
{
	m_nSteps = 1;
	SLAM.resetAllParameters();
	UpdateData(FALSE);
	isSelectSource = false;
	isSelectOdometry = false;
	OnPaint();
}

void CMonoSLAMView::OnBnClickedExit()
{
	// TODO: 在此添加控件通知处理程序代码
	// =========释放写入视频资源??????????
	//cvReleaseVideoWriter(&writer);

	PostQuitMessage(0);
}

