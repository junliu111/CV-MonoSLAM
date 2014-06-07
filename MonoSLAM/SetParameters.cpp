/** @file
* @author Jun Liu
* @date 2014-1-1
* @version v1.0.0
*/
/*=========================================================================
*
* This part is used for parameters setting in MFC dialog.
*
*=========================================================================*/

#include "stdafx.h"
#include "MonoSLAM.h"
#include "SetParameters.h"
#include "afxdialogex.h"


// CSetParameters 对话框

IMPLEMENT_DYNAMIC(CSetParameters, CDialogEx)

CSetParameters::CSetParameters(CWnd* pParent /*=NULL*/)
	: CDialogEx(CSetParameters::IDD, pParent)
{

}

CSetParameters::~CSetParameters()
{
}

void CSetParameters::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);

	// Setting finding KeyPoints parameters
	DDX_Text(pDX, IDC_DEEP, deep);
	DDX_Text(pDX, IDC_BLOCK_SIZE, blockSize);
	DDX_Text(pDX, IDC_INIT_NUM, nInitialRaws);
	DDX_Text(pDX, IDC_MIN_NUM, minNUM);
	DDX_Text(pDX, IDC_QUALITY_LEVEL, qualityLevel);
	DDX_Text(pDX, IDC_PROCESS_NUM, nProcessRaws);
	DDX_Text(pDX, IDC_MIN_DIST, minDist);
	DDX_Check(pDX, IDC_CHECK_RANSAC, isUseRANSAC);

	// Setting threshold
	DDX_Text(pDX, IDC_THRESHOLD_MATCHING, THRESHOLD_MATCH_PATCH);
	DDX_Text(pDX, IDC_THRESHOLD_RANSAC, THRESHOLD_RANSAC);

	// Setting noise value
	DDX_Text(pDX, IDC_SIGMA_MEASURE, sigmaMeasure);
	DDX_Text(pDX, IDC_SIGMA_RHO, sigmaRHO);
	DDX_Text(pDX, IDC_ODO_A1, a1);
	DDX_Text(pDX, IDC_ODO_A2, a2);
	DDX_Text(pDX, IDC_ODO_A3, a3);
	DDX_Text(pDX, IDC_ODO_A4, a4);

	// Setting recording output path
	DDX_Check(pDX, IDC_CHECK_ROBOT, isRecordRobotInfo);
	DDX_Check(pDX, IDC_CHECK_FEATURES, isRecordFeaturesInfo);
	DDX_Text(pDX, IDC_ROBOT_PATH, recordRobotDir);
	DDX_Text(pDX, IDC_FEATURES_PATH, recordFeaturesDir);
}

BEGIN_MESSAGE_MAP(CSetParameters, CDialogEx)
	ON_BN_CLICKED(IDC_BTN_OK, &CSetParameters::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BTN_ROBOT, &CSetParameters::OnBnClickedBtnRobot)
	ON_BN_CLICKED(IDC_BTN_FEATURES, &CSetParameters::OnBnClickedBtnFeatures)
	ON_BN_CLICKED(IDC_CHECK_RANSAC, &CSetParameters::OnBnClickedCheckRansac)
	ON_BN_CLICKED(IDC_CHECK_ROBOT, &CSetParameters::OnBnClickedCheckRobot)
	ON_BN_CLICKED(IDC_CHECK_FEATURES, &CSetParameters::OnBnClickedCheckFeatures)
	ON_BN_CLICKED(IDC_CHECK_HARRIS, &CSetParameters::OnBnClickedCheckHarris)
	ON_BN_CLICKED(IDC_BTN_CANCEL, &CSetParameters::OnBnClickedCancel)
END_MESSAGE_MAP()

BOOL CSetParameters::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	initialDisplay();

	return TRUE;
}

// CSetParameters 消息处理程序
void CSetParameters::initialDisplay (void)
{
	// For Harris display
	if (BST_CHECKED == IsDlgButtonChecked(IDC_CHECK_HARRIS))
	{
		((CEdit*)GetDlgItem(IDC_HARRIS))->SetWindowText("TRUE");
	} 
	else if (BST_UNCHECKED == IsDlgButtonChecked(IDC_CHECK_HARRIS))
	{
		((CEdit*)GetDlgItem(IDC_HARRIS))->SetWindowText("FALSE");
	}

	// For RANSAC display
	if (BST_CHECKED == IsDlgButtonChecked(IDC_CHECK_RANSAC))
	{
		((CEdit*)GetDlgItem(IDC_THRESHOLD_RANSAC))->EnableWindow(TRUE);
	} 
	else if (BST_UNCHECKED == IsDlgButtonChecked(IDC_CHECK_RANSAC))
	{
		((CEdit*)GetDlgItem(IDC_THRESHOLD_RANSAC))->EnableWindow(FALSE);
	}

	// For robot's information
	if (TRUE == isRecordRobotInfo)
	{
		((CButton*)GetDlgItem(IDC_CHECK_ROBOT))->SetCheck(TRUE);
		((CEdit*)GetDlgItem(IDC_ROBOT_PATH))->EnableWindow(TRUE);
		((CButton*)GetDlgItem(IDC_BTN_ROBOT))->EnableWindow(TRUE);
		
	} 
	else if (FALSE == isRecordRobotInfo)
	{
		((CButton*)GetDlgItem(IDC_CHECK_ROBOT))->SetCheck(FALSE);
		((CEdit*)GetDlgItem(IDC_ROBOT_PATH))->EnableWindow(FALSE);
		((CButton*)GetDlgItem(IDC_BTN_ROBOT))->EnableWindow(FALSE);
	}

	// For features' information
	if (TRUE == isRecordFeaturesInfo)
	{
		((CButton*)GetDlgItem(IDC_CHECK_FEATURES))->SetCheck(TRUE);
		((CEdit*)GetDlgItem(IDC_FEATURES_PATH))->EnableWindow(TRUE);
		((CButton*)GetDlgItem(IDC_BTN_FEATURES))->EnableWindow(TRUE);
	} 
	else
	{
		((CButton*)GetDlgItem(IDC_CHECK_FEATURES))->SetCheck(FALSE);
		((CEdit*)GetDlgItem(IDC_FEATURES_PATH))->EnableWindow(FALSE);
		((CButton*)GetDlgItem(IDC_BTN_FEATURES))->EnableWindow(FALSE);
	}
}

void CSetParameters::OnBnClickedCheckHarris()
{
	if (BST_CHECKED == ((CButton*)GetDlgItem(IDC_CHECK_HARRIS))->GetCheck())
	{
		((CEdit*)GetDlgItem(IDC_HARRIS))->SetWindowText("TRUE");
		isUseHarris = TRUE;
		((CButton*)GetDlgItem(IDC_CHECK_HARRIS))->SetCheck(TRUE);
	}
	else if (BST_UNCHECKED == ((CButton*)GetDlgItem(IDC_CHECK_HARRIS))->GetCheck())
	{
		GetDlgItem(IDC_HARRIS)->SetWindowText("FALSE");
		isUseHarris = FALSE;
		((CButton*)GetDlgItem(IDC_CHECK_HARRIS))->SetCheck(FALSE);
	}
}

void CSetParameters::OnBnClickedCheckRansac()
{
	if (BST_CHECKED == ((CButton*)GetDlgItem(IDC_CHECK_RANSAC))->GetCheck())
	{
		GetDlgItem(IDC_THRESHOLD_RANSAC)->EnableWindow(TRUE);
		isUseRANSAC = TRUE;
		((CButton*)GetDlgItem(IDC_CHECK_RANSAC))->SetCheck(TRUE);
	}
	else if (BST_UNCHECKED == ((CButton*)GetDlgItem(IDC_CHECK_RANSAC))->GetCheck())
	{
		GetDlgItem(IDC_THRESHOLD_RANSAC)->EnableWindow(FALSE);
		isUseRANSAC = FALSE;
		((CButton*)GetDlgItem(IDC_CHECK_RANSAC))->SetCheck(FALSE);
	}
}

void CSetParameters::OnBnClickedCheckRobot()
{
	if (BST_CHECKED == IsDlgButtonChecked(IDC_CHECK_ROBOT))
	{
		((CEdit*)GetDlgItem(IDC_ROBOT_PATH))->EnableWindow(TRUE);
		((CButton*)GetDlgItem(IDC_BTN_ROBOT))->EnableWindow(TRUE);
		isRecordRobotInfo = TRUE;
		((CButton*)GetDlgItem(IDC_CHECK_ROBOT))->SetCheck(TRUE);
	}
	else if (BST_UNCHECKED == IsDlgButtonChecked(IDC_CHECK_ROBOT))
	{
		((CEdit*)GetDlgItem(IDC_ROBOT_PATH))->EnableWindow(FALSE);
		((CButton*)GetDlgItem(IDC_BTN_ROBOT))->EnableWindow(FALSE);
		isRecordRobotInfo = FALSE;
		((CButton*)GetDlgItem(IDC_CHECK_ROBOT))->SetCheck(FALSE);
	}
}

void CSetParameters::OnBnClickedCheckFeatures()
{
	if (BST_CHECKED == IsDlgButtonChecked(IDC_CHECK_FEATURES))
	{
		GetDlgItem(IDC_FEATURES_PATH)->EnableWindow(TRUE);
		GetDlgItem(IDC_BTN_FEATURES)->EnableWindow(TRUE);
		isRecordFeaturesInfo = TRUE;
		((CButton*)GetDlgItem(IDC_CHECK_FEATURES))->SetCheck(TRUE);
	}
	else if (BST_UNCHECKED == IsDlgButtonChecked(IDC_CHECK_FEATURES))
	{
		GetDlgItem(IDC_FEATURES_PATH)->EnableWindow(FALSE);
		GetDlgItem(IDC_BTN_FEATURES)->EnableWindow(FALSE);
		isRecordFeaturesInfo = FALSE;
		((CButton*)GetDlgItem(IDC_CHECK_FEATURES))->SetCheck(FALSE);
	}
}

void CSetParameters::OnBnClickedBtnRobot()
{
	CString strFilter = _T("文本文件(*.txt)|*.txt|所有文件(*.*)|*.*||");

	CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY | 
		OFN_OVERWRITEPROMPT, strFilter, this);
	dlg.m_ofn.lpstrTitle = _T("Choose file to store Robot-Info");

	if(dlg.DoModal() == IDOK)
	{
		recordRobotDir = dlg.GetPathName();
	}

	((CEdit*)GetDlgItem(IDC_ROBOT_PATH))->SetWindowText(recordRobotDir);
}

void CSetParameters::OnBnClickedBtnFeatures()
{
	CString strFilter = _T("文本文件(*.txt)|*.txt|所有文件(*.*)|*.*||");

	CFileDialog dlg(TRUE, NULL, NULL, OFN_HIDEREADONLY | 
		OFN_OVERWRITEPROMPT, strFilter, this);
	dlg.m_ofn.lpstrTitle = _T("Choose file to store Features-Info");

	if(dlg.DoModal() == IDOK)
	{
		recordFeaturesDir = dlg.GetPathName();
	}

	((CEdit*)GetDlgItem(IDC_FEATURES_PATH))->SetWindowText(recordFeaturesDir);
}

void CSetParameters::OnBnClickedOk()
{
	CDialogEx::OnOK();
}

void CSetParameters::OnBnClickedCancel()
{
	CDialogEx::OnCancel();
}


