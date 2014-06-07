#pragma once

#include "afxwin.h"
// CSetParameters 对话框

class CSetParameters : public CDialogEx
{
	DECLARE_DYNAMIC(CSetParameters)

// 操作
public:
	// For finding keypoints
	BOOL isUseHarris;
	double deep;
	int blockSize;
	double qualityLevel;
	int nInitialRaws;
	int nProcessRaws;
	int minNUM;
	double minDist;

	// For setting threshold
	double THRESHOLD_MATCH_PATCH;
	BOOL isUseRANSAC;
	double THRESHOLD_RANSAC;

	// For setting noise value
	double sigmaMeasure;
	double sigmaRHO;
	double a1;
	double a2;
	double a3;
	double a4;

	// For setting recording path
	BOOL isRecordRobotInfo;
	BOOL isRecordFeaturesInfo;
	CString recordRobotDir;
	CString recordFeaturesDir;

public:
	CSetParameters(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CSetParameters();

// 对话框数据
	enum { IDD = IDD_PARAMETER };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	virtual BOOL OnInitDialog(); // 构造后第一次调用

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedCheckHarris();
	afx_msg void OnBnClickedCheckRansac();
	afx_msg void OnBnClickedCheckRobot();
	afx_msg void OnBnClickedCheckFeatures();
	afx_msg void OnBnClickedBtnRobot();
	afx_msg void OnBnClickedBtnFeatures();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();

	void initialDisplay (void);
};
