// MonoSLAMView.h : CMonoSLAMView 类的接口
//


#pragma once

#include "SLAM.h"
#include "SetParameters.h"
#include "OpenGlDisplay.h"
#include "afxcmn.h"
#include "afxwin.h"

class CMonoSLAMView : public CFormView
{
protected: // 仅从序列化创建
	CMonoSLAMView();
	DECLARE_DYNCREATE(CMonoSLAMView)

public:
	enum{ IDD = IDD_MONOSLAM_FORM };

// 属性
public:
	CMonoSLAMDoc* GetDocument() const;

// 操作
public:
	HACCEL m_hAcc;

	bool isSelectSource;
	bool isSelectOdometry;
	BOOL isOpenCMD;	// 是否打开CMD

	int m_model;
	int m_nSteps;		// 一次性执行多少个循环
	int flag4PlayType;

	CString srcPathName;
	CString srcExtName;
	CString odoPathName;

	COpenGlDisplay glDisplay;
	CvVideoWriter *writer;
	CSLAM SLAM;

// 重写
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	virtual void OnInitialUpdate(); // 构造后第一次调用

// 实现
public:
	virtual ~CMonoSLAMView(); 
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()
public:
	BOOL PreTranslateMessage(MSG* pMsg);// Capture keyboard information
	afx_msg void OnPaint();
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnBnClickedOpenCmd();
	afx_msg void OnBnClickedSourceFile();
	afx_msg void OnBnClickedOdometryFile();
	afx_msg void OnBnClickedRadioDebug();
	afx_msg void OnBnClickedRadioRelease();
	afx_msg void OnBnClickedSetParameters();
	afx_msg void OnBnClickedSLAM();
	afx_msg void OnBnClickedAuto();
	afx_msg void OnBnClickedRestart();
	afx_msg void OnBnClickedExit();
	afx_msg void OnBnClickedCheckShowPath();
	afx_msg void OnBnClickedCheckShowOdometry();
	void getSourceStartIndex (void);
	void informBeforProcess (void);
};

#ifndef _DEBUG  // MonoSLAMView.cpp 中的调试版本
inline CMonoSLAMDoc* CMonoSLAMView::GetDocument() const
   { return reinterpret_cast<CMonoSLAMDoc*>(m_pDocument); }
#endif

