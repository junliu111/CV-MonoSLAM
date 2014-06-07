// MonoSLAMDoc.cpp : CMonoSLAMDoc 类的实现
//

#include "stdafx.h"
#include "MonoSLAM.h"

#include "MonoSLAMDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMonoSLAMDoc

IMPLEMENT_DYNCREATE(CMonoSLAMDoc, CDocument)

BEGIN_MESSAGE_MAP(CMonoSLAMDoc, CDocument)
END_MESSAGE_MAP()


// CMonoSLAMDoc 构造/析构

CMonoSLAMDoc::CMonoSLAMDoc()
{
	// TODO: 在此添加一次性构造代码

}

CMonoSLAMDoc::~CMonoSLAMDoc()
{
}

BOOL CMonoSLAMDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: 在此添加重新初始化代码
	// (SDI 文档将重用该文档)

	return TRUE;
}




// CMonoSLAMDoc 序列化

void CMonoSLAMDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: 在此添加存储代码
	}
	else
	{
		// TODO: 在此添加加载代码
	}
}


// CMonoSLAMDoc 诊断

#ifdef _DEBUG
void CMonoSLAMDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CMonoSLAMDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CMonoSLAMDoc 命令
