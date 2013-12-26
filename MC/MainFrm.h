// MainFrm.h : CMainFrame 类的接口
//


#pragma once

#include "ChildView.h"
#include "../ModelComp/required.h"

class CMainFrame : public CFrameWnd
{
	
public:
	CMainFrame();
protected: 
	DECLARE_DYNAMIC(CMainFrame)

// 属性
public:

// 操作
public:

// 重写
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	virtual BOOL OnCmdMsg(UINT nID, int nCode, void* pExtra, AFX_CMDHANDLERINFO* pHandlerInfo);

// 实现
public:
	virtual ~CMainFrame();

#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

private:
	void SetStatus(LPCTSTR text) { m_wndStatusBar.SetPaneText(0, text); }
	void ResetStatus() { m_wndStatusBar.SetPaneText(0, _T("就绪")); }

protected:  // 控件条嵌入成员
	CStatusBar  m_wndStatusBar;
	CToolBar    m_wndToolBar;
	CChildView    m_wndView;
	CProgressCtrl m_progressCtrl;

	Handle(AIS_Shape)					m_rootAISShape;
	TopoDS_Shape						m_rootTopoShape;

// 生成的消息映射函数
protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSetFocus(CWnd *pOldWnd);
	afx_msg void OnFileOpen();

	DECLARE_MESSAGE_MAP()
};


