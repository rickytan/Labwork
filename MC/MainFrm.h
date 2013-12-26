// MainFrm.h : CMainFrame ��Ľӿ�
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

// ����
public:

// ����
public:

// ��д
public:
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	virtual BOOL OnCmdMsg(UINT nID, int nCode, void* pExtra, AFX_CMDHANDLERINFO* pHandlerInfo);

// ʵ��
public:
	virtual ~CMainFrame();

#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

private:
	void SetStatus(LPCTSTR text) { m_wndStatusBar.SetPaneText(0, text); }
	void ResetStatus() { m_wndStatusBar.SetPaneText(0, _T("����")); }

protected:  // �ؼ���Ƕ���Ա
	CStatusBar  m_wndStatusBar;
	CToolBar    m_wndToolBar;
	CChildView    m_wndView;
	CProgressCtrl m_progressCtrl;

	Handle(AIS_Shape)					m_rootAISShape;
	TopoDS_Shape						m_rootTopoShape;

// ���ɵ���Ϣӳ�亯��
protected:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSetFocus(CWnd *pOldWnd);
	afx_msg void OnFileOpen();

	DECLARE_MESSAGE_MAP()
};


