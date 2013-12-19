// MC.h : MC 应用程序的主头文件
//
#pragma once

#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"       // 主符号
#include "../ModelComp/required.h"

// CMCApp:
// 有关此类的实现，请参阅 MC.cpp
//

class CMCApp : public CWinApp
{
public:
	CMCApp();


// 重写
public:
	virtual BOOL InitInstance();

// 实现
	Handle(V3d_Viewer) GetViewer() { return m_viewer; }
	Handle(AIS_InteractiveContext) GetAISContext() { return m_context; }

public:
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
	afx_msg void OnFileOpen();

private:
	Handle(AIS_InteractiveContext)		m_context;
	Handle(V3d_Viewer)					m_viewer;
	Handle(Graphic3d_GraphicDriver)		m_graphicDriver;
};

extern CMCApp theApp;