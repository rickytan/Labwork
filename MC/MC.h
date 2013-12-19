// MC.h : MC Ӧ�ó������ͷ�ļ�
//
#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"       // ������
#include "../ModelComp/required.h"

// CMCApp:
// �йش����ʵ�֣������ MC.cpp
//

class CMCApp : public CWinApp
{
public:
	CMCApp();


// ��д
public:
	virtual BOOL InitInstance();

// ʵ��
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