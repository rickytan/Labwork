
// OfflineFution.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// COfflineFutionApp: 
// �йش����ʵ�֣������ OfflineFution.cpp
//

class COfflineFutionApp : public CWinApp
{
public:
	COfflineFutionApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
    virtual int ExitInstance();
};

extern COfflineFutionApp theApp;