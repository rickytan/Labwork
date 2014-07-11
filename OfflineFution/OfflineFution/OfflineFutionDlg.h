
// OfflineFutionDlg.h : ͷ�ļ�
//

#pragma once

#include "ImageRenderer.h"
#include "NuiSensorChooserUI.h"
#include "KinectFusionParams.h"
#include "KinectFusionProcessor.h"

// COfflineFutionDlg �Ի���
class COfflineFutionDlg : public CDialogEx
{
// ����
public:
	COfflineFutionDlg(CWnd* pParent = NULL);	// ��׼���캯��
    ~COfflineFutionDlg();

// �Ի�������
	enum { IDD = IDD_OFFLINEFUTION_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��
    void SetStatusMessage(LPCTSTR szText);


// ʵ��
protected:
	HICON m_hIcon;

    void InitializeUIControls();

	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
    virtual void OnOK();
    virtual void OnCancel();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
    afx_msg void OnDestroy();
	afx_msg HCURSOR OnQueryDragIcon();
    afx_msg void OnActivateApp(BOOL bActive, DWORD dwThreadID);
    afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
    afx_msg void OnFrameReady();
    afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
    DECLARE_MESSAGE_MAP()
private:
    CSliderCtrl m_SliderMin, m_SliderMax;
    CStatusBar m_StatusBar;

    NuiSensorChooserUI * m_pSensorChooserUI;
    KinectFusionProcessor m_processor;
    KinectFusionParams m_params;
    ID2D1Factory * m_pD2D1Factory;
    ImageRenderer * m_pDepthViewRenderer;
    DWORD m_tickLastStatus;
    DWORD m_fFramesPerSecond;
    BOOL m_bInitializeError;
public:
    afx_msg void OnTRBNThumbPosChangingSliderMin(NMHDR *pNMHDR, LRESULT *pResult);
};
