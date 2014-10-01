
// OfflineFutionDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "OfflineFution.h"
#include "OfflineFutionDlg.h"
#include "afxdialogex.h"
#include "Helper.h"

#include <iostream>
#include <ostream>

#include <NuiApi.h>
#include <NuiKinectFusionApi.h>
#include <NuiKinectFusionCameraPoseFinder.h>
#include <NuiKinectFusionDepthProcessor.h>
#include <NuiSensor.h>
#include <NuiSensorChooser.h>



#ifdef _DEBUG
#define new DEBUG_NEW
#endif

enum 
{
    WM_FRAMEREADY               = WM_USER,
    WM_UPDATE_SENSOR_STATUS,
    WM_UPDATE_NEAR_MODE
};

#define ON_WM_FRAMEREADY() \
{ WM_FRAMEREADY, 0, 0, 0, AfxSig_vv, \
    (AFX_PMSG)(AFX_PMSGW) \
    (static_cast<void (AFX_MSG_CALL CWnd::*)(void)> (&ThisClass::OnFrameReady)) },

static DWORD cStatusTimeoutInMilliseconds = 5000;

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// COfflineFutionDlg 对话框



COfflineFutionDlg::COfflineFutionDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(COfflineFutionDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

    m_pDepthViewRenderer = new ImageRenderer();
    m_pDrawTrackingResiduals = new ImageRenderer();
}

COfflineFutionDlg::~COfflineFutionDlg()
{
    SAFE_DELETE(m_pSensorChooserUI);
    SAFE_DELETE(m_pDepthViewRenderer);
    SAFE_DELETE(m_pDrawTrackingResiduals);
}

void COfflineFutionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_SLIDER_MIN, m_SliderMin);
    DDX_Control(pDX, IDC_SLIDER_MAX, m_SliderMax);
}

BEGIN_MESSAGE_MAP(COfflineFutionDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
    ON_WM_ACTIVATEAPP()
    ON_WM_FRAMEREADY()
    ON_WM_HSCROLL()
END_MESSAGE_MAP()


// COfflineFutionDlg 消息处理程序

BOOL COfflineFutionDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

    SetStatusMessage(L"Initializing...");

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO:  在此添加额外的初始化代码
    InitializeUIControls();

    D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2D1Factory);

    int width = m_params.m_cDepthWidth;
    int height = m_params.m_cDepthHeight;

    HRESULT hr = m_pDepthViewRenderer->Initialize(GetDlgItem(IDC_RECONSTRUCTION_VIEW)->m_hWnd,
        m_pD2D1Factory,
        width,
        height, 
        width * sizeof(ULONG));
    if (FAILED(hr)) {
        SetStatusMessage(L"Failed to initialize the depth View.");
        m_bInitializeError = TRUE;
    }

    hr = m_pDrawTrackingResiduals->Initialize(GetDlgItem(IDC_TRACKING_VIEW)->m_hWnd,
        m_pD2D1Factory,
        width,
        height,
        width * sizeof(ULONG));

    if (FAILED(hr)) {
        SetStatusMessage(L"Failed to initialize the tracking View.");
        m_bInitializeError = TRUE;
    }

    if (FAILED(m_processor.SetWindow(m_hWnd, WM_FRAMEREADY, WM_UPDATE_SENSOR_STATUS, WM_UPDATE_NEAR_MODE)) ||
        FAILED(m_processor.SetParams(m_params)) ||
        FAILED(m_processor.StartProcessing()))
    {
        m_bInitializeError = true;
    }

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void COfflineFutionDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void COfflineFutionDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;
        
		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR COfflineFutionDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void COfflineFutionDlg::OnActivateApp(BOOL bActive, DWORD dwThreadID)
{
    
}

void COfflineFutionDlg::OnDestroy()
{
    m_processor.StopProcessing();

}

void COfflineFutionDlg::OnOK()
{
}

void COfflineFutionDlg::OnCancel()
{
    CDialogEx::OnCancel();
}


void COfflineFutionDlg::InitializeUIControls()
{
    RECT rc;
    GetClientRect(&rc);

    POINT ptCenterTop;
    ptCenterTop.x = (rc.right - rc.left) / 2;
    ptCenterTop.y = 0;

    // Create the sensor chooser UI control to show sensor status
    m_pSensorChooserUI = new NuiSensorChooserUI(m_hWnd, IDC_SENSORCHOOSER, ptCenterTop);
    m_pSensorChooserUI->UpdateSensorStatus(NuiSensorChooserStatusInitializing);

    m_SliderMin.SetRange(350, 8000, TRUE);
    m_SliderMin.SetPos(350);
    m_SliderMax.SetRange(350, 8000, TRUE);
    m_SliderMax.SetPos(3000);
}
void COfflineFutionDlg::SetStatusMessage(LPCTSTR szText)
{
    size_t length = 0;
    if (FAILED(StringCchLength(
        szText,
        KinectFusionProcessorFrame::StatusMessageMaxLen,
        &length)))
    {
        length = 0;
    }

    if (length > 0)
    {
        SendDlgItemMessage(IDC_STATUS_BAR, WM_SETTEXT, 0, (LPARAM)szText);
        m_tickLastStatus = GetTickCount();
    }
    else {
        // Clear the status message after a timeout (as long as frames are flowing)
        if (GetTickCount() - m_tickLastStatus > cStatusTimeoutInMilliseconds &&
            m_fFramesPerSecond > 0)
        {
            SendDlgItemMessage(IDC_STATUS_BAR, WM_SETTEXT, 0, 0);
            m_tickLastStatus = GetTickCount();
        }
    }
}

void COfflineFutionDlg::OnFrameReady()
{
    KinectFusionProcessorFrame const* pFrame = nullptr;

    MSG msg;
    while (PeekMessage(&msg, m_hWnd, WM_FRAMEREADY, WM_FRAMEREADY, PM_REMOVE)) {}

    const int FRAME_SKIP_COUNT = 30;
    static int frameCounter = FRAME_SKIP_COUNT;

    m_processor.LockFrame(&pFrame);
    if (m_processor.IsVolumeInitialized())
    {
        if (m_processor.IsCameraPoseFinderAvailable()) {
            Matrix4 trans = m_processor.GetWorldToCameraTransform();
            Eigen::Matrix4f mat = Helper::convertToEigenMatrix(trans);
            //mat(3, 2) = -mat(3, 2);
            std::stringstream ss;
            ss << mat.block<3, 1>(0, 3).transpose() << std::endl;
            std::string s = ss.str();
            std::wstring ws;
            ws.assign(s.begin(), s.end());
            SetStatusMessage(ws.c_str());
            if (frameCounter++ > FRAME_SKIP_COUNT) {
                NUI_FUSION_IMAGE_FRAME * depthFloat = m_processor.GetDepthFloatImage();
                PCloud cloud = Helper::depthFloatToPointCloud(depthFloat);
                Helper::saveSequenceTo("./data", cloud, mat);
                frameCounter = 0;
            }
        }
        m_pDepthViewRenderer->Draw(pFrame->m_pDepthRGBX, pFrame->m_cbImageSize);
        //m_pDrawReconstruction->Draw(pFrame->m_pReconstructionRGBX, pFrame->m_cbImageSize);
        m_pDrawTrackingResiduals->Draw(pFrame->m_pTrackingDataRGBX, pFrame->m_cbImageSize);
        //SetStatusMessage(pFrame->m_statusMessage);
    }
    m_processor.UnlockFrame();
}

void COfflineFutionDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
    m_params.m_fMinDepthThreshold = 0.001f * m_SliderMin.GetPos();
    m_params.m_fMaxDepthThreshold = 0.001f * m_SliderMax.GetPos();
    
    // update text
    WCHAR str[MAX_PATH];
    swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMinDepthThreshold);
    SetDlgItemText(IDC_MIN_DEPTH_DISTANCE, str);
    swprintf_s(str, ARRAYSIZE(str), L"%4.2fm", m_params.m_fMaxDepthThreshold);
    SetDlgItemText(IDC_MAX_DEPTH_DISTANCE, str);


    m_processor.SetParams(m_params);
}

