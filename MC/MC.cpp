// MC.cpp : 定义应用程序的类行为。
//

#include "stdafx.h"
#include "MC.h"
#include "MainFrm.h"
#include <V3d_Viewer.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Graphic3d_GraphicDriver.hxx>

#ifdef _DEBUG
//#define new DEBUG_NEW
#endif


// CMCApp

BEGIN_MESSAGE_MAP(CMCApp, CWinApp)
	ON_COMMAND(ID_APP_ABOUT, &CMCApp::OnAppAbout)
	ON_COMMAND(ID_FILE_OPEN, &CMCApp::OnFileOpen)
END_MESSAGE_MAP()


// CMCApp 构造

CMCApp::CMCApp()
{
	// TODO: 在此处添加构造代码，
	// 将所有重要的初始化放置在 InitInstance 中
	try
	{
		Handle(Aspect_DisplayConnection) displayConnection;
		m_graphicDriver = Graphic3d::InitGraphicDriver(displayConnection);

		TCollection_ExtendedString a3DName("Visu3D");
		m_viewer = new V3d_Viewer(
			m_graphicDriver,
			a3DName.ToExtString(),
			"",
			1000.0,
			V3d_XposYnegZpos,
			Quantity_NOC_GRAY30,
			V3d_ZBUFFER,
			V3d_GOURAUD,
			V3d_WAIT, 
			Standard_True,
			Standard_False);
		m_viewer->SetDefaultLights();
		m_viewer->SetLightOn();

		m_context = new AIS_InteractiveContext(m_viewer);
		m_context->SetDeviationCoefficient(0.0008);
	}
	catch (Standard_Failure)
	{
		AfxMessageBox(_T("初始化图形驱动失败！程序无法运行！"));
		ExitProcess(1);
	}
}


// 唯一的一个 CMCApp 对象

CMCApp theApp;


// CMCApp 初始化

BOOL CMCApp::InitInstance()
{
	// 如果一个运行在 Windows XP 上的应用程序清单指定要
	// 使用 ComCtl32.dll 版本 6 或更高版本来启用可视化方式，
	//则需要 InitCommonControlsEx()。否则，将无法创建窗口。
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// 将它设置为包括所有要在应用程序中使用的
	// 公共控件类。
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();

	// 初始化 OLE 库
	if (!AfxOleInit())
	{
		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}
	AfxEnableControlContainer();
	// 标准初始化
	// 如果未使用这些功能并希望减小
	// 最终可执行文件的大小，则应移除下列
	// 不需要的特定初始化例程
	// 更改用于存储设置的注册表项
	// TODO: 应适当修改该字符串，
	// 例如修改为公司或组织名
	SetRegistryKey(_T("应用程序向导生成的本地应用程序"));
	// 若要创建主窗口，此代码将创建新的框架窗口
	// 对象，然后将其设置为应用程序的主窗口对象
	CMainFrame* pFrame = new CMainFrame;
	if (!pFrame)
		return FALSE;
	m_pMainWnd = pFrame;
	// 创建并加载框架及其资源
	pFrame->LoadFrame(IDR_MAINFRAME,
		WS_OVERLAPPEDWINDOW | FWS_ADDTOTITLE, NULL,
		NULL);






	// 唯一的一个窗口已初始化，因此显示它并对其进行更新
	pFrame->ShowWindow(SW_SHOW);
	pFrame->UpdateWindow();
	// 仅当具有后缀时才调用 DragAcceptFiles
	//  在 SDI 应用程序中，这应在 ProcessShellCommand 之后发生
	return TRUE;
}


// CMCApp 消息处理程序




// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialog
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

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// 用于运行对话框的应用程序命令
void CMCApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}


// CMCApp 消息处理程序


void CMCApp::OnFileOpen()
{
	// TODO: 在此添加命令处理程序代码
	CFileDialog dialog(
		TRUE, // TRUE for FileOpen, FALSE for FileSaveAs
		NULL,
		NULL,
		OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		TEXT("STEP file(*.step;*.stp)|*.step;*.stp|All Files (*.*)|*.*||"),
		m_pMainWnd);

	if (dialog.DoModal() == IDOK) {
		CString filePath = dialog.GetPathName();
		char file_str[1024] = {0};
		WideCharToMultiByte(CP_ACP, 0, filePath.GetString(), filePath.GetLength(), file_str, 1024, NULL, NULL);
	
		SetCursor(this->LoadStandardCursor(IDC_WAIT));

		STEPControl_Reader reader;
		if (reader.ReadFile(file_str) != IFSelect_RetDone) {
			AfxMessageBox(_T("文件打开错误！"));
			SetCursor(this->LoadStandardCursor(IDC_ARROW));
			return;
		}


		Standard_Integer nbRoots = reader.NbRootsForTransfer();
		cout << "Number of roots in STEP file: " << nbRoots << endl;
		Standard_Integer NbTrans = reader.TransferRoots();
		// translates all transferable roots, and returns the number of
		//successful translations
		cout << "STEP roots transferred: " << NbTrans << endl;
		cout << "Number of resulting shapes is: " << reader.NbShapes() << endl;
		m_rootTopoShape = reader.OneShape();
		if (!m_rootTopoShape.IsNull()) {
			m_rootAISShape = new AIS_Shape(m_rootTopoShape);
			m_context->SetColor(m_rootAISShape, Quantity_NOC_RED);
			m_context->SetMaterial(m_rootAISShape, Graphic3d_NOM_GOLD);
			m_context->SetDisplayMode(m_rootAISShape, 1);
			m_context->Display(m_rootAISShape);
			SetCursor(this->LoadStandardCursor(IDC_ARROW));
			//PostMessage(m_pMainWnd, WM_PAINT, 0, 0);
			m_pMainWnd->Invalidate();
		}
		else {
			AfxMessageBox(_T("模型为空！"));
		}

		BRepPrimAPI_MakeWedge w(20, 20, 20, 3);
		TopoDS_Shape ws = w.Solid();
		Handle(AIS_Shape) ais = new AIS_Shape(ws);
		m_context->SetColor(ais, Quantity_NOC_GREEN);
		m_context->Display(ais);

		(((CMainFrame*)m_pMainWnd)->GetView())->Reset();
	}
}
