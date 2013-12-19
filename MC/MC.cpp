// MC.cpp : ����Ӧ�ó��������Ϊ��
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


// CMCApp ����

CMCApp::CMCApp()
{
	// TODO: �ڴ˴���ӹ�����룬
	// ��������Ҫ�ĳ�ʼ�������� InitInstance ��
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
		AfxMessageBox(_T("��ʼ��ͼ������ʧ�ܣ������޷����У�"));
		ExitProcess(1);
	}
}


// Ψһ��һ�� CMCApp ����

CMCApp theApp;


// CMCApp ��ʼ��

BOOL CMCApp::InitInstance()
{
	// ���һ�������� Windows XP �ϵ�Ӧ�ó����嵥ָ��Ҫ
	// ʹ�� ComCtl32.dll �汾 6 ����߰汾�����ÿ��ӻ���ʽ��
	//����Ҫ InitCommonControlsEx()�����򣬽��޷��������ڡ�
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// ��������Ϊ��������Ҫ��Ӧ�ó�����ʹ�õ�
	// �����ؼ��ࡣ
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();

	// ��ʼ�� OLE ��
	if (!AfxOleInit())
	{
		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}
	AfxEnableControlContainer();
	// ��׼��ʼ��
	// ���δʹ����Щ���ܲ�ϣ����С
	// ���տ�ִ���ļ��Ĵ�С����Ӧ�Ƴ�����
	// ����Ҫ���ض���ʼ������
	// �������ڴ洢���õ�ע�����
	// TODO: Ӧ�ʵ��޸ĸ��ַ�����
	// �����޸�Ϊ��˾����֯��
	SetRegistryKey(_T("Ӧ�ó��������ɵı���Ӧ�ó���"));
	// ��Ҫ���������ڣ��˴��뽫�����µĿ�ܴ���
	// ����Ȼ��������ΪӦ�ó���������ڶ���
	CMainFrame* pFrame = new CMainFrame;
	if (!pFrame)
		return FALSE;
	m_pMainWnd = pFrame;
	// ���������ؿ�ܼ�����Դ
	pFrame->LoadFrame(IDR_MAINFRAME,
		WS_OVERLAPPEDWINDOW | FWS_ADDTOTITLE, NULL,
		NULL);






	// Ψһ��һ�������ѳ�ʼ���������ʾ����������и���
	pFrame->ShowWindow(SW_SHOW);
	pFrame->UpdateWindow();
	// �������к�׺ʱ�ŵ��� DragAcceptFiles
	//  �� SDI Ӧ�ó����У���Ӧ�� ProcessShellCommand ֮����
	return TRUE;
}


// CMCApp ��Ϣ�������




// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
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

// �������жԻ����Ӧ�ó�������
void CMCApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}


// CMCApp ��Ϣ�������


void CMCApp::OnFileOpen()
{
	// TODO: �ڴ���������������
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
			AfxMessageBox(_T("�ļ��򿪴���"));
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
			AfxMessageBox(_T("ģ��Ϊ�գ�"));
		}

		BRepPrimAPI_MakeWedge w(20, 20, 20, 3);
		TopoDS_Shape ws = w.Solid();
		Handle(AIS_Shape) ais = new AIS_Shape(ws);
		m_context->SetColor(ais, Quantity_NOC_GREEN);
		m_context->Display(ais);

		(((CMainFrame*)m_pMainWnd)->GetView())->Reset();
	}
}
