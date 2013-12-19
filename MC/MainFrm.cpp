// MainFrm.cpp : CMainFrame 类的实现
//

#include "stdafx.h"
#include "MC.h"

#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMainFrame

IMPLEMENT_DYNAMIC(CMainFrame, CFrameWnd)

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWnd)
	ON_WM_CREATE()
	ON_WM_SETFOCUS()
	ON_COMMAND(ID_FILE_OPEN, &CMainFrame::OnFileOpen)
END_MESSAGE_MAP()

static UINT indicators[] =
{
	ID_SEPARATOR,           // 状态行指示器
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
};


// CMainFrame 构造/析构

CMainFrame::CMainFrame()
{
	// TODO: 在此添加成员初始化代码
}

CMainFrame::~CMainFrame()
{
}


int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CFrameWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	// 创建一个视图以占用框架的工作区
	if (!m_wndView.Create(NULL, NULL, AFX_WS_DEFAULT_VIEW,
		CRect(0, 0, 0, 0), this, AFX_IDW_PANE_FIRST, NULL))
	{
		TRACE0("未能创建视图窗口\n");
		return -1;
	}
	
	if (!m_wndToolBar.CreateEx(this, TBSTYLE_TRANSPARENT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndToolBar.LoadToolBar(IDR_MAINFRAME))
	{
		TRACE0("未能创建工具栏\n");
		return -1;      // 未能创建
	}

	if (!m_wndStatusBar.Create(this) ||
		!m_wndStatusBar.SetIndicators(indicators,
		  sizeof(indicators)/sizeof(UINT)))
	{
		TRACE0("未能创建状态栏\n");
		return -1;      // 未能创建
	}

	// TODO: 如果不需要可停靠工具栏，则删除这三行
	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndToolBar);

	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWnd::PreCreateWindow(cs) )
		return FALSE;
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	cs.dwExStyle &= ~WS_EX_CLIENTEDGE;
	cs.lpszClass = AfxRegisterWndClass(0);
	return TRUE;
}


// CMainFrame 诊断

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWnd::Dump(dc);
}

#endif //_DEBUG


// CMainFrame 消息处理程序

void CMainFrame::OnSetFocus(CWnd* /*pOldWnd*/)
{
	// 将焦点前移到视图窗口
	m_wndView.SetFocus();
}

BOOL CMainFrame::OnCmdMsg(UINT nID, int nCode, void* pExtra, AFX_CMDHANDLERINFO* pHandlerInfo)
{
	// 让视图第一次尝试该命令
	if (m_wndView.OnCmdMsg(nID, nCode, pExtra, pHandlerInfo))
		return TRUE;

	// 否则，执行默认处理
	return CFrameWnd::OnCmdMsg(nID, nCode, pExtra, pHandlerInfo);
}



void CMainFrame::OnFileOpen()
{
	// TODO: 在此添加命令处理程序代码
	// TODO: 在此添加命令处理程序代码
	CFileDialog dialog(
		TRUE, // TRUE for FileOpen, FALSE for FileSaveAs
		NULL,
		NULL,
		OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		TEXT("STEP file(*.step;*.stp)|*.step;*.stp|All Files (*.*)|*.*||"),
		this);

	if (dialog.DoModal() == IDOK) {
		CString filePath = dialog.GetPathName();
		char file_str[1024] = {0};
		WideCharToMultiByte(CP_ACP, 0, filePath.GetString(), filePath.GetLength(), file_str, 1024, NULL, NULL);

		SetCursor(AfxGetApp()->LoadStandardCursor(IDC_WAIT));

		STEPControl_Reader reader;
		if (reader.ReadFile(file_str) != IFSelect_RetDone) {
			AfxMessageBox(_T("文件打开错误！"));
			SetCursor(AfxGetApp()->LoadStandardCursor(IDC_ARROW));
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
			Handle(AIS_InteractiveContext) m_context = ((CMCApp*)AfxGetApp())->GetAISContext();
			m_context->RemoveAll();

			m_rootAISShape = new AIS_Shape(m_rootTopoShape);
			//m_context->SetColor(m_rootAISShape, Quantity_NOC_SALMON);
			//m_context->SetMaterial(m_rootAISShape, Graphic3d_NOM_METALIZED);
			m_context->SetDisplayMode(m_rootAISShape, true);
			m_context->Display(m_rootAISShape);

			SetCursor(AfxGetApp()->LoadStandardCursor(IDC_ARROW));
		}
		else {
			SetCursor(AfxGetApp()->LoadStandardCursor(IDC_ARROW));
			AfxMessageBox(_T("模型为空！"));
		}

		m_wndView.Reset();
	}
}
