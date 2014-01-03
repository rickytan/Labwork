// ChildView.cpp : CChildView 类的实现
//

#include "stdafx.h"
#include "MC.h"
#include "ChildView.h"
#include <V3d_View.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <WNT_Window.hxx>

#ifdef _DEBUG
//#define new DEBUG_NEW
#endif

IMPLEMENT_DYNAMIC(CChildView, CWnd);

BEGIN_MESSAGE_MAP(CChildView, CWnd)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_LBUTTONDBLCLK()
	ON_WM_MBUTTONDOWN()
	ON_WM_MBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_MOUSEWHEEL()
	ON_WM_MOUSEHOVER()
	ON_WM_MOUSEMOVE()
	ON_WM_CONTEXTMENU()
	ON_WM_KEYDOWN()
	ON_WM_SIZE()
	ON_COMMAND(ID_RESET, &CChildView::OnReset)
	ON_COMMAND(ID_DRAW_SOLID, &CChildView::OnDrawSolid)
	ON_COMMAND(ID_DRAW_WIRE, &CChildView::OnDrawWire)
	ON_COMMAND(ID_BACKGROUND_COLOR, &CChildView::OnBackgroundColor)
	ON_COMMAND(ID_DRAW_BOUNDING_BOX, &CChildView::OnDrawBoundingBox)
END_MESSAGE_MAP()

CChildView::CChildView()
: m_winWidth(0)
, m_winHeight(0)
, m_shouldRotate(FALSE)
, m_mouseMoved(FALSE)
, m_mouse(0, 0)
{

}

CChildView::~CChildView()
{
}

void CChildView::Reset()
{
	m_view->Reset();

	m_view->SetProj(V3d_Zpos);
	if (!m_view.IsNull())
		m_view->FitAll();
	m_view->ZFitAll();
}

void CChildView::ShowGrid(Standard_Boolean show)
{
	Handle(V3d_Viewer) aViewer = m_view->Viewer();
	if (show) {
		Handle(Graphic3d_AspectMarker3d) aGridAspect = new Graphic3d_AspectMarker3d(Aspect_TOM_BALL,Quantity_NOC_WHITE,4);
		aViewer->SetGridEcho(aGridAspect);
		Standard_Integer aWidth=0, aHeight=0, anOffset=0;
		m_view->Window()->Size(aWidth,aHeight);
		aViewer->SetRectangularGridGraphicValues(aWidth,aHeight,anOffset);
		aViewer->ActivateGrid(Aspect_GT_Rectangular, Aspect_GDM_Lines);
	}
	else
	{
		aViewer->DeactivateGrid();
		aViewer->Update();
	}
}

// CChildView 消息处理程序

BOOL CChildView::PreCreateWindow(CREATESTRUCT& cs) 
{
	if (!CWnd::PreCreateWindow(cs))
		return FALSE;

	cs.dwExStyle |= WS_EX_CLIENTEDGE;
	cs.style &= ~WS_BORDER;
	cs.lpszClass = AfxRegisterWndClass(CS_HREDRAW|CS_VREDRAW|CS_DBLCLKS, 
		::LoadCursor(NULL, IDC_ARROW), reinterpret_cast<HBRUSH>(COLOR_WINDOW+1), NULL);

	return TRUE;
}

int CChildView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CWnd::OnCreate(lpCreateStruct) == -1)
		return -1;

	Handle(V3d_Viewer) aViewer = ((CMCApp*)AfxGetApp())->GetViewer();
	aViewer->DefaultPerspectiveView();
	aViewer->SetDefaultTypeOfView (V3d_PERSPECTIVE);
	//aViewer->SetDefaultBackgroundColor(Quantity_NOC_GRAY);
	aViewer->SetDefaultBgGradientColors(Quantity_NOC_GRAY30, Quantity_NOC_BLACK);
	m_view = aViewer->CreateView();
	m_view->SetAntialiasingOn();

	HWND win = GetSafeHwnd();
	Handle(WNT_Window) aWNTWindow = new WNT_Window(win);
	m_view->SetWindow(aWNTWindow);
	if (!aWNTWindow->IsMapped())
		aWNTWindow->Map();

	Handle(AIS_InteractiveContext) m_context = ((CMCApp*)AfxGetApp())->GetAISContext();
	m_context->OpenLocalContext();
	//m_context->ActivateStandardMode(TopAbs_FACE);

	ShowGrid();
	Reset();
	return 0;
}

void CChildView::OnMButtonDown(UINT nFlags, CPoint point)
{
	m_mouseMoved = FALSE;
	V3d_View *m_view;
	AIS_InteractiveContext *m_context;

}

void CChildView::OnMButtonUp(UINT nFlags, CPoint point)
{

}

void CChildView::OnLButtonUp(UINT nFlags, CPoint point)
{
	//V3d_View *m_view;
	if (!m_mouseMoved) {
		AIS_InteractiveContext *context = NULL;
		if (!((CMCApp*)AfxGetApp())->GetAISContext().IsNull())
			context = (AIS_InteractiveContext *)((CMCApp*)AfxGetApp())->GetAISContext().Access();
		context->MoveTo(point.x, point.y, m_view);
		if (nFlags & MK_SHIFT)
			context->ShiftSelect();
		else
			context->Select();

		
		
		context->InitSelected();
		if (context->MoreSelected()) {
			Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(context->SelectedInteractive());
			//TopoDS_Solid solid = TopoDS::Solid(context->SelectedShape());
			//context->OpenLocalContext();
			//context->Activate(context->Current(), 4);
		}
		
	}
	m_shouldRotate = FALSE;
}

void CChildView::OnLButtonDown(UINT nFlags, CPoint point)
{
	//V3d_View *m_view;
	m_shouldRotate = TRUE;
	m_mouseMoved = FALSE;
	m_view->StartRotation(point.x, point.y);
}

void CChildView::OnLButtonDblClk(UINT nFlags, CPoint point)
{

}

void CChildView::OnRButtonDown(UINT nFlags, CPoint point)
{
	m_showContextMenu = TRUE;
	m_mouseMoved = FALSE;
	m_view->StartZoomAtPoint(point.x, point.y);
}

void CChildView::OnRButtonUp(UINT nFlags, CPoint point)
{
	if (m_showContextMenu) {
		OnContextMenu(this, point);
	}
	m_showContextMenu = FALSE;
	m_shouldRotate = FALSE;
}

void CChildView::OnContextMenu(CWnd* pWnd, CPoint pos)
{
	CMenu menu;
	menu.LoadMenu(IDR_POPUPMENU);
	CMenu *popup = menu.GetSubMenu(0);

	if (m_view->ComputedMode()) {
		popup->CheckMenuItem(ID_DRAW_WIRE, MF_CHECKED | MF_BYCOMMAND);
		popup->CheckMenuItem(ID_DRAW_SOLID, MF_UNCHECKED | MF_BYCOMMAND);
	}
	else {
		popup->CheckMenuItem(ID_DRAW_WIRE, MF_UNCHECKED | MF_BYCOMMAND);
		popup->CheckMenuItem(ID_DRAW_SOLID, MF_CHECKED | MF_BYCOMMAND);
	}


	ClientToScreen(&pos);
	popup->TrackPopupMenu(TPM_LEFTALIGN | TPM_RIGHTBUTTON, pos.x, pos.y, this->GetTopLevelFrame());

}

BOOL CChildView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	//V3d_View *m_view;
	BOOL isShiftDown = nFlags & MK_SHIFT;
	Standard_Real scaleDelta = isShiftDown ? 0.96 : 0.8;
	Standard_Real scale = m_view->Scale();
	if (zDelta < 0)
		scale /= scaleDelta;
	else
		scale *= scaleDelta;

	m_view->SetScale(scale);

	return TRUE;
}

void CChildView::OnMouseHover(UINT nFlags, CPoint point)
{
	m_mouse = point;
	m_mouseMoved = true;
}

void CChildView::OnMouseMove(UINT nFlags, CPoint point)
{
	//V3d_View *m_view;
	BOOL isShiftDown = nFlags & MK_SHIFT;

	Standard_Real dx = point.x - m_mouse.x;
	Standard_Real dy = m_mouse.y - point.y;
	if (nFlags & MK_LBUTTON && m_shouldRotate) {
		m_view->Rotation(point.x, point.y);
	}
	else if (nFlags & MK_MBUTTON) {
		Standard_Real transFactor =  (isShiftDown ? 2.0 : 5.0) / m_view->Scale();
		m_view->Pan(dx, dy);
	}
	else if (nFlags & MK_RBUTTON) {
		m_showContextMenu = FALSE;
		m_view->ZoomAtPoint(m_mouse.x, m_mouse.y, point.x, point.y);
	}
	else
		((CMCApp*)AfxGetApp())->GetAISContext()->MoveTo(point.x, point.y, m_view);

	m_mouse = point;
	m_mouseMoved = TRUE;
}

void CChildView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	if (nFlags & MK_SHIFT) {

	}
}

void CChildView::OnPaint() 
{
	CPaintDC dc(this); // 用于绘制的设备上下文

	m_view->Redraw();
}

void CChildView::OnSize(UINT nType, int cx, int cy)
{
	if (m_winHeight != cy ||
		m_winWidth != cx) {

			m_winWidth = cx;
			m_winHeight = cy;

			if (!m_view.IsNull())
				m_view->MustBeResized();

			Handle(V3d_Viewer) aViewer = m_view->Viewer();
			if (aViewer->IsActive()) {
				Standard_Integer aWidth=0, aHeight=0, anOffset=0;
				m_view->Window()->Size(aWidth,aHeight);
				aViewer->SetRectangularGridGraphicValues(aWidth,aHeight,anOffset);
				aViewer->Update();
			}
	}
}
void CChildView::OnReset()
{
	// TODO: 在此添加命令处理程序代码
	Reset();
}

void CChildView::OnDrawSolid()
{
	// TODO: 在此添加命令处理程序代码
	if (m_view->ComputedMode()) {
		SetCursor(AfxGetApp()->LoadCursor(IDC_WAIT));
		m_view->SetComputedMode(Standard_False);
		SetCursor(AfxGetApp()->LoadCursor(IDC_ARROW));
	}
}

void CChildView::OnDrawWire()
{
	// TODO: 在此添加命令处理程序代码
	if (!m_view->ComputedMode()) {
		SetCursor(AfxGetApp()->LoadCursor(IDC_WAIT));
		m_view->SetComputedMode(Standard_True);
		SetCursor(AfxGetApp()->LoadCursor(IDC_ARROW));
	}
}

void CChildView::OnDrawBoundingBox()
{
	// TODO: 在此添加命令处理程序代码
	if (!m_view->ComputedMode()) {
		SetCursor(AfxGetApp()->LoadCursor(IDC_WAIT));
		m_view->SetComputedMode(2);
		SetCursor(AfxGetApp()->LoadCursor(IDC_ARROW));
	}
}

void CChildView::OnBackgroundColor()
{
	// TODO: 在此添加命令处理程序代码
	Standard_Real R1;
	Standard_Real G1;
	Standard_Real B1;
	m_view->BackgroundColor(Quantity_TOC_RGB,R1,G1,B1);
	COLORREF m_clr ;
	m_clr = RGB(R1*255,G1*255,B1*255);

	CColorDialog dlg(m_clr);
	if (dlg.DoModal() == IDOK) {
		m_clr = dlg.GetColor();
		R1 = GetRValue(m_clr)/255.;
		G1 = GetGValue(m_clr)/255.;
		B1 = GetBValue(m_clr)/255.;
		m_view->SetBackgroundColor(Quantity_TOC_RGB, R1, G1, B1);
		m_view->Redraw();
	}
}

