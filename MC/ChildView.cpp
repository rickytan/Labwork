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
	ON_WM_MBUTTONDOWN()
	ON_WM_MBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_MOUSEWHEEL()
	ON_WM_MOUSEHOVER()
	ON_WM_MOUSEMOVE()
	ON_WM_CONTEXTMENU()
	ON_WM_SIZE()
	ON_COMMAND(ID_RESET, &CChildView::OnReset)
END_MESSAGE_MAP()

CChildView::CChildView()
: m_winWidth(0)
, m_winHeight(0)
, m_rotate()
, m_translate()
{

}

CChildView::~CChildView()
{
}

void CChildView::Reset()
{
	m_view->Reset();

	m_view->SetProj(V3d_Zpos);
	m_view->FitAll();
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

	HWND win = GetSafeHwnd();
	Handle(WNT_Window) aWNTWindow = new WNT_Window(win);
	m_view->SetWindow(aWNTWindow);
	if (!aWNTWindow->IsMapped())
		aWNTWindow->Map();

	Reset();
	return 0;
}

void CChildView::OnMButtonDown(UINT nFlags, CPoint point)
{
	V3d_View *m_view;

}

void CChildView::OnMButtonUp(UINT nFlags, CPoint point)
{

}

void CChildView::OnLButtonUp(UINT nFlags, CPoint point)
{
	//V3d_View *m_view;
	AIS_InteractiveContext *context;
	
}

void CChildView::OnLButtonDown(UINT nFlags, CPoint point)
{
	//V3d_View *m_view;
	m_view->StartRotation(point.x, point.y);
}

void CChildView::OnRButtonDown(UINT nFlags, CPoint point)
{
	m_showContextMenu = TRUE;
	m_view->StartZoomAtPoint(point.x, point.y);
}

void CChildView::OnRButtonUp(UINT nFlags, CPoint point)
{
	if (m_showContextMenu) {
		OnContextMenu(this, point);
	}
}

void CChildView::OnContextMenu(CWnd* pWnd, CPoint pos)
{
	CMenu menu;
	menu.LoadMenu(IDR_POPUPMENU);
	CMenu *popup = menu.GetSubMenu(0);
	
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

}

void CChildView::OnMouseMove(UINT nFlags, CPoint point)
{
	//V3d_View *m_view;
	BOOL isShiftDown = nFlags & MK_SHIFT;

	Standard_Real dx = point.x - m_mouse.x;
	Standard_Real dy = m_mouse.y - point.y;
	if (nFlags & MK_MBUTTON) {
		Standard_Real transFactor =  (isShiftDown ? 2.0 : 5.0) / m_view->Scale();
		m_view->Pan(dx, dy);
	}
	if (nFlags & MK_LBUTTON) {
		m_view->Rotation(point.x, point.y);
	}
	if (nFlags & MK_RBUTTON) {
		m_showContextMenu = FALSE;
		m_view->ZoomAtPoint(m_mouse.x, m_mouse.y, point.x, point.y);
	}

	m_mouse = point;
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
	}
}
void CChildView::OnReset()
{
	// TODO: 在此添加命令处理程序代码
	Reset();
}
