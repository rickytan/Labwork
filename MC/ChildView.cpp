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
	ON_WM_MOUSEHOVER()
	ON_WM_MOUSEMOVE()
	ON_WM_SIZE()
END_MESSAGE_MAP()

CChildView::CChildView()
{

}

CChildView::~CChildView()
{
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

	m_view->SetProj(V3d_Zneg);
	m_view->FitAll();

	HWND win = GetSafeHwnd();
	Handle(WNT_Window) aWNTWindow = new WNT_Window(win);
	m_view->SetWindow(aWNTWindow);
	if (!aWNTWindow->IsMapped())
		aWNTWindow->Map();

	Standard_Integer w = 800, h = 600;
	aWNTWindow->Size(w, h);

	::PostMessage( GetSafeHwnd(), WM_SIZE, SIZE_RESTORED, MAKELONG(w, h));

	return 0;
}

void CChildView::OnLButtonUp(UINT nFlags, CPoint point)
{

}

void CChildView::OnLButtonDown(UINT nFlags, CPoint point)
{
	
}

void CChildView::OnMouseHover(UINT nFlags, CPoint point)
{


}

void CChildView::OnMouseMove(UINT nFlags, CPoint point)
{
	if (nFlags & MK_MBUTTON) {

	}
}

void CChildView::OnPaint() 
{
	CPaintDC dc(this); // 用于绘制的设备上下文

	m_view->Redraw();
}

void CChildView::OnSize(UINT nType, int cx, int cy)
{
	if (!m_view.IsNull())
		m_view->MustBeResized();
}