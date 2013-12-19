// ChildView.h : CChildView 类的接口
//


#pragma once
#include "../ModelComp/required.h"

// CChildView 窗口

class CChildView : public CWnd
{
// 构造
public:
	CChildView();
	DECLARE_DYNAMIC(CChildView)
// 属性
public:

// 操作
public:

// 重写
protected:

	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

// 实现
public:
	virtual ~CChildView();
	Handle(V3d_View) GetV3dView() { return m_view; }
	void Reset();

	// 生成的消息映射函数
protected:
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnMouseHover(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint pos);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);

	afx_msg void OnReset();

	DECLARE_MESSAGE_MAP()

private:
	Handle(V3d_View)				m_view;

	int								m_winWidth, m_winHeight;
	CPoint							m_mouse;

	BOOL							m_showContextMenu;

	Standard_Real					m_scale;
	gp_Vec 							m_translate;
	gp_Quaternion					m_rotate;

};

