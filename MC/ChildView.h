// ChildView.h : CChildView ��Ľӿ�
//


#pragma once
#include "../ModelComp/required.h"

// CChildView ����

class CChildView : public CWnd
{
// ����
public:
	CChildView();
	DECLARE_DYNAMIC(CChildView)
// ����
public:

// ����
public:

// ��д
protected:

	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);

// ʵ��
public:
	virtual ~CChildView();
	Handle(V3d_View) GetV3dView() { return m_view; }

	// ���ɵ���Ϣӳ�亯��
protected:
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnMouseHover(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	DECLARE_MESSAGE_MAP()

private:
	Handle(V3d_View)				m_view;

	int								m_winWidth, m_winHeight;
	float							m_scale;
	gp_Vec3							m_translate;
};

