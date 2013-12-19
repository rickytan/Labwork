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
	void Reset();

	// ���ɵ���Ϣӳ�亯��
protected:
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnMouseHover(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	DECLARE_MESSAGE_MAP()

private:

	void Scale(float scale) { m_scale = scale; m_view->SetScale(m_scale); }
	void Translate(gp_Vec vec) { m_translate = vec; m_view->Translate(m_translate.X(), m_translate.Y(), m_translate.Z()); }
	void Rotate(gp_Quaternion rot) { m_rotate = rot; m_view->Rotate(m_rotate.X(), m_rotate.Y(), m_rotate.Z()); }

	Handle(V3d_View)				m_view;

	int								m_winWidth, m_winHeight;
	CPoint							m_mouse;

	Standard_Real					m_scale;
	gp_Vec 							m_translate;
	gp_Quaternion					m_rotate;
};

