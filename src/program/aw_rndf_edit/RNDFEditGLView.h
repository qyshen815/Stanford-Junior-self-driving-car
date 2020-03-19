#ifndef RNDFEDITGLVIEW_H
#define RNDFEDITGLVIEW_H

#include "glwidget.h"

#include <gl_support.h>

namespace vlr {

class RNDFEditGLView : public GLWidget
 {
     Q_OBJECT

public:
	GLuint gridMapTexture;
	GLuint tex_type;

public:
	RNDFEditGLView(QWidget *parent = 0);
    ~RNDFEditGLView();

public slots:

protected:
     void initializeGL();
     void paintGL();
//     void resizeGL(int width, int height);

private:
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent* event);

	void drawGrid(double center_x, double center_y);
	void drawSelected(double center_x, double center_y);
	void drawTrajectory(double center_x, double center_y);
	void drawSmoothedLaneStack(double center_x, double center_y);

	FontRenderer fr;
};

} // namespace vlr

#endif // RNDFEDITGLVIEW_H
