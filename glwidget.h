#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QMainWindow>
#include <QGLWidget>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
#include <QMouseEvent>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <QCoreApplication>
#include <QtOpenGL/QGLShader>
#include <QtOpenGL/QGLShaderProgram>
#include <QtOpenGL/QGLFunctions>
#include <QtOpenGL/QGLBuffer>
#include <pcl/surface/mls.h>


class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    GLWidget(QWidget *parent = 0);
//    ~glwidget();
    void initializeGL();//初始化
    void resizeGL(int width, int height);//更改形状
    void paintGL();//绘制
    float trans;

    float r_x;
    float r_y;
    float r_z;
    float s;
    float s_distance;

    bool show_matched_points;

    std::vector<std::vector<double> > click_points;

    std::vector<std::vector<double> > source_points;
    std::vector<std::vector<double> > target_points;
    pcl::PointCloud<pcl::PointXYZRGB> pc;

    QPoint last_pos;

private:
//    QGLShaderProgram shaderProgram;
//    QGLBuffer vbo, ebo;

    void mousePressEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void drawAxis();

};

#endif // GLWIDGET_H
