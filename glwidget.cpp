#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent) : QGLWidget(parent)
{
    trans = 0.0;
    r_x = 0.0;
    r_y = 0.0;
    r_z = 0.0;
    s_distance = -10.0;
    show_matched_points = false;
//    pc.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}


void GLWidget::initializeGL()
{
    std::cout << "GLWidget init..." << std::endl;

    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

}

void GLWidget::resizeGL(int width, int height)
{std::cout << "feature points size:" << click_points.size() << std::endl;
    for(int i = 0; i < click_points.size(); i++)
    {
        std::cout << click_points[i][0] << ", " << click_points[i][1] << ", " << click_points[i][2] << std::endl;
    }
    std::cout << "resizeGL:" << width << " " << height << std::endl;
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    gluPerspective(60.0, width/height, 1.0, 1000);
//    glRotatef(trans, 0.0, 1.0, 0.0);

//    glOrtho(-width/30,width/30,-height/30,height/30,-10,10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
//    glTranslatef(0.1, 0.1, 0.1);
//    glRotatef(180, 1.0, 0.0, 0.0);
    std::cout << "resizeGL..." << std::endl;

}

void GLWidget::paintGL()
{
//    std::cout << "paintGL..." << std::endl;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    glViewport(0, 0, width()/2, height()/2);

//    glBegin(GL_TRIANGLES);
//        glColor3f(1.0, 0.0, 0.0);
//        glVertex3f(-1.0, -1.0, -2.0);
//        glColor3f(0.0, 1.0, 0.0);
//        glVertex3f(1.0, -1.0, -2.0);
//        glColor3f(0.0, 0.0, 1.0);
//        glVertex3f( 0.0, 1.0, -2.0);

//    glEnd();

//    glBegin(GL_TRIANGLES);
//        glColor3f(1.0, 0.0, 0.0);
//        glVertex3f(-1.0, -1.0, 2.0);
//        glColor3f(1.0, 0.0, 0.0);
//        glVertex3f(1.0, -1.0, 2.0);
//        glColor3f(1.0, 0.0, 0.0);
//        glVertex3f( 0.0, 1.0, 2.0);
//    glEnd();
    glPointSize(0.05);
    if(pc.points.size())
    {
        glBegin(GL_POINTS);
        for(int i = 0; i < pc.points.size(); i++)
        {
            pcl::PointXYZRGB point = pc.points[i];
            glColor3f(point.r / 255.0, point.g / 255.0, point.b / 255.0);
            glVertex3f(point.x, point.y, point.z);
        }
        glEnd();
    }

    if(click_points.size())
    {
        glPointSize(3.0);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);

        for(int i = 0; i < click_points.size(); i++)
        {
//            std::cout << click_points[i][0] << ", " << click_points[i][1] << ", " << click_points[i][2] << std::endl;
            glVertex3f(click_points[i][0], click_points[i][1], click_points[i][2]);
        }
        glEnd();
    }

    if(show_matched_points && source_points.size() == target_points.size())
    {
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        for(int i = 0; i < source_points.size(); i++)
        {
            glVertex3d(source_points[i][0], source_points[i][1], source_points[i][2]);
            glVertex3d(target_points[i][0], target_points[i][1], target_points[i][2]);
        }
        glEnd();
    }

    drawAxis();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, width()/height(), 0.01, 1000);
    glTranslatef(0.0, 0.0, s_distance);
    glRotatef(r_y, 0.0, 1.0, 0.0);
    glRotatef(r_z, 0.0, 0.0, 1.0);

//    std::cout << "feature points size:" << click_points.size() << std::endl;
//    for(int i = 0; i < click_points.size(); i++)
//    {
//        std::cout << click_points[i][0] << ", " << click_points[i][1] << ", " << click_points[i][2] << std::endl;
//    }

}

void GLWidget::drawAxis()
{
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(1.0, 0.0, 0.0);

        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 1.0, 0.0);

        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 1.0);
    glEnd();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    last_pos = event->pos();
//    std::cout << last_pos.x() << ", " << last_pos.y() << std::endl;
}

void GLWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
//    std::cout << event->pos().x() << ", " << event->pos().y() << std::endl;
    if(event->buttons() == Qt::LeftButton)
    {
        double modelview[16], projection[16];
        int viewport[4];

        GLfloat winx, winy, winz;
        GLdouble posx, posy, posz;
        glGetIntegerv(GL_VIEWPORT, viewport);
        glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
        glGetDoublev(GL_PROJECTION_MATRIX, projection);

        winx = event->pos().x();
        winy = height() - event->pos().y();
        glReadPixels((int)winx, (int)winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);
        gluUnProject(winx, winy, winz, modelview, projection, viewport, &posx, &posy, &posz);
        std::cout << posx << ", " << posy << ", " << posz << std::endl;

        std::vector<double> point = {posx, posy, posz};
        click_points.push_back(point);

        std::cout << "feature points size:" << click_points.size() << std::endl;
        for(int i = 0; i < click_points.size(); i++)
        {
            std::cout << click_points[i][0] << ", " << click_points[i][1] << ", " << click_points[i][2] << std::endl;
        }
    }
    else if(event->buttons() == Qt::RightButton)
    {
        if(click_points.size())
        {
            std::vector<std::vector<double> >::iterator iter = click_points.end();
            click_points.erase(iter);

            std::cout << "feature points size:" << click_points.size() << std::endl;
            for(int i = 0; i < click_points.size(); i++)
            {
                std::cout << click_points[i][0] << ", " << click_points[i][1] << ", " << click_points[i][2] << std::endl;
            }
        }
    }
    updateGL();

}


void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
//    r_x = 0;
//    r_y = 0;
//    r_z = 0;
    GLfloat dx = GLfloat(event->x() - last_pos.x()) / width();
    GLfloat dy = GLfloat(event->y() - last_pos.y()) / height();

    if(event->buttons() & Qt::LeftButton){

        r_z += 180 * dy;
        r_y += 180 * dx;
//        r_x -= dy;
//        r_y -= dx;
//        std::cout << r_x << std::endl;

        updateGL();

    }

//    else if(event->buttons() & Qt::RightButton){

//        r_x -= 180 * dy;
//        r_y -= 180 * dx;
//        r_x -= dy;
//        r_z -= dx;

//        updateGL();
//    }

    last_pos = event->pos();
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    if(event->delta() > 0)
    {
        s_distance += 0.1;
        updateGL();
    }
    else
    {

        s_distance -= 0.1;
        updateGL();
    }
//    std::cout << event->delta() << std::endl;
}
