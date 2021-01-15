#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <string.h>
#include <typeinfo>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <glwidget.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <algorithm>
#include <opencv4/opencv2/opencv.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_actionOpenFile_1_triggered();
    void on_actionOpenFile_2_triggered();

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;

    void ReadPointCloudFile(std::string filepath, int type, int widget_num);
//    void ShowFusionPointCloud(Eigen::Matrix3d rotate, Eigen::Vector3d transform);
    void ShowFusionPointCloud(Eigen::Matrix4d R_T);
    void ToCvFormat(std::vector<std::vector<double> >& click_points, std::vector<cv::Point3f>& feature_points);
    cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3f>& srcPoints, const std::vector<cv::Point3f>&  dstPoints);
};

#endif // MAINWINDOW_H
