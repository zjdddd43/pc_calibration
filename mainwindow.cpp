#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/eigen.hpp>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_actionOpenFile_1_triggered()
{
//    std::cout << "111" << std::endl;
    QString filepath = QFileDialog::getOpenFileName(this,"choose file","./", "PLY Files(*.ply);;PCD Files(*.pcd)");
//    std::cout << filepath.toStdString() << std::endl;

    std::string s = filepath.toStdString();
    std::string file_type = s.substr(s.size() - 4, s.size() - 1);
//    std::cout << file_type << std::endl;

    if(file_type == ".ply") ReadPointCloudFile(s, 1, 1);
    else if(file_type == ".pcd") ReadPointCloudFile(s, 2, 1);
    else std::cout << "file type error..." << std::endl;

//    ui->openGLWidget_3->pc += ui->openGLWidget_2->pc;

}

void MainWindow::on_actionOpenFile_2_triggered()
{
//    std::cout << "222" << std::endl;#include <pcl/common/transforms.h>
    QString filepath = QFileDialog::getOpenFileName(this,"choose file","./", "PLY Files(*.ply);;PCD Files(*.pcd)");
//    std::cout << filepath.toStdString() << std::endl;

    std::string s = filepath.toStdString();
    std::string file_type = s.substr(s.size() - 4, s.size() - 1);
//    std::cout << file_type << std::endl;

    if(file_type == ".ply") ReadPointCloudFile(s, 1, 2);
    else if(file_type == ".pcd") ReadPointCloudFile(s, 2, 2);
    else std::cout << "file type error..." << std::endl;

    ui->openGLWidget_3->pc += ui->openGLWidget->pc;
}

void MainWindow::ReadPointCloudFile(std::string filepath, int type, int widget_num)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    if(type == 1)
    {
//        pcl::io::loadPLYFile(filepath, ui->openGLWidget_2->pc);
        pcl::io::loadPLYFile(filepath, cloud);
//        std::cout << ui->openGLWidget_2->pc.points.size() << std::endl;
    }
    else if(type == 2)
    {
        pcl::io::loadPCDFile(filepath, cloud);
    }

    if(widget_num == 1) ui->openGLWidget_2->pc = cloud;
    else ui->openGLWidget->pc = cloud;

}


//void MainWindow::on_pushButton_clicked()
//{
//    std::cout << "push buttion..." << std::endl;

//    std::vector<std::vector<double> >& points1 = ui->openGLWidget_2->click_points;
//    std::vector<std::vector<double> >& points2 = ui->openGLWidget->click_points;

//    Eigen::MatrixXd points1_matrix;
//    Eigen::MatrixXd points2_matrix;
//    Eigen::MatrixXd first_point_error;
//    Eigen::Matrix3d rotate;

////    Eigen::Matrix<double, points1.size(), 3> points1_matrix;
////    Eigen::Matrix<double, points2.size(), 3> points2_matrix;

//    int point_size = points1.size() > points2.size() ? points2.size() : points1.size();

//    if(point_size != 0)
//    {
//        points1_matrix.resize(point_size, 3);
//        points2_matrix.resize(point_size, 3);
////        first_point_error.resize(point_size, 3);

//        if(point_size >= 3)
//        {
//            for(int i = 0; i < point_size; i++)
//            {
//                points1_matrix.row(i) = Eigen::VectorXd::Map(&points1[i][0], points1[i].size());
//                points2_matrix.row(i) = Eigen::VectorXd::Map(&points2[i][0], points2[i].size());
////                first_point_error.row(i) = Eigen::Vector3d(points2[0][0] - points1[0][0], points2[0][1] - points1[0][1], points2[0][2] - points1[0][2]);
//            }
//            // 输出特征点矩阵
//            std::cout << "Left widget feature points' matrix" << std::endl;
//            std::cout << points1_matrix << std::endl;
//            std::cout << "Right widget feature points' matrix" << std::endl;
//            std::cout << points2_matrix << std::endl;
//            std::cout << "fist point error" << std::endl;
//            std::cout << first_point_error << std::endl;

//            // 将第一个特征点位移至重合
////            for(int j = 0; j < 3; j++)
////            {
////                points1_matrix.colopenGLWidget_3(j) += first_point_error.col(j);
////            }
////            std::cout << "Left widget feature points trans" << std::endl;
////            std::cout << points1_matrix << std::endl;

//            // 计算旋转矩阵
////            rotate = points1_matrix.ldlt().solve(points2_matrix);
////            rotate = points1_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(points2_matrix);
//            rotate = points1_matrix.colPivHouseholderQr().solve(points2_matrix);
//            std::cout << "rotate matrix :" << std::endl;
//            std::cout << rotate << std::endl;

//            Eigen::Vector3d T(0,0,0);

////            ShowFusionPointCloud(rotate, first_point_error.row(0));
//            ShowFusionPointCloud(rotate, T);

//        }
//        else std::cout << "not enough feature points. " << std::endl;
//    }
//    else std::cout << "no feature points." << std::endl;

//}

void MainWindow::on_pushButton_clicked()
{
    std::cout << "push buttion..." << std::endl;

    std::vector<std::vector<double> >& points1 = ui->openGLWidget_2->click_points;
    std::vector<std::vector<double> >& points2 = ui->openGLWidget->click_points;

    std::vector<cv::Point3f> src_points;
    std::vector<cv::Point3f> tar_points;
    ToCvFormat(points1, src_points);
    ToCvFormat(points2, tar_points);

    cv::Mat transform_matrix = Get3DR_TransMatrix(src_points, tar_points);
    std::cout << transform_matrix << std::endl;

    Eigen::Matrix4d matrix_4d;
    cv::cv2eigen(transform_matrix, matrix_4d);
    std::cout << matrix_4d << std::endl;

    Eigen::Matrix3d matrix_3d;
    matrix_3d = matrix_4d.block<3,3>(0,0);
//    std::cout << matrix_3d << std::endl;

    Eigen::Quaterniond q(matrix_3d);
//    std::cout << "q:" << q.coeffs() << std::endl;

    std::cout << "x y z q.w q.x q.y q.z" << std::endl;
    std::cout << matrix_4d(0,3) << " " << matrix_4d(1,3) << " " << matrix_4d(2,3) << " ";
    std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;

    ShowFusionPointCloud(matrix_4d);

}

void MainWindow::ShowFusionPointCloud(Eigen::Matrix4d R_T)
{
    pcl::PointCloud<pcl::PointXYZRGB>& left_widget_pc = ui->openGLWidget_2->pc;
    pcl::PointCloud<pcl::PointXYZRGB>& right_widget_pc = ui->openGLWidget->pc;

    pcl::PointCloud<pcl::PointXYZRGB> trans_pc;
    pcl::transformPointCloud(left_widget_pc, trans_pc, R_T);

    ui->openGLWidget_3->pc += trans_pc;

//    ui->openGLWidget_3->source_points = ui->openGLWidget_2->click_points;
//    ui->openGLWidget_3->target_points = ui->openGLWidget->click_points;
//    ui->openGLWidget_3->show_matched_points = true;
    ui->openGLWidget_3->update();

}

//void MainWindow::ShowFusionPointCloud(Eigen::Matrix3d R, Eigen::Vector3d T)
//{
//    Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
////    Eigen::Isometry3d test = Eigen::Isometry3d::Identity();
////    test.rotate(R);
////    test.pretranslate(T);
//    transform_matrix.block<3,3>(0, 0) = R;
////    matrix.block<1,3>(0, 3) = transform;
//    transform_matrix(0,3) = T[0];
//    transform_matrix(1,3) = T[1];
//    transform_matrix(2,3) = T[2];
//    std::cout << "transform matrix :" << std::endl;
//    std::cout << transform_matrix << std::endl;

//    pcl::PointCloud<pcl::PointXYZRGB>& left_widget_pc = ui->openGLWidget_2->pc;
//    pcl::PointCloud<pcl::PointXYZRGB>& right_widget_pc = ui->openGLWidget->pc;

//    pcl::PointCloud<pcl::PointXYZRGB> trans_pc;
////    pcl::PointCloud<pcl::PointXYZRGB> projection_pc;

//    //
////    double projection[16];
////    glGetDoublev(GL_PROJECTION_MATRIX, projection);
////    Eigen::Matrix4d projection_matrix;
////    for(int i = 0; i < 4; i+= 4)
////    {
////        projection_matrix(i, 0) = projection[i*4];
////        projection_matrix(i, 1) = projection[i*4 + 1];
////        projection_matrix(i, 2) = projection[i*4 + 2];
////        projection_matrix(i, 3) = projection[i*4 + 3];
////    }


//    pcl::transformPointCloud(left_widget_pc, trans_pc, transform_matrix);


//    std::cout << trans_pc.size() << std::endl;

////    trans_pc += right_widget_pc;
////    std::cout << trans_pc.size() << std::endl;

//    ui->openGLWidget_3->pc += trans_pc;

//    ui->openGLWidget_3->source_points = ui->openGLWidget_2->click_points;
//    ui->openGLWidget_3->target_points = ui->openGLWidget->click_points;
//    ui->openGLWidget_3->show_matched_points = true;
//    ui->openGLWidget_3->update();

//}

void MainWindow::ToCvFormat(std::vector<std::vector<double> >& click_points, std::vector<cv::Point3f>& feature_points)
{
    cv::Point3f point;
    for(int i = 0; i < click_points.size(); i++)
    {
        point.x = click_points[i][0];
        point.y = click_points[i][1];
        point.z = click_points[i][2];
        feature_points.push_back(point);
    }
}


cv::Mat MainWindow::Get3DR_TransMatrix(const std::vector<cv::Point3f>& srcPoints, const std::vector<cv::Point3f>&  dstPoints)
{
    double srcSumX = 0.0f;
    double srcSumY = 0.0f;
    double srcSumZ = 0.0f;

    double dstSumX = 0.0f;
    double dstSumY = 0.0f;
    double dstSumZ = 0.0f;

    //至少三组点
    if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
    {
        return cv::Mat();
    }

    int pointsNum = srcPoints.size();
    for (int i = 0; i < pointsNum; ++i)
    {
        srcSumX += srcPoints[i].x;
        srcSumY += srcPoints[i].y;
        srcSumZ += srcPoints[i].z;

        dstSumX += dstPoints[i].x;
        dstSumY += dstPoints[i].y;
        dstSumZ += dstPoints[i].z;
    }

    cv::Point3d centerSrc, centerDst;

    centerSrc.x = double(srcSumX / pointsNum);
    centerSrc.y = double(srcSumY / pointsNum);
    centerSrc.z = double(srcSumZ / pointsNum);

    centerDst.x = double(dstSumX / pointsNum);
    centerDst.y = double(dstSumY / pointsNum);
    centerDst.z = double(dstSumZ / pointsNum);

    //Mat::Mat(int rows, int cols, int type)
    cv::Mat srcMat(3, pointsNum, CV_64FC1);
    cv::Mat dstMat(3, pointsNum, CV_64FC1);

    for (int i = 0; i < pointsNum; ++i)//N组点
    {
        //三行
        srcMat.at<double>(0, i) = srcPoints[i].x - centerSrc.x;
        srcMat.at<double>(1, i) = srcPoints[i].y - centerSrc.y;
        srcMat.at<double>(2, i) = srcPoints[i].z - centerSrc.z;

        dstMat.at<double>(0, i) = dstPoints[i].x - centerDst.x;
        dstMat.at<double>(1, i) = dstPoints[i].y - centerDst.y;
        dstMat.at<double>(2, i) = dstPoints[i].z - centerDst.z;

    }

    cv::Mat matS = srcMat * dstMat.t();

    cv::Mat matU, matW, matV;
    cv::SVDecomp(matS, matW, matU, matV);

    cv::Mat matTemp = matU * matV;
    double det = cv::determinant(matTemp);//行列式的值

    double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
    cv::Mat matM(3, 3, CV_64FC1, datM);

    cv::Mat matR = matV.t() * matM * matU.t();

    double* datR = (double*)(matR.data);
    double delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2]);
    double delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);
    double delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);


    //生成RT齐次矩阵(4*4)
    cv::Mat R_T = (cv::Mat_<double>(4, 4) <<
        matR.at<double>(0, 0), matR.at<double>(0, 1), matR.at<double>(0, 2), delta_X,
        matR.at<double>(1, 0), matR.at<double>(1, 1), matR.at<double>(1, 2), delta_Y,
        matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2), delta_Z,
        0, 0, 0, 1
        );

    return R_T;
}
