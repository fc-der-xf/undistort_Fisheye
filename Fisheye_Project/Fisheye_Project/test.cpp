#include <opencv2\opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <fstream>
using namespace std;
using namespace cv;

int main() {
	cv::Vec4d distortion_coeffs;
	distortion_coeffs[0] = 0.04205792769789696;
	distortion_coeffs[1] = -0.02283179759979248;
	distortion_coeffs[2] = -0.0015250984579324722;
	distortion_coeffs[3] = 0.0004033915465697646;

	double k1 = distortion_coeffs[0], k2 = distortion_coeffs[1], k3 = distortion_coeffs[2], k4 = distortion_coeffs[3];
	double fx = 368.1839904785156;
	double fy = 354.2261657714844;
	double cx = 656.9208984375;
	double cy = 396.4649963378906;
	
	cv::Matx33d intrinsic_matrix(368.1839904785156, 0 , 656.9208984375, 0, 354.2261657714844, 396.4649963378906, 0, 0, 1);
	//我不理解，明明程序里面都是有个641 * 8的操作的...
	//但是乘上去之后反而出问题了，不乘的话倒是可以正常的畸变校正...


	vector<double> rotation_vector; // 旋转向量
	vector<double> translation_vector; //平移向量

	Mat image = imread("frames_1.jpg");
	Size image_size = image.size();

	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);

	fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
	Mat t = image.clone();
	cv::remap(image, t, mapx, mapy, INTER_LINEAR);
	//pass
	//这里校正是没问题的，但是为什么用undistortPoints就出问题了呢

	Point2f src_point = Point(281.0, 497.0);
	vector<Point2f> src_points;
	src_points.push_back(src_point);
	vector<Point2f> dst_points;
	Mat R1 = Mat::zeros(3, 3, CV_32F);
	fisheye::undistortPoints(src_points, dst_points, intrinsic_matrix, distortion_coeffs);
	
	//因为那是归一化之后的坐标！还要再进行别的操作
	//校正后的像素坐标算法
	double x_undistort = dst_points[0].x * fx + cx;
	double y_undistort = dst_points[0].y * fy + cy;

	//为什么↓这种方法就不行呢
	Mat test_img;
	fisheye::undistortImage(image, test_img, intrinsic_matrix, distortion_coeffs);
	
	cv::Mat gray_img = cv::imread("frames_1.jpg", 0);
	int rows = gray_img.rows, cols = gray_img.cols;
	cv::Mat undistort_img = cv::Mat(rows, cols, CV_8UC1);

	//现在这段可以大致完成任务了
	for (int v = 0; v < rows; v++) {
		for (int u = 0; u < cols; u++) {
			double x = (u - cx) / fx, y = (v - cy) / fy;
			double r = sqrt(x * x + y * y); //像点距图像中心的径向距离
			double theta = atan(r);
			double thetad = theta * (1 + k1 * theta + k2 * pow(theta, 4) + k3 * pow(theta, 6) + k4 * pow(theta, 8));
			//double x_distorted = x * (1 + k1 * r + k2 * r * r * r * r + k3 * pow(r,6) + k4 * pow(r, 8));
			//double y_distorted = y * (1 + k1 * r + k2 * r * r * r * r + k3 * pow(r, 6) + k4 * pow(r, 8));

			double u_distorted = fx * x * thetad / r + cx;
			double v_distorted = fy * y * thetad / r + cy;

			if ((int)x == 425) {
				cout << u_distorted << "," << v_distorted << endl;
			}

			/*if ((int)(fx * dst_points[0].x * thetad / r + cx) == 425) {
				cout << u << "," << v << endl;
			}*/

			//if (u == 425 && v == 405) {
			//	cout << fx * dst_points[0].x * thetad / r + cx << endl;
			//	cout << fy * dst_points[0].y * thetad / r + cy << endl;
			//	/*cout << x_distorted << "," << y_distorted << endl;*/
			//	//这里反而算出来了-0.610545,0.0015438...这不是undistortPoint的值吗...不完全是，但是差不多
			//	// //所以才说还要再乘以内参的值是吧
			//	//看了一下，undistortPoint好像传进去的是世界坐标系的点
			//	cout << u_distorted << "," << v_distorted << endl;
			//}

			//最近邻插值
			if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
				undistort_img.at<uchar>(v, u) = gray_img.at<uchar>((int)v_distorted, (int)u_distorted);
			}
			else {
				undistort_img.at<uchar>(v, u) = 0;
			}
		}
	}

	cv::imwrite("undistort_frames_1.jpg", undistort_img);

	//验证外参的排列方式
	/*cv::Vec3d rotation_vector;
	cv::Vec3d translation_vector;

	rotation_vector[0] = 0.8495314121246338;
	rotation_vector[1] = 0.0769149661064148;
	rotation_vector[2] = -0.14422720670700073;

	translation_vector[0] = -0.001887165242806077;
	translation_vector[1] = -0.009032193571329117;
	translation_vector[2] = 1.117197036743164;*/
	


	cv::waitKey(0);
	return 0;
}