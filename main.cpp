#include"RigidBody.h"
#include<opencv2/opencv.hpp>
#include<time.h>
#include <iomanip>
#include<fstream>  
#include<sstream>
#include<direct.h>  
#include<string.h>
using namespace std;

int viewSizex = 600;
int viewSizey = 300;
cv::Mat canvas = cv::Mat::zeros(viewSizex, viewSizey, CV_8UC3);
char buffer[120];
void CanvasClear(cv::Mat &canvas)
{
	canvas = cv::Mat::zeros(viewSizey, viewSizex, CV_8UC3);
	for (int j = 0; j < viewSizey; j++)
	{
		for (int i = 0; i < viewSizex; i++)
		{
			canvas.at<cv::Vec3b>(j, i)(0) = 236;
			canvas.at<cv::Vec3b>(j, i)(1) = 255;
			canvas.at<cv::Vec3b>(j, i)(2) = 236;
		}
	}
}


//void CanvasDraw(cv::Mat &canvas, RigidBody body, double scale, Vector2d viewcenter) 
//{
//	Vector2d start, end, center;
//	for (int i = 0, j = body.pointList.size() - 1; i < body.pointList.size(); j = i++)
//	{
//		start = viewcenter + scale*(body.pointList[i].position - viewcenter);
//		end = viewcenter + scale*(body.pointList[j].position - viewcenter);
//		cv::line(canvas, cv::Point(start[0], viewSizey - start[1]), cv::Point(end[0], viewSizey - end[1]), cv::Scalar(100, 200, 100), 2);
//	}
//
//	center = viewcenter + scale*(body.barycenter - viewcenter);
//	cv::circle(canvas, cv::Point(center[0], viewSizey - center[1]), 2, cv::Scalar(0, 0, 250), -1);
//}


void CanvasDraw(cv::Mat &canvas, RigidBody body, double scale, Vector2d viewcenter)
{
	Vector2d start, end, center;
	cv::Point plist[6];
	int nps[] = { 5 };
	std::vector<cv::Point > contour;
	contour.reserve(6);
	for (int i = 0; i < body.pointList.size(); i++)
	{
		start = viewcenter + scale*(body.pointList[i].position - viewcenter);
		contour.push_back(cv::Point(start[0], viewSizey - start[1]));
		
	}
	

	std::vector<std::vector<cv::Point >> contours;
	contours.push_back(contour);

	Vector2d ground;
	ground = viewcenter + scale*(Vector2d(-10, 0.98) - viewcenter);
	cv::line(canvas, cv::Point(ground[0], viewSizey - ground[1]), cv::Point(ground[0] + 1800, viewSizey - ground[1]), cv::Scalar(188, 188, 188), 2);

	cv::polylines(canvas, contours, true, cv::Scalar(242, 233, 196), 2, cv::LINE_AA);
	cv::fillPoly(canvas, contours, cv::Scalar(242, 233, 196));
	for (int i = 0, j = body.pointList.size() - 1; i < body.pointList.size(); j = i++)
	{
		start = viewcenter + scale*(body.pointList[i].position - viewcenter);
		contour.push_back(cv::Point(start[0], viewSizey - start[1]));
		end = viewcenter + scale*(body.pointList[j].position - viewcenter);
		cv::line(canvas, cv::Point(start[0], viewSizey - start[1]), cv::Point(end[0], viewSizey - end[1]), cv::Scalar(255, 182, 147), 2);
	}
	center = viewcenter + scale*(body.barycenter - viewcenter);
	cv::line(canvas, cv::Point(center[0], viewSizey - center[1]), cv::Point(center[0], viewSizey - center[1] + 40), cv::Scalar(101, 251, 255), 2);
	cv::circle(canvas, cv::Point(center[0], viewSizey - center[1]), 4, cv::Scalar(188, 188, 255), -1);
}

string getTime()
{
	time_t timep;
	time(&timep);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H.%M.%S", localtime(&timep));
	return tmp;
}

void CreateFolder()
{
	//获取当前程序路径，建立与时间同名的文件夹
	string time = getTime();
	char pack[30];
	char op[300] = "md ";
	strcpy_s(pack, time.c_str());
	_getcwd(buffer, 120);
	cout << buffer << endl;
	strcat_s(buffer, "\\");
	strcat_s(buffer, pack);
	strcat_s(op, buffer);
	system(op);
	strcat_s(buffer, "\\");
}

void CanvasOutput(int num, cv::Mat* canvas)
{
	string over = ".png";
	stringstream ss;
	string str;
	cout << num << endl;
	ss << num;
	ss >> str;
	str += over;
	str = buffer + str;
	cv::imwrite(str, *canvas);
}


int main()
{
	RigidBody body;

	//body.AddPoint(Vector2d(1, 1), 1, Vector2d(0, 0));
	//body.AddPoint(Vector2d(1.5, 1), 5, Vector2d(0, 0));
	//body.AddPoint(Vector2d(1.5, 3), 15, Vector2d(0, 0));
	//body.AddPoint(Vector2d(1, 3), 4, Vector2d(0, 0));
	//body.AddPoint(Vector2d(0.5, 2.5), 3, Vector2d(0, 0));
	//body.AddPoint(Vector2d(0.5, 1.5), 1, Vector2d(0, 0));


	//body.AddPoint(Vector2d(1.5, 1), 2, Vector2d(0, 0));
	//body.AddPoint(Vector2d(1.5, 3), 6, Vector2d(0, 0));
	//body.AddPoint(Vector2d(0.5, 3), 5, Vector2d(0, 0));
	//body.AddPoint(Vector2d(0.5, 1), 1, Vector2d(0, 0));

	body.AddPoint(Vector2d(1, 1), 4, Vector2d(0, 0));
	body.AddPoint(Vector2d(1.5, 1), 15, Vector2d(0, 0));
	body.AddPoint(Vector2d(1.5, 3), 5, Vector2d(0, 0));
	body.AddPoint(Vector2d(1, 3), 1, Vector2d(0, 0));
	body.AddPoint(Vector2d(0.5, 2.5), 1, Vector2d(0, 0));
	body.AddPoint(Vector2d(0.5, 1.5), 3, Vector2d(0, 0));



	body.InitialBarycenter();
	
	//Initial rotateCenter
	body.rotateCenter = Vector2d(1, 1);
	body.omega = 0.9;
	int ji = 0;
	CreateFolder();
	for (int i = 0; i < 1000; i++)
	{
		body.Solve();
		body.Update(0.008);
		body.Collision();
		CanvasClear(canvas);
		CanvasDraw(canvas, body, 80, Vector2d(-5.5, 0.5));
		cv::imshow("RigidBody", canvas);
		cv::waitKey(10);
		if (i % 5 == 0)
		{
			CanvasOutput(ji, &canvas);
			ji++;
		}
	}
	
}