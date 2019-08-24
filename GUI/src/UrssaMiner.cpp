#include "UrssaMiner.h"
#include <iostream>
#include <opencv2/opencv.hpp> 
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>



using namespace System;
using namespace System::Windows::Forms;



[STAThread]
int main(array<String^>^ args) {

	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	GUI::UrssaMiner form;
	Application::Run(%form);

	
	transform();

	/*
	std::cout << H.rows << std::endl;
	std::cout << H.cols << std::endl;
	for (int i = 0; i < H.rows; i++) {
		for (int j = 0; j < H.cols; j++) {
			std::cout << H.at<double>(i, j) << ", ";
		}
		std::cout << std::endl;
	}
	*/
	
	
	std::cin.get();

	return 0;

}

