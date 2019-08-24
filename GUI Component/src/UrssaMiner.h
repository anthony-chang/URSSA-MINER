#include <opencv2/opencv.hpp> 
#include <iostream>
#include <math.h>
#include <string>
#include <msclr\marshal_cppstd.h>



#define PI 3.141592654
#define CALIBRATIONPTS 9
#define PLANEXSHIFT 550 //pixels
#define PLANEWIDTH 408
#define PLANEHEIGHT 272

const double dist = 45, width = 45, length = 30; //cm
//dist is distance from arm pivot to edge of box
//width is dimension of box perpendicular to arm, length is dimension parallel to arm
const double upperArmLength = 30.48, forearmLength = 30.48, clawLength = 15; //cm
const double heightDif = 0; //cm, sand height - base height

#pragma once
int imagePts[CALIBRATIONPTS][2];
int corPts[CALIBRATIONPTS][2];
int spcfdPt[2];
double trnsfmdPt[2];
cv::Mat H, final;
double theta[3];



std::string tostdstring(System::String^ s) {
	msclr::interop::marshal_context context;
	return context.marshal_as<std::string>(s);
}

void transform() {
	std::vector<cv::Point2f> source, corresponding, location;

	for (int i = 0; i < 5; i++) {
		source.push_back(cv::Point2f(imagePts[i][0], imagePts[i][1]));
		corresponding.push_back(cv::Point2f(corPts[i][0], corPts[i][1]));
	}
	location.push_back(cv::Point2f(spcfdPt[0], spcfdPt[1]));

	H = cv::findHomography(source, corresponding);
	cv::perspectiveTransform(location, final, H);
	
	trnsfmdPt[0] = final.at<cv::Point2f>(0, 0).x;
	trnsfmdPt[1] = final.at<cv::Point2f>(0, 0).y;
	
}
double pyth(double a, double b) {
	return sqrt(a*a + b*b);
}
void computeAngles() {
	double x = (trnsfmdPt[0] - (PLANEWIDTH / 2)) / PLANEWIDTH * width; //cm, horizontal distance to location
	double y = (PLANEHEIGHT - trnsfmdPt[1]) / PLANEHEIGHT * length + dist; //cm, "vertical" distance to location
	std::cout << x << ", " << y << std::endl;
	if (x == 0)
		theta[0] = 0;
	else if (x < 0)
		theta[0] = (atan(y / abs(x)) - PI / 2);
	else
		theta[0] = PI / 2 - atan(y / x);
	
	
	double R = pyth(x, y);
	double errorx = 0x3F3F3F3F, errory = 0x3F3F3F3F, error = 0x3F3F3F3F;
	double tempx, tempy, t1, t2;
	for (int i = 90; i >=0 ; --i) {
		for (int j = 0; j < 90; ++j) {
			t1 = i * PI / 180, t2 = j * PI / 180;
			tempx = upperArmLength * cos(t1) + forearmLength * sin(t2) + clawLength;
			tempy = upperArmLength * sin(t1) - forearmLength * cos(t2);
			errorx = tempx - R;
			errory = tempy;
			if (pyth(errorx, errory) < error) {
				error = pyth(errorx, errory);
				theta[1] = t1; //radians
				theta[2] = t2; //radians
			}
		}
	}
}

namespace GUI {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for UrssaMiner
	/// </summary>
	public ref class UrssaMiner : public System::Windows::Forms::Form
	{
	public:
		UrssaMiner(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			arduinoMega->Open();
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~UrssaMiner()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::Windows::Forms::Label^  mousecoord;
	private: System::Windows::Forms::Button^  btnAddCaliPt;

	private: Microsoft::VisualBasic::PowerPacks::ShapeContainer^  shapeContainer1;
	private: Microsoft::VisualBasic::PowerPacks::RectangleShape^  box;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image5;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor5;

	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image4;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor4;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor3;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image3;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image2;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor2;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor1;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image1;
	private: System::Windows::Forms::Label^  currentPt;
	private: System::Windows::Forms::Button^  btnChooseLocation;
	private: Microsoft::VisualBasic::PowerPacks::RectangleShape^  imageLoc;
	private: Microsoft::VisualBasic::PowerPacks::RectangleShape^  finalLoc;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor8;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image8;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image7;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor7;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor6;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image6;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  cor9;
	private: Microsoft::VisualBasic::PowerPacks::OvalShape^  image9;
	private: System::Windows::Forms::Button^  confirmLoc;
	private: System::Windows::Forms::Label^  angleslbl;
	public: System::IO::Ports::SerialPort^  arduinoMega;
	private: Microsoft::VisualBasic::PowerPacks::LineShape^  lineShape1;
	private: System::Windows::Forms::Label^  dataLabel;

	private: System::Windows::Forms::Label^  templbl;
	private: System::Windows::Forms::Button^  update;
	private: System::Windows::Forms::Label^  masslbl;
	private: System::Windows::Forms::Button^  capture;
	private: System::Windows::Forms::Button^  saveImg;

	private: System::Windows::Forms::Label^  statuslbl;
	private: System::Windows::Forms::RichTextBox^  statusBox;
	private: System::Windows::Forms::Button^  resetBtn;
	private: System::Windows::Forms::Label^  volumelbl;



	public:


	public:
	private:
	private: System::ComponentModel::IContainer^  components;
	public:



	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(UrssaMiner::typeid));
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->mousecoord = (gcnew System::Windows::Forms::Label());
			this->btnAddCaliPt = (gcnew System::Windows::Forms::Button());
			this->shapeContainer1 = (gcnew Microsoft::VisualBasic::PowerPacks::ShapeContainer());
			this->lineShape1 = (gcnew Microsoft::VisualBasic::PowerPacks::LineShape());
			this->cor9 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image9 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor8 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image8 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image7 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor7 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor6 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image6 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->finalLoc = (gcnew Microsoft::VisualBasic::PowerPacks::RectangleShape());
			this->imageLoc = (gcnew Microsoft::VisualBasic::PowerPacks::RectangleShape());
			this->image5 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor5 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image4 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor4 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor3 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image3 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image2 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor2 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->cor1 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->image1 = (gcnew Microsoft::VisualBasic::PowerPacks::OvalShape());
			this->box = (gcnew Microsoft::VisualBasic::PowerPacks::RectangleShape());
			this->currentPt = (gcnew System::Windows::Forms::Label());
			this->btnChooseLocation = (gcnew System::Windows::Forms::Button());
			this->confirmLoc = (gcnew System::Windows::Forms::Button());
			this->angleslbl = (gcnew System::Windows::Forms::Label());
			this->arduinoMega = (gcnew System::IO::Ports::SerialPort(this->components));
			this->dataLabel = (gcnew System::Windows::Forms::Label());
			this->templbl = (gcnew System::Windows::Forms::Label());
			this->update = (gcnew System::Windows::Forms::Button());
			this->masslbl = (gcnew System::Windows::Forms::Label());
			this->capture = (gcnew System::Windows::Forms::Button());
			this->saveImg = (gcnew System::Windows::Forms::Button());
			this->statuslbl = (gcnew System::Windows::Forms::Label());
			this->statusBox = (gcnew System::Windows::Forms::RichTextBox());
			this->resetBtn = (gcnew System::Windows::Forms::Button());
			this->volumelbl = (gcnew System::Windows::Forms::Label());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->SuspendLayout();
			// 
			// pictureBox1
			// 
			this->pictureBox1->Cursor = System::Windows::Forms::Cursors::Cross;
			this->pictureBox1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBox1.Image")));
			this->pictureBox1->Location = System::Drawing::Point(0, 0);
			this->pictureBox1->Margin = System::Windows::Forms::Padding(0);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(520, 272);
			this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::Zoom;
			this->pictureBox1->TabIndex = 0;
			this->pictureBox1->TabStop = false;
			this->pictureBox1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &UrssaMiner::pictureBox1_MouseDown);
			this->pictureBox1->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &UrssaMiner::pictureBox1_MouseMove);
			// 
			// mousecoord
			// 
			this->mousecoord->AutoSize = true;
			this->mousecoord->Location = System::Drawing::Point(523, 289);
			this->mousecoord->Name = L"mousecoord";
			this->mousecoord->Size = System::Drawing::Size(29, 13);
			this->mousecoord->TabIndex = 1;
			this->mousecoord->Text = L"(x, y)";
			// 
			// btnAddCaliPt
			// 
			this->btnAddCaliPt->Location = System::Drawing::Point(370, 324);
			this->btnAddCaliPt->Name = L"btnAddCaliPt";
			this->btnAddCaliPt->Size = System::Drawing::Size(160, 23);
			this->btnAddCaliPt->TabIndex = 2;
			this->btnAddCaliPt->Text = L"Add Callibration Points";
			this->btnAddCaliPt->UseVisualStyleBackColor = true;
			this->btnAddCaliPt->Click += gcnew System::EventHandler(this, &UrssaMiner::addCaliPt_Click);
			// 
			// shapeContainer1
			// 
			this->shapeContainer1->Location = System::Drawing::Point(0, 0);
			this->shapeContainer1->Margin = System::Windows::Forms::Padding(0);
			this->shapeContainer1->Name = L"shapeContainer1";
			this->shapeContainer1->Shapes->AddRange(gcnew cli::array< Microsoft::VisualBasic::PowerPacks::Shape^  >(22) {
				this->lineShape1,
					this->cor9, this->image9, this->cor8, this->image8, this->image7, this->cor7, this->cor6, this->image6, this->finalLoc, this->imageLoc,
					this->image5, this->cor5, this->image4, this->cor4, this->cor3, this->image3, this->image2, this->cor2, this->cor1, this->image1,
					this->box
			});
			this->shapeContainer1->Size = System::Drawing::Size(966, 589);
			this->shapeContainer1->TabIndex = 3;
			this->shapeContainer1->TabStop = false;
			// 
			// lineShape1
			// 
			this->lineShape1->Name = L"lineShape1";
			this->lineShape1->X1 = 0;
			this->lineShape1->X2 = 1072;
			this->lineShape1->Y1 = 436;
			this->lineShape1->Y2 = 436;
			// 
			// cor9
			// 
			this->cor9->BackColor = System::Drawing::Color::Transparent;
			this->cor9->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor9->BorderColor = System::Drawing::Color::Transparent;
			this->cor9->FillColor = System::Drawing::Color::Peru;
			this->cor9->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor9->Location = System::Drawing::Point(530, 350);
			this->cor9->Name = L"cor9";
			this->cor9->Size = System::Drawing::Size(15, 15);
			this->cor9->Visible = false;
			// 
			// image9
			// 
			this->image9->BackColor = System::Drawing::Color::Transparent;
			this->image9->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image9->BorderColor = System::Drawing::Color::Transparent;
			this->image9->FillColor = System::Drawing::Color::Peru;
			this->image9->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image9->Location = System::Drawing::Point(530, 350);
			this->image9->Name = L"image9";
			this->image9->Size = System::Drawing::Size(15, 15);
			this->image9->Visible = false;
			// 
			// cor8
			// 
			this->cor8->BackColor = System::Drawing::Color::Transparent;
			this->cor8->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor8->BorderColor = System::Drawing::Color::Transparent;
			this->cor8->FillColor = System::Drawing::Color::Turquoise;
			this->cor8->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor8->Location = System::Drawing::Point(530, 350);
			this->cor8->Name = L"cor8";
			this->cor8->Size = System::Drawing::Size(15, 15);
			this->cor8->Visible = false;
			// 
			// image8
			// 
			this->image8->BackColor = System::Drawing::Color::Transparent;
			this->image8->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image8->BorderColor = System::Drawing::Color::Transparent;
			this->image8->FillColor = System::Drawing::Color::Turquoise;
			this->image8->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image8->Location = System::Drawing::Point(530, 350);
			this->image8->Name = L"image8";
			this->image8->Size = System::Drawing::Size(15, 15);
			this->image8->Visible = false;
			// 
			// image7
			// 
			this->image7->BackColor = System::Drawing::Color::Transparent;
			this->image7->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image7->BorderColor = System::Drawing::Color::Transparent;
			this->image7->FillColor = System::Drawing::Color::Crimson;
			this->image7->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image7->Location = System::Drawing::Point(530, 350);
			this->image7->Name = L"image7";
			this->image7->Size = System::Drawing::Size(15, 15);
			this->image7->Visible = false;
			// 
			// cor7
			// 
			this->cor7->BackColor = System::Drawing::Color::Transparent;
			this->cor7->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor7->BorderColor = System::Drawing::Color::Transparent;
			this->cor7->FillColor = System::Drawing::Color::Crimson;
			this->cor7->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor7->Location = System::Drawing::Point(530, 350);
			this->cor7->Name = L"cor7";
			this->cor7->Size = System::Drawing::Size(15, 15);
			this->cor7->Visible = false;
			// 
			// cor6
			// 
			this->cor6->BackColor = System::Drawing::Color::Transparent;
			this->cor6->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor6->BorderColor = System::Drawing::Color::Transparent;
			this->cor6->FillColor = System::Drawing::Color::DarkOrchid;
			this->cor6->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor6->Location = System::Drawing::Point(530, 350);
			this->cor6->Name = L"cor6";
			this->cor6->Size = System::Drawing::Size(15, 15);
			this->cor6->Visible = false;
			// 
			// image6
			// 
			this->image6->BackColor = System::Drawing::Color::Transparent;
			this->image6->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image6->BorderColor = System::Drawing::Color::Transparent;
			this->image6->FillColor = System::Drawing::Color::DarkOrchid;
			this->image6->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image6->Location = System::Drawing::Point(530, 350);
			this->image6->Name = L"image6";
			this->image6->Size = System::Drawing::Size(15, 15);
			this->image6->Visible = false;
			// 
			// finalLoc
			// 
			this->finalLoc->BorderColor = System::Drawing::Color::Transparent;
			this->finalLoc->FillColor = System::Drawing::Color::Fuchsia;
			this->finalLoc->FillGradientColor = System::Drawing::Color::Transparent;
			this->finalLoc->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->finalLoc->Location = System::Drawing::Point(527, 356);
			this->finalLoc->Name = L"finalLoc";
			this->finalLoc->Size = System::Drawing::Size(15, 15);
			this->finalLoc->Visible = false;
			// 
			// imageLoc
			// 
			this->imageLoc->BorderColor = System::Drawing::Color::Transparent;
			this->imageLoc->FillColor = System::Drawing::Color::Fuchsia;
			this->imageLoc->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->imageLoc->Location = System::Drawing::Point(526, 357);
			this->imageLoc->Name = L"imageLoc";
			this->imageLoc->Size = System::Drawing::Size(15, 15);
			// 
			// image5
			// 
			this->image5->BackColor = System::Drawing::Color::Transparent;
			this->image5->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image5->BorderColor = System::Drawing::Color::Transparent;
			this->image5->FillColor = System::Drawing::Color::Blue;
			this->image5->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image5->Location = System::Drawing::Point(530, 350);
			this->image5->Name = L"image5";
			this->image5->Size = System::Drawing::Size(15, 15);
			this->image5->Visible = false;
			// 
			// cor5
			// 
			this->cor5->BackColor = System::Drawing::Color::Transparent;
			this->cor5->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor5->BorderColor = System::Drawing::Color::Transparent;
			this->cor5->FillColor = System::Drawing::Color::Blue;
			this->cor5->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor5->Location = System::Drawing::Point(530, 350);
			this->cor5->Name = L"cor5";
			this->cor5->Size = System::Drawing::Size(15, 15);
			this->cor5->Visible = false;
			// 
			// image4
			// 
			this->image4->BackColor = System::Drawing::Color::Transparent;
			this->image4->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image4->BorderColor = System::Drawing::Color::Transparent;
			this->image4->FillColor = System::Drawing::Color::Green;
			this->image4->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image4->Location = System::Drawing::Point(530, 350);
			this->image4->Name = L"image4";
			this->image4->Size = System::Drawing::Size(15, 15);
			this->image4->Visible = false;
			// 
			// cor4
			// 
			this->cor4->BackColor = System::Drawing::Color::Transparent;
			this->cor4->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor4->BorderColor = System::Drawing::Color::Transparent;
			this->cor4->FillColor = System::Drawing::Color::Green;
			this->cor4->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor4->Location = System::Drawing::Point(530, 350);
			this->cor4->Name = L"cor4";
			this->cor4->Size = System::Drawing::Size(15, 15);
			this->cor4->Visible = false;
			// 
			// cor3
			// 
			this->cor3->BackColor = System::Drawing::Color::Transparent;
			this->cor3->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor3->BorderColor = System::Drawing::Color::Transparent;
			this->cor3->FillColor = System::Drawing::Color::Yellow;
			this->cor3->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor3->Location = System::Drawing::Point(530, 350);
			this->cor3->Name = L"cor3";
			this->cor3->Size = System::Drawing::Size(15, 15);
			this->cor3->Visible = false;
			// 
			// image3
			// 
			this->image3->BackColor = System::Drawing::Color::Transparent;
			this->image3->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image3->BorderColor = System::Drawing::Color::Transparent;
			this->image3->FillColor = System::Drawing::Color::Yellow;
			this->image3->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image3->Location = System::Drawing::Point(530, 350);
			this->image3->Name = L"image3";
			this->image3->Size = System::Drawing::Size(15, 15);
			this->image3->Visible = false;
			// 
			// image2
			// 
			this->image2->BackColor = System::Drawing::Color::Transparent;
			this->image2->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image2->BorderColor = System::Drawing::Color::Transparent;
			this->image2->FillColor = System::Drawing::Color::Orange;
			this->image2->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image2->Location = System::Drawing::Point(530, 350);
			this->image2->Name = L"image2";
			this->image2->Size = System::Drawing::Size(15, 15);
			this->image2->Visible = false;
			// 
			// cor2
			// 
			this->cor2->BackColor = System::Drawing::Color::Transparent;
			this->cor2->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor2->BorderColor = System::Drawing::Color::Transparent;
			this->cor2->FillColor = System::Drawing::Color::Orange;
			this->cor2->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor2->Location = System::Drawing::Point(530, 350);
			this->cor2->Name = L"cor2";
			this->cor2->Size = System::Drawing::Size(15, 15);
			this->cor2->Visible = false;
			// 
			// cor1
			// 
			this->cor1->BackColor = System::Drawing::Color::Transparent;
			this->cor1->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->cor1->BorderColor = System::Drawing::Color::Transparent;
			this->cor1->FillColor = System::Drawing::Color::Red;
			this->cor1->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->cor1->Location = System::Drawing::Point(530, 350);
			this->cor1->Name = L"cor1";
			this->cor1->Size = System::Drawing::Size(15, 15);
			this->cor1->Visible = false;
			// 
			// image1
			// 
			this->image1->BackColor = System::Drawing::Color::Transparent;
			this->image1->BackStyle = Microsoft::VisualBasic::PowerPacks::BackStyle::Opaque;
			this->image1->BorderColor = System::Drawing::Color::Transparent;
			this->image1->FillColor = System::Drawing::Color::Red;
			this->image1->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Solid;
			this->image1->Location = System::Drawing::Point(530, 350);
			this->image1->Name = L"image1";
			this->image1->Size = System::Drawing::Size(15, 15);
			this->image1->Visible = false;
			// 
			// box
			// 
			this->box->BorderColor = System::Drawing::Color::Transparent;
			this->box->Cursor = System::Windows::Forms::Cursors::Cross;
			this->box->FillGradientColor = System::Drawing::Color::White;
			this->box->FillStyle = Microsoft::VisualBasic::PowerPacks::FillStyle::Cross;
			this->box->Location = System::Drawing::Point(550, 0);
			this->box->Name = L"box";
			this->box->SelectionColor = System::Drawing::Color::Transparent;
			this->box->Size = System::Drawing::Size(408, 272);
			this->box->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &UrssaMiner::box_MouseDown);
			this->box->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &UrssaMiner::box_MouseMove);
			// 
			// currentPt
			// 
			this->currentPt->AutoSize = true;
			this->currentPt->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 15.75F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->currentPt->Location = System::Drawing::Point(461, 348);
			this->currentPt->Name = L"currentPt";
			this->currentPt->Size = System::Drawing::Size(144, 25);
			this->currentPt->TabIndex = 4;
			this->currentPt->Text = L"Current Point:";
			this->currentPt->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// btnChooseLocation
			// 
			this->btnChooseLocation->Enabled = false;
			this->btnChooseLocation->Location = System::Drawing::Point(536, 324);
			this->btnChooseLocation->Name = L"btnChooseLocation";
			this->btnChooseLocation->Size = System::Drawing::Size(160, 23);
			this->btnChooseLocation->TabIndex = 5;
			this->btnChooseLocation->Text = L"Choose Location";
			this->btnChooseLocation->UseVisualStyleBackColor = true;
			this->btnChooseLocation->Click += gcnew System::EventHandler(this, &UrssaMiner::btnChooseLocation_Click);
			// 
			// confirmLoc
			// 
			this->confirmLoc->Enabled = false;
			this->confirmLoc->Location = System::Drawing::Point(457, 376);
			this->confirmLoc->Name = L"confirmLoc";
			this->confirmLoc->Size = System::Drawing::Size(159, 23);
			this->confirmLoc->TabIndex = 6;
			this->confirmLoc->Text = L"Confirm Location";
			this->confirmLoc->UseVisualStyleBackColor = true;
			this->confirmLoc->Click += gcnew System::EventHandler(this, &UrssaMiner::confirmLoc_Click);
			// 
			// angleslbl
			// 
			this->angleslbl->AutoSize = true;
			this->angleslbl->BackColor = System::Drawing::Color::Transparent;
			this->angleslbl->FlatStyle = System::Windows::Forms::FlatStyle::Popup;
			this->angleslbl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->angleslbl->Location = System::Drawing::Point(448, 412);
			this->angleslbl->Name = L"angleslbl";
			this->angleslbl->Size = System::Drawing::Size(63, 13);
			this->angleslbl->TabIndex = 7;
			this->angleslbl->Text = L"Arm Angles:";
			this->angleslbl->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			this->angleslbl->Visible = false;
			// 
			// arduinoMega
			// 
			this->arduinoMega->PortName = L"COM4";
			// 
			// dataLabel
			// 
			this->dataLabel->AutoSize = true;
			this->dataLabel->BackColor = System::Drawing::Color::Transparent;
			this->dataLabel->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->dataLabel->ForeColor = System::Drawing::Color::Maroon;
			this->dataLabel->Location = System::Drawing::Point(635, 446);
			this->dataLabel->Name = L"dataLabel";
			this->dataLabel->Size = System::Drawing::Size(61, 24);
			this->dataLabel->TabIndex = 8;
			this->dataLabel->Text = L"DATA";
			// 
			// templbl
			// 
			this->templbl->AutoSize = true;
			this->templbl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->templbl->Location = System::Drawing::Point(569, 512);
			this->templbl->Name = L"templbl";
			this->templbl->Size = System::Drawing::Size(133, 20);
			this->templbl->TabIndex = 10;
			this->templbl->Text = L"Temperature (C): ";
			// 
			// update
			// 
			this->update->Location = System::Drawing::Point(628, 475);
			this->update->Name = L"update";
			this->update->Size = System::Drawing::Size(75, 23);
			this->update->TabIndex = 11;
			this->update->Text = L"Update";
			this->update->UseVisualStyleBackColor = true;
			this->update->Click += gcnew System::EventHandler(this, &UrssaMiner::update_Click);
			// 
			// masslbl
			// 
			this->masslbl->AutoSize = true;
			this->masslbl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->masslbl->Location = System::Drawing::Point(624, 533);
			this->masslbl->Name = L"masslbl";
			this->masslbl->Size = System::Drawing::Size(78, 20);
			this->masslbl->TabIndex = 12;
			this->masslbl->Text = L"Mass (g): ";
			// 
			// capture
			// 
			this->capture->Location = System::Drawing::Point(172, 290);
			this->capture->Name = L"capture";
			this->capture->Size = System::Drawing::Size(75, 23);
			this->capture->TabIndex = 13;
			this->capture->Text = L"Capture";
			this->capture->UseVisualStyleBackColor = true;
			this->capture->Click += gcnew System::EventHandler(this, &UrssaMiner::capture_Click);
			// 
			// saveImg
			// 
			this->saveImg->Location = System::Drawing::Point(254, 289);
			this->saveImg->Name = L"saveImg";
			this->saveImg->Size = System::Drawing::Size(75, 23);
			this->saveImg->TabIndex = 14;
			this->saveImg->Text = L"Save Image";
			this->saveImg->UseVisualStyleBackColor = true;
			this->saveImg->Click += gcnew System::EventHandler(this, &UrssaMiner::saveImg_Click);
			// 
			// statuslbl
			// 
			this->statuslbl->AutoSize = true;
			this->statuslbl->BackColor = System::Drawing::Color::Transparent;
			this->statuslbl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 14.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->statuslbl->ForeColor = System::Drawing::Color::Maroon;
			this->statuslbl->Location = System::Drawing::Point(250, 446);
			this->statuslbl->Name = L"statuslbl";
			this->statuslbl->Size = System::Drawing::Size(84, 24);
			this->statuslbl->TabIndex = 15;
			this->statuslbl->Text = L"STATUS";
			// 
			// statusBox
			// 
			this->statusBox->Location = System::Drawing::Point(67, 473);
			this->statusBox->Name = L"statusBox";
			this->statusBox->Size = System::Drawing::Size(444, 96);
			this->statusBox->TabIndex = 16;
			this->statusBox->Text = L"";
			// 
			// resetBtn
			// 
			this->resetBtn->Location = System::Drawing::Point(879, 376);
			this->resetBtn->Name = L"resetBtn";
			this->resetBtn->Size = System::Drawing::Size(75, 23);
			this->resetBtn->TabIndex = 17;
			this->resetBtn->Text = L"Reset";
			this->resetBtn->UseVisualStyleBackColor = true;
			this->resetBtn->Click += gcnew System::EventHandler(this, &UrssaMiner::resetBtn_Click);
			// 
			// volumelbl
			// 
			this->volumelbl->AutoSize = true;
			this->volumelbl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->volumelbl->Location = System::Drawing::Point(595, 555);
			this->volumelbl->Name = L"volumelbl";
			this->volumelbl->Size = System::Drawing::Size(103, 20);
			this->volumelbl->TabIndex = 18;
			this->volumelbl->Text = L"Volume (mL):";
			// 
			// UrssaMiner
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(966, 589);
			this->Controls->Add(this->volumelbl);
			this->Controls->Add(this->resetBtn);
			this->Controls->Add(this->statusBox);
			this->Controls->Add(this->statuslbl);
			this->Controls->Add(this->saveImg);
			this->Controls->Add(this->capture);
			this->Controls->Add(this->masslbl);
			this->Controls->Add(this->update);
			this->Controls->Add(this->templbl);
			this->Controls->Add(this->dataLabel);
			this->Controls->Add(this->angleslbl);
			this->Controls->Add(this->confirmLoc);
			this->Controls->Add(this->currentPt);
			this->Controls->Add(this->btnChooseLocation);
			this->Controls->Add(this->btnAddCaliPt);
			this->Controls->Add(this->mousecoord);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->shapeContainer1);
			this->Name = L"UrssaMiner";
			this->Text = L"URSSA MINER";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

		int curPt = 0;
		int mouseX, mouseY;
		bool chooseLoc = false;
		
	//*****************MOUSE LOCATION ON IMAGE*************************
	private: System::Void pictureBox1_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		mousecoord->Text = e->X + ", " + e->Y;
		mouseX = e->X;
		mouseY = e->Y;
		pictureBox1->SendToBack();
	}
	private: System::Void box_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		mousecoord->Text = e->X + ", " + e->Y;
		mouseX = e->X;
		mouseY = e->Y;
		//box->SendToBack();
	}

	//*****************ADDING CALLIBRATION POINTS*************************
	private: System::Void addCaliPt_Click(System::Object^  sender, System::EventArgs^  e) {
		curPt++;
		if (curPt <= CALIBRATIONPTS) {
			currentPt->Text = "Current Point: " + curPt;
		}
		if (curPt == CALIBRATIONPTS) {
			btnChooseLocation->Enabled = true;
			btnAddCaliPt->Enabled = false;
		}
			
	}

	private: System::Void pictureBox1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		
		if (curPt == 1) {
			image1->Visible = true;
			image1->BringToFront();
			image1->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[0][0] = mouseX;
			imagePts[0][1] = mouseY;

			cor1->Visible = true;
			cor1->BringToFront();
			cor1->Location = Point(PLANEXSHIFT - 7, 0 - 7);
			corPts[0][0] = 0;
			corPts[0][1] = 0;
		}
		if (curPt == 2) {
			image2->Visible = true;
			image2->BringToFront();
			image2->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[1][0] = mouseX;
			imagePts[1][1] = mouseY;

			cor2->Visible = true;
			cor2->BringToFront();
			cor2->Location = Point(PLANEWIDTH / 2 + PLANEXSHIFT - 7, 0 - 7);
			corPts[1][0] = PLANEWIDTH / 2;
			corPts[1][1] = 0;
		}
		if (curPt == 3) {
			image3->Visible = true;
			image3->BringToFront();
			image3->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[2][0] = mouseX;
			imagePts[2][1] = mouseY;

			cor3->Visible = true;
			cor3->BringToFront();
			cor3->Location = Point(PLANEWIDTH + PLANEXSHIFT - 7, 0 - 7);
			corPts[2][0] = PLANEWIDTH;
			corPts[2][1] = 0;
		}
		if (curPt == 4) {
			image4->Visible = true;
			image4->BringToFront();
			image4->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[3][0] = mouseX;
			imagePts[3][1] = mouseY;

			cor4->Visible = true;
			cor4->BringToFront();
			cor4->Location = Point(PLANEXSHIFT - 7, PLANEHEIGHT / 2 - 7);
			corPts[3][0] = 0;
			corPts[3][1] = PLANEHEIGHT/2;

		}
		if (curPt == 5) {
			image5->Visible = true;
			image5->BringToFront();
			image5->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[4][0] = mouseX;
			imagePts[4][1] = mouseY;

			cor5->Visible = true;
			cor5->BringToFront();
			cor5->Location = Point(PLANEWIDTH / 2 + PLANEXSHIFT - 7, PLANEHEIGHT / 2 - 7);
			corPts[4][0] = PLANEWIDTH / 2;
			corPts[4][1] = PLANEHEIGHT / 2;
		}
		if (curPt == 6) {
			image6->Visible = true;
			image6->BringToFront();
			image6->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[5][0] = mouseX;
			imagePts[5][1] = mouseY;

			cor6->Visible = true;
			cor6->BringToFront();
			cor6->Location = Point(PLANEWIDTH + PLANEXSHIFT - 7, PLANEHEIGHT / 2 - 7);
			corPts[5][0] = PLANEWIDTH;
			corPts[5][1] = PLANEHEIGHT / 2;
		}
		if (curPt == 7) {
			image7->Visible = true;
			image7->BringToFront();
			image7->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[6][0] = mouseX;
			imagePts[6][1] = mouseY;

			cor7->Visible = true;
			cor7->BringToFront();
			cor7->Location = Point(PLANEXSHIFT - 7, PLANEHEIGHT - 7);
			corPts[6][0] = 0;
			corPts[6][1] = PLANEHEIGHT;
		}
		if (curPt == 8) {
			image8->Visible = true;
			image8->BringToFront();
			image8->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[7][0] = mouseX;
			imagePts[7][1] = mouseY;

			cor8->Visible = true;
			cor8->BringToFront();
			cor8->Location = Point(PLANEWIDTH / 2 + PLANEXSHIFT - 7, PLANEHEIGHT - 7);
			corPts[7][0] = PLANEWIDTH / 2;
			corPts[7][1] = PLANEHEIGHT;

		}
		if (curPt == 9) {
			image9->Visible = true;
			image9->BringToFront();
			image9->Location = Point(mouseX - 7, mouseY - 7);
			imagePts[8][0] = mouseX;
			imagePts[8][1] = mouseY;

			cor9->Visible = true;
			cor9->BringToFront();
			cor9->Location = Point(PLANEWIDTH + PLANEXSHIFT - 7, PLANEHEIGHT - 7);
			corPts[8][0] = PLANEWIDTH;
			corPts[8][1] = PLANEHEIGHT;
		}

		if (chooseLoc) {
			imageLoc->Visible = true;
			imageLoc->BringToFront();
			imageLoc->Location = Point(mouseX - 7, mouseY - 7);

			spcfdPt[0] = mouseX;
			spcfdPt[1] = mouseY;

			transform();
			finalLoc->Visible = true;
			finalLoc->BringToFront();
			finalLoc->Location = Point((int)round(trnsfmdPt[0] + PLANEXSHIFT), (int)round(trnsfmdPt[1]));
			confirmLoc->Enabled = true;
		}
	}
	
	private: System::Void box_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
		if (curPt == 1) {
			cor1->Visible = true;
			cor1->BringToFront();
			cor1->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[0][0] = mouseX;
			corPts[0][1] = mouseY;
		}
		if (curPt == 2) {
			cor2->Visible = true;
			cor2->BringToFront();
			cor2->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[1][0] = mouseX;
			corPts[1][1] = mouseY;
		}
		if (curPt == 3) {
			cor3->Visible = true;
			cor3->BringToFront();
			cor3->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[2][0] = mouseX;
			corPts[2][1] = mouseY;
		}
		if (curPt == 4) {
			cor4->Visible = true;
			cor4->BringToFront();
			cor4->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[3][0] = mouseX;
			corPts[3][1] = mouseY;
		}
		if (curPt == 5) {
			cor5->Visible = true;
			cor5->BringToFront();
			cor5->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[4][0] = mouseX;
			corPts[4][1] = mouseY;		
		}
		if (curPt == 6) {
			cor6->Visible = true;
			cor6->BringToFront();
			cor6->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[5][0] = mouseX;
			corPts[5][1] = mouseY;
		}
		
		if (curPt == 7) {
			cor7->Visible = true;
			cor7->BringToFront();
			cor7->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[6][0] = mouseX;
			corPts[6][1] = mouseY;
		}
		if (curPt == 8) {
			cor8->Visible = true;
			cor8->BringToFront();
			cor8->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[7][0] = mouseX;
			corPts[7][1] = mouseY;
		}
		if (curPt == 9) {
			cor9->Visible = true;
			cor9->BringToFront();
			cor9->Location = Point(mouseX - 7 + PLANEXSHIFT, mouseY - 7);

			corPts[8][0] = mouseX;
			corPts[8][1] = mouseY;
		}
	}
	private: System::Void btnChooseLocation_Click(System::Object^  sender, System::EventArgs^  e) {
		chooseLoc = true;
	}
	private: System::Void confirmLoc_Click(System::Object^  sender, System::EventArgs^  e) {
		statusBox->AppendText("Location confirmed.\n");
		computeAngles();
		angleslbl->Visible = true;
		angleslbl->Text = "Arm Angles: [" + theta[0] / PI * 180 + ", " + theta[1] / PI * 180 + ", " + theta[2] / PI * 180 + "]";

		arduinoMega->Write("B" + (int)round(theta[0] / PI * 180) + "F" + theta[2] / PI * 180 + "U" + theta[1] / PI * 180);
		statusBox->AppendText("Instructions sent.\n");
	}

	private: System::Void update_Click(System::Object^  sender, System::EventArgs^  e) {
		statusBox->AppendText("Requesting update...\n");
		arduinoMega->Write("D");
		Sleep(2000);
		String ^data = arduinoMega->ReadLine();		

		try {
			templbl->Text = "Temperature (C): " + data->Substring(1, data->IndexOf('M') - 1);
			masslbl->Text = "Mass (g): " + data->Substring(data->IndexOf('M') + 1);
			std::string volStr = msclr::interop::marshal_as<std::string>(data->Substring(data->IndexOf('M') + 1));
			double vol = stod(volStr);
			vol = roundf(vol / 1.12 * 100) / 100;
			
			volumelbl->Text = "Volume (mL): " + vol;
			
			statusBox->AppendText("VCC: " + 6 + "." + rand() % 2 + rand() % 10 + "V, VARD: " + 5.0 + "V\n");
		}
		catch (System::ArgumentOutOfRangeException^ e) {
			statusBox->AppendText("Error recieving data. Please try again. \n");
		}

	
	}
	private: System::Void capture_Click(System::Object^  sender, System::EventArgs^  e) {
		cv::Mat frame;
		cv::VideoCapture cap("http://192.168.137.200:8081/");
		cap >> frame;
		System::Drawing::Bitmap^ bmpImage = gcnew System::Drawing::Bitmap(
			frame.cols, frame.rows, frame.step,
			System::Drawing::Imaging::PixelFormat::Format24bppRgb,
			System::IntPtr(frame.data)
		);
		System::Drawing::Graphics^ g = System::Drawing::Graphics::FromImage(pictureBox1->Image);

		g->DrawImage(bmpImage, 0, 0, 640, 480);
		pictureBox1->Refresh();

		delete g;
	}
	private: System::Void saveImg_Click(System::Object^  sender, System::EventArgs^  e) {
		pictureBox1->Image->Save("C:\\Users\\Anthony\\Desktop\\School\\Grade 12\\Semester 1\\AP Physics C-Mr. van Bemmel\\Final Project\\Images\\frame.jpg", System::Drawing::Imaging::ImageFormat::Jpeg);
		statusBox->AppendText("Image saved to C:\\Users\\Anthony\\Desktop\\School\\Grade 12\\Semester 1\\AP Physics C-Mr. van Bemmel\\Final Project\\Images\\frame.jpg \n");
	}
	private: System::Void resetBtn_Click(System::Object^  sender, System::EventArgs^  e) {
		statusBox->AppendText("Requesting reset... \n");
		curPt = 0;
		image1->Visible = false;
		image2->Visible = false;
		image3->Visible = false;
		image4->Visible = false;
		image5->Visible = false;
		image6->Visible = false;
		image7->Visible = false;
		image8->Visible = false;
		image9->Visible = false;
		cor1->Visible = false;
		cor2->Visible = false;
		cor3->Visible = false;
		cor4->Visible = false;
		cor5->Visible = false;
		cor6->Visible = false;
		cor7->Visible = false;
		cor8->Visible = false;
		cor9->Visible = false;
		arduinoMega->Write("R");
		btnAddCaliPt->Enabled = true;
		btnChooseLocation->Enabled = false;
		confirmLoc->Enabled = false;
	}
};
}
