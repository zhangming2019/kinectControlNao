
#include <conio.h>
#include <Windows.h>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <NuiApi.h>
#include <KinectInteraction.h>
#include <fstream>
#include <atlstr.h>
#include <strsafe.h>
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alerror/alerror.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <qi/os.hpp>
//#include "Matrix.h"
using namespace std;
using namespace cv;
using namespace Eigen;
using Eigen::MatrixXd;
const double pi = 3.14159265358979323846;
#define SafeRelease(X) if(X) delete X;
//----------------------------------------------------
#define _WINDOWS
INuiSensor            *m_pNuiSensor;
string stateStr[2] = {"none","none"};
INuiInteractionStream *m_nuiIStream;




class CIneractionClient:public INuiInteractionClient
{
public:
	CIneractionClient(){;}
	~CIneractionClient(){;}
	STDMETHOD(GetInteractionInfoAtLocation)(THIS_ DWORD skeletonTrackingId, NUI_HAND_TYPE handType, FLOAT x, FLOAT y, _Out_ NUI_INTERACTION_INFO *pInteractionInfo)
	{        
		if(pInteractionInfo)
		{
			pInteractionInfo->IsPressTarget         = FALSE;//must add
			pInteractionInfo->IsGripTarget          = TRUE;    //must add
 			pInteractionInfo->PressTargetControlId  = 0;
 			pInteractionInfo->PressAttractionPointX = 0.f;
 			pInteractionInfo->PressAttractionPointY = 0.f;
			return S_OK;
		}
		return E_POINTER;
		return S_OK; 
	}
 
	STDMETHODIMP_(ULONG)    AddRef()                                    { return 2;     }
	STDMETHODIMP_(ULONG)    Release()                                   { return 1;     }
	STDMETHODIMP            QueryInterface(REFIID riid, void **ppv)     { return S_OK;  }
};
CIneractionClient m_nuiIClient;







//--------------------------------------------------------------------
HANDLE m_hNextColorFrameEvent;
HANDLE m_hNextDepthFrameEvent;
HANDLE m_hNextSkeletonEvent	 ;
HANDLE m_hNextInteractionEvent;
HANDLE m_pColorStreamHandle	  ;
HANDLE m_pDepthStreamHandle  ;
HANDLE m_hEvNuiProcessStop   ;
NUI_IMAGE_VIEW_AREA area     ;
//DepthImageFormat depthImageFormat = Resolution640x480Fps30;
//-----------------------------------------------------------------------------------

// *********************************nao robot****************************************//
const float ShoulderToElbowLength = 105.00 ;
const float ElbowToWristLength = 55.95;
const float WristToHandLength = 57.75;
float LShouldToWrist[3] ={0,0,0};
float LShouldToHand[3] ={0,0,0};
float LShouldToElbow[3] ={0,0,0};
float LElbowToWrist[3] = {0,0,0};
float RShouldToWrist[3] ={0,0,0};
float RShouldToElbow[3] ={0,0,0};
float RElbowToWrist[3] = {0,0,0};
float resultPosition[3] = {0,0,0};
float l1 = 0.015;
float l2 = 0.105;
float l3 = 0.055;
const float FrameError = 0.0001;
const float FrameMaxError = 0.7;
bool DEBUGE = false;
String connectNao= "10.10.10.100";
AL::ALMotionProxy motion(connectNao, 9559);
double HAngle0 = 0.0;
double HAngle1 = 0.0;
double HAngle2 = 0.0;
double HAngle3 = 0.0;
float angle0 = 1.44242,angle1,angle2,angle3,angle4,angle5,angle6,angle7,angle8,angle9,angle10;
//commandAngles[1.44242, 0.223037, -1.20182, -0.418269, 0.3, 1.44262, -0.22326, 1.20182, 0.418131, 0.3]
double LShoulderPitchResult = 1.44242 ,LShoulderRollResult =0.223037,LElboYawResult = -1.20182,LElbowRollResult =-0.418269,LHand = 0.3;
double RShoulderPitchResult = 1.44262 ,RShoulderRollResult = -0.22326,RElboYawResult = 1.20182,RElbowRollResult = 0.418131,RHand =  0.3;
double bufferAngle0[5]={0,0,0,0,0};
double bufferAngle1[5]={0,0,0,0,0};
double bufferAngle2[5]={0,0,0,0,0};
double bufferAngle3[5]={0,0,0,0,0};
bool startFilter = false;
int indexBuffer=0;
Vector3d fkine(vector<float> commandAngles);
bool NewtonIK = true;
Matrix3d unit = Matrix3d::Identity(3,3);
bool transfer;
// ************************************动力学雅可比矩阵****************************** //
struct DHMatrix{
	float jointAngle;
	float linkOffset;
	float linkLenth;
	float linkTwist;
	Matrix4d Transformation;
	Matrix4d RotzAngle;
	Matrix4d TranszLenth;
	Matrix4d TransxAngle;
	Matrix4d RotxAngle;
	void setDHParam(float jointAngleFlag,float linkOffsetFlag,float linkLenthFlag,float linkTwistFlag ){
		jointAngle = jointAngleFlag;
		linkOffset = linkOffsetFlag;
		linkLenth = linkLenthFlag;
		linkTwist = linkTwistFlag;
	}
	Matrix4d setTransformation(){
		RotzAngle(0,0) = cos(jointAngle);
		RotzAngle(0,1) = -sin(jointAngle);
		RotzAngle(0,2) = 0;
		RotzAngle(0,3) = 0;
		RotzAngle(1,0) = sin(jointAngle);
		RotzAngle(1,1) = cos(jointAngle);
		RotzAngle(1,2) = 0;
		RotzAngle(1,3) = 0;
		RotzAngle(2,0) = 0;
		RotzAngle(2,1) = 0;
		RotzAngle(2,2) = 1;
		RotzAngle(2,3) = 0;
		RotzAngle(3,0) = 0;
		RotzAngle(3,1) = 0;
		RotzAngle(3,2) = 0;
		RotzAngle(3,3) = 1;

		TranszLenth(0,0) = 1;
		TranszLenth(0,1) = 0;
		TranszLenth(0,2) = 0;
		TranszLenth(0,3) = 0;
		TranszLenth(1,0) = 0;
		TranszLenth(1,1) = 1;
		TranszLenth(1,2) = 0;
		TranszLenth(1,3) = 0;
		TranszLenth(2,0) = 0;
		TranszLenth(2,1) = 0;
		TranszLenth(2,2) = 1;
		TranszLenth(2,3) = linkOffset;
		TranszLenth(3,0) = 0;
		TranszLenth(3,1) = 0;
		TranszLenth(3,2) = 0;
		TranszLenth(3,3) = 1;

		TransxAngle(0,0) = 1;
		TransxAngle(0,1) = 0;
		TransxAngle(0,2) = 0;
		TransxAngle(0,3) = linkLenth;
		TransxAngle(1,0) = 0;
		TransxAngle(1,1) = 1;
		TransxAngle(1,2) = 0;
		TransxAngle(1,3) = 0;
		TransxAngle(2,0) = 0;
		TransxAngle(2,1) = 0;
		TransxAngle(2,2) = 1;
		TransxAngle(2,3) = 0;
		TransxAngle(3,0) = 0;
		TransxAngle(3,1) = 0;
		TransxAngle(3,2) = 0;
		TransxAngle(3,3) = 1;

		RotxAngle(0,0) = 1;
		RotxAngle(0,1) = 0;
		RotxAngle(0,2) = 0;
		RotxAngle(0,3) = 0;
		RotxAngle(1,0) = 0;
		RotxAngle(1,1) = cos(linkTwist);
		RotxAngle(1,2) = -sin(linkTwist);
		RotxAngle(1,3) = 0;
		RotxAngle(2,0) = 0;
		RotxAngle(2,1) = sin(linkTwist);
		RotxAngle(2,2) = cos(linkTwist);
		RotxAngle(2,3) = 0;
		RotxAngle(3,0) = 0;
		RotxAngle(3,1) = 0;
		RotxAngle(3,2) = 0;
		RotxAngle(3,3) = 1;
		Transformation = RotzAngle * TranszLenth *TransxAngle*RotxAngle;
		return Transformation;
	}
};
double round(double r)  
{  
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);  
}  
double conventAngle(double angleTemp){
	
	double temp = angleTemp / 2 / pi;
	temp = round(temp);
	double angleChange = angleTemp - temp * 2 * pi; 
	return angleChange;
}
void testconventAngle(){
	double ceshi1 = 1.7865;
	cout << "conventAngle" <<conventAngle(1.7865)<<endl;
	cout << sin(ceshi1) << "=="<<sin(conventAngle(1.7865))<< endl;
	
	 ceshi1 = -1.7865;
	cout << "conventAngle" <<conventAngle(-1.7865)<<endl;
	cout << sin(ceshi1) << "=="<<sin(conventAngle(-1.7865))<< endl;


}

MatrixXd getLeftHandJacobian(float Angle0,float Angle1,float Angle2,float Angle3){
	MatrixXd jacobian(3,4);
	jacobian(0,0) = l3*cos(Angle0)*sin(Angle2)*sin(Angle3) +\
		l3*sin(Angle0)*sin(Angle1)*cos(Angle2)*sin(Angle3) -\
		l3*cos(Angle3)*sin(Angle0)*cos(Angle1) - l2*cos(Angle1)*sin(Angle0) + l1*sin(Angle0)*sin(Angle1);
	jacobian(0,1) = -l3*cos(Angle0)*cos(Angle1)*cos(Angle2)*sin(Angle3) -l3*cos(Angle3)*cos(Angle0)\
		*sin(Angle1) - l2*cos(angle0)*sin(angle1)-l1*cos(Angle0)*cos(Angle1);
	jacobian(0,2) = l3*sin(Angle0)*cos(angle2)*sin(Angle3) + l3*cos(Angle0)*sin(Angle1)*sin(Angle2)*sin(Angle3);
	jacobian(0,3) = l3*sin(Angle0)*sin(Angle2)*cos(Angle3) -l3 *cos(Angle0)*sin(Angle1)*cos(Angle2)*cos(Angle3)\
		-l3*cos(Angle1)*cos(Angle0)*sin(Angle3);
	jacobian(1,0) = -l3*cos(Angle0)*sin(Angle1)*cos(Angle2)*sin(Angle3)+l3*sin(Angle0)*cos(Angle2)*sin(Angle3)\
		+l3*cos(Angle0)*cos(Angle1)*cos(Angle3);
	jacobian(1,1) = -l3*sin(Angle0)*cos(Angle1)*cos(Angle2)*sin(Angle3) - l3*sin(Angle0)*sin(Angle1)*cos(Angle3)-\
		l2*sin(Angle0)*sin(Angle1) -l1*sin(Angle0)*cos(Angle1);
	jacobian(1,2) = l3*sin(Angle0)*sin(Angle1)*sin(Angle2)*sin(Angle3)-l3*cos(Angle0)*cos(Angle2)*sin(Angle3);
	jacobian(1,3) = -l3*sin(Angle0)*sin(Angle1)*cos(Angle2)*cos(Angle3)\
		-l3*cos(Angle0)*sin(Angle2)*cos(Angle3)-l3*sin(Angle0)*cos(Angle1)*sin(Angle3);

	jacobian(1,0) = -jacobian(1,0);
	jacobian(1,1) = -jacobian(1,1);
	jacobian(1,2) = -jacobian(1,2);
	jacobian(1,3) = -jacobian(1,3);

	jacobian(2,0) = 0;
	jacobian(2,1) = -l3*sin(Angle1)*cos(Angle2)*sin(Angle3) + l3*cos(Angle1)*cos(Angle3)+l2*cos(Angle1)-l1*sin(Angle1);
	jacobian(2,2) = -l3*cos(Angle1)*sin(Angle2)*sin(Angle3);
	jacobian(2,3) = l3*cos(Angle1)*cos(Angle2)*cos(Angle3) -l3*sin(Angle1)*sin(Angle3);
	//cout <<"jacobian:"<<jacobian<<endl;
	return jacobian;
}
//******************************************角度限制************************************//
//*  -2.0857 <= angle0 <= 2.0857
//*	 -0.3142 <= angle1 <= 1.3265
//*  -2.0857 <= angle2 <= 2.0857
//*  -1.5446 <= angle3 <= -0.0349
double setHangle(float AngleMax,float AngleMin, float inputAngle){
	double Hangle = (AngleMax - AngleMin)*(AngleMax - AngleMin)*(2*inputAngle-AngleMax-AngleMin)/(4*(AngleMax-inputAngle)*(AngleMax-inputAngle)*(inputAngle-AngleMin)*(inputAngle-AngleMin));
	Hangle = abs(Hangle);
	return Hangle;
}
Matrix4d getWeightMatrix(vector<float> currentAngle){
	Matrix4d WeightMatrix;
	double w0;
	double w1;
	double w2;
	double w3;
	double HAngle0Flag =setHangle(2.0857,-2.0857,currentAngle[0]);
	double HAngle1Flag =setHangle(1.3265,-0.3142,currentAngle[1]);
	double HAngle2Flag =setHangle( 2.0857,-2.0857,currentAngle[2]);
	double HAngle3Flag =setHangle(-0.0349,-1.5446,currentAngle[3]);
	if(HAngle0Flag >= HAngle0){
		w0 = 1 + HAngle0Flag;
	}else{
		w0 = 1;
	}
	if(HAngle1Flag >= HAngle1){
		w1 = 1 + HAngle1Flag ;
	}else{
		w1 = 1;
	}
	if(HAngle2Flag >= HAngle2){
		w2 = 1 + HAngle2Flag ;
	}else{
		w2 = 1;
	}
	if(HAngle3Flag >= HAngle3){
		w3 = 1 + HAngle3Flag;
	}else{
		w3 = 1;
	}
	HAngle0 = setHangle(2.0857,-2.0857,currentAngle[0]);
	HAngle1 = setHangle(1.3265,-0.3142,currentAngle[1]);
	HAngle2 = setHangle( 2.0857,-2.0857,currentAngle[2]);
	HAngle3 = setHangle(-0.0349,-1.5446,currentAngle[3]);

	WeightMatrix(0,0) =w0;
	WeightMatrix(0,1) =0;
	WeightMatrix(0,2) =0;
	WeightMatrix(0,3) =0;
	WeightMatrix(1,0) =0;
	WeightMatrix(1,1) =w1;
	WeightMatrix(1,2) =0;
	WeightMatrix(1,3) =0;
	WeightMatrix(2,0) =0;
	WeightMatrix(2,1) =0;
	WeightMatrix(2,2) =w2;
	WeightMatrix(2,3) =0;
	WeightMatrix(3,0) =0;
	WeightMatrix(3,1) =0;
	WeightMatrix(3,2) =0;
	WeightMatrix(3,3) =w3;

	//cout <<"WeightMatrix:"<<WeightMatrix<<endl;
	return WeightMatrix;
}
//*************************逆运动学算法用牛顿跌打法求解*************************//
Vector4d ikine(vector<float> tagertPos){
	double targetX;
	double targetY;
	double targetZ;
	double currentX;
	double currentY;
	double currentZ;
	Vector3d temp1;
	int temp ;
	int space = 0;
	int loopNumber = 1000;
	double e_pos = 0.00001;
	bool useSensorValues = false;
	MatrixXd jacobian(3,4);
	Matrix4d WeightMatrix;
	AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll");
	bool useSensors   = false;
	std::vector<float> commandAngles = motion.getAngles(names, useSensors);//得到当前角度
	//cout << "commandAngles：" <<commandAngles << endl;
	temp1 = fkine(commandAngles);//得到当前位置

	std::string name1 = "LWristYaw";
	std::vector<float> result = motion.getPosition(name1, space, useSensorValues);
	//std::cout << name1 << ":" << std::endl;

	//std::cout << "currentPosition (x, y, z): " << result.at(0) << ", " << result.at(2) - 0.1<< ", " << result.at(1) -0.098 << std::endl;
//	cout <<"temp1" <<endl<<temp1<<endl;
	currentX =  temp1(0);
	currentY =  temp1(1);
	currentZ =  temp1(2);
	//设置目标位置
	targetX = tagertPos[0];
	targetY = tagertPos[1];
	targetZ = tagertPos[2];

	//std::cout << "targetPosition (x, y, z): " << targetX << ", " << targetY<< ", " << targetZ << std::endl;
	//cin>>temp;
	//进入迭代部分
	Vector3d changeX ;
	Vector3d targetPosition;
	Vector3d currentPosition;
	Vector4d changeAngle ;
	Vector4d currentAngle;
	currentAngle(0) = commandAngles[0];
	currentAngle(1) = commandAngles[1];
	currentAngle(2) = commandAngles[2];
	currentAngle(3) = commandAngles[3];
	
	targetPosition(0) = targetX;
	targetPosition(1) = targetY;
	targetPosition(2) = targetZ;

	currentPosition(0) = currentX;
	currentPosition(1) = currentY;
	currentPosition(2) = currentZ;
	for(int loop = 0;loop < 500;loop++){
		changeX = targetPosition - currentPosition ;
	/*	cout << "changeX" << endl << changeX << endl;
		cout << "currentAngle" << endl << currentAngle << endl;*/
		//根据当前角度设置雅可比矩阵和权重矩阵
		jacobian = getLeftHandJacobian(currentAngle(0),currentAngle(1),currentAngle(2),currentAngle(3));
		commandAngles[0] = currentAngle(0);
		commandAngles[1] = currentAngle(1);
		commandAngles[2] = currentAngle(2);
		commandAngles[3] = currentAngle(3);
		WeightMatrix = getWeightMatrix(commandAngles);
		/*cout <<"WeightMatrix" << endl<< WeightMatrix<<endl;
		cout << "jacobian" << endl<< jacobian<<endl;*/
		//cout << "changeX" << endl<< changeX<<endl;
		//得到相应的微分角度
		changeAngle = WeightMatrix.inverse()*jacobian.transpose()*(jacobian*WeightMatrix.inverse()*jacobian.transpose()).inverse()*changeX;
		///changeAngle = jacobian.transpose()*(jacobian*jacobian.transpose()).inverse()*changeX;
		//cout << "changeAngle" << endl << changeAngle << endl;
		double epos = sqrt(changeAngle(0)*changeAngle(0) + changeAngle(1)*changeAngle(1) + changeAngle(2)*changeAngle(2));
		if(epos < e_pos){
			transfer = true;
			cout << "****************************************************************" << endl;
			cout <<"commandAngles"<<commandAngles<< endl;
			cout <<"WeightMatrix"<<endl<<WeightMatrix<< endl;
			cout <<"targetPosition"<<targetPosition<< endl;
			double errorTemp;
			Vector3d error3D = targetPosition - currentPosition;
			currentPosition = fkine(commandAngles);
			cout <<"epos" <<epos <<"<<<"<<e_pos<<endl;
			cout <<"currentAngle" <<currentAngle <<endl;
			cout << "currentPosition" << currentPosition <<endl;
			cout << "error"<<targetPosition - currentPosition<< endl;
			errorTemp = sqrt(error3D(0)*error3D(0) + error3D(1)*error3D(1) + error3D(2)*error3D(2));
			cout << "errorTemp" << errorTemp << endl;
			if(errorTemp > 0.01){
				system("pause");
			}
			break;
		}
		if(loop ==499){
			transfer =false;
			cout << "****************************************************************" << endl;
			cout <<"WeightMatrix"<< endl <<WeightMatrix<< endl;
			cout <<"targetPosition"<<targetPosition<< endl;
			double errorTemp;
			Vector3d error3D = targetPosition - currentPosition;
			currentPosition = fkine(commandAngles);
			cout <<"commandAngles"<<commandAngles<< endl;
			cout <<"epos" <<epos <<"<<<"<<e_pos<<endl;
			cout <<"currentAngle" <<currentAngle <<endl;
			cout << "currentPosition" << currentPosition <<endl;
			cout << "error"<<targetPosition - currentPosition<< endl;
			errorTemp = sqrt(error3D(0)*error3D(0) + error3D(1)*error3D(1) + error3D(2)*error3D(2));
			cout << "errorTemp" << errorTemp << endl;
			if(errorTemp > 0.01){
				system("pause");
			}
		}
		//更新当前角度
		currentAngle = currentAngle + changeAngle;
		commandAngles[0] = currentAngle(0);
		commandAngles[1] = currentAngle(1);
		commandAngles[2] = currentAngle(2);
		commandAngles[3] = currentAngle(3);
		//根据前向运动学求Xnew
		currentPosition = fkine(commandAngles);
	}	
	DEBUGE =true;
	if(DEBUGE){
		currentPosition = fkine(commandAngles);
		cout <<"currentAngle" <<currentAngle <<endl;
		cout << "currentPosition" << currentPosition <<endl;
		cout << "error"<<targetPosition - currentPosition<< endl;
	}
	currentAngle(0) = conventAngle(currentAngle(0));
	currentAngle(1) = conventAngle(currentAngle(1));
	currentAngle(2) = conventAngle(currentAngle(2));
	currentAngle(3) = conventAngle(currentAngle(3));
	/*cout << "currentAngle" <<currentAngle << endl;
	cout << "currentPosition"<< currentPosition<<endl;*/
	return currentAngle;
}

//*************************逆运动学测试**********************//
//*********************anglechange =  inv(W)*j'*inv([j*inv(W)*j'])*changeX 
void ikineTest(){
	double targetX;
	double targetY;
	double targetZ;
	double currentX;
	double currentY;
	double currentZ;
	Vector3d temp1;
	int temp ;
	int space = 0;
	int loopNumber = 1000;
	double e_pos = 0.00001;
	bool useSensorValues = false;
	MatrixXd jacobian(3,4);
	Matrix4d WeightMatrix;
	AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll");
	bool useSensors   = false;
	std::vector<float> commandAngles = motion.getAngles(names, useSensors);//得到当前角度
	cout << "commandAngles：" <<commandAngles << endl;
	temp1 = fkine(commandAngles);//得到当前位置

	std::string name1 = "LWristYaw";
	std::vector<float> result = motion.getPosition(name1, space, useSensorValues);
	std::cout << name1 << ":" << std::endl;

	std::cout << "currentPosition (x, y, z): " << result.at(0) << ", " << result.at(2) - 0.1<< ", " << result.at(1) -0.098 << std::endl;
	cout <<"temp1" <<endl<<temp1<<endl;
	currentX =  temp1(0);
	currentY =  temp1(1);
	currentZ =  temp1(2);
	//设置目标位置
	targetX = 0.00976838;
	targetY = -0.0294509;
	targetZ = -0.155502;

	std::cout << "targetPosition (x, y, z): " << targetX << ", " << targetY<< ", " << targetZ << std::endl;

	//cin>>temp;
//进入迭代部分
	Vector3d changeX ;
	Vector3d targetPosition;
	Vector3d currentPosition;
	Vector4d changeAngle ;
	Vector4d currentAngle;
	currentAngle(0) = commandAngles[0];
	currentAngle(1) = commandAngles[1];
	currentAngle(2) = commandAngles[2];
	currentAngle(3) = commandAngles[3];
	targetPosition(0) = targetX;
	targetPosition(1) = targetY;
	targetPosition(2) = targetZ;
	currentPosition(0) = currentX;
	currentPosition(1) = currentY;
	currentPosition(2) = currentZ;
	for(int loop = 0;loop < loopNumber;loop++){
		changeX = targetPosition - currentPosition ;
	/*	cout << "changeX" << endl << changeX << endl;
		cout << "currentAngle" << endl << currentAngle << endl;*/
		//根据当前角度设置雅可比矩阵和权重矩阵
		jacobian = getLeftHandJacobian(currentAngle(0),currentAngle(1),currentAngle(2),currentAngle(3));
		commandAngles[0] = currentAngle(0);
		commandAngles[1] = currentAngle(1);
		commandAngles[2] = currentAngle(2);
		commandAngles[3] = currentAngle(3);
		WeightMatrix = getWeightMatrix(commandAngles);
		/*cout <<"WeightMatrix" << endl<< WeightMatrix<<endl;
		cout << "jacobian" << endl<< jacobian<<endl;*/
		//cout << "changeX" << endl<< changeX<<endl;
		//得到相应的微分角度
		changeAngle = WeightMatrix.inverse()*jacobian.transpose()*(jacobian*WeightMatrix.inverse()*jacobian.transpose()).inverse()*changeX;
		//changeAngle = jacobian.transpose()*(jacobian*jacobian.transpose()).inverse()*changeX;
		//cout << "changeAngle" << endl << changeAngle << endl;
		double epos = sqrt(changeAngle(0)*changeAngle(0) + changeAngle(1)*changeAngle(1) + changeAngle(2)*changeAngle(2));
		if(epos < e_pos){
			break;
		}
		//更新当前角度
		currentAngle = currentAngle + changeAngle;
		commandAngles[0] = currentAngle(0);
		commandAngles[1] = currentAngle(1);
		commandAngles[2] = currentAngle(2);
		commandAngles[3] = currentAngle(3);
		//根据前向运动学求Xnew
		currentPosition = fkine(commandAngles);
	}


	
	currentAngle(0) = conventAngle(currentAngle(0));
	currentAngle(1) = conventAngle(currentAngle(1));
	currentAngle(2) = conventAngle(currentAngle(2));
	currentAngle(3) = conventAngle(currentAngle(3));
	cout << "currentAngle" <<currentAngle << endl;
	cout << "currentPosition"<< currentPosition<<endl;

	// AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll","RShoulderPitch", "RShoulderRoll","RElbowYaw","RElbowRoll");
	AL::ALValue angles      = AL::ALValue::array(currentAngle(0),currentAngle(1) ,currentAngle(2),currentAngle(3));
	cout << "angles:" << angles << endl;
	fkine(angles);
	cin>>temp;

	float fractionMaxSpeed  = 0.1f;
	motion.setStiffnesses(names, AL::ALValue::array(1.0f,1.0f,1.0f,1.0f));
	qi::os::sleep(1.0f);
	motion.setAngles(names, angles, fractionMaxSpeed);
	cin>>temp;
	result = motion.getPosition(name1, space, useSensorValues);
	std::cout << name1 << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(2) - 0.1<< ", " << result.at(1) -0.098 << std::endl;
	system("pause");
}
//**
//* 函数描述
//* @author Administrator
//* @Time 2017-05-10 16:27:01
//* @param 各个关节角度
//* @return 末端位置 
//*/
Vector3d fkine(vector<float> commandAngles){
	DHMatrix dHMatrix0to1;
	Matrix4d dh0to1;
	dHMatrix0to1.setDHParam(commandAngles[0],0.0,0.0,pi / 2);
	dh0to1 = dHMatrix0to1.setTransformation();

	DHMatrix dHMatrix1to2;
	Matrix4d dh1to2;
	dHMatrix1to2.setDHParam(commandAngles[1] + pi /2 ,0.0,l1,pi / 2);
	dh1to2 = dHMatrix1to2.setTransformation();

	DHMatrix dHMatrix2to3;
	Matrix4d dh2to3;
	dHMatrix2to3.setDHParam(commandAngles[2] + pi ,l2,0.0,pi / 2);
	dh2to3 = dHMatrix2to3.setTransformation();
	
	DHMatrix dHMatrix3to4;
	Matrix4d dh3to4;
	dHMatrix3to4.setDHParam(commandAngles[3] + pi ,0.0,0.0,pi / 2);
	dh3to4 = dHMatrix3to4.setTransformation();
	

	DHMatrix dHMatrix4to5;
	Matrix4d dh4to5;
	dHMatrix4to5.setDHParam(0 ,l3,0.0,0.0);
	dh4to5 = dHMatrix4to5.setTransformation();

	Matrix4d effecter;
	Vector3d effecterPosition;
	effecter = dh0to1*dh1to2*dh2to3*dh3to4*dh4to5;
	effecterPosition(0) = effecter(0,3);
	effecterPosition(1) = -effecter(1,3);
	effecterPosition(2) = effecter(2,3);
	bool DBUGE = false;
	if(DBUGE){
		cout << "effecter: "<<endl;
		cout<<effecter<<endl;
		cout << "正向运动学终端位置:" << endl; 
		cout << "x:" << effecterPosition(0) <<  "y:" <<effecterPosition(1)<<"z:" <<effecterPosition(2);
	}
	return effecterPosition;
}
Vector3d fRkine(vector<float> commandAngles){
	DHMatrix dHMatrix0to1;
	Matrix4d dh0to1;
	dHMatrix0to1.setDHParam(commandAngles[0],0.0,0.0,pi / 2);
	dh0to1 = dHMatrix0to1.setTransformation();
	 

	DHMatrix dHMatrix1to2;
	Matrix4d dh1to2;
	dHMatrix1to2.setDHParam(commandAngles[1] - pi /2 ,0.0,l1,-pi / 2);
	dh1to2 = dHMatrix1to2.setTransformation();
	

	DHMatrix dHMatrix2to3;
	Matrix4d dh2to3;
	dHMatrix2to3.setDHParam(commandAngles[2] + pi,l2,0.0,-pi / 2);
	dh2to3 = dHMatrix2to3.setTransformation();
	

	DHMatrix dHMatrix3to4;
	Matrix4d dh3to4;
	dHMatrix3to4.setDHParam(commandAngles[3] + pi ,0.0,0.0,-pi / 2);
	dh3to4 = dHMatrix3to4.setTransformation();
	

	DHMatrix dHMatrix4to5;
	Matrix4d dh4to5;
	dHMatrix4to5.setDHParam(0   ,l3,0.0,0.0);
	dh4to5 = dHMatrix4to5.setTransformation();
	if(DEBUGE){
		cout<< "dh0to1:" << endl;
		cout<< dh0to1 << endl;
		cout<< "dh1to2:" << endl;
		cout<< dh1to2 << endl;
		cout<< "dh2to3:" << endl;
		cout<< dh2to3 << endl;
		cout<< "dh3to4:" << endl;
		cout<< dh3to4 << endl;
		cout<< "dh4to5:" << endl;
		cout<< dh4to5 << endl;
	}
	Matrix4d effecter;
	Vector3d effecterPosition;
	effecter = dh0to1*dh1to2*dh2to3*dh3to4*dh4to5;
	effecterPosition(0) = effecter(0,3);
	effecterPosition(1) = -effecter(1,3);
	effecterPosition(2) = effecter(2,3);
	bool DBUGE = false;
	if(DBUGE){
		cout << "effecter: "<<endl;
		cout<<effecter<<endl;
		cout << "正向运动学终端位置:" << endl; 
		cout << "x:" << effecterPosition(0) <<  "y:" <<effecterPosition(1)<<"z:" <<effecterPosition(2);
	}
	return effecterPosition;
}
void fRkineTest(){
	AL::ALValue names  = AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw");
	AL::ALValue angles  =  AL::ALValue::array(1.0f,0.1f,1.0f,1.0f,0.0f);
	float fractionmaxspeed  = 0.1f;
	motion.setStiffnesses(names,  AL::ALValue::array(1.0f,1.0f,1.0f,1.0f,1.0f));
	qi::os::sleep(1.0f);
	motion.setAngles(names, angles, fractionmaxspeed);
	int temp; 
	cin >> temp;
	bool useSensors   = false;
	std::vector<float> commandAngles = motion.getAngles(names,useSensors);
	/*int temp; 
	cin >> temp;*/
	cout << "commandAngles:" <<	commandAngles<<endl;
	DHMatrix dHMatrix0to1;
	Matrix4d dh0to1;
	dHMatrix0to1.setDHParam(commandAngles[0],0.0,0.0,pi / 2);
	dh0to1 = dHMatrix0to1.setTransformation();
	cout<< "dh0to1:" << endl;
	cout<< dh0to1 << endl;

	DHMatrix dHMatrix1to2;
	Matrix4d dh1to2;
	dHMatrix1to2.setDHParam(commandAngles[1] - pi /2 ,0.0,l1,-pi / 2);
	dh1to2 = dHMatrix1to2.setTransformation();
	cout<< "dh1to2:" << endl;
	cout<< dh1to2 << endl;

	DHMatrix dHMatrix2to3;
	Matrix4d dh2to3;
	dHMatrix2to3.setDHParam(commandAngles[2] + pi,l2,0.0,-pi / 2);
	dh2to3 = dHMatrix2to3.setTransformation();
	cout<< "dh2to3:" << endl;
	cout<< dh2to3 << endl;

	DHMatrix dHMatrix3to4;
	Matrix4d dh3to4;
	dHMatrix3to4.setDHParam(commandAngles[3] + pi ,0.0,0.0,-pi / 2);
	dh3to4 = dHMatrix3to4.setTransformation();
	cout<< "dh3to4:" << endl;
	cout<< dh3to4 << endl;

	DHMatrix dHMatrix4to5;
	Matrix4d dh4to5;
	dHMatrix4to5.setDHParam(commandAngles[4] + pi  ,l3,0.0,0.0);
	dh4to5 = dHMatrix4to5.setTransformation();
	cout<< "dh4to5:" << endl;
	cout<< dh4to5 << endl;

	Matrix4d effecter;
	effecter = dh0to1*dh1to2*dh2to3*dh3to4*dh4to5;
	cout << "effecter: "<<endl;
	cout<<effecter<<endl;
	//commandAngles[0] = commandAngles[0];
	///*double effectx = l3 *sin(commandAngles[0])*sin(commandAngles[2])*sin(commandAngles[3]) - l3*cos(commandAngles[0])*sin(commandAngles[1])*cos(commandAngles[2])*sin(commandAngles[3])+ l3*cos(commandAngles[3])*cos(commandAngles[0])*cos(commandAngles[1])+l2*cos(commandAngles[0])*cos(commandAngles[1]) - l1*cos(commandAngles[0])*sin(commandAngles[1]);
	//double effecty = -l3*sin(commandAngles[0])*sin(commandAngles[1])*cos(commandAngles[2])*sin(commandAngles[3]) -l3*cos(commandAngles[0])*sin(commandAngles[2])*sin(commandAngles[3])+l3*sin(commandAngles[0])*cos(commandAngles[1])*cos(commandAngles[3]) + l2*sin(commandAngles[0])*cos(commandAngles[1]) -l1*sin(commandAngles[0])*sin(commandAngles[1]);
	//double effectz = l3*cos(commandAngles[1])*cos(commandAngles[2])*sin(commandAngles[3])+l3*sin(commandAngles[1])*cos(commandAngles[3]) + l2*sin(commandAngles[1])+l1*cos(commandAngles[1]);*/
	//double effectx = effecter(0)(3);
	//double effecty = effecter(1)(3);
	//double effectz = effecter(2)(3);
	/*cout << "x:" << effectx <<"y"<<effecty<<"z"<<effectz;*/
	int space = 0;
	bool useSensorValues = false;
	std::string name = "RWristYaw";
	std::vector<float> result = motion.getPosition(name, space, useSensorValues);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(2) - 0.1<< ", " << result.at(1) + 0.098 << std::endl;
	name = "RShoulderPitch";
	result = motion.getPosition(name, space, useSensorValues);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(2)<< ", " << result.at(1)<< std::endl;
	/*MatrixXd Transformation(4,4);
	 MatrixXd m(2,2);
	  m(0,0) = 3;
	  m(1,0) = 2.5;
	  m(0,1) = -1;
	  m(1,1) = m(1,0) + m(0,1);
	  std::cout << m << std::endl;
	int temp;
	cin >> temp;*/
}
void fkineTest(){
	AL::ALValue names  = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw");
	AL::ALValue angles  =  AL::ALValue::array(1.0f,1.4f,1.0f,-1.0f,0.0f);
	float fractionmaxspeed  = 0.1f;
	motion.setStiffnesses(names,  AL::ALValue::array(1.0f,1.0f,1.0f,1.0f,1.0f));
	qi::os::sleep(1.0f);
	motion.setAngles(names, angles, fractionmaxspeed);
	int temp; 
	cin >> temp;
	bool useSensors   = false;
	std::vector<float> commandAngles = motion.getAngles(names,useSensors);
	/*int temp; 
	cin >> temp;*/
	cout << "commandAngles:" <<	commandAngles<<endl;
	DHMatrix dHMatrix0to1;
	Matrix4d dh0to1;
	dHMatrix0to1.setDHParam(commandAngles[0],0.0,0.0,pi / 2);
	dh0to1 = dHMatrix0to1.setTransformation();
	cout<< "dh0to1:" << endl;
	cout<< dh0to1 << endl;

	DHMatrix dHMatrix1to2;
	Matrix4d dh1to2;
	dHMatrix1to2.setDHParam(commandAngles[1] + pi /2 ,0.0,l1,pi / 2);
	dh1to2 = dHMatrix1to2.setTransformation();
	cout<< "dh1to2:" << endl;
	cout<< dh1to2 << endl;

	DHMatrix dHMatrix2to3;
	Matrix4d dh2to3;
	dHMatrix2to3.setDHParam(commandAngles[2] + pi ,l2,0.0,pi / 2);
	dh2to3 = dHMatrix2to3.setTransformation();
	cout<< "dh2to3:" << endl;
	cout<< dh2to3 << endl;

	DHMatrix dHMatrix3to4;
	Matrix4d dh3to4;
	dHMatrix3to4.setDHParam(commandAngles[3] + pi ,0.0,0.0,pi / 2);
	dh3to4 = dHMatrix3to4.setTransformation();
	cout<< "dh3to4:" << endl;
	cout<< dh3to4 << endl;

	DHMatrix dHMatrix4to5;
	Matrix4d dh4to5;
	dHMatrix4to5.setDHParam(commandAngles[4] ,l3,0.0,0.0);
	dh4to5 = dHMatrix4to5.setTransformation();
	cout<< "dh4to5:" << endl;
	cout<< dh4to5 << endl;

	Matrix4d effecter;
	effecter = dh0to1*dh1to2*dh2to3*dh3to4*dh4to5;
	cout << "effecter: "<<endl;
	cout<<effecter<<endl;
	commandAngles[0] = commandAngles[0];
	double effectx = l3 *sin(commandAngles[0])*sin(commandAngles[2])*sin(commandAngles[3]) - l3*cos(commandAngles[0])*sin(commandAngles[1])*cos(commandAngles[2])*sin(commandAngles[3])+ l3*cos(commandAngles[3])*cos(commandAngles[0])*cos(commandAngles[1])+l2*cos(commandAngles[0])*cos(commandAngles[1]) - l1*cos(commandAngles[0])*sin(commandAngles[1]);
	double effecty = -l3*sin(commandAngles[0])*sin(commandAngles[1])*cos(commandAngles[2])*sin(commandAngles[3]) -l3*cos(commandAngles[0])*sin(commandAngles[2])*sin(commandAngles[3])+l3*sin(commandAngles[0])*cos(commandAngles[1])*cos(commandAngles[3]) + l2*sin(commandAngles[0])*cos(commandAngles[1]) -l1*sin(commandAngles[0])*sin(commandAngles[1]);
	double effectz = l3*cos(commandAngles[1])*cos(commandAngles[2])*sin(commandAngles[3])+l3*sin(commandAngles[1])*cos(commandAngles[3]) + l2*sin(commandAngles[1])+l1*cos(commandAngles[1]);

	cout << "x:" << effectx <<"y"<<effecty<<"z"<<effectz;
	int space = 0;
	bool useSensorValues = false;
	std::string name = "LWristYaw";
	std::vector<float> result = motion.getPosition(name, space, useSensorValues);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(2) - 0.1<< ", " << result.at(1) -0.098 << std::endl;
	/*MatrixXd Transformation(4,4);
	 MatrixXd m(2,2);
	  m(0,0) = 3;
	  m(1,0) = 2.5;
	  m(0,1) = -1;
	  m(1,1) = m(1,0) + m(0,1);
	  std::cout << m << std::endl;
	int temp;
	cin >> temp;*/
}
//**************************************数据分析*************************************//
bool writeToText = false;
bool firstRecord = true;
long FirstTime;
long time1;
ofstream ocout;
ofstream ocout1;
bool DATAVIEW = false;
//cv 图像显示
Mat skeletonImage;
Mat depthImage; 
Mat image;
CvPoint skeletonPoint[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT]={cvPoint(0,0)};
CvPoint colorPoint[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT]={cvPoint(0,0)};
void drawSkeleton(Mat &image, CvPoint pointSet[], int whichone);
void getDepthImage(HANDLE &depthEvent, HANDLE &depthStreamHandle, Mat &depthImage,CvPoint skeletonPoint1[6][20],NUI_SKELETON_FRAME skeletonFrame) ;
float getAngleForSkel(NUI_SKELETON_DATA skel,int i);
void getAndDrawAngle(const NUI_SKELETON_DATA & skel);
bool tracked[NUI_SKELETON_COUNT]={FALSE};
const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
int getLeftHandPostion(const NUI_SKELETON_DATA & skel);
void controlNao();
float angleArr[1000]; 
int arrIndex = 0;

int DrawColor(HANDLE h)
{
	NUI_IMAGE_FRAME imageFrame = { 0 };
    HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame( h, 0, &imageFrame );
	NUI_LOCKED_RECT colorData;
	imageFrame.pFrameTexture->LockRect( 0, &colorData, 0, 0 );
	image = cv::Mat(480, 640, CV_8UC4, colorData.pBits );
	area = imageFrame.ViewArea;
	m_pNuiSensor->NuiImageStreamReleaseFrame( h, &imageFrame );
	cv::imshow( "colorImage", image );
	return 0;
}
 
int DrawDepth(HANDLE h)
{
	NUI_IMAGE_FRAME pImageFrame;
	INuiFrameTexture* pDepthImagePixelFrame;
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame( h, 0, &pImageFrame );
	BOOL nearMode = TRUE;
	m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &pImageFrame, &nearMode, &pDepthImagePixelFrame);
	INuiFrameTexture * pTexture = pDepthImagePixelFrame;
	NUI_LOCKED_RECT LockedRect;  
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );  
	if( LockedRect.Pitch != 0 )
	{
		HRESULT hr = m_nuiIStream->ProcessDepth(LockedRect.size,PBYTE(LockedRect.pBits),pImageFrame.liTimeStamp);
		if( FAILED( hr ) )
		{
			cout<<"Process Depth failed"<<endl;
		}
	}
	pTexture->UnlockRect(0);
	m_pNuiSensor->NuiImageStreamReleaseFrame( h, &pImageFrame );
	return 0;
}
 
int DrawSkeleton()
{
	//NUI_IMAGE_FRAME imageFrame = { 0 };
	bool queryPeople = false;
	NUI_SKELETON_FRAME SkeletonFrame = {0};
	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame( 0, &SkeletonFrame );
	if( FAILED( hr ) )
	{
		cout<<"Get Skeleton Image Frame Failed"<<endl;
		return -1;
	}
	bool bFoundSkeleton = true;
	static int static_one_is_enough=0;
	if(static_one_is_enough==0)
	{
		cout<<"find skeleton !"<<endl;
		static_one_is_enough++;
	}



	const NUI_TRANSFORM_SMOOTH_PARAMETERS SomewhatLatentParams = {0.5f, 0.1f, 0.5f, 0.1f, 0.1f};
	m_pNuiSensor->NuiTransformSmooth(&SkeletonFrame,&SomewhatLatentParams); 
	//m_pNuiSensor->NuiTransformSmooth(&SkeletonFrame,NULL);
	skeletonImage.setTo(0);
	for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i)
    {
        NUI_SKELETON_TRACKING_STATE trackingState = SkeletonFrame.SkeletonData[i].eTrackingState; 
        if (NUI_SKELETON_TRACKED == trackingState)
        {
				queryPeople = true;
				if(SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED && SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_CENTER] != NUI_SKELETON_POSITION_NOT_TRACKED){
					float fx,fy;
					//getAndDrawAngle(SkeletonFrame.SkeletonData[i]);
					if(firstRecord){
						FirstTime = (long)SkeletonFrame.liTimeStamp.QuadPart;
						firstRecord = false;
					}
					time1 = SkeletonFrame.liTimeStamp.QuadPart - FirstTime;
					//cout<<"time:"<<time1 << endl;
				/*	cout << "z: " << SkeletonFrame.SkeletonData[i].SkeletonPositions[0].z<<endl;*/
					writeToText = false;
					if(writeToText){
						ocout <<SkeletonFrame.SkeletonData[i].SkeletonPositions[5].x<<","<<SkeletonFrame.SkeletonData[i].SkeletonPositions[5].y<<","<<SkeletonFrame.SkeletonData[i].SkeletonPositions[5].z<<","<< time1 << endl;
						//ocout1 <<-LShoulderPitch<<" "<< LShoulderRoll<<" "<<LElbowYaw<<" "<<LElbowRoll<<" "<<time1<<endl;
					}
					
					if(SkeletonFrame.SkeletonData[i].SkeletonPositions[0].z > 2 && SkeletonFrame.SkeletonData[i].SkeletonPositions[0].z < 3){
						getLeftHandPostion(SkeletonFrame.SkeletonData[i]);
					}

				
					for(int j = 0 ; j < NUI_SKELETON_POSITION_COUNT ; j++){
						 NuiTransformSkeletonToDepthImage(SkeletonFrame.SkeletonData[i].SkeletonPositions[j], &fx, &fy,NUI_IMAGE_RESOLUTION_640x480 );//将骨骼坐标转化到深度图像中去，第一个参数是骨骼位置vector4 后面是深度图像相对位置变量	 
						 skeletonPoint[i][j].x = (int)fx;     
						 skeletonPoint[i][j].y = (int)fy;
					}
					for(int j = 0; j < NUI_SKELETON_POSITION_COUNT ; j++){
						if (SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[j] != NUI_SKELETON_POSITION_NOT_TRACKED){
							circle(skeletonImage, skeletonPoint[i][j], 3, cvScalar(0, 0, 255), 1, 8, 0);
							long  x=0;
							long  y=0;
							NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480,&area,skeletonPoint[i][j].x,skeletonPoint[i][j].y,0,&x,&y);
							colorPoint[i][j].x =  x;
							colorPoint[i][j].y =  y;
							//circle(image, colorPoint[i][j], 3, cvScalar(0, 0, 255), 1, 8, 0);
							tracked[i] = TRUE;
						} 
					}
					drawSkeleton(skeletonImage, skeletonPoint[i], i);//第一个参数是Mat，第二个是cvpoint  第三个是第几个人
				}	
        }
    }
	if(queryPeople == false){
		cout<<"没有人在视野中!"<<endl;
		return -1;
	}else{
		imshow("skeletonImage", skeletonImage); //显示图像 
	}
	Vector4 vTemp;
	m_pNuiSensor->NuiAccelerometerGetCurrentReading(&vTemp);
	hr =m_nuiIStream->ProcessSkeleton(NUI_SKELETON_COUNT, 
		SkeletonFrame.SkeletonData,
		&vTemp,
		SkeletonFrame.liTimeStamp);
	if( FAILED( hr ) )
	{
		cout<<"Process Skeleton failed"<<endl;
	}
	waitKey(1);
	return 0;
}
 
int ShowInteraction()
{
	NUI_INTERACTION_FRAME Interaction_Frame;
	HRESULT hr = m_nuiIStream->GetNextFrame( 0,&Interaction_Frame );
	if(hr != S_OK)
	{
		if(hr == E_POINTER)
			cout<<"E_POINTER          "<<endl;
		else if(hr == E_NUI_FRAME_NO_DATA)
		{
			cout<<"E_NUI_FRAME_NO_DATA"<<endl;
		}
		return -1;
	}
	int trackingID = 0;
	int event;
	for(int i=0 ; i<NUI_SKELETON_COUNT ; i++)
	{
		COORD pos = {0,0};
		HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);    //函数句柄
		SetConsoleCursorPosition(hOut, pos);   
		static int frameCount=0;
		frameCount++;
		for(int j=0;j<NUI_USER_HANDPOINTER_COUNT;j++)
		{
			if( ( frameCount%3 )==1 )
			{
				trackingID = Interaction_Frame.UserInfos[i].SkeletonTrackingId;
				event = Interaction_Frame.UserInfos[i].HandPointerInfos[j].HandEventType;
				DWORD state = Interaction_Frame.UserInfos[i].HandPointerInfos[j].State;
				NUI_HAND_TYPE type = Interaction_Frame.UserInfos[i].HandPointerInfos[j].HandType;
				if(type==NUI_HAND_TYPE_NONE)
					continue;
				if((state&&NUI_HANDPOINTER_STATE_TRACKED)==0)
					continue;
				if((state&&NUI_HANDPOINTER_STATE_ACTIVE)==0)
					continue;
				cout<<"id="<<trackingID<<"--------HandEventType=";
				if(event == NUI_HAND_EVENT_TYPE_GRIP)
				{
					stateStr[j] = "Grip ！！！...";
					cout<<"Grip ！！！   ";
				}
				else if(event == NUI_HAND_EVENT_TYPE_GRIPRELEASE)
				{
					stateStr[j] = "Grip Release...";
					cout<<"Grip Release "; 

				}
				else
				{
					cout << stateStr[j];
				}
				cout<<"    HandType=";
				
				if(type==NUI_HAND_TYPE_NONE)
					cout<<"No    Hand";
				else if(type==NUI_HAND_TYPE_LEFT)
					cout<<"Left  Hand";
				else if(type==NUI_HAND_TYPE_RIGHT)
					cout<<"Right Hand";
				cout<<endl;
			}
		}
	}
	return 0;
}
 
DWORD WINAPI KinectDataThread(LPVOID pParam)
{
	HANDLE hEvents[5] = {m_hEvNuiProcessStop,m_hNextColorFrameEvent,m_hNextDepthFrameEvent,m_hNextSkeletonEvent,m_hNextInteractionEvent};
	while(1)
	{
		int nEventIdx;
		nEventIdx=WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),hEvents,FALSE,100);
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hEvNuiProcessStop, 0))
		{
			break;
		}
		// Process signal events
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0))
		{
			DrawColor(m_pColorStreamHandle);
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0))
		{
			DrawDepth(m_pDepthStreamHandle);
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0))
		{
			DrawSkeleton();
		}
		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextInteractionEvent, 0))
		{
			//system("pause");
			ShowInteraction();
		}
		waitKey(1);
	}
 
	CloseHandle(m_hEvNuiProcessStop);
	m_hEvNuiProcessStop = NULL;
	CloseHandle( m_hNextSkeletonEvent );
	CloseHandle( m_hNextDepthFrameEvent );
	CloseHandle( m_hNextColorFrameEvent );
	CloseHandle( m_hNextInteractionEvent );
	m_pNuiSensor->NuiShutdown();
	SafeRelease(m_pNuiSensor);
	return 0;
}
 
DWORD ConnectKinect()//dword double world 每个word两个字节长度 一个字节8位及32位
{
	/*int temp;
	cin >> temp;*/
	skeletonImage.create(480, 640, CV_8UC3); 
	INuiSensor * pNuiSensor;
	HRESULT hr;
	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);//获取链接个数
	if (FAILED(hr))
	{
		cout<<"无kinect链接"<<endl;
		return hr;
	}
	
	 //Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}
		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}
		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}
	if (NULL != m_pNuiSensor)
	{
		if (SUCCEEDED(hr))
		{   
			hr = m_pNuiSensor->NuiInitialize(\
				NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX|\
				NUI_INITIALIZE_FLAG_USES_COLOR|\
				NUI_INITIALIZE_FLAG_USES_SKELETON);
			if( hr != S_OK )
			{
				cout<<"初始化kinect失败"<<endl;
				return hr;
			}
			// cin >> temp;
			m_hNextColorFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
			m_pColorStreamHandle = NULL;
 
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480, 0, 2, 
				m_hNextColorFrameEvent, &m_pColorStreamHandle);
			if( FAILED( hr ) )
			{
				cout<<"Could not open image stream video"<<endl;
				return hr;
			}

			m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
			m_pDepthStreamHandle = NULL;
 
			hr = m_pNuiSensor->NuiImageStreamOpen( 
				NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
				NUI_IMAGE_RESOLUTION_640x480, 0, 2, 
				m_hNextDepthFrameEvent, &m_pDepthStreamHandle);
			if( FAILED( hr ) )
			{ 
				cout<<"Could not open depth stream video"<<endl;
				return hr;
			}
			m_hNextSkeletonEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
			hr = m_pNuiSensor->NuiSkeletonTrackingEnable( 
				m_hNextSkeletonEvent, 
				NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE//|
				);
			if( FAILED( hr ) )
			{
				cout<<"Could not open skeleton stream video"<<endl;
				return hr;
			}
		}
	}
	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		cout<<"No ready Kinect found!"<<endl;
		return E_FAIL;
	}
	return hr;
}
void movingAveragefilter(double bufAngle0,double bufAngle1,double bufAngle2,double bufAngle3){
	double result0 = 0.0;
	double result1 = 0.0;
	double result2 = 0.0;
	double result3 = 0.0;
	bufferAngle0[indexBuffer] = bufAngle0;
	bufferAngle1[indexBuffer] = bufAngle1;
	bufferAngle2[indexBuffer] = bufAngle2;
	bufferAngle3[indexBuffer] = bufAngle3;
	indexBuffer++;
	if(indexBuffer == 5){
		indexBuffer=0;
		startFilter = true;
	}
	for(int i=0;i<5;i++){
		result0 = result0 + bufferAngle0[i];
		result1 = result1 + bufferAngle1[i];
		result2 = result2 + bufferAngle2[i];
		result3 = result3 + bufferAngle3[i];
	}
	result0 = result0 / 5;
	result1 = result1 / 5;
	result2 = result2 / 5;
	result3 = result3 / 5;
	if(startFilter){
		LShoulderPitchResult = result0;
		LShoulderRollResult = result1;
		LElboYawResult = result2;
		LElbowRollResult = result3;
	}else{
		LShoulderPitchResult = bufAngle0;
		LShoulderRollResult = bufAngle1;
		LElboYawResult = bufAngle2;
		LElbowRollResult = bufAngle3;
	}
}

void calculLeftAngle(){
	float LShoulderPitch = 0;
	float LShoulderPitchAcos = 0;
	float LShoulderPitchAtan = 0;
	float LShoulderPitchAsin = 0;
	float LShoulderRoll = 0;
	float LElbowYaw = 0;
	float LElbowYawAcos = 0;
	float LElbowYawAsin = 0;
	float LElbowRoll=0;

	float RShoulderPitch = 0;
	float RShoulderPitchAcos = 0;
	float RShoulderPitchAtan = 0;
	float RShoulderPitchAsin = 0;
	float RShoulderRoll = 0;
	float RElbowYaw = 0;
	float RElbowYawAcos = 0;
	float RElbowYawAsin = 0;
	float RElbowRoll=0;

	float resultAngle[4];

	float Lerror0;
	float Lerror1;
	float Lerror2;
	float Lerror3;

	float Rerror0;
	float Rerror1;
	float Rerror2;
	float Rerror3;
	//从机器人的躯干坐标系转移到肩部坐标系
	float flag = LShouldToElbow[1] ;
	LShouldToElbow[1] = LShouldToElbow[2];
	LShouldToElbow[2] = flag;

	flag = LElbowToWrist[1];
	LElbowToWrist[1] = LElbowToWrist[2];
	LElbowToWrist[2] = flag;

	flag = RShouldToElbow[1] ;
	RShouldToElbow[1] = RShouldToElbow[2];
	RShouldToElbow[2] = flag;

	flag = RElbowToWrist[1];
	RElbowToWrist[1] = RElbowToWrist[2];
	RElbowToWrist[2] = flag;


	LShoulderRoll = asin(LShouldToElbow[2] / sqrt(l1*l1+l2*l2)) + atan(-l1 / l2);
	//cout << "LShoulderRoll::" << LShoulderRoll <<endl;
	//cout << "LShoulderRoll::" << LShoulderRoll <<endl;
	LShoulderPitchAcos = acos(LShouldToElbow[0] / (l2*cos(LShoulderRoll) - l1*sin(LShoulderRoll)));
	//cout << "LShoulderPitchAcos::" << LShoulderPitchAcos<<endl;	
	LShoulderPitchAsin = asin(LShouldToElbow[1] / (l2*cos(LShoulderRoll) - l1*sin(LShoulderRoll)));
	//cout << "LShoulderPitchAsin::" << LShoulderPitchAsin<<endl;	
	LShoulderPitchAtan = atan(LShouldToElbow[1] / LShouldToElbow[0]);
	//cout << "LShoulderPitchAtan::" << LShoulderPitchAtan<<endl;
	if(LShoulderPitchAsin > 0){
		LShoulderPitch =  LShoulderPitchAcos;
		//cout << "LShoulderPitch:" << -LShoulderPitchAcos <<  endl;
	}else{
		LShoulderPitch =  -LShoulderPitchAcos;
		//cout << "LShoulderPitch:" << LShoulderPitchAcos <<  endl;
	}
	//angle0 = -LShoulderPitch;
	//cout << "LShoulderRoll::" << LShoulderRoll <<endl;
	//angle1 = LShoulderRoll;
	float LElbowToWristTemp[3] = {0,0,0};
	LElbowToWristTemp[0] = -sin(LShoulderRoll)*cos(LShoulderPitch)*LElbowToWrist[0]  - sin(LShoulderPitch)* sin(LShoulderRoll)*LElbowToWrist[1] + cos(LShoulderRoll)*LElbowToWrist[2];
	LElbowToWristTemp[1] = sin(LShoulderPitch)*LElbowToWrist[0]-cos(LShoulderPitch)*LElbowToWrist[1];
	LElbowToWristTemp[2] = cos(LShoulderPitch)*cos(LShoulderRoll)*LElbowToWrist[0] + sin(LShoulderPitch)*cos(LShoulderRoll)*LElbowToWrist[1] + sin(LShoulderRoll)*LElbowToWrist[2];
	LElbowYaw =  -atan(LElbowToWristTemp[1] / LElbowToWristTemp[0]);
	//cout << "LElbowYaw::" << LElbowYaw<<endl;
	float error1 = sqrt(LElbowToWristTemp[0]*LElbowToWristTemp[0] + LElbowToWristTemp[1]*LElbowToWristTemp[1] ) / l3;
	//cout <<"error1:" << error1<<endl;
	if(error1 > 1){
		error1 = 1;
	}
	LElbowRoll = -asin(error1);
	//cout << "LElbowRoll::" << LElbowRoll<<endl;
	LElbowYawAcos = acos(-LElbowToWristTemp[0] / sqrt(LElbowToWristTemp[0]*LElbowToWristTemp[0] +LElbowToWristTemp[1]*LElbowToWristTemp[1]));
	//cout << "LElbowYawAcos::" << LElbowYawAcos<<endl;
	LElbowYawAsin = asin(-LElbowToWristTemp[1] / sqrt(LElbowToWristTemp[0]*LElbowToWristTemp[0] +LElbowToWristTemp[1]*LElbowToWristTemp[1]));
	//cout << "LElbowYawAsin::" << LElbowYawAsin<<endl;
	if(LElbowYawAsin > 0){
		LElbowYaw = -LElbowYawAcos;
	}else{
		LElbowYaw = LElbowYawAcos;
	}
	//cout << "LElbowYaw::" << LElbowYaw<<endl;
	//cout << "LElbowRoll::" << LElbowRoll<<endl;
	Lerror0 = abs(angle0 + LShoulderPitch);
	Lerror1 = abs(angle1 - LShoulderRoll);
	Lerror2 = abs(angle2 - LElbowYaw);
	Lerror3 = abs(angle3 - LElbowRoll);
	//cout << "time:"<<time1 << "angle0"<< angle0 << endl;
	//***********************去抖动操作******************************//
	if((Lerror0 < FrameMaxError)){
		angle0 = -LShoulderPitch;
	}
	if(Lerror1 < FrameMaxError){
		angle1 = LShoulderRoll;
	}
	if(Lerror2 < FrameMaxError){
		angle2 = LElbowYaw;
	}
	if(Lerror3 < FrameMaxError){
		angle3 = LElbowRoll;
	}
	DATAVIEW = false;
	if(DATAVIEW){
		ocout <<angle0<<","<<angle1<<","<<angle2<<","<<angle3<< ","<< time1 << endl;
		ocout1 <<-LShoulderPitch<<","<< LShoulderRoll<<","<<LElbowYaw<<","<<LElbowRoll<<","<<time1<<endl;
	}
	//滑动平均滤波
	//movingAveragefilter(angle0,angle1,angle2,angle3);
	//
	LShoulderPitchResult = angle0;
	LShoulderRollResult = angle1;
	LElboYawResult = angle2;
	LElbowRollResult = angle3;
	bool BIHUAN = false;
	if(BIHUAN){
		vector<float> Angles(4);
		Vector3d position;
		Angles[0] = angle0;
		Angles[1] = angle1;
		Angles[2] = angle2;
		Angles[3] = angle3;
		position = fkine(Angles);
	/*	cout << "angle：" <<Angles << endl;
		cout << " position:" << endl <<position<< endl;
		cout << "position真实："<< endl <<"x::" << LShouldToElbow[0] + LElbowToWrist[0] <<"y::" <<  LShouldToElbow[1] + LElbowToWrist[1]  <<"z::"<< LShouldToElbow[2] + LElbowToWrist[2] <<endl;*/
		DATAVIEW = false;
		if(DATAVIEW){
			ocout <<position(0)<<"  "<<position(1)<<"  "<<position(2)<<"  "<< endl;
			ocout1 << LShouldToElbow[0] + LElbowToWrist[0]<<"  "<<  LShouldToElbow[1] + LElbowToWrist[1]<<"  "<< LShouldToElbow[2] + LElbowToWrist[2]<<endl;
		}
		Vector3d errorPosition ;
		errorPosition(0) = position(0) - (LShouldToElbow[0] + LElbowToWrist[0]) ;
		errorPosition(1) = position(1) - (LShouldToElbow[1] + LElbowToWrist[1]) ;
		errorPosition(2) = position(2) - (LShouldToElbow[2] + LElbowToWrist[2]) ;
		double poverError = sqrt(errorPosition(0)*errorPosition(0) + errorPosition(1)*errorPosition(1)+errorPosition(2)*errorPosition(2));
		//cout << "error ： " << poverError << endl;
		if(poverError > 0.04){
			cout << "结果不对***********************************************************" << endl;
		}else{
			LShoulderPitchResult = angle0;
			LShoulderRollResult = angle1;
			LElboYawResult = angle2;
			LElbowRollResult = angle3;
		}

 	}


	RShoulderRoll = asin(RShouldToElbow[2] / sqrt(l1*l1+l2*l2)) + atan(l1 / l2);
	RShoulderPitchAcos = acos(RShouldToElbow[0] / (l2*cos(RShoulderRoll) + l1*sin(RShoulderRoll)));
	//cout << "LShoulderPitchAcos::" << LShoulderPitchAcos<<endl;	
	RShoulderPitchAsin = asin(RShouldToElbow[1] / (l2*cos(RShoulderRoll) + l1*sin(RShoulderRoll)));
	//cout << "LShoulderPitchAsin::" << LShoulderPitchAsin<<endl;	
	RShoulderPitchAtan = atan(RShouldToElbow[1] / RShouldToElbow[0]);

	if(RShoulderPitchAsin > 0){
		RShoulderPitch =  RShoulderPitchAcos;
		//cout << "LShoulderPitch:" << -LShoulderPitchAcos <<  endl;
	}else{
		RShoulderPitch =  -RShoulderPitchAcos;
		//cout << "LShoulderPitch:" << LShoulderPitchAcos <<  endl;
	}
	float RElbowToWristTemp[3] = {0,0,0};
	RElbowToWristTemp[0] = sin(RShoulderRoll)*cos(RShoulderPitch)*RElbowToWrist[0]  + sin(RShoulderPitch)* sin(RShoulderRoll)*RElbowToWrist[1] - cos(RShoulderRoll)*RElbowToWrist[2];
	RElbowToWristTemp[1] = -sin(RShoulderPitch)*RElbowToWrist[0] + cos(RShoulderPitch)*RElbowToWrist[1];
	RElbowToWristTemp[2] = cos(RShoulderPitch)*cos(RShoulderRoll)*RElbowToWrist[0] + sin(RShoulderPitch)*cos(RShoulderRoll)*RElbowToWrist[1] + sin(RShoulderRoll)*RElbowToWrist[2];
	RElbowYaw =  -atan(RElbowToWristTemp[1] / RElbowToWristTemp[0]);
	error1 = sqrt(RElbowToWristTemp[0]*RElbowToWristTemp[0] + RElbowToWristTemp[1]*RElbowToWristTemp[1] ) / l3;
	//cout <<"error1:" << error1<<endl;
	if(error1 > 1){
		error1 = 1;
	}
	RElbowRoll = asin(error1);

	RElbowYawAcos = acos(-RElbowToWristTemp[0] / sqrt(RElbowToWristTemp[0]*RElbowToWristTemp[0] +RElbowToWristTemp[1]*RElbowToWristTemp[1]));
	//cout << "LElbowYawAcos::" << LElbowYawAcos<<endl;
	RElbowYawAsin = asin(-RElbowToWristTemp[1] / sqrt(RElbowToWristTemp[0]*RElbowToWristTemp[0] +RElbowToWristTemp[1]*RElbowToWristTemp[1]));
	//cout << "LElbowYawAsin::" << LElbowYawAsin<<endl;
	if(RElbowYawAsin > 0){
		RElbowYaw = -RElbowYawAcos;
	}else{
		RElbowYaw = RElbowYawAcos;
	}

	Lerror0 = abs(angle4 + RShoulderPitch);
	Lerror1 = abs(angle5 - RShoulderRoll);
	Lerror2 = abs(angle6 - RElbowYaw);
	Lerror3 = abs(angle7 - RElbowRoll);
	
	//***********************去抖动操作******************************//
	if(Lerror0 > FrameError){
		angle4 = -RShoulderPitch;
	}
	if(Lerror1 > FrameError){
		angle5 = RShoulderRoll;
	}
	if(Lerror2 > FrameError){
		angle6 = RElbowYaw;
	}
	if(Lerror3 > FrameError){
		angle7 = RElbowRoll;
	}
	DATAVIEW = false;
	if(DATAVIEW){
		ocout <<angle0<<","<<angle1<<","<<angle2<<","<<angle3<< ","<< time1 << endl;
		ocout1 <<-RShoulderPitch<<","<< RShoulderRoll<<","<<RElbowYaw<<","<<RElbowRoll<<","<<time1<<endl;
	}
	RShoulderPitchResult = angle4;
	RShoulderRollResult = angle5;
	RElboYawResult = angle6;
	RElbowRollResult = angle7;
	BIHUAN = false;
	if(BIHUAN){
		vector<float> Angles(4);
		Vector3d position;
		Angles[0] = angle4;
		Angles[1] = angle5;
		Angles[2] = angle6;
		Angles[3] = angle7;
		position = fRkine(Angles);
		Vector3d errorPosition ;
		errorPosition(0) = position(0) - (RShouldToElbow[0] + RElbowToWrist[0]) ;
		errorPosition(1) = position(1) - (RShouldToElbow[1] + RElbowToWrist[1]) ;
		errorPosition(2) = position(2) - (RShouldToElbow[2] + RElbowToWrist[2]) ;
		double poverError = sqrt(errorPosition(0)*errorPosition(0) + errorPosition(1)*errorPosition(1)+errorPosition(2)*errorPosition(2));
		//cout << "error ： " << poverError << endl;
		if(poverError > 0.04){
			cout << "结果不对***********************************************************" << endl;
		}else{
			RShoulderPitchResult = angle4;
			RShoulderRollResult = angle5;
			RElboYawResult = angle6;
			RElbowRollResult = angle7;
		}
	}	
	/*angle0 = -LShoulderPitch;
	angle1 = LShoulderRoll;
	angle2 = LElbowYaw;
	angle3 = LElbowRoll;*/
	//LElbowYaw = asin(LElbowToWristTemp[1] / l3 / sin(LElbowRoll));
	//cout << "LElbowYaw::" << LElbowYaw<<endl;
}
void calculRightHandAngle(){
	//DHMatrix RHandDh0to1;
	//RHandDh0to1
}
//Position (x, y, z): 0.0637386, 0.132009, -0.100519
void testNaoAPI(){
	motion.rest();
	motion.wakeUp();
	AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll");
	AL::ALValue angles      = AL::ALValue::array(0,1.36,0,0/*,angle4, angle5,angle6,angle7*/);
	float fractionMaxSpeed  = 0.1f;
	motion.setStiffnesses(names, AL::ALValue::array(1.0f,1.0f,1.0f,1.0f/*,1.0f,1.0f,1.0f,1.0f*/));
	motion.setAngles(names, angles, fractionMaxSpeed);
	qi::os::sleep(2.0f);
	string name = "LHand";
	int space = 0;
	std::vector<float> result = motion.getPosition(name, space, false);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(1)
			<< ", " << result.at(2) << std::endl;

	name = "LShoulderPitch";
	space = 0;
	result = motion.getPosition(name, space, false);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(1)
			<< ", " << result.at(2) << std::endl;
	name = "LShoulderRoll";
	space = 0;
	result = motion.getPosition(name, space, false);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(1)
			<< ", " << result.at(2) << std::endl;
	name = "LElbowYaw";
	space = 0;
	result = motion.getPosition(name, space, false);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(1)
			<< ", " << result.at(2) << std::endl;

	name = "LElbowRoll";
	space = 0;
	result = motion.getPosition(name, space, false);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(1)
			<< ", " << result.at(2) << std::endl;
	name = "LWristYaw";
	space = 0;
	result = motion.getPosition(name, space, false);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(1)
			<< ", " << result.at(2) << std::endl;

	name = "LArm";
	space = 0;
	result = motion.getPosition(name, space, false);
	std::cout << name << ":" << std::endl;
	std::cout << "Position (x, y, z): " << result.at(0) << ", " << result.at(1)
			<< ", " << result.at(2) << std::endl;
}
void testLeg(){

}
void testHand(){
	AL::ALValue names       = AL::ALValue::array("RHand", "LHand");
	AL::ALValue anglesGrip      = AL::ALValue::array(0.01f, 0.01f);//同时握拳
	AL::ALValue anglesRelease      = AL::ALValue::array(0.718f, 0.718f);//同时张开
	AL::ALValue anglesLRRG      = AL::ALValue::array(0.01f, 0.718f);//左手握拳右手张开
	AL::ALValue anglesLGRR      = AL::ALValue::array(0.718f, 0.01f);//左手张开右手握拳
	float fractionMaxSpeed  = 0.55f;
	motion.setStiffnesses(names, AL::ALValue::array(1.0f, 1.0f));
	if(stateStr[0] =="Grip ！！！..." && stateStr[1] =="Grip ！！！..."){
			LHand = 0.718f;
			RHand = 0.718f;
			motion.setAngles(names,anglesGrip,fractionMaxSpeed);
			cout<<"左手握拳"<<endl;

	}else if(stateStr[0] =="Grip ！！！..."  && stateStr[1] !="Grip ！！！..." ){
			RHand = 0.718f;
			LHand = 0.01f;
			motion.setAngles(names,anglesLGRR,fractionMaxSpeed);
			cout<<"左手张开"<<endl;
	}else if(stateStr[0] !="Grip ！！！..." && stateStr[1] =="Grip ！！！..."){
			LHand = 0.718f;
			RHand = 0.01f;
		    motion.setAngles(names,anglesLRRG,fractionMaxSpeed);
	}else{
			LHand = 0.01f;
			RHand = 0.01f;
			motion.setAngles(names,anglesRelease,fractionMaxSpeed);
	}

}
void Hand(){
	//AL::ALValue names       = AL::ALValue::array("RHand", "LHand");
	//AL::ALValue anglesGrip      = AL::ALValue::array(0.01f, 0.01f);//同时握拳
	//AL::ALValue anglesRelease      = AL::ALValue::array(0.718f, 0.718f);//同时张开
	//AL::ALValue anglesLRRG      = AL::ALValue::array(0.01f, 0.718f);//左手握拳右手张开
	//AL::ALValue anglesLGRR      = AL::ALValue::array(0.718f, 0.01f);//左手张开右手握拳
	//float fractionMaxSpeed  = 1.0f;
	//motion.setStiffnesses(names, AL::ALValue::array(1.0f, 1.0f));
	if(stateStr[0] =="Grip ！！！..." && stateStr[1] =="Grip ！！！..."){
			RHand = 0.01f;
			LHand = 0.01;
			//motion.setAngles(names,anglesGrip,fractionMaxSpeed);
			//cout<<"左手握拳"<<endl;

	}else if(stateStr[0] =="Grip ！！！..."  && stateStr[1] !="Grip ！！！..." ){
			LHand =0.01f;
			RHand = 0.718f;
			//motion.setAngles(names,anglesLGRR,fractionMaxSpeed);
			//cout<<"左手张开"<<endl;
	}else if(stateStr[0] !="Grip ！！！..." && stateStr[1] =="Grip ！！！..."){
			RHand = 0.01f;
			LHand = 0.718f;
		    //motion.setAngles(names,anglesLRRG,fractionMaxSpeed);
	}else{
			RHand = 0.718f;
			LHand = 0.718f;
			//motion.setAngles(names,anglesRelease,fractionMaxSpeed);
	}

}
void controlUpperBodyNao(){
	Hand();
	AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll","LHand","RShoulderPitch", "RShoulderRoll","RElbowYaw","RElbowRoll","RHand");
	AL::ALValue angles      = AL::ALValue::array(LShoulderPitchResult, LShoulderRollResult,LElboYawResult,LElbowRollResult,LHand,RShoulderPitchResult, RShoulderRollResult,RElboYawResult,RElbowRollResult,RHand);
	float fractionMaxSpeed  = 0.8f;
	motion.setStiffnesses(names, AL::ALValue::array(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f));
	motion.setAngles(names, angles, fractionMaxSpeed);
}
void testOriginalAngle(){
	AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll","LHand","RShoulderPitch", "RShoulderRoll","RElbowYaw","RElbowRoll","RHand");
	bool useSensors   = false;
	std::vector<float> commandAngles = motion.getAngles(names,useSensors);
	cout << "commandAngles"<< commandAngles <<endl;
	system("pause");
}
int main()
{
	/*fRkineTest();
	system("pause");*/
	/*testNaoAPI();
	system("pause");*/
	motion.rest();
	motion.wakeUp();
	//testOriginalAngle();
	
	AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll");
	bool useSensors   = false;
	std::vector<float> commandAngles = motion.getAngles(names,useSensors);

	HAngle0 = setHangle(2.0857,-2.0857,commandAngles[0]);
	HAngle1 = setHangle(1.3265,-0.3142,commandAngles[1]);
	HAngle2 = setHangle( 2.0857,-2.0857,commandAngles[2]);
	HAngle3 = setHangle(-0.0349,-1.5446,commandAngles[3]);

	//cout << "HAngle0" << HAngle0 << "HAngle1" << HAngle1 << "HAngle2" << HAngle2 << "HAngle3" << HAngle3 << endl;
	//testconventAngle();
	//system("pause");
	/*ikineTest();*/
	////fkineTest();
	ocout.open("angle.txt");
	ocout1.open("originalAngle.txt");

	/*motion.rest();
	motion.wakeUp();*/
	ConnectKinect();
	HRESULT hr;
	m_hNextInteractionEvent = CreateEvent( NULL,TRUE,FALSE,NULL );
	m_hEvNuiProcessStop = CreateEvent(NULL,TRUE,FALSE,NULL);
	hr =  NuiCreateInteractionStream(m_pNuiSensor,(INuiInteractionClient *)&m_nuiIClient,&m_nuiIStream);
	if( FAILED( hr ) )
	{
		cout<<"Could not open Interation stream video"<<endl;
		return hr;
	}
	hr = m_nuiIStream->Enable(m_hNextInteractionEvent);
	if( FAILED( hr ) )
	{
		cout<<"Could not open Interation stream video"<<endl;
		return hr;
	}
	HANDLE m_hProcesss = CreateThread(NULL, 0, KinectDataThread, 0, 0, 0);
	while(1)
	{
		//testHand();
		controlUpperBodyNao();
		//Sleep(1);
	}
	m_pNuiSensor->NuiShutdown();
	SafeRelease(m_pNuiSensor);
	return 0;
}

void drawSkeleton(Mat &image, CvPoint pointSet[], int whichone)   
{   
    CvScalar color;   
    switch(whichone) //跟踪不同的人显示不同的颜色   
    {   
    case 0:   
        color = cvScalar(255);   
        break;   
    case 1:   
        color = cvScalar(0,255);   
        break;   
    case 2:   
        color = cvScalar(0, 0, 255);   
        break;   
    case 3:   
        color = cvScalar(255, 255, 0);   
        break;   
    case 4:   
        color = cvScalar(255, 0, 255);   
        break;   
    case 5:   
        color = cvScalar(0, 255, 255);   
        break;   
    }   
    if((pointSet[NUI_SKELETON_POSITION_HEAD].x!=0 || pointSet[NUI_SKELETON_POSITION_HEAD].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HEAD], pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SPINE].x!=0 || pointSet[NUI_SKELETON_POSITION_SPINE].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SPINE], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SPINE].x!=0 || pointSet[NUI_SKELETON_POSITION_SPINE].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SPINE], pointSet[NUI_SKELETON_POSITION_HIP_CENTER], color, 2);   
   
   // 左上肢   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HAND_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_HAND_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], pointSet[NUI_SKELETON_POSITION_HAND_LEFT], color, 2);   
   
   // 右上肢   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], pointSet[NUI_SKELETON_POSITION_HAND_RIGHT], color, 2);   
   
   // 左下肢   
    if((pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HIP_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_CENTER], pointSet[NUI_SKELETON_POSITION_HIP_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_HIP_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_LEFT], pointSet[NUI_SKELETON_POSITION_KNEE_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_KNEE_LEFT], pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].y!=0) &&    
        (pointSet[NUI_SKELETON_POSITION_FOOT_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_FOOT_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT], pointSet[NUI_SKELETON_POSITION_FOOT_LEFT], color, 2);   
   
   // 右下肢   
    if((pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_CENTER], pointSet[NUI_SKELETON_POSITION_HIP_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_RIGHT], pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT],color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT], pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT], pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT], color, 2);   
}   
void getDepthImage(HANDLE &depthEvent, HANDLE &depthStreamHandle, Mat &depthImage,CvPoint skeletonPoint1[6][20],NUI_SKELETON_FRAME skeletonFrame)   
{   
    const NUI_IMAGE_FRAME *depthFrame = NULL;   
   
    NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame);   //从刚才打开数据流的流句柄中得到该帧数据，读取到的数据地址存于pImageFrame
    INuiFrameTexture *pTexture = depthFrame->pFrameTexture;     
   
    NUI_LOCKED_RECT LockedRect;   
    pTexture->LockRect(0, &LockedRect, NULL, 0);     
   //4.3、提取数据帧到LockedRect，它包括两个数据对象：pitch每行字节数，pBits第一个字节location
//	并锁定数据，这样当我们读数据的时候，kinect就不会去修改它  
    RGBQUAD q;   
  
    if( LockedRect.Pitch != 0 ) //判断数据是否有效  
    {   
		//讲数据转换成opencv的Mat格式
        for (int i=0; i<depthImage.rows; i++)  //循环Mat的行 
        {   
            uchar *ptr = depthImage.ptr<uchar>(i);   //第i行的指针
            uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;  //每个字节代表一个颜色信息，直接用uchar
            USHORT *pBufferRun = (USHORT*) pBuffer;   //这里需要转换因为每个数据是两个字节，存储的同上面的颜色信息不同
              
            for (int j=0; j<depthImage.cols; j++)   
            {   
                int player = pBufferRun[j]&7;   //获得ID信息
                int data = (pBufferRun[j]&0xfff8) >> 3;   //获得图像信息
                   
                uchar imageData = 255-(uchar)(256*data/0x0fff);   
                q.rgbBlue = q.rgbGreen = q.rgbRed = 0;   
			//	根据不同用户调节用户的颜色
                switch(player)   
                {   
                    case 0:     
                        q.rgbRed = imageData / 2;     
                        q.rgbBlue = imageData / 2;     
                        q.rgbGreen = imageData / 2;     
                        break;     
                    case 1:      
                        q.rgbRed = imageData;     
                        break;     
                    case 2:     
                        q.rgbGreen = imageData;     
                        break;     
                    case 3:     
                        q.rgbRed = imageData / 4;     
                        q.rgbGreen = q.rgbRed*4;  //这里利用乘的方法，而不用原来的方法可以避免不整除的情况   
                        q.rgbBlue = q.rgbRed*4;  //可以在后面的getTheContour()中配合使用，避免遗漏一些情况   
                        break;     
                    case 4:     
                        q.rgbBlue = imageData / 4;    
                        q.rgbRed = q.rgbBlue*4;     
                        q.rgbGreen = q.rgbBlue*4;     
                        break;     
                    case 5:     
                        q.rgbGreen = imageData / 4;    
                        q.rgbRed = q.rgbGreen*4;     
                        q.rgbBlue = q.rgbGreen*4;     
                        break;     
                    case 6:     
                        q.rgbRed = imageData / 2;     
                        q.rgbGreen = imageData / 2;      
                        q.rgbBlue = q.rgbGreen*2;     
                        break;     
                    case 7:     
                        q.rgbRed = 255 - ( imageData / 2 );     
                        q.rgbGreen = 255 - ( imageData / 2 );     
                        q.rgbBlue = 255 - ( imageData / 2 );   
                }      
                ptr[3*j] = q.rgbBlue;   
                ptr[3*j+1] = q.rgbGreen;   
                ptr[3*j+2] = q.rgbRed;   
            }   
        }   
    }   
    else   
    {   
        cout << "捕捉深度图像出现错误" << endl;   
    }   

	for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i){
		 NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
	    if(NUI_SKELETON_TRACKED == trackingState){
			if ( tracked[i] ) {
				for(int j = 0; j < NUI_SKELETON_POSITION_COUNT ; j++){
					if(((skeletonPoint[i][j].x)	!= 0 && (skeletonPoint[i][j].y)!=0)){
						circle(depthImage, skeletonPoint[i][j], 3, cvScalar(0, 0, 255), 1, 8, 0);
					}
				}

			}
			drawSkeleton(depthImage, skeletonPoint[i], i);//第一个参数是Mat，第二个是cvpoint  第三个是第几个人人
		}
	}
}
int getLeftHandPostion(const NUI_SKELETON_DATA & skel){
	float ShouldToElbow[3] ={0,0,0};
	float ShouldToElbowUnit[3] ={0,0,0};
	float ElbowToWrist[3] ={0,0,0};
	float ElbowToWristUnit[3] ={0,0,0};
	float naoTorsoToShoulder[3] = {0,0,0};
	float robotShouldToElbow[3] = {0,0,0};
	float robotElbowToWrist[3] ={0,0,0};
	float robotElbowToHand[3] ={0,0,0};

	float LElbowToHand[3] = {0,0,0};
	/*naoTorsoToShoulder[0] = 0;
	naoTorsoToShoulder[1] = 98;
	naoTorsoToShoulder[2] = 100;*/
	ShouldToElbow[0] = skel.SkeletonPositions[5].x - skel.SkeletonPositions[4].x;
	ShouldToElbow[1] = skel.SkeletonPositions[5].y - skel.SkeletonPositions[4].y;
	ShouldToElbow[2] = skel.SkeletonPositions[5].z - skel.SkeletonPositions[4].z;
	ShouldToElbowUnit[0] = ShouldToElbow[0] / sqrt(ShouldToElbow[0]*ShouldToElbow[0] +ShouldToElbow[1]*ShouldToElbow[1]+ ShouldToElbow[2]*ShouldToElbow[2]);
	ShouldToElbowUnit[1] = ShouldToElbow[1] / sqrt(ShouldToElbow[0]*ShouldToElbow[0] +ShouldToElbow[1]*ShouldToElbow[1]+ ShouldToElbow[2]*ShouldToElbow[2]);
	ShouldToElbowUnit[2] = ShouldToElbow[2] / sqrt(ShouldToElbow[0]*ShouldToElbow[0] +ShouldToElbow[1]*ShouldToElbow[1]+ ShouldToElbow[2]*ShouldToElbow[2]);
	ElbowToWrist[0] = skel.SkeletonPositions[6].x - skel.SkeletonPositions[5].x;
	ElbowToWrist[1] = skel.SkeletonPositions[6].y - skel.SkeletonPositions[5].y;
	ElbowToWrist[2] = skel.SkeletonPositions[6].z - skel.SkeletonPositions[5].z;
	ElbowToWristUnit[0] = ElbowToWrist[0] / sqrt(ElbowToWrist[0]*ElbowToWrist[0] +ElbowToWrist[1]*ElbowToWrist[1]+ ElbowToWrist[2]*ElbowToWrist[2]);
	ElbowToWristUnit[1] = ElbowToWrist[1] / sqrt(ElbowToWrist[0]*ElbowToWrist[0] +ElbowToWrist[1]*ElbowToWrist[1]+ ElbowToWrist[2]*ElbowToWrist[2]);
	ElbowToWristUnit[2] = ElbowToWrist[2] / sqrt(ElbowToWrist[0]*ElbowToWrist[0] +ElbowToWrist[1]*ElbowToWrist[1]+ ElbowToWrist[2]*ElbowToWrist[2]);
	robotShouldToElbow[0] = ShouldToElbowUnit[0] * ShoulderToElbowLength/ 1000;
	robotShouldToElbow[1] = ShouldToElbowUnit[1] * ShoulderToElbowLength/ 1000;
	robotShouldToElbow[2] = ShouldToElbowUnit[2] * ShoulderToElbowLength/ 1000;
	//cout << "robotShouldToElbow[0] :: " << robotShouldToElbow[2] << "robotShouldToElbow[1] ::"<<robotShouldToElbow[0] << "robotShouldToElbow[2] ::"<<robotShouldToElbow[1] <<endl;
	//cout << "robotShouldToElbow[0] :: " << -robotShouldToElbow[2] << "robotShouldToElbow[1] ::"<<-robotShouldToElbow[0] + 0.098 << "robotShouldToElbow[2] ::"<<robotShouldToElbow[1] + 0.1 <<endl;
	robotElbowToWrist[0] = ElbowToWristUnit[0] * (ElbowToWristLength) / 1000;
	robotElbowToWrist[1] = ElbowToWristUnit[1] * (ElbowToWristLength) / 1000;
	robotElbowToWrist[2] = ElbowToWristUnit[2] * (ElbowToWristLength) / 1000;
	//cout << "robotElbowToWrist[0] :: " << robotElbowToWrist[2] << "robotShouldToElbow[1] ::"<<robotElbowToWrist[0] << "robotShouldToElbow[2] ::"<<robotElbowToWrist[1] <<endl;
	LShouldToElbow[0] = -robotShouldToElbow[2];
	LShouldToElbow[1] = -robotShouldToElbow[0];
	LShouldToElbow[2] = robotShouldToElbow[1];

	//cout << "LShouldToElbow" << endl;
	//cout <<LShouldToElbow[0]<< ", " << LShouldToElbow[1]<< ", " <<LShouldToElbow[2]<< ", " <<endl;
	LElbowToWrist[0] = -robotElbowToWrist[2];
	LElbowToWrist[1] = -robotElbowToWrist[0];
	LElbowToWrist[2] = robotElbowToWrist[1];
	//cout << "LElbowToWrist" << endl;
	//cout <<LElbowToWrist[0]<< ", " << LElbowToWrist[1]<< ", " <<LElbowToWrist[2]<< ", " <<endl;
	NewtonIK = false;
	if(NewtonIK){
		LShouldToWrist[0] = LElbowToWrist[0] + LShouldToElbow[0];
		LShouldToWrist[1] = LElbowToWrist[1] + LShouldToElbow[1];
		LShouldToWrist[2] = LElbowToWrist[2] + LShouldToElbow[2];
	}
	bool naoAPI = false;
	if(naoAPI){
		robotElbowToHand[0] =  ElbowToWristUnit[0] * (ElbowToWristLength + WristToHandLength) / 1000; 
		robotElbowToHand[1] =  ElbowToWristUnit[1] * (ElbowToWristLength + WristToHandLength) / 1000;  
		robotElbowToHand[2] =  ElbowToWristUnit[2] * (ElbowToWristLength + WristToHandLength) / 1000;  

		LElbowToHand[0] = -robotElbowToHand[2];
		LElbowToHand[1] = -robotElbowToHand[0];
		LElbowToHand[2] = robotElbowToHand[1];

		LShouldToHand[0] = LElbowToHand[0] + LShouldToElbow[0];
		LShouldToHand[1] = LElbowToHand[1] + LShouldToElbow[1] + 0.098;
		LShouldToHand[2] = LElbowToHand[2] + LShouldToElbow[2] + 0.1;
		cout << "LShouldToHand"<<LShouldToHand[0]<< ","<<LShouldToHand[1] <<  ","<<LShouldToHand[2] <<endl;
	}

	ShouldToElbow[0] = skel.SkeletonPositions[9].x - skel.SkeletonPositions[8].x;
	ShouldToElbow[1] = skel.SkeletonPositions[9].y - skel.SkeletonPositions[8].y;
	ShouldToElbow[2] = skel.SkeletonPositions[9].z - skel.SkeletonPositions[8].z;
	ShouldToElbowUnit[0] = ShouldToElbow[0] / sqrt(ShouldToElbow[0]*ShouldToElbow[0] +ShouldToElbow[1]*ShouldToElbow[1]+ ShouldToElbow[2]*ShouldToElbow[2]);
	ShouldToElbowUnit[1] = ShouldToElbow[1] / sqrt(ShouldToElbow[0]*ShouldToElbow[0] +ShouldToElbow[1]*ShouldToElbow[1]+ ShouldToElbow[2]*ShouldToElbow[2]);
	ShouldToElbowUnit[2] = ShouldToElbow[2] / sqrt(ShouldToElbow[0]*ShouldToElbow[0] +ShouldToElbow[1]*ShouldToElbow[1]+ ShouldToElbow[2]*ShouldToElbow[2]);
	ElbowToWrist[0] = skel.SkeletonPositions[10].x - skel.SkeletonPositions[9].x;
	ElbowToWrist[1] = skel.SkeletonPositions[10].y - skel.SkeletonPositions[9].y;
	ElbowToWrist[2] = skel.SkeletonPositions[10].z - skel.SkeletonPositions[9].z;
	ElbowToWristUnit[0] = ElbowToWrist[0] / sqrt(ElbowToWrist[0]*ElbowToWrist[0] +ElbowToWrist[1]*ElbowToWrist[1]+ ElbowToWrist[2]*ElbowToWrist[2]);
	ElbowToWristUnit[1] = ElbowToWrist[1] / sqrt(ElbowToWrist[0]*ElbowToWrist[0] +ElbowToWrist[1]*ElbowToWrist[1]+ ElbowToWrist[2]*ElbowToWrist[2]);
	ElbowToWristUnit[2] = ElbowToWrist[2] / sqrt(ElbowToWrist[0]*ElbowToWrist[0] +ElbowToWrist[1]*ElbowToWrist[1]+ ElbowToWrist[2]*ElbowToWrist[2]);
	robotShouldToElbow[0] = ShouldToElbowUnit[0] * ShoulderToElbowLength/ 1000;
	robotShouldToElbow[1] = ShouldToElbowUnit[1] * ShoulderToElbowLength/ 1000;
	robotShouldToElbow[2] = ShouldToElbowUnit[2] * ShoulderToElbowLength/ 1000;
	//cout << "robotShouldToElbow[0] :: " << robotShouldToElbow[2] << "robotShouldToElbow[1] ::"<<robotShouldToElbow[0] << "robotShouldToElbow[2] ::"<<robotShouldToElbow[1] <<endl;
	//cout << "robotShouldToElbow[0] :: " << -robotShouldToElbow[2] << "robotShouldToElbow[1] ::"<<-robotShouldToElbow[0] + 0.098 << "robotShouldToElbow[2] ::"<<robotShouldToElbow[1] + 0.1 <<endl;
	robotElbowToWrist[0] = ElbowToWristUnit[0] * (ElbowToWristLength) / 1000;
	robotElbowToWrist[1] = ElbowToWristUnit[1] * (ElbowToWristLength) / 1000;
	robotElbowToWrist[2] = ElbowToWristUnit[2] * (ElbowToWristLength) / 1000;
	//cout << "robotElbowToWrist[0] :: " << robotElbowToWrist[2] << "robotShouldToElbow[1] ::"<<robotElbowToWrist[0] << "robotShouldToElbow[2] ::"<<robotElbowToWrist[1] <<endl;
	RShouldToElbow[0] = -robotShouldToElbow[2];
	RShouldToElbow[1] = -robotShouldToElbow[0];
	RShouldToElbow[2] = robotShouldToElbow[1];

	//cout << "LShouldToElbow" << endl;
	//cout <<LShouldToElbow[0]<< ", " << LShouldToElbow[1]<< ", " <<LShouldToElbow[2]<< ", " <<endl;
	RElbowToWrist[0] = -robotElbowToWrist[2];
	RElbowToWrist[1] = -robotElbowToWrist[0];
	RElbowToWrist[2] = robotElbowToWrist[1];

	controlNao();
	//calculAngle();
	//cout << "robotElbowToWrist[0] :: " << -robotElbowToWrist[2] << "robotElbowToWrist[1] ::"<<-robotElbowToWrist[0] + 0.098 << "robotElbowToWrist[2] ::"<<robotElbowToWrist[1] + 0.1 <<endl;
	/*resultPosition[0] = (robotShouldToElbow[0] + robotElbowToWrist[0] + naoTorsoToShoulder[0]) /  1000 ;
	resultPosition[1] = (robotShouldToElbow[1] + robotElbowToWrist[1] + naoTorsoToShoulder[1]) /  1000 ;
	resultPosition[2] = (robotShouldToElbow[2] + robotElbowToWrist[2] + naoTorsoToShoulder[2]) /  1000 ;
	cout << "resultPosition[0] :: " << -resultPosition[2] << "resultPosition[1] ::"<<-resultPosition[0] + 0.098 << "resultPosition[2] ::"<<resultPosition[1] + 0.1 <<endl;*/
	return 0;
}
float getAngleForSkel(NUI_SKELETON_DATA skel,int i){
	float a[3] = { 0,0,0 };
	float b[3] = { 0,0,0 };
	float c[3] = { 0,0,0 };
	float d[3] = { 0,0,0 };
	float e[3] = { 0,0,0 };
	float q, p, j,n,angle;
	float PI = 3.1415926;
	int m = 1;
	switch (i)
	{
	case 0:                                                      //lshoulderpitch 机器人角度范围 （82.7 -127.2）  （-1.44 -2.22）
	{
			  a[0] = skel.SkeletonPositions[5].y - skel.SkeletonPositions[4].y;
			  a[1] = skel.SkeletonPositions[5].z - skel.SkeletonPositions[4].z;
			  a[2] = skel.SkeletonPositions[5].x - skel.SkeletonPositions[4].x;
			  b[0] = skel.SkeletonPositions[1].y - skel.SkeletonPositions[2].y;
			  b[1] = skel.SkeletonPositions[1].z - skel.SkeletonPositions[2].z;
			  if (a[0] > 0)
				  m = -1;
			  q = a[0] * b[0] + a[1] * b[1];
	          p = sqrt(a[0] * a[0] + a[1] * a[1]);
	          j = sqrt(b[0] * b[0] + b[1] * b[1]);
			  if((a[1] > 0)&&(a[0] < 0))                         //后下方
				  angle = m*(acos(q / p / j) + PI / 2);
			  else if((a[1] < 0)&&(a[0] < 0))                   //前下
				  angle = m*(PI / 2 - acos(q / p / j));
			  else if((a[1] < 0)&&(a[0] > 0))                   //前上
				  angle = m*(acos(q / p / j) - PI / 2);
			  else if((a[1] > 0)&&(a[0] > 0))                   //后上
				  angle = m*(PI * 3/2 - acos(q / p / j));
			  else if(a[1] == 0)
				  angle = 0;
			  else if((a[2] == 0)&&(a[0] > 0))
				  angle = -PI / 2;
			  else if((a[2] == 0)&&(a[0] < 0))
				  angle =  PI / 2;
			 
			  break;
	}
	case 2:                                                           //lshoulderroll
	{
			  a[0] = skel.SkeletonPositions[5].x - skel.SkeletonPositions[4].x;
			  a[1] = skel.SkeletonPositions[5].y - skel.SkeletonPositions[4].y;
			  a[2] = skel.SkeletonPositions[5].z - skel.SkeletonPositions[4].z;
			  b[0] = skel.SkeletonPositions[8].x - skel.SkeletonPositions[4].x;
			  b[1] = skel.SkeletonPositions[8].y - skel.SkeletonPositions[4].y;
			  b[2] = skel.SkeletonPositions[8].z - skel.SkeletonPositions[4].z;
			  if (a[0] > 0)
				  m = -1;

			  q = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	          p = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	          j = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
			 
			  
			  if (a[1]<0&&a[0]>0)                    //右下
				  angle = m*(PI /2 - acos(q / p / j));
			  else if(a[1]<0&&a[0]<0)                //左下
				  angle = m*(acos(q / p / j) - PI /2);
			  else if(a[1]>0&&a[0]<0)                //左上                    ..........................................
				  angle = m*(acos(q / p / j) - PI /2);
			  else if(a[1]>0&&a[0]>0)                //右上
				  angle = 0;
			   else if(a[1]==0)
                  angle = 0;         // m*asin(abs(a[0])/n)
			  else if(a[0]==0)        //上臂处于垂直平面
			      angle = 0;

			  if(p/j<0.3)
				  angle = 0;
			  break;
	}
	case 1:                                                                 //lelbowroll
		{
		a[0] = skel.SkeletonPositions[4].x - skel.SkeletonPositions[5].x;
	    a[1] = skel.SkeletonPositions[4].y - skel.SkeletonPositions[5].y;
	    a[2] = skel.SkeletonPositions[4].z - skel.SkeletonPositions[5].z;
		b[0] = skel.SkeletonPositions[6].x - skel.SkeletonPositions[5].x;
	    b[1] = skel.SkeletonPositions[6].y - skel.SkeletonPositions[5].y;
	    b[2] = skel.SkeletonPositions[6].z - skel.SkeletonPositions[5].z;

		q = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	    p = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	    j = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
		if (acos(q / p / j) > PI / 2)
			angle = acos(q / p / j) - PI;
		else
			angle = -PI/2;
		break;
		}
	case 3:                                                               //lelbowyaw
		{
		a[0] = skel.SkeletonPositions[4].x - skel.SkeletonPositions[5].x;
	    a[1] = skel.SkeletonPositions[4].y - skel.SkeletonPositions[5].y;
	    a[2] = skel.SkeletonPositions[4].z - skel.SkeletonPositions[5].z;
		b[0] = skel.SkeletonPositions[6].x - skel.SkeletonPositions[5].x;
	    b[1] = skel.SkeletonPositions[6].y - skel.SkeletonPositions[5].y;
	    b[2] = skel.SkeletonPositions[6].z - skel.SkeletonPositions[5].z;

		c[0] = skel.SkeletonPositions[8].x - skel.SkeletonPositions[4].x;
	    c[1] = skel.SkeletonPositions[8].y - skel.SkeletonPositions[4].y;
	    c[2] = skel.SkeletonPositions[8].z - skel.SkeletonPositions[4].z;
		d[0] = skel.SkeletonPositions[5].x - skel.SkeletonPositions[4].x;
	    d[1] = skel.SkeletonPositions[5].y - skel.SkeletonPositions[4].y;
	    d[2] = skel.SkeletonPositions[5].z - skel.SkeletonPositions[4].z;
		float abcross[3] = {0,0,0};
		float cdcross[3] = {0,0,0};
		
		abcross[0] = a[1]*b[2] - a[2]*b[1];
		abcross[1] = a[2]*b[0] - a[0]*b[2];
		abcross[2] = a[0]*b[1] - a[1]*b[0];

		cdcross[0] = c[1]*d[2] - c[2]*d[1];
		cdcross[1] = c[2]*d[0] - c[0]*d[2];
		cdcross[2] = c[0]*d[1] - c[1]*d[0];
		q = abcross[0] * cdcross[0] + abcross[1] * cdcross[1] + abcross[2] * cdcross[2];
	    p = sqrt(abcross[0] * abcross[0] + abcross[1] * abcross[1] + abcross[2] * abcross[2]);
	    j = sqrt(cdcross[0] * cdcross[0] + cdcross[1] * cdcross[1] + cdcross[2] * cdcross[2]);

		if(cdcross[0]*b[0] + cdcross[1]*b[1] + cdcross[2]*b[2]>0)
			angle = - acos(q / p / j);
		else if(cdcross[0]*b[0] + cdcross[1]*b[1] + cdcross[2]*b[2]==0)
			angle = 0;
		//*****************************************************************angle = 0;
			
		else if(cdcross[0]*b[0] + cdcross[1]*b[1] + cdcross[2]*b[2]<0)
			angle = acos(q / p / j);
		else angle = PI/2;  

	
		break;
		}

	case 4:                                                           //rshoulderpitch
		{
		      a[0] = skel.SkeletonPositions[9].y - skel.SkeletonPositions[8].y;
			  a[1] = skel.SkeletonPositions[9].z - skel.SkeletonPositions[8].z;
			  a[2] = skel.SkeletonPositions[9].x - skel.SkeletonPositions[8].x;
			  b[0] = skel.SkeletonPositions[1].y - skel.SkeletonPositions[2].y;
			  b[1] = skel.SkeletonPositions[1].z - skel.SkeletonPositions[2].z;

			   if (a[0] > 0)
				  m = -1;

			  q = a[0] * b[0] + a[1] * b[1];
	          p = sqrt(a[0] * a[0] + a[1] * a[1]);
	          j = sqrt(b[0] * b[0] + b[1] * b[1]);

			  if((a[1] > 0)&&(a[0] < 0))                         //后下方
				  angle = m*(acos(q / p / j) + PI /2);
			  else if((a[1] < 0)&&(a[0] < 0))                   //前下
				  angle = m*(PI /2 - acos(q / p / j));
			  else if((a[1] < 0)&&(a[0] > 0))                   //前上
				  angle = m*(acos(q / p / j) - PI /2);
			  else if((a[1] > 0)&&(a[0] > 0))                   //后上
				  angle = m*(PI *3/2 - acos(q / p / j));
			  else if(a[1] == 0)
				  angle = 0;
			  else if((a[2] == 0)&&(a[0] > 0))             //手肘呈直角往上
				  angle = -PI /2;
			  else if((a[2] == 0)&&(a[0] < 0))               //手肘呈直角往下
				  angle = PI /2;
			   
			break;
		}

	case 5:                                                //rshoulderroll
		{
			  a[0] = skel.SkeletonPositions[9].x - skel.SkeletonPositions[8].x;
			  a[1] = skel.SkeletonPositions[9].y - skel.SkeletonPositions[8].y;
			  a[2] = skel.SkeletonPositions[9].z - skel.SkeletonPositions[8].z;
			  b[0] = skel.SkeletonPositions[4].x - skel.SkeletonPositions[8].x;
			  b[1] = skel.SkeletonPositions[4].y - skel.SkeletonPositions[8].y;
			  b[2] = skel.SkeletonPositions[4].z - skel.SkeletonPositions[8].z;
			  if (a[0] > 0)
				  m = -1;

			  q = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	          p = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	          j = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] *b[2]);
			  n = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
			  
			  if (a[1]<0&&a[0]>0)                    //右下
				  angle = m*(acos(q / p / j) - PI/2);              ///PI/2
			  else if(a[1]<0&&a[0]<0)              //左下
				  angle = m*( PI /2 - acos(q / p / j));
			  else if(a[1]>0&&a[0]<0)              //左上                    ..........................................
				  angle = 0;
			  else if(a[1]>0&&a[0]>0)              //右上
				  angle = m*(acos(q / p / j) - PI/2);
			   else if(a[1]==0)
                  angle = 0;         //
			  else if(a[0]==0)        //上臂处于垂直平面
			      angle = 0;
		break;
		}
	case 6:                                             //relbowroll
		{
		a[0] = skel.SkeletonPositions[8].x - skel.SkeletonPositions[9].x;
	    a[1] = skel.SkeletonPositions[8].y - skel.SkeletonPositions[9].y;
	    a[2] = skel.SkeletonPositions[8].z - skel.SkeletonPositions[9].z;
		b[0] = skel.SkeletonPositions[10].x - skel.SkeletonPositions[9].x;
	    b[1] = skel.SkeletonPositions[10].y - skel.SkeletonPositions[9].y;
	    b[2] = skel.SkeletonPositions[10].z - skel.SkeletonPositions[9].z;

		q = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	    p = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	    j = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
		if (acos(q / p / j) > PI/2)
			angle = PI-acos(q / p / j);
		else
			angle = PI/2;
		break;
		}
	case 7:                                                                  //relbowyaw
		{
		a[0] = skel.SkeletonPositions[10].x - skel.SkeletonPositions[9].x;
	    a[1] = skel.SkeletonPositions[10].y - skel.SkeletonPositions[9].y;
	    a[2] = skel.SkeletonPositions[10].z - skel.SkeletonPositions[9].z;
		b[0] = skel.SkeletonPositions[8].x - skel.SkeletonPositions[9].x;
	    b[1] = skel.SkeletonPositions[8].y - skel.SkeletonPositions[9].y;
	    b[2] = skel.SkeletonPositions[8].z - skel.SkeletonPositions[9].z;

		c[0] = skel.SkeletonPositions[9].x - skel.SkeletonPositions[8].x;
	    c[1] = skel.SkeletonPositions[9].y - skel.SkeletonPositions[8].y;
	    c[2] = skel.SkeletonPositions[9].z - skel.SkeletonPositions[8].z;
		d[0] = skel.SkeletonPositions[4].x - skel.SkeletonPositions[8].x;
	    d[1] = skel.SkeletonPositions[4].y - skel.SkeletonPositions[8].y;
	    d[2] = skel.SkeletonPositions[4].z - skel.SkeletonPositions[8].z;
		float abcross[3] = {0,0,0};
		float cdcross[3] = {0,0,0};
		
		abcross[0] = a[1]*b[2] - a[2]*b[1];
		abcross[1] = a[2]*b[0] - a[0]*b[2];
		abcross[2] = a[0]*b[1] - a[1]*b[0];

		cdcross[0] = c[1]*d[2] - c[2]*d[1];
		cdcross[1] = c[2]*d[0] - c[0]*d[2];
		cdcross[2] = c[0]*d[1] - c[1]*d[0];
		q = abcross[0] * cdcross[0] + abcross[1] * cdcross[1] + abcross[2] * cdcross[2];
	    p = sqrt(abcross[0] * abcross[0] + abcross[1] * abcross[1] + abcross[2] * abcross[2]);
	    j = sqrt(cdcross[0] * cdcross[0] + cdcross[1] * cdcross[1] + cdcross[2] * cdcross[2]);

		if(cdcross[0]*a[0] + cdcross[1]*a[1] + cdcross[2]*a[2]>0)
			angle = acos(q / p / j);
		else if(cdcross[0]*a[0] + cdcross[1]*a[1] + cdcross[2]*a[2]==0)
			angle = 0;
		//*****************************************************************angle = 0;
			
		else if(cdcross[0]*a[0] + cdcross[1]*a[1] + cdcross[2]*a[2]<0)
			angle = - acos(q / p / j);
		else angle = PI/2;  
		break;
		}

	case 8:
		{
		a[0] = skel.SkeletonPositions[2].x - skel.SkeletonPositions[1].x;                                             //skel.SkeletonPositions[2].x - skel.SkeletonPositions[1].x;
	    a[1] = skel.SkeletonPositions[2].y - skel.SkeletonPositions[1].y;                                             //skel.SkeletonPositions[2].y - skel.SkeletonPositions[1].y;
	    a[2] = skel.SkeletonPositions[2].z - skel.SkeletonPositions[1].z;                                              //skel.SkeletonPositions[2].z - skel.SkeletonPositions[1].z;
		
		d[0] = (skel.SkeletonPositions[8].x + skel.SkeletonPositions[4].x)/2;
	    d[1] = (skel.SkeletonPositions[8].y + skel.SkeletonPositions[4].y)/2;
	    d[2] = (skel.SkeletonPositions[8].z + skel.SkeletonPositions[4].z)/2;
		b[0] = skel.SkeletonPositions[3].x - d[0];
	    b[1] = skel.SkeletonPositions[3].y - d[1];
	    b[2] = skel.SkeletonPositions[3].z - d[2];



		q = a[1] * b[1] + a[2] * b[2];
	    p = sqrt(a[1] * a[1] + a[2] * a[2]);
	    j = sqrt(b[1] * b[1] + b[2] * b[2]);

		if(b[2]<0)
			angle = acos(q / p / j);
		else if(b[2]>0)
			angle = -acos(q / p / j);
		else
			angle = 0;
		
			break;
		}
	case 9:
		{
		d[0] =  (skel.SkeletonPositions[8].x + skel.SkeletonPositions[4].x)/2;                                                 //(skel.SkeletonPositions[8].x + skel.SkeletonPositions[4].x)/2;
	    d[1] =  (skel.SkeletonPositions[8].y + skel.SkeletonPositions[4].y)/2;                                                                          //(skel.SkeletonPositions[8].y + skel.SkeletonPositions[4].y)/2;
	    d[2] =  (skel.SkeletonPositions[8].z + skel.SkeletonPositions[4].z)/2;                                                                             //(skel.SkeletonPositions[8].z + skel.SkeletonPositions[4].z)/2;
		a[0] = skel.SkeletonPositions[1].x - skel.SkeletonPositions[3].x;
	    a[1] = skel.SkeletonPositions[1].y - skel.SkeletonPositions[3].y;
	    a[2] = skel.SkeletonPositions[1].z - skel.SkeletonPositions[3].z;
		b[0] = skel.SkeletonPositions[2].x - skel.SkeletonPositions[3].x;
	    b[1] = skel.SkeletonPositions[2].y - skel.SkeletonPositions[3].y;
	    b[2] = skel.SkeletonPositions[2].z - skel.SkeletonPositions[3].z;

		float abcross[3] = {0,0,0};	
		abcross[0] = a[1]*b[2] - a[2]*b[1];
		abcross[1] = a[2]*b[0] - a[0]*b[2];
		abcross[2] = a[0]*b[1] - a[1]*b[0];
		

		e[0] = skel.SkeletonPositions[4].x - skel.SkeletonPositions[8].x;
	    e[1] = skel.SkeletonPositions[4].y - skel.SkeletonPositions[8].y;
	    e[2] = skel.SkeletonPositions[4].z - skel.SkeletonPositions[8].z;

		q = abcross[0] * e[0] + abcross[1] * e[1] + abcross[2] * e[2];
	    p = sqrt(abcross[0] * abcross[0] + abcross[1] * abcross[1] +abcross[2] * abcross[2]);
	    j = sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
			
		
		if(d[0]<skel.SkeletonPositions[3].x)                      
		{  if(acos(q / p/j)<PI/2)
		    angle = - acos(q / p/j);
		else
			angle =0;
		}

		else if(d[0]>skel.SkeletonPositions[3].x)                      
		{  if(acos(q / p/j)<PI/2)
		    angle = acos(q / p/j);
		else
			angle =0;
		}
		else 
			angle = 0;
		break;
		}
	}
	

	return(angle);

}

void getAndDrawAngle(const NUI_SKELETON_DATA & skel)
{      
	angle1 = getAngleForSkel(skel, 0);
	angle2 = getAngleForSkel(skel, 1);
	angle3 = getAngleForSkel(skel, 2);
	angle4 = getAngleForSkel(skel, 3);
	angle5 = getAngleForSkel(skel, 4);
	angle6 = getAngleForSkel(skel, 5);
	angle7 = getAngleForSkel(skel, 6);
	angle8 = getAngleForSkel(skel, 7);
	angle9 = getAngleForSkel(skel, 8);
	angle10 = getAngleForSkel(skel, 9);

	if(angle1 >-2.0857&& angle1 <2.0857)    
	    angle1 = angle1;
	else{
		if (angle1 > 0)
	         angle1 = 2.0857;
		else
			angle1 = -2.0857;
	}
	angleArr[arrIndex]  = angle1;
	arrIndex = arrIndex + 1;
	cout<<"角度"<< angle1<< endl;


	if(angle3 > -0.3142 && angle3 < 1.3265)    
	    angle3 = angle3;
	else
		{
			if (angle3 > 0)
				angle3 = 1.3265;
			else 
				angle3 = -0.3142;
	}

	if( angle2 > -1.5446 && angle2 < -0.0349)    
	    angle2 = angle2;
	else
		{
		if (angle2 > -0.0349)
	         angle2 = -0.0349;
		else 
			angle2 = -1.5446;
	}

	if(angle4 > -2.0857 && angle4 < 2.0857)    
	    angle4=angle4;
	else
		{
			if (angle4 > 2.0857)
				angle4=2.0857;
			else 
				angle4=-2.0857;
	}

	if(angle5 > -2.0857 && angle5 < 2.0857)             //rshoulderpitch
	    angle5 = angle5;
	else
		{
			if (angle5 > 2.0857)
				angle5 = 2.0857;
			else 
				angle5 = -2.0857;
	}
	
	if(angle6>-1.3265&&angle6<0.3142)               //rshoulderroll
	    angle6=angle6;
	else
		{
			if (angle6>0.3142)
	         angle6=0.3142;
			else angle6=-1.3265;
	}

	if(angle7>0.0349&&angle7<1.5446)              //relbowroll
	    angle7=angle7;
	else
		{
		if (angle7>1.5446)
	         angle7=1.5446;
		else
			angle7=0.0349;
	}

	if(angle8>-2.0857&&angle8<2.0857)         //relbowyaw
	    angle8=angle8;
	else
		{
			if (angle8>2.0857)
				 angle8=2.0857;
			else 
				angle8=-2.0857;
	}

	if(angle9 >-0.6720 && angle9<0.5149)         //headpitch
	    angle9=angle9;
	else
		{
			if (angle9>0.5149)
				angle9=0.5149;
			else 
				angle9=-0.6720;
	}

	if(angle10>-1.5&&angle10<1.5)         //headyaw      活动区域缩小
	    angle10=angle10;
	else
		{if (angle10>1.5)
	         angle10=1.5;
		else 
			angle10=-1.5;
	}
}
void smoothAngleData(){
	
}
void ikNaoApi(){
	

} 
void controlNao(){
	  float resultLeft[4];
	 // vector<float> commandAngles(4);
	  NewtonIK = false;
	  if(NewtonIK){
		vector<float> tagertPos(3);
		Vector4d ikAngle;
		tagertPos[0] = LShouldToWrist[0];
		tagertPos[1] = LShouldToWrist[2];
		tagertPos[2] = LShouldToWrist[1];
		//cout << "tagertPos" << tagertPos << endl;
		ikAngle = ikine(tagertPos);	
		////cout << "ikAngle" << ikAngle << endl;
		if(transfer){
			AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll"/*,"RShoulderPitch", "RShoulderRoll","RElbowYaw","RElbowRoll"*/);
			AL::ALValue angles      = AL::ALValue::array(ikAngle(0), ikAngle(1),ikAngle(2),ikAngle(3)/*,angle4, angle5,angle6,angle7*/);
			float fractionMaxSpeed  = 0.1f;
			motion.setStiffnesses(names, AL::ALValue::array(1.0f,1.0f,1.0f,1.0f/*,1.0f,1.0f,1.0f,1.0f*/));
			motion.setAngles(names, angles, fractionMaxSpeed);
		}
	  } 
	  bool naoAPI = false;
	  if(naoAPI){
		  std::string chainName = "LArm";
		  int space = 0;
		  std::vector<float> position(6, 0.0f); 
		  position[0] = LShouldToHand[0];
		  position[1] = LShouldToHand[1];
		  position[2] = LShouldToHand[2];
		  float fractionMaxSpeed = 0.9f;
		  motion.setPositions(chainName, space, position, fractionMaxSpeed, 7);
		  //qi::os::sleep(2.0f);
	  }
	  bool jiexijie = true;
	  if(jiexijie){
		  calculLeftAngle();
		  //testHand();
		  //Sleep(1);
		  /*AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll","RShoulderPitch", "RShoulderRoll","RElbowYaw","RElbowRoll");
		  AL::ALValue angles      = AL::ALValue::array(LShoulderPitchResult, LShoulderRollResult,LElboYawResult,LElbowRollResult,RShoulderPitchResult, RShoulderRollResult,RElboYawResult,RElbowRollResult);
		  float fractionMaxSpeed  = 0.55f;
		  motion.setStiffnesses(names, AL::ALValue::array(1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f));
		  motion.setAngles(names, angles, fractionMaxSpeed);*/
	  }
	/*  calculLeftAngle();
	
	  AL::ALValue names       = AL::ALValue::array("LShoulderPitch", "LShoulderRoll","LElbowYaw","LElbowRoll");
	  AL::ALValue angles      = AL::ALValue::array(angle0, angle1,angle2,angle3);
	  float fractionMaxSpeed  = 0.55f;
	  motion.setStiffnesses(names, AL::ALValue::array(1.0f,1.0f,1.0f,1.0f));
	  motion.setAngles(names, angles, fractionMaxSpeed);*/
}

