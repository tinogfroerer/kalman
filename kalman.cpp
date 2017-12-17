// estimator2.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <cmath>
#include <string>

using namespace std;
using namespace cv;

typedef struct {
	Mat m;
} MatWrapper;

ostream& operator<<(ostream& stream, const MatWrapper& mat) {
	stream << mat.m.at<double>(0) << ';' << mat.m.at<double>(1) << ';' << \
	mat.m.at<double>(2) << ';' << mat.m.at<double>(3) << endl;
	return stream;
}

istream& operator>>(istream& stream, MatWrapper& mat) {
	string str;
	if (stream.eof()) return stream;
	getline(stream, str);
	//cout << "COUT " << str << endl; 
	// Tokenize
	for (int i = 0; i < 4; i++) {
		if(str.empty()) return stream;
		size_t pos = str.find_first_of(',', 0);
		mat.m.at<double>(i) = stof(str.substr(0,pos));
		if (pos >= str.npos) return stream;
		str = str.substr(pos+1,str.npos-pos-1);
	}
	return stream;
}

void init_filter(KalmanFilter& filter)
{
	const double dt = 1.0f/445.0;
	const double damping = 0.8;
	const double r = 7.519e-8;
	const double qp = 1.0e-8;
	const double qv = 1.2e-6;
	
	Mat transitionMatrix = Mat::eye(4,4,CV_64F);
	transitionMatrix.at<double>(0,2) = dt;
	transitionMatrix.at<double>(1,3) = dt;
	transitionMatrix.at<double>(2,2) -= damping*dt;
	transitionMatrix.at<double>(3,3) -= damping*dt;
	filter.transitionMatrix = transitionMatrix.clone();
	
	Mat measurementMatrix = Mat::eye(2,4,CV_64F);
	filter.measurementMatrix = measurementMatrix.clone();
	
	Mat measurementNoiseCov = r * Mat::eye(2,2,CV_64F);
	filter.measurementNoiseCov = measurementNoiseCov.clone();
	
	Mat processNoiseCov = qp * Mat::eye(4,4,CV_64F);
	processNoiseCov.at<double>(2,2) = qv;
	processNoiseCov.at<double>(3,3) = qv;
	filter.processNoiseCov = processNoiseCov.clone();
	
	// Initialize P
	filter.errorCovPost = processNoiseCov.clone();
	
}

int main()
{
	
	KalmanFilter filter(4,2,0,CV_64F); // State is p,v, measurement only p
	init_filter(filter);
	
		cout << "A =\n" << filter.transitionMatrix << endl;
		cout << "H =\n" << filter.measurementMatrix << endl;
		cout << "R =\n" << filter.measurementNoiseCov << endl;	
		cout << "Q =\n" << filter.processNoiseCov << endl;
		cout << "P =\n" << filter.errorCovPost << endl;
		
	//Mat in = Mat(4,1,CV_64F);
	//Mat out = Mat(4,1,CV_64F);
	Mat measurement = Mat(2,1,CV_64F);
	MatWrapper inw = {Mat(4,1,CV_64F)};
	MatWrapper outw = {Mat(4,1,CV_64F)};
	
	// Set state to estimated state with differentials
	cin >> inw;
	filter.statePre = inw.m;
	//measurement.at<double>(0) = in.at<double>(0);
	//measurement.at<double>(1) = in.at<double>(1);
	cout << "X =\n" << filter.statePre << endl;
	
	// Let the kalman filter do its job
	while (!cin.eof()) {
		// Predict next state
		filter.predict();
		// Get actual next state
		cin >> inw;
		measurement.at<double>(0) = inw.m.at<double>(0);
		measurement.at<double>(1) = inw.m.at<double>(1);
		filter.correct(measurement);
		outw.m = filter.statePost;
		cout << outw;
		cerr << "P =\n" << filter.errorCovPost << endl;
		cerr << "K =\n" << filter.gain << endl;
		//cout << w;
	}
	
	return 0;
}


/*
// untested! Maybe use opencv kalman filter instead?
static void kalman(const Vector2f z, Vector4f& x, Matrix4f& P, const double dt)
{
	// Kalman filtering constants
	const double e = 0.8f; // Damping constant in 1/s
	const double q = 0.0001f; // process covariance coeff in 1/s^2
	const double r = 0.006f; // measurement covariance coeff in 1/s^2
	// Kalman filter matrices
	Matrix4f A = Matrix4f::Identity();
	A.topRightCorner(2, 2) = dt*Matrix2f::Identity();
	A.bottomLeftCorner(2, 2) -= e*dt*Matrix2f::Identity();
	Matrix<double, 2, 4> H = MatrixXf::Zero(2,4);
	H.topLeftCorner(2, 2) = Matrix2f::Identity();
	Matrix4f Q = Matrix4f::Identity()*q*dt*dt;
	Matrix2f R = Matrix2f::Identity()*r*dt*dt;

	Vector4f x_estimate = A*x;
	Matrix4f P_estimate = A*P*A.transpose() + Q;

	Matrix2f temp = H*P_estimate.transpose()*H.transpose() + R;
	Matrix<double, 4, 2> K = P_estimate*H.transpose()*temp.inverse();
	x = x_estimate + K*(z - H*x_estimate);
	P = (Matrix4f::Identity() - K*H)*P_estimate;
}*/
