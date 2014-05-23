#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv2\imgproc\imgproc.hpp>
#include <fstream>

using namespace cv;

struct imageData {
	std::string imageName = "";
	double latitude = 0;
	double longitude = 0;
	double altitudeFeet = 0;
	double altitudeMeter = 0;
	double roll = 0;
	double pitch = 0;
	double yaw = 0;
};

struct cameraProperties {
	double focalLength = 0;		// meters
	double averageAltitude = 0;	// meters
	double pixelSize = 0;		// meters
	double minLat = 0;
	double minLon = 0;
	double maxLat = 0;
	double maxLon = 0;
	double pixelPerMeter = 0;
};

void parseFile(std::vector<imageData>* imageVector);
void setCameraProperties(cameraProperties* camera, std::vector<imageData>* imageVector);
double gpsToDeltaMeter(double lat1, double lon1, double lat2, double lon2);

int main(){

	// create vector with struct containing data in text file
	std::vector<imageData> imageVector;
	parseFile(&imageVector);

	// camera properties for Ricoh GR Digial III
	cameraProperties camera;
	setCameraProperties(&camera, &imageVector);
		
	// calculate distances between lats lons
	// http://www.gpsvisualizer.com/calculators
	//double deltaLatMeters = 2023;
	//double deltaLonMeters = 2347;
	double deltaLatMeters = gpsToDeltaMeter(camera.maxLat, camera.minLon, camera.minLat, camera.minLon);
	double deltaLonMeters = gpsToDeltaMeter(camera.maxLat, camera.minLon, camera.maxLat, camera.maxLon);

	// scale factor
	float sf = 0.1;

	// divide by 4 otherwise cv::Mat won't fit in memory... :P
	int numCols = (deltaLonMeters * camera.pixelPerMeter) * sf;
	int numRows = (deltaLatMeters * camera.pixelPerMeter) * sf;
	cv::Mat superStitch(numRows, numCols, CV_8UC3);
	cv::Mat tempResult;
	cv::Mat mask;
	
	std::string imagePath;
	double deltaLat = 0.0;
	double deltaLon = 0.0;
	double resizeValue = 0.0;
	cv::namedWindow("window", cv::WINDOW_NORMAL);
	for (int i = 0; i < (imageVector.size() - 1); i++){
		std::cout << i << std::endl;

		// read in image
		imagePath = "C:\\Users\\Larso\\Documents\\Visual Studio 2013\\Projects\\SuperStitch\\Data\\Images\\";
		imagePath = imagePath.append(imageVector.at(i).imageName);
		cv::Mat image = cv::imread(imagePath, 1);
		resizeValue = sf * imageVector.at(i).altitudeMeter / camera.averageAltitude;
		cv::resize(image, image, Size(), resizeValue, resizeValue, INTER_AREA);
				
		// get GPS data
		deltaLat = gpsToDeltaMeter(camera.maxLat, camera.minLon, imageVector.at(i).latitude, camera.minLon);
		deltaLon = gpsToDeltaMeter(camera.maxLat, camera.minLon, camera.maxLat, imageVector.at(i).longitude);
		
		// offset values
		int centerCol = (deltaLon * camera.pixelPerMeter * sf);
		int centerRow = (deltaLat * camera.pixelPerMeter * sf);

		// offset
		cv::Matx23f M1(1, 0, centerCol - image.cols / 2, 0, 1, centerRow - image.rows / 2);
		warpAffine(image, tempResult, M1, superStitch.size());
		//mask = tempResult > 0;
		//tempResult.copyTo(superStitch, mask);
		//imshow("window", superStitch);
		//cv::waitKey(3000);

		// rotate around center point
		cv::Matx23f M2 = getRotationMatrix2D(Point2f(centerCol, centerRow), imageVector.at(i).yaw+90, 1);
		warpAffine(tempResult, tempResult, M2, superStitch.size());
		mask = tempResult > 0;
		tempResult.copyTo(superStitch, mask);
		imshow("window", superStitch);
		cv::waitKey(3000);

		/*
		// transformation matrix M - rotates and translates the original image and places it on superStitch
		// http://docs.opencv.org/trunk/doc/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html
		// search for "Rotation" is above URL
		int centerCol = (deltaLon * camera.pixelPerMeter * sf);
		int centerRow = (deltaLat * camera.pixelPerMeter * sf);
		float headingRads = imageVector.at(i).yaw * 3.14 / 180.0;
		float a = cos(headingRads);
		float b = sin(headingRads);
		cv::Matx44f M(a, b, (1-a)*(image.cols/2) - b*(image.rows/2), centerCol,
			-b, a, b*(image.cols / 2) + (1 - a)*(image.rows / 2), centerRow,
			0, 0, 0, 1);
		cv::warpPerspective(image, tempResult, M, superStitch.size());
		*/
		//mask = tempResult > 0;
		//tempResult.copyTo(superStitch, mask);
		//imshow("window", superStitch);
		//cv::waitKey(1000);
		
	}

	// create window for display
	cv::namedWindow("window", cv::WINDOW_NORMAL);
	while (1){
		imshow("window", superStitch);
		cv::waitKey(33);
	}

	return 0;
}

void parseFile(std::vector<imageData>* imageVector){
	std::string textFile = "C:\\Users\\Larso\\Documents\\Visual Studio 2013\\Projects\\SuperStitch\\Data\\GPS\\GPS-IMU_noheader.txt";
	std::ifstream fileReader;
	fileReader.open(textFile);
	if (fileReader.is_open()){
		char word[50];
		while (!fileReader.eof()){
			imageData id;
			for (int i = 0; i < 7; i++){
				fileReader >> word;
				if (i == 0)	{ id.imageName = word; }
				else if (i == 1)	{ id.latitude = atof(word); }
				else if (i == 2)	{ id.longitude = atof(word); }
				else if (i == 3)	{
					id.altitudeFeet = atof(word);
					id.altitudeMeter = id.altitudeFeet * 0.3048;
				}
				else if (i == 4)	{ id.yaw = atof(word); }
				else if (i == 5)	{ id.pitch = atof(word); }
				else if (i == 6)	{ id.roll = atof(word); }
			}
			imageVector->push_back(id);
		}
	}
	fileReader.close();
}

void setCameraProperties(cameraProperties* camera, std::vector<imageData>* imageVector){
	camera->focalLength = 6E-3;				// camera specific (meters)
	camera->pixelSize = 2.0394737E-6;		// camera specific (meters)
	double sum = 0;
	double minLat = imageVector->at(0).latitude;
	double minLon = imageVector->at(0).longitude;
	double maxLat = imageVector->at(0).latitude;
	double maxLon = imageVector->at(0).longitude;
	for (int i = 0; i<(imageVector->size() - 1); i++){
		sum += imageVector->at(i).altitudeMeter;
		if (imageVector->at(i).latitude < minLat){
			minLat = imageVector->at(i).latitude;
		}
		if (imageVector->at(i).longitude < minLon){
			minLon = imageVector->at(i).longitude;
		}
		if (imageVector->at(i).latitude > maxLat){
			maxLat = imageVector->at(i).latitude;
		}
		if (imageVector->at(i).longitude > maxLon){
			maxLon = imageVector->at(i).longitude;
		}
	}
	camera->averageAltitude = sum / (imageVector->size() - 1);
	camera->minLat = minLat;
	camera->minLon = minLon;
	camera->maxLat = maxLat;
	camera->maxLon = maxLon;
	camera->pixelPerMeter = (camera->focalLength) / (camera->averageAltitude * camera->pixelSize);
}

double gpsToDeltaMeter(double lat1, double lon1, double lat2, double lon2){
	// use Haversine to calculate distances
	// http://www.movable-type.co.uk/scripts/latlong.html
	
	float radiusEarthMeters = 6378100;
	float lat1Rads = lat1 * 3.14 / 180;
	float lat2Rads = lat2 * 3.14 / 180;
	float deltaLatRads = (lat2 - lat1) * 3.14 / 180;
	float deltaLonRads = (lon2 - lon1) * 3.14 / 180;

	double a = (sin(deltaLatRads / 2) * sin(deltaLatRads / 2)) +
		(cos(lat1Rads) * cos(lat2Rads)) *
		(sin(deltaLonRads / 2) * sin(deltaLonRads / 2));

	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	return radiusEarthMeters * c;
}