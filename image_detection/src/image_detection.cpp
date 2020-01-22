#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>

using namespace cv;

class Image
{
	private:
	
	sensor_msgs::Image raw;
	cv_bridge::CvImagePtr cv_ptr,processed;
	
	Mat Original;
		
	public:
	
	int value_1_min,value_1_max;
	int value_2_min,value_2_max;
	int value_3_min,value_3_max;

	ros::Publisher image1,image2;
	
	Image(){
			value_1_min=0;
			value_1_max=255;
			value_2_min=70;
			value_2_max=255;
			value_3_min=100;
			value_3_max=255;
			namedWindow("Processed");
			namedWindow("Result");
			
			/// Create Trackbars
			createTrackbar("value_1","Processed",&value_1_min,value_1_max,NULL);
			createTrackbar("value_2","Processed",&value_2_min,value_2_max,NULL);
			createTrackbar("value_3","Processed",&value_3_min,value_3_max,NULL);
			createTrackbar("value_1_max","Processed",&value_1_max,255,NULL);
			createTrackbar("value_2_max","Processed",&value_2_max,255,NULL);
			createTrackbar("value_3_max","Processed",&value_3_max,255,NULL);	
		}
	
	~Image(){
			destroyWindow("Processed");
			destroyWindow("Result");
		}
		
		void readImage(const sensor_msgs::Image::ConstPtr &msg){
			//write callback topic  to private opencv variable
			try
			{
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			  //for showing the mono image...
			  processed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}
			
			processed->image=processImage(cv_ptr->image);
			image1.publish(processed->toImageMsg());
			processed->image=processGaussian (processed->image);
			cv_ptr->image=processHugh(processed->image,cv_ptr->image);
			image2.publish(processed->toImageMsg());
			
			cv::imshow("Processed", processed->image);					// image after processing
			cv::imshow("Result", cv_ptr->image);
			on_trackbar(value_1_min, 0 );
			cv::waitKey(3);
			
		}
		
		Mat processImage(Mat input)
		{
			Mat output;
			Mat HSV;
			cvtColor(input,HSV,COLOR_BGR2HSV);
			inRange(HSV,Scalar(value_1_min,value_2_min,value_3_min),Scalar(value_1_max,value_2_max,value_3_max),output);								
				//smooth the processed image for better ball detection
			
			return output;
		}
		
		Mat processGaussian (Mat input)
		{
			Mat output;
			
			GaussianBlur(input,				// input image
						 output,				// function output
						 Size(15,15),			// smoothing window width and height in pixels
						 1.5);					// sigma value, determines how much the image will be blurred
			
			return output;
		}
		
		Mat processHugh(Mat input,Mat color)
		{
			Mat output=color;
			struct
			{
				double x;
				double y;
				double z;
				bool detected;
			}Ball_Position;
			
			vector<cv::Vec3f> vecCircles;				// 3 element vector of floats, this will be the pass by reference output of HoughCircles()
			vector<cv::Vec3f>::iterator itrCircles;		// iterator for the circles vector
			
			HoughCircles(input,						// input image
						 vecCircles,				// function output 
						 CV_HOUGH_GRADIENT,			// two-pass algorithm for detecting circles
						 2,							// size of image / this value = "accumulator resolution"
						 input.rows / 4,		// min distance in pixels between the centers of the detected circles
						 100,						// high threshold of Canny edge detector (called by cvHoughCircles)						
						 50,						// low threshold of Canny edge detector (set at 1/2 previous value)
						 15,						// min circle radius (minimum circle radius calibrated in laboratory conditions)
						 100);						// max circle radius (maximum circle radius calibrated in laboratory conditions)

		for(itrCircles = vecCircles.begin(); itrCircles != vecCircles.end(); itrCircles++) {
			//write infomation to structure:
			Ball_Position.x=(*itrCircles)[0];
			Ball_Position.y=(*itrCircles)[1];
			Ball_Position.z=(*itrCircles)[2];
			printf("ball detected at %f / %f /%f \n",Ball_Position.x,Ball_Position.y,Ball_Position.z);
			
			/*
			if((*itrCircles)[0] >= 300){
				//cout << "ball detected!!!";
				break;
				}
			*/
												// draw small green circle at center of detected object
			cv::circle(output,													// draw on original image
					   Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),			// center point of circle
					   3,															// radius of circle in pixels
					   Scalar(0,255,0),												// draw pure green (remember, its BGR, not RGB)
					   CV_FILLED);													// thickness, fill in the circle

												// draw red circle around the detected object
			cv::circle(output,													// draw on original image
					   Point((int)(*itrCircles)[0], (int)(*itrCircles)[1]),			// center point of circle
					   (int)(*itrCircles)[2],										// radius of circle in pixels
					   Scalar(0,0,255),												// draw pure red (remember, its BGR, not RGB)
					   3);															// thickness of circle in pixels
		}
			return output;
		}
		//callbackfunction for the sliders
		void on_trackbar( int, void* )
		{
				
		}
			
};


int main(int argc, char** argv) {

	ros::init(argc, argv, "ball detector node");		
	ros::NodeHandle n("~");
	Image img;
	
	std::string image_raw,out1,out2;
	n.param<std::string>("image_raw",image_raw,"/raw");
	n.param<std::string>("image_out1",out1,"/image1"); 
	n.param<std::string>("image_out2",out2,"/image2"); 
	
	img.image1=n.advertise<sensor_msgs::Image>(out1.c_str(),10);
	img.image2=n.advertise<sensor_msgs::Image>(out2.c_str(),10);

	ros::Subscriber imageSub=n.subscribe(image_raw.c_str(),1,&Image::readImage,&img);
		
	ros::spin();	
	return 0;	
}
