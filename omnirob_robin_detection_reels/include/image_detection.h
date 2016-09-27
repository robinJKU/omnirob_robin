#include <ros/ros.h>

#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <reels_robin_object.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include <math.h> 
#include "boost/multi_array.hpp"
#include "tf/transform_datatypes.h"
#include <ctime>

using namespace cv;

//static const std::string OPENCV_WINDOW = "Image window";

class ImageProcessor{

  	std::vector<AR_Marker> markers;
  	boost::array<double, 9> K;
	std::vector<double> D;
  	Mat Image;
	bool reel_presence;
  
public:

  	ImageProcessor() {

    	//namedWindow(OPENCV_WINDOW, CV_WINDOW_NORMAL );
	reel_presence=false;

  	}

  	/*~ImageProcessor(){

    		destroyWindow(OPENCV_WINDOW);
  	}*/

	void set_markers(AR_Marker &marker){

	        ROS_INFO("Marker broadcasted");
	        markers.push_back(marker);
	}

	void set_intrinsic_matrix(boost::array<double, 9> &Km,std::vector<double> &Dm){

	        ROS_INFO("Matrix broadcasted");
	        K=Km;
		D=Dm;
	}

	void set_image(Mat &ImageIn){

        	ROS_INFO("Image broadcasted");
	        Image=ImageIn;
	}

	void get_reel_presence(bool &reel_presence_in){

	        reel_presence_in=reel_presence;

	}


	void elaboration(){

		//imshow(OPENCV_WINDOW, Image);
    		//waitKey(1000);
		ROS_INFO("Read the position of the markers 3, 4, 5");
		tf::Vector3 position3,position4,position5;
		getMarkerPosition(position3,position4,position5);
		ROS_INFO("Read of the intrinsic matrix of the camera");

		unsigned int u_3,v_3,u_4,v_4,u_5,v_5;
		conversion_camera_pixel(position3, u_3, v_3);
		conversion_camera_pixel(position4, u_4, v_4);
		conversion_camera_pixel(position5, u_5, v_5);

		unsigned int width= u_4-u_3;
		unsigned int height= (v_4-v_3)/2;

		Rect myROI(u_3, v_3, width, height);
		Mat croppedImage = Image(myROI);

		//imshow("Cropped_image", croppedImage);
    		//waitKey(3000);
		Mat dst;
		
		//Application of a Gaussian blurring (with increasing size of the kernel)

	   	for (int i=1; i<7; i=i+2){ 

			GaussianBlur( croppedImage, dst, Size( i, i ), 0, 0 );
      			//imshow( "Smoothing with Gaussian Kernel", dst );

   		}

		//Conversion in a HVS image
		Mat imgHSV;
		cvtColor(dst, imgHSV, CV_BGR2HSV);
		int channels=imgHSV.channels();
		int nRows=imgHSV.rows;
		int nCols=imgHSV.cols;
		
		//Separation of the image with the yellow color
  		Mat imgThresholded;
		int iLowH,iLowS=150,iLowV=50,iHighH,iHighS=255,iHighV=255;
		iLowH=22; //lowest value of yellow's hue
		iHighH=38; //highest value of yellow's hue

  		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		//imshow( "Yellow", imgThresholded );
		//waitKey(1000);

		erode(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//imshow( "Open", imgThresholded );
		//waitKey(1000);

		dilate(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//imshow( "Close", imgThresholded );
		//waitKey(1000);

		int n_yellow=0;

            	for( int x = 0; x < imgThresholded.rows; x++ ){

              		for ( int y = 0; y < imgThresholded.cols; y++ ){

				if ( imgThresholded.at<uchar>(x,y) == 255 ) {

					n_yellow++;
                         
                         	}
			}
                } 

		//ROS_INFO("Number of yellow pixels after opening and closing= %d",n_yellow);

		if (n_yellow>0){
		
			reel_presence=true;			
			
			//Filling of the image
	    		Mat im_floodfill = imgThresholded.clone();
    			floodFill(im_floodfill, Point(0,0), Scalar(255));

	    		// Invert floodfilled image
    			Mat im_floodfill_inv;
    			bitwise_not(im_floodfill, im_floodfill_inv);

			Mat im_out = (imgThresholded | im_floodfill_inv);
 		
			//imshow("Floodfilled Image", im_floodfill);
    			//imshow("Inverted Floodfilled Image that is the pin", im_floodfill_inv);
			//imshow("Final image", im_out);
		
			vector<Vec3f> circles_reel, circles_pin;
			Mat croppedImage_2= croppedImage.clone();
			Mat croppedImage_3= croppedImage.clone();

			//Application of the Hough circle transform 

			clock_t time_a = clock();
			houghCircle (im_floodfill_inv, circles_pin);//Pin
			houghCircle (im_out, circles_reel);//Reel
			clock_t time_b = clock();

			//ROS_INFO("Number of circle of type pin = %d",(int)circles_pin.size());
			//ROS_INFO("Number of circle of type pin = %d",(int)circles_reel.size());

			if(circles_pin.size()==0){
				ROS_INFO("No circle found of type pin");
			}
			if(circles_reel.size()==0){
				ROS_INFO("No circle found of type reel");
			}
			
			for (int i=0;i<circles_pin.size();i++){

				//ROS_INFO("Center of the circle pin (pixel) x = %d y = %d",(int)circles_pin[i][0], (int)circles_pin[i][1]);
				//ROS_INFO("Radius of the circle pin (pixel) r = %d",(int)circles_pin[i][2]);
std::cout<<"\n---------------------------DIAMETER OF THE PIN FOUND WITH THE ANALYSIS OF THE RGB IMAGE (HOUGH)"<<endl;
				ROS_INFO("Radius of the circle pin (m) r = %f",circles_pin[i][2]/K[0]*position5[2]);
std::cout<<endl;
   				Point center_pin(cvRound(circles_pin[i][0]), cvRound(circles_pin[i][1]));
   				int radius_pin = cvRound(circles_pin[i][2]);

   				// circle center pin
   				circle( croppedImage, center_pin, 3, Scalar(0,255,0), -1, 8, 0 );
   				// circle outline pin
   				circle( croppedImage, center_pin, radius_pin, Scalar(0,0,255), 3, 8, 0 );

			}

			for (int j=0;j<circles_reel.size();j++){



				//ROS_INFO("Center of the circle reel (pixel) x = %d y = %d",(int)circles_reel[j][0], (int)circles_reel[j][1]);
				//ROS_INFO("Radius of the circle reel (pixel) r = %d",(int)circles_reel[j][2]);
std::cout<<"\n---------------------------DIAMETER OF THE REEL FOUND WITH THE ANALYSIS OF THE RGB IMAGE (HOUGH)"<<endl;
				ROS_INFO("Radius of the circle reel (m) r = %f",circles_reel[j][2]/K[0]*position5[2]);
std::cout<<endl;

   				Point center_reel(cvRound(circles_reel[j][0]), cvRound(circles_reel[j][1]));
   				int radius_reel = cvRound(circles_reel[j][2]);

   				// circle center reel
   				circle( croppedImage, center_reel, 3, Scalar(0,255,0), -1, 8, 0 );
   				// circle outline reel
   				circle( croppedImage, center_reel, radius_reel, Scalar(0,0,255), 3, 8, 0 );

 			}

			//imshow( "Circle found with Hough Circle", croppedImage);
		
			ROS_INFO("Application of the Ransac algorithm for circles");
    			Point2d bestCircleCenter_reel, bestCircleCenter_pin;
    			double bestCircleRadius_reel, bestCirclePercentage_reel, bestCircleRadius_pin, bestCirclePercentage_pin;

			clock_t time_c = clock();
			ransacCircle (im_floodfill_inv, bestCircleCenter_pin, bestCircleRadius_pin, bestCirclePercentage_pin);//Pin
			ransacCircle (im_out, bestCircleCenter_reel, bestCircleRadius_reel, bestCirclePercentage_reel);//Reel
			clock_t time_d = clock();

			//ROS_INFO("Center of the circle pin (pixel) x = %d y = %d",(int)bestCircleCenter_pin.x, (int)bestCircleCenter_pin.y);
			//ROS_INFO("Radius of the circle pin (pixel) r = %d",(int)bestCircleRadius_pin);
std::cout<<"\n---------------------------DIAMETER OF THE PIN FOUND WITH THE ANALYSIS OF THE RGB IMAGE (RANSAC CIRCLE)"<<endl;
			ROS_INFO("Radius of the circle pin (m) r = %f",bestCircleRadius_pin/K[0]*position5[2]);
std::cout<<endl;

			//ROS_INFO("Center of the circle reel (pixel) x = %d y = %d",(int)bestCircleCenter_reel.x, (int)bestCircleCenter_reel.y);
			//ROS_INFO("Radius of the circle reel (pixel) r = %d",(int)bestCircleRadius_reel);
std::cout<<"\n---------------------------DIAMETER OF THE REEL FOUND WITH THE ANALYSIS OF THE RGB IMAGE (RANSAC CIRCLE)"<<endl;
			ROS_INFO("Radius of the circle reel (m) r = %f",bestCircleRadius_reel/K[0]*position5[2]);
std::cout<<endl;

			Point center_ransac_pin(cvRound(bestCircleCenter_pin.x), cvRound(bestCircleCenter_pin.y));
			int radius_ransac_pin = cvRound(bestCircleRadius_pin);

			Point center_ransac_reel(cvRound(bestCircleCenter_reel.x), cvRound(bestCircleCenter_reel.y));
			int radius_ransac_reel = cvRound(bestCircleRadius_reel);

			// circle center pin
			circle( croppedImage_2, center_ransac_pin, 3, Scalar(0,255,0), -1, 8, 0 );
			// circle outline pin
			circle( croppedImage_2, center_ransac_pin, radius_ransac_pin, Scalar(0,0,255), 3, 8, 0 );

			// circle center reel
			circle( croppedImage_2, center_ransac_reel, 3, Scalar(0,255,0), -1, 8, 0 );
			// circle outline reel
			circle( croppedImage_2, center_ransac_reel, radius_ransac_reel, Scalar(0,0,255), 3, 8, 0 );
		
			//imshow( "Ransac Circle", croppedImage_2);

			ROS_INFO("Application of the Ransac algorithm for ellipses");
    			Point2d bestEllipseCenter_reel, bestEllipseCenter_pin;
    			double bestEllipse_a_pin, bestEllipse_b_pin, bestAngle_pin, bestEllipsePercentage_pin,a_real_pin,b_real_pin;
			double bestEllipse_a_reel, bestEllipse_b_reel, bestAngle_reel, bestEllipsePercentage_reel,a_real_reel,b_real_reel;

			clock_t time_e = clock();
			ransacEllipse (im_floodfill_inv, bestEllipseCenter_pin, bestEllipse_a_pin, bestEllipse_b_pin, bestAngle_pin, bestEllipsePercentage_pin);
			ransacEllipse (im_out, bestEllipseCenter_reel, bestEllipse_a_reel, bestEllipse_b_reel, bestAngle_reel, bestEllipsePercentage_reel);
			clock_t time_f = clock();

			//ROS_INFO("Center of the ellipse pin (pixel) x = %f y = %f",bestEllipseCenter_pin.x, bestEllipseCenter_pin.y);
			//ROS_INFO("Semiaxis a of the ellipse pin (pixel) a = %f",bestEllipse_a_pin);
			//ROS_INFO("Semiaxis a of the ellipse pin (pixel) b = %f",bestEllipse_b_pin);
std::cout<<"\n---------------------------SEMIAXIS OF THE PIN FOUND WITH THE ANALYSIS OF THE RGB IMAGE (RANSAC ELLIPSE)"<<endl;

			conversion_semiaxis(bestEllipseCenter_pin,bestEllipse_a_pin,bestEllipse_b_pin,bestAngle_pin,position5[2],a_real_pin,b_real_pin);

			ROS_INFO("Semiaxis a of the ellipse pin (m) a = %f",a_real_pin);
			ROS_INFO("Semiaxis b of the ellipse pin (m) b = %f",b_real_pin);
std::cout<<endl;

			//ROS_INFO("Center of the ellipse reel (pixel) x = %f y = %f",bestEllipseCenter_reel.x, bestEllipseCenter_reel.y);
			//ROS_INFO("Semiaxis a of the ellipse reel (pixel) a = %f",bestEllipse_a_reel);
			//ROS_INFO("Semiaxis a of the ellipse reel (pixel) b = %f",bestEllipse_b_reel);
std::cout<<"\n---------------------------SEMIAXIS OF THE REEL FOUND WITH THE ANALYSIS OF THE RGB IMAGE (RANSAC ELLIPSE)"<<endl;

			conversion_semiaxis(bestEllipseCenter_reel,bestEllipse_a_reel,bestEllipse_b_reel,bestAngle_reel,position5[2],a_real_reel,b_real_reel);

			ROS_INFO("Semiaxis a of the ellipse reel (m) a = %f",a_real_reel);
			ROS_INFO("Semiaxis b of the ellipse reel (m) b = %f",b_real_reel);
std::cout<<endl;

			Point center_ellipse_pin(cvRound(bestEllipseCenter_pin.x), cvRound(bestEllipseCenter_pin.y));
			int axis_a_pin = cvRound(bestEllipse_a_pin);
			int axis_b_pin = cvRound(bestEllipse_b_pin);

			Point center_ellipse_reel(cvRound(bestEllipseCenter_reel.x), cvRound(bestEllipseCenter_reel.y));
			int axis_a_reel = cvRound(bestEllipse_a_reel);
			int axis_b_reel = cvRound(bestEllipse_b_reel);

			// ellipse outline pin
			circle( croppedImage_3, center_ellipse_pin, 3, Scalar(0,255,0), -1, 8, 0 );
			ellipse( croppedImage_3, center_ellipse_pin, Size(axis_a_pin,axis_b_pin),bestAngle_pin*180/M_PI,0.0,360, Scalar(0,0,255), 3, 8, 0);

			// ellipse outline reel
			circle( croppedImage_3, center_ellipse_reel, 3, Scalar(0,255,0), -1, 8, 0 );
			ellipse( croppedImage_3, center_ellipse_reel, Size(axis_a_reel,axis_b_reel),bestAngle_reel*180/M_PI,0.0,360, Scalar(0,0,255), 3, 8, 0);

		
			//imshow( "Ransac Ellipse", croppedImage_3);

			double time_hough=(double)(time_b-time_a) / CLOCKS_PER_SEC;
			double time_RANSAC_circle=(double)(time_d-time_c) / CLOCKS_PER_SEC;
			double time_RANSAC_ellipse=(double)(time_f-time_e) / CLOCKS_PER_SEC;

			ROS_INFO("Time for calculation with Hough = %f",time_hough);
			ROS_INFO("Time for calculation with RANSAC circle = %f",time_RANSAC_circle);
			ROS_INFO("Time for calculation with RANSAC ellipse = %f",time_RANSAC_ellipse);

		}else{

			ROS_INFO("No yellow objects found");
		}
		//waitKey(1000);
		//destroyWindow("Image window");

	}


	void conversion_semiaxis(Point2d &EllipseCenter,double &Ellipse_a,double &Ellipse_b,double &angle,double &z,double &a_real,double &b_real){

		double ax=EllipseCenter.x+Ellipse_a*cos(angle);
		double ay=EllipseCenter.y+Ellipse_a*sin(angle);
		double bx=EllipseCenter.x+Ellipse_b*cos(angle+M_PI/2);
		double by=EllipseCenter.y+Ellipse_b*sin(angle+M_PI/2);

		double cx_real=EllipseCenter.x/K[0]*z;
		double cy_real=EllipseCenter.y/K[4]*z;
		double ax_real=ax/K[0]*z;
		double ay_real=ay/K[4]*z;
		double bx_real=bx/K[0]*z;
		double by_real=by/K[4]*z;

		a_real=sqrt((ax_real-cx_real)*(ax_real-cx_real)+(ay_real-cy_real)*(ay_real-cy_real));
		b_real=sqrt((bx_real-cx_real)*(bx_real-cx_real)+(by_real-cy_real)*(by_real-cy_real));

	}

	void conversion_camera_pixel(tf::Vector3 &position,unsigned int &u_p,unsigned int &v_p){

		double x_n=position[0]/position[2];
		double y_n=position[1]/position[2];
		double rq=pow(x_n,2.0)+pow(y_n,2.0);

		double x_nd=x_n*(1+D[0]*rq+D[1]*pow(rq,2.0)+D[4]*pow(rq,3.0))/(1+D[5]*rq+D[6]*pow(rq,2.0)+D[7]*pow(rq,3.0))+2*D[2]*x_n*y_n+D[3]*(rq+2*pow(x_n,2.0));
		double y_nd=y_n*(1+D[0]*rq+D[1]*pow(rq,2.0)+D[4]*pow(rq,3.0))/(1+D[5]*rq+D[6]*pow(rq,2.0)+D[7]*pow(rq,3.0))+D[2]*(rq+2*pow(y_n,2.0))+2*D[3]*x_n*y_n;

		double u=K[0]*x_nd+K[2];
		double v=K[4]*y_nd+K[5];
		u_p=(unsigned int) u;
		v_p=(unsigned int) v;

	} 


	void gaussianBlur (Mat &inputImage, Mat &outputImage){

	   	for (int i=1; i<5; i=i+2){ 
      		
			GaussianBlur( inputImage, outputImage, Size( i, i ), 0, 0 );
      			//imshow( "Smoothing with Gauss", outputImage );
      			//waitKey(1000);
   		}

	}

	void houghCircle (Mat &inputImage, vector<Vec3f> &circles){

		Mat blurred_image;
		int lowThreshold=50;
		int const max_lowThreshold = 150;
		int ratio = 3;
		int kernel_size = 3;
		gaussianBlur (inputImage, blurred_image);
	 
		// Apply the Hough Transform to find the circles
  		HoughCircles(blurred_image, circles, CV_HOUGH_GRADIENT, 1, blurred_image.rows/8, 100, 20, 0, 0 );
		//HoughCircles(inputImage, circles, CV_HOUGH_GRADIENT, 1, inputImage.rows/8, 10, 20, 0, 0 );

	}


	void ransacCircle (Mat &inputImage, Point2d &bestCircleCenter, double &bestCircleRadius, double &bestCirclePercentage){

		Mat blurred_image,canny;
		bestCirclePercentage=0;
		int lowThreshold=50;
		int ratio = 3;
		int kernel_size = 3;
		gaussianBlur (inputImage, blurred_image);
		Canny( blurred_image, canny, lowThreshold, lowThreshold*ratio, kernel_size );
		Mat canny_bin;
		threshold(canny, canny_bin, 0, 255, CV_THRESH_BINARY);
		//imshow( "canny_bin", canny_bin);
		//waitKey(3000);
	 	std::vector<cv::Point2d> edgePositions;

 		for(unsigned int x=0; x<canny_bin.rows; x++){

     			for(unsigned int y=0; y<canny_bin.cols; y++){

         			if(canny_bin.at<unsigned char>(x,y) == 255){

					edgePositions.push_back(cv::Point2d(y,x));
				}
     			}
 		}


		//--------------Automatic calculus of iteration for rannsac----------

		/*double p=0.9; //probability of success
		double w=(double)edgePositions.size()/(double)(canny_bin.rows*canny_bin.cols);
		ROS_INFO("Total number of pixel = %d",(int)canny_bin.rows*canny_bin.cols);
		ROS_INFO("Total number of inliers = %d",(int)edgePositions.size());
		double n_iteration=(log(1.0-p)/log(1.0-pow (w,3.0)));
		ROS_INFO("Automatic number of iteration for RANSAC n = %f",n_iteration);*/
		//-------------------------------------------------------------------


    		// create distance transform to efficiently evaluate distance to nearest edge
    		Mat dt,dist,canny_inv;
		bitwise_not(canny_bin, canny_inv);

		//imshow( "inv_canny", canny_inv);
		//waitKey(3000);

    		distanceTransform(canny_inv, dt,CV_DIST_L1, 3);
	    	normalize(dt, dist, 0, 1., NORM_MINMAX);
    		//imshow("Distance Transform Image", dist);
		//waitKey(3000);
    		double minRadius = 0.0;

		//-------------------------------------------------
    		int maxNrOfIterations = 1000;
		//-------------------------------------------------

    		for(unsigned int its=0; its< maxNrOfIterations; its++){


        		unsigned int idx1 = rand()%edgePositions.size();
        		unsigned int idx2 = rand()%edgePositions.size();
        		unsigned int idx3 = rand()%edgePositions.size();

        		// we need 3 different samples:
        		if(idx1 == idx2) continue;
        		if(idx1 == idx3) continue;
        		if(idx3 == idx2) continue;

        		// create circle from 3 points:
        		Point2d center; double radius;double rate;
        		getCircle(edgePositions[idx1],edgePositions[idx2],edgePositions[idx3],center,radius);

        		//verify or falsify the circle by inlier counting:
        		verifyCircle(dt,center,radius, rate);

        		// update best circle information if necessary
			if(rate >= bestCirclePercentage && radius >= minRadius){

   	 				bestCirclePercentage = rate;
    					bestCircleRadius = radius;
    					bestCircleCenter = center;

			}
				
    		}
	}


	void ransacEllipse (Mat &inputImage, Point2d &bestEllipseCenter, double &bestEllipse_a, double &bestEllipse_b, double &bestAngle, double &bestEllipsePercentage){

		Mat blurred_image,canny;
		bestEllipsePercentage=0.0;
		int lowThreshold=50;
		int ratio = 3;
		int kernel_size = 3;
		gaussianBlur (inputImage, blurred_image);
		Canny( blurred_image, canny, lowThreshold, lowThreshold*ratio, kernel_size );
		Mat canny_bin;
		threshold(canny, canny_bin, 0, 255, CV_THRESH_BINARY);
		//imshow( "canny_bin", canny_bin);
		//waitKey(3000);
	 	std::vector<cv::Point2d> edgePositions;

 		for(unsigned int x=0; x<canny_bin.rows; x++){

     			for(unsigned int y=0; y<canny_bin.cols; y++){

         			if(canny_bin.at<unsigned char>(x,y) == 255){

					edgePositions.push_back(cv::Point2d(y,x));
				}
     			}
 		}


		//--------------Automatic calculus of iteration for rannsac----------

		/*double p=0.9; //probability of success
		double w=(double)edgePositions.size()/(double)(canny_bin.rows*canny_bin.cols);
		ROS_INFO("Total number of pixel = %d",(int)canny_bin.rows*canny_bin.cols);
		ROS_INFO("Total number of inliers = %d",(int)edgePositions.size());
		double n_iteration=(log(1.0-p)/log(1.0-pow (w,5.0)));
		ROS_INFO("Automatic number of iteration for RANSAC n = %f",n_iteration);*/
		//-------------------------------------------------------------------


    		// create distance transform to efficiently evaluate distance to nearest edge
    		Mat dt,dist,canny_inv;
		bitwise_not(canny_bin, canny_inv);

		//imshow( "inv_canny", canny_inv);
		//waitKey(3000);

    		distanceTransform(canny_inv, dt,CV_DIST_L1, 3);
	    	//normalize(dt, dist, 0, 1., NORM_MINMAX);
    		//imshow("Distance Transform Image", dist);
		//waitKey(3000);
    		double min_simi_axis = 0.0;

		//-------------------------------------------------
    		int maxNrOfIterations = 1000;
		//-------------------------------------------------

    		for(int its=0; its< maxNrOfIterations; its++){


        		unsigned int idx1 = rand()%edgePositions.size();
        		unsigned int idx2 = rand()%edgePositions.size();
        		unsigned int idx3 = rand()%edgePositions.size();
        		unsigned int idx4 = rand()%edgePositions.size();
        		unsigned int idx5 = rand()%edgePositions.size();


        		// we need 5 different samples:
        		if(idx1 == idx2) continue;
        		if(idx1 == idx3) continue;
        		if(idx1 == idx4) continue;
        		if(idx1 == idx5) continue;
        		if(idx2 == idx3) continue;
        		if(idx2 == idx4) continue;
        		if(idx2 == idx5) continue;
        		if(idx3 == idx4) continue;
        		if(idx3 == idx5) continue;
        		if(idx4 == idx5) continue;

        		// create ellipse from 5 points:
        		Point2d center; double semi_axis1,semi_axis2,angle;double rate;
        		getEllipse(edgePositions[idx1],edgePositions[idx2],edgePositions[idx3],edgePositions[idx4],edgePositions[idx5],center,semi_axis1,semi_axis2,angle);

        		//verify or falsify the ellipse by inlier counting:
        		verifyEllipse(dt,center,semi_axis1,semi_axis2,angle, rate);

        		// update best ellipse information if necessary
			if(rate >= bestEllipsePercentage && semi_axis1 >= min_simi_axis){

   	 				bestEllipsePercentage = rate;
    					bestEllipse_a =semi_axis1 ;
					bestEllipse_b =semi_axis2 ;
					bestAngle=angle;
					bestEllipseCenter = center;
			}
				
    		}
	}


	void verifyCircle(cv::Mat &dt, cv::Point2d &center, double &radius,double &rate){

 		unsigned int counter = 0;
 		unsigned int inlier = 0;
 		double minInlierDist = 2.0;
 		double maxInlierDist = radius/25.0;

 		// choose samples along the circle and count inlier percentage
 		for(double t =0; t<2*M_PI; t+= 0.05){
     
			counter++;
     			float cX = radius*cos(t) + center.x;
     			float cY = radius*sin(t) + center.y;

     			if(cX < dt.cols && cX >= 0 && cY < dt.rows && cY >= 0 && dt.at<float>(cY,cX) < maxInlierDist){

        			inlier++;

     			}
 		}

		rate=(double)inlier/(double)counter;
	}


	void verifyEllipse(cv::Mat &dt, cv::Point2d &center, double &semi_axis1,double &semi_axis2,double &angle,double &rate){

 		int counter = 0;
 		int inlier = 0;
 		double minInlierDist = 2.0;
 		double maxInlierDist = semi_axis2/25.0;
		double a=semi_axis1;
		double b=semi_axis2;
		double c=sqrt(a*a-b*b);
		double e=c/a;

 		// choose samples along the ellipse and count inlier percentage
 		for(double t =0; t<2*M_PI; t+= 0.05){
     
			counter++;
			double r=(a*b)/sqrt(b*cos(t)*b*cos(t)+a*sin(t)*a*sin(t));
     			float cX = center.x+r*cos(t+angle);
     			float cY = center.y+r*sin(t+angle);

     			if(cX < dt.cols && cX >= 0 && cY < dt.rows && cY >= 0 && dt.at<float>(cY,cX) < maxInlierDist){

        			inlier++;

     			}
 		}
		
		rate=(double)inlier/(double)counter;
	}


	void getCircle(Point2d &p1,Point2d &p2,Point2d &p3,Point2d &center, double &radius){
  
		double x1 = p1.x;
  		double x2 = p2.x;
  		double x3 = p3.x;

  		double y1 = p1.y;
  		double y2 = p2.y;
  		double y3 = p3.y;

		Eigen::MatrixXd A(3,3),D(3,3),E(3,3),F(3,3);

		A<<x1,y1,1,
		   x2,y2,1,
		   x3,y3,1;

		D<<x1*x1+y1*y1,y1,1,
		   x2*x2+y2*y2,y2,1,
		   x3*x3+y3*y3,y3,1;

		E<<x1*x1+y1*y1,x1,1,
		   x2*x2+y2*y2,x2,1,
		   x3*x3+y3*y3,x3,1;

		F<<x1*x1+y1*y1,x1,y1,
		   x2*x2+y2*y2,x2,y2,
		   x3*x3+y3*y3,x3,y3;

		double a=A.determinant();
		double d=-D.determinant();
		double e=E.determinant();
		double f=-F.determinant();

		center.x =-d/(2*a);
		center.y =-e/(2*a);
		radius=sqrt((d*d+e*e)/(4*a*a)-f/a);
	}


	void getEllipse(Point2d &p1,Point2d &p2,Point2d &p3,Point2d &p4,Point2d &p5,Point2d &center, double &semi_axis1,double &semi_axis2,double &angle){
  
		double x1 = p1.x;
  		double x2 = p2.x;
  		double x3 = p3.x;
		double x4 = p4.x;
		double x5 = p5.x;

  		double y1 = p1.y;
  		double y2 = p2.y;
  		double y3 = p3.y;
  		double y4 = p4.y;
  		double y5 = p5.y;

		Eigen::MatrixXd A(5,5),B(5,5),C(5,5),D(5,5),F(5,5),G(5,5);

		A<<x1*y1,y1*y1,x1,y1,1,
		   x2*y2,y2*y2,x2,y2,1,
		   x3*y3,y3*y3,x3,y3,1,
		   x4*y4,y4*y4,x4,y4,1,
		   x5*y5,y5*y5,x5,y5,1;

		B<<x1*x1,y1*y1,x1,y1,1,
		   x2*x2,y2*y2,x2,y2,1,
		   x3*x3,y3*y3,x3,y3,1,
		   x4*x4,y4*y4,x4,y4,1,
		   x5*x5,y5*y5,x5,y5,1;

		C<<x1*x1,x1*y1,x1,y1,1,
                   x2*x2,x2*y2,x2,y2,1,
		   x3*x3,x3*y3,x3,y3,1,
		   x4*x4,x4*y4,x4,y4,1,
		   x5*x5,x5*y5,x5,y5,1;

		D<<x1*x1,x1*y1,y1*y1,y1,1,
		   x2*x2,x2*y2,y2*y2,y2,1,
		   x3*x3,x3*y3,y3*y3,y3,1,
		   x4*x4,x4*y4,y4*y4,y4,1,
		   x5*x5,x5*y5,y5*y5,y5,1;

		F<<x1*x1,x1*y1,y1*y1,x1,1,
		   x2*x2,x2*y2,y2*y2,x2,1,
		   x3*x3,x3*y3,y3*y3,x3,1,
		   x4*x4,x4*y4,y4*y4,x4,1,
		   x5*x5,x5*y5,y5*y5,x5,1;

		G<<x1*x1,x1*y1,y1*y1,x1,y1,
		   x2*x2,x2*y2,y2*y2,x2,y2,
		   x3*x3,x3*y3,y3*y3,x3,y3,
		   x4*x4,x4*y4,y4*y4,x4,y4,
		   x5*x5,x5*y5,y5*y5,x5,y5;

		double a=A.determinant();
		double b=-B.determinant();
		double c=C.determinant();
		double d=-D.determinant();
		double f=F.determinant();
		double g=-G.determinant();

		double x0=(2*c*d-b*f)/(b*b-4*a*c);
		double y0=(2*a*f-b*d)/(b*b-4*a*c);

		double num=2*((a*f*f-b*d*f+c*d*d)/(4*a*c-b*b)-g);
		double den1=a+c-sqrt((a-c)*(a-c)+b*b);
		double den2=a+c+sqrt((a-c)*(a-c)+b*b);

		if (abs(den1)<abs(den2)){
		semi_axis1=sqrt(num/den1);
		semi_axis2=sqrt(num/den2);
		}else if (abs(den1)>abs(den2)){
		semi_axis2=sqrt(num/den1);
		semi_axis1=sqrt(num/den2);		
		}
		
		if(b==0 && a<c){
			angle=0;
		}else if(b==0 && a>c){
			angle=M_PI/2;
		}else if(b!=0 && a<c){
			angle=1/2*atan2(b,(a-c));
		}else if(b!=0 && a>c){
			angle=M_PI/2+1/2*atan2(b,(a-c));
		}else{
			angle=0;
		}

		center.x=x0;
		center.y=y0;
	}


	void getMarkerPosition(tf::Vector3 &position3,tf::Vector3 &position4, tf::Vector3 &position5){

		for (int k=0;k<markers.size();k++){

			if(markers[k].marker_id==3){
			position3[0]=markers[k].observed_pose.position.x;
			position3[1]=markers[k].observed_pose.position.y;
			position3[2]=markers[k].observed_pose.position.z;
			}
			else if(markers[k].marker_id==4){
			position4[0]=markers[k].observed_pose.position.x;
			position4[1]=markers[k].observed_pose.position.y;
			position4[2]=markers[k].observed_pose.position.z;
			}
			else if(markers[k].marker_id==5){
			position5[0]=markers[k].observed_pose.position.x;
			position5[1]=markers[k].observed_pose.position.y;
			position5[2]=markers[k].observed_pose.position.z;
			}
		}

	}
};

