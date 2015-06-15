
#include "openCvFunctions.h"
#include "ConversionTools.hxx"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

extern char* object;

double angle(cv:: Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// This function uses OpenCV to track red blob's x, y coordinates and returns them
// as well as returing a threshold map of the red object. 
void rgbImage(cv::Mat imgOriginal ,int* xpos, int* ypos, int* radi)
{
    ///////static int count = 0;
    // Declare variables for radius and center storage
    cv::Point2f center;
    float radius = 0;
    center.x = 0;
    center.y = 0;

    // Sets HSV values to search for the red object.
    //low was 100 if that needs to change back
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    if (object[0] == 49)
    {
        //
        //printf("Object 1 locating...");
        iLowH = 100;
        iHighH = 130;

        iLowS = 100; 
        iHighS = 255;

        iLowV = 49;
        iHighV = 255;
    }
    else if (object[0] == 50)
    {
        //
        //printf("Object 2 locating...");
        iLowH = 0;
        iHighH = 10;

        iLowS = 100; 
        iHighS = 255;

        iLowV = 49;
        iHighV = 255;
    }
    else if (object[0]==51)
    {
        //
        //printf("Object 3 locating...");
        iLowH = 50;
        iHighH = 90;

        iLowS = 50; 
        iHighS = 255;

        iLowV = 100;
        iHighV = 255;
    }
    else
    {
        //
        //printf("Object 3 locating...");
        iLowH = 80;
        iHighH = 179;

        iLowS = 0; 
        iHighS = 255;

        iLowV = 0;
        iHighV = 255;
    }
//oragne is 0-10
   ////////// if (count==50)
   ///////////     imwrite( "origional.jpg", imgOriginal ); 
    cv::Mat imgHSV;
    cv::Mat imgLines = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );

    // Convert the captured frame from BGR to HSV
    cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); 
    
    // Variable to store thresholded image.
    cv::Mat imgThresholded = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );
   
    cv::Mat imgThresholdedFiltered = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );
    cv::Mat ball;

    // Finds pixel values in the set HSV range (red).
    inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); 
     //////////if (count==50)
      //////////  imwrite( "threshold.jpg", imgThresholded); 
  
    // morphological opening (removes small objects from the foreground)
    // the cv::Size function sets the size of objects removed. Increase to get rid +of large objects
    // decrease to leave in smaller objects.
    erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)) ); 

    //morphological closing (removes small holes from the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    ///////////////if (count==50)
    /////////imwrite( "filteredthreshold.jpg", imgThresholded); 
    //Calculate the moments of the thresholded image
    cv::Moments oMoments = cv::moments(imgThresholded);

    imgOriginal.copyTo(ball,imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    if (dArea > 10000)
    {
        //Prepare the image for findContours
        cv::cvtColor(ball, ball, CV_BGR2GRAY);
        cv::threshold(ball, ball, 0, 255, CV_THRESH_BINARY);

        //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = ball.clone();
        cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
        cv::findContours( contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
    
        //Draw the contours
        cv::Mat contourImage(ball.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);

        cv::RNG rng(12345);
        ///////  cv::Mat drawing1 = cv::Mat::zeros( imgThresholded.size(), CV_8UC3 );
        ////// for( int i = 0; i< contours.size(); i++ )
        ////////// {
        //////     cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        ///////////     drawContours( drawing1, contours, i, color, 2, 8,0, 0, cv::Point(0,0) );
        ////// }
        //////// if (count==50)
        /////////     imwrite( "contourimage.jpg", drawing1); 

        // approximate contours
        std::vector<std::vector<cv::Point> > contours_pol( contours.size() );
        for( int i = 0; i < contours.size(); i++ ) 
        {
            approxPolyDP(cv::Mat(contours[i]), contours_pol[i], 5, true );
        }
        //cv::RNG rng(12345);
    
        // merge all contours into one vector
        std::vector<cv::Point> merged_contour_points;
        for (int i = 0; i < contours_pol.size(); i++) 
        {
            for (int j = 0; j < contours_pol[i].size(); j++) 
            {
                merged_contour_points.push_back(contours_pol[i][j]);
            }
        }
    
        // find circle that encloses merged contours
        vector<cv::vector<cv::Point> > contours_poly(1);
        vector<cv::Rect> boundRect( 1);
        vector<cv::Point2f>centers(1);
        
        for( int i = 0; i < 1; i++ )
        { 
            cv::approxPolyDP( cv::Mat(merged_contour_points), contours_poly[i], 3, true );
            cv::minEnclosingCircle( (cv::Mat)merged_contour_points, centers[i], radius );
        }

    
        // Draw Circle onto rgb image 
        cv::Mat drawing = cv::Mat::zeros( imgThresholded.size(), CV_8UC3 );
        for( int i = 0; i< 1; i++ )
        {
            cv::Scalar color = cv::Scalar(0,0,225);
            //cv::Scalar color = cv::Scalar( rng.uniform(0, 0), rng.uniform(0,0), rng.uniform(50,225) );
            cv::circle(imgOriginal,centers[i], radius, color, 2, 8, 0 );
        }
        /////cv::Mat immgLines = cv::Mat::zeros(imgOriginal.size(), CV_8UC3 );;
        ////////circle(immgLines, cv::Point((int)centers[0].x, (int)centers[0].y), 2,cv::Scalar(30,144,255), 4,8, 0);
        ////////if (count==50)
         ////////imwrite( "final.jpg", imgOriginal+drawing+immgLines); 
        // Return value of circle center and radius
        *xpos = (int)centers[0].x;
        *ypos = (int)centers[0].y;
        *radi = (int)radius;
         //cout <<"Radius: " << radius;
        
    }
    //////count++;
    //show the thresholded image
    imshow("Thresholded Image", imgThresholded); 

}



// This function uses OpenCV to track red blob's x, y coordinates and returns them
// as well as returing a threshold map of the red object. 
void rgbImage2(cv::Mat imgOriginal ,int* xpos, int* ypos, int* radi)
{
    ///////static int count = 0;
    // Declare variables for radius and center storage
    cv::Point2f center;
    float radius = 0;
    center.x = 0;
    center.y = 0;

    // Sets HSV values to search for the red object.
    //low was 100 if that needs to change back
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    // if (object[0] == 49)
    // {
        //
        //printf("Object 1 locating...");
        int iLowH1 = 100;
        int iHighH1 = 130;

        int iLowS1 = 100; 
        int iHighS1 = 255;

        int iLowV1 = 49;
        int iHighV1 = 255;
    // }
    // else if (object[0] == 50)
    // {
        //
        //printf("Object 2 locating...");
        int iLowH2 = 0;
        int iHighH2 = 50;

        int iLowS2 = 120; 
        int iHighS2 = 255;

        int iLowV2 = 100;
        int iHighV2 = 255;
    // }
    // else
    // {
        //
        //printf("Object 3 locating...");
        int iLowH3 = 50;
        int iHighH3 = 90;

        int iLowS3 = 50; 
        int iHighS3 = 255;

        int iLowV3 = 100;
        int iHighV3 = 255;
 //   }

//oragne is 0-10
   ////////// if (count==50)
   ///////////     imwrite( "origional.jpg", imgOriginal ); 
    cv::Mat imgHSV;
    cv::Mat imgLines = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );

    // Convert the captured frame from BGR to HSV
    cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); 
    
    // Variable to store thresholded image.
    cv::Mat imgThresholded = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );
    cv::Mat imgThresholded1 = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );
    cv::Mat imgThresholded2 = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );
    cv::Mat imgThresholded3 = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );
   
    cv::Mat imgThresholdedFiltered = cv::Mat::zeros( imgOriginal.size(), CV_8UC3 );
    cv::Mat ball;

    // Finds pixel values in the set HSV range (red).
    inRange(imgHSV, cv::Scalar(iLowH1, iLowS1, iLowV1), cv::Scalar(iHighH1, iHighS1, iHighV1), imgThresholded1); 
    inRange(imgHSV, cv::Scalar(iLowH2, iLowS2, iLowV2), cv::Scalar(iHighH2, iHighS2, iHighV2), imgThresholded2); 
    inRange(imgHSV, cv::Scalar(iLowH3, iLowS3, iLowV3), cv::Scalar(iHighH3, iHighS3, iHighV3), imgThresholded3); 


    imgThresholded = imgThresholded1+imgThresholded2+imgThresholded3;

    erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1)) ); 
    // dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    // erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::Moments oMoments = cv::moments(imgThresholded);

    imgOriginal.copyTo(ball,imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    if (dArea > 10000)
    {
        //Prepare the image for findContours
        cv::cvtColor(ball, ball, CV_BGR2GRAY);
        cv::threshold(ball, ball, 0, 255, CV_THRESH_BINARY);

        //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = ball.clone();
        cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
        cv::findContours( contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
    
        //Draw the contours
        cv::Mat contourImage(ball.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);

        cv::RNG rng(12345);
        ///////  cv::Mat drawing1 = cv::Mat::zeros( imgThresholded.size(), CV_8UC3 );
        ////// for( int i = 0; i< contours.size(); i++ )
        ////////// {
        //////     cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        ///////////     drawContours( drawing1, contours, i, color, 2, 8,0, 0, cv::Point(0,0) );
        ////// }
        //////// if (count==50)
        /////////     imwrite( "contourimage.jpg", drawing1); 

        // approximate contours
        std::vector<cv::Point> approx;
        cv::Mat drawing = cv::Mat::zeros( imgThresholded.size(), CV_8UC3 );
        cv::Scalar color = cv::Scalar(0,0,0);
        cv::Scalar color2 = cv::Scalar(0,0,0);
        double maxCosine;
        int type = 0;
        cout<<"starting looking at contours this image\n";
        for( int i = 0; i < contours.size(); i++ ) 
        {
            approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.04, true );
            type=0;
            if(approx.size() == 4 && fabs(contourArea(cv::Mat(approx))) > 100 && isContourConvex(cv::Mat(approx)))
            {
                type = 1;
                cout <<"There's a square here!\n";
                string label = "Square";
                for (int c = 0; c<approx.size(); c++)
                {
                    // for( int j = 2; j < 5; j++ )
                    // {
                    //     double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                    //     maxCosine = MAX(maxCosine, cosine);
                    // }
                    // if( maxCosine < 0.3 )
                    // {   
                        int next_c = c+1;
                        if (next_c >=approx.size())
                        {
                            next_c=0;
                        }
                        cv::line(imgOriginal, approx[c], approx[next_c], color, 2, 8, 0);

                    //}
                }
                int c = 0;
                cv::putText(imgOriginal, label, approx[c], 1, 1.0, color2, 2.0);
            }
            if(approx.size() > 4 && fabs(contourArea(cv::Mat(approx))) > 100 && isContourConvex(cv::Mat(approx)))
            {
                type = 2;
                cout <<"There's a circle here!\n";
                string label = "Circle";

                for (int c = 0; c<approx.size(); c++)
                {
                    // for( int j = 2; j < 5; j++ )
                    // {
                    //     double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                    //     maxCosine = MAX(maxCosine, cosine);
                    // }
                    // if( maxCosine < 0.3 )
                    // {   
                        int next_c = c+1;
                        if (next_c >=approx.size())
                        {
                            next_c=0;
                        }
                        cv::line(imgOriginal, approx[c], approx[next_c], color, 2, 8, 0);
                    //}
                }
                int c = 0;
                cv::putText(imgOriginal, label, approx[c], 1, 1.0, color2, 2.0);
            }
            if(approx.size() < 4 && fabs(contourArea(cv::Mat(approx))) > 100 && isContourConvex(cv::Mat(approx)))
            {
                type = 3;
                cout <<"There's a triangle here!\n";
                string label = "Triangle";
   
                for (int c = 0; c<approx.size(); c++)
                {
                    // for( int j = 2; j < 5; j++ )
                    // {
                    //     double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                    //     maxCosine = MAX(maxCosine, cosine);
                    // }
                    // if( maxCosine < 0.3 )
                    // {   
                        int next_c = c+1;
                        if (next_c >=approx.size())
                        {
                            next_c=0;
                        }
                        cv::line(imgOriginal, approx[c], approx[next_c], color, 2, 8, 0);
                    //}
                }
                int c = 0;
                cv::putText(imgOriginal, label, approx[c], 1, 1.0, color2, 2.0);
            }
            // if (type!=0){
            //     for (int c = 0; c<approx.size(); c++)
            //     {
            //         // for( int j = 2; j < 5; j++ )
            //         // {
            //         //     double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
            //         //     maxCosine = MAX(maxCosine, cosine);
            //         // }
            //         // if( maxCosine < 0.3 )
            //         // {   
            //             int next_c = c+1;
            //             if (next_c >=approx.size())
            //             {
            //                 next_c=0;
            //             }
            //             cv::line(drawing, approx[c], approx[next_c], color, 2, 8, 0);
            //         //}
            //     }
            // }
        }
            if (cv::waitKey(10) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            {
      // cout << "esc key is pressed by user" << endl;
 
                while(1)
                {
                    if (cv::waitKey(30) == 27) 
                    break;
                }
            }

                // if(approx.size() == 6 && fabs(contourArea(cv::Mat(approx))) > 100 && isContourConvex(cv::Mat(approx))){
                // type = 3;
                // cout <<": "
                // for (int c = 0; c<approx.size(); c++)
                // {
                //     // for( int j = 2; j < 5; j++ )
                //     // {
                //     //     double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                //     //     maxCosine = MAX(maxCosine, cosine);
                //     // }
                //     // if( maxCosine < 0.3 )
                //     // {   
                //         int next_c = c+1;
                //         if (next_c >=approx.size())
                //         {
                //             next_c=0;
                //         }
                //         cv::line(drawing, approx[c], approx[next_c], color, 2, 8, 0);
                //     //}
                // }
        //     }
        // }
        //cv::RNG rng(12345);

        // for (int c = 0; c<approx.size(); c++)
        // {
        //     int next_c = c+1;
        //     if (next_c >=approx.size())
        //     {
        //         next_c=0;
        //     }
        //     cv::line(drawing, approx[c], approx[next_c], color, 2, 8, 0);
        // }
        cv::imshow( "Lines", imgOriginal); 
        // // merge all contours into one vector
        // std::vector<cv::Point> merged_contour_points;
        // for (int i = 0; i < contours_pol.size(); i++) 
        // {
        //     for (int j = 0; j < contours_pol[i].size(); j++) 
        //     {
        //         merged_contour_points.push_back(contours_pol[i][j]);
        //     }
        // }
    
        // // find circle that encloses merged contours
        // vector<cv::vector<cv::Point> > contours_poly(1);
        // vector<cv::Rect> boundRect( 1);
        // vector<cv::Point2f>centers(1);
        
        // for( int i = 0; i < 1; i++ )
        // { 
        //     cv::approxPolyDP( cv::Mat(merged_contour_points), contours_poly[i], 3, true );
        //     cv::minEnclosingCircle( (cv::Mat)merged_contour_points, centers[i], radius );
        // }

    
        // // Draw Circle onto rgb image 
        // cv::Mat drawing = cv::Mat::zeros( imgThresholded.size(), CV_8UC3 );
        // for( int i = 0; i< 1; i++ )
        // {
        //     cv::Scalar color = cv::Scalar(0,0,225);
        //     //cv::Scalar color = cv::Scalar( rng.uniform(0, 0), rng.uniform(0,0), rng.uniform(50,225) );
        //     cv::circle(imgOriginal,centers[i], radius, color, 2, 8, 0 );
        // }
        // /////cv::Mat immgLines = cv::Mat::zeros(imgOriginal.size(), CV_8UC3 );;
        // ////////circle(immgLines, cv::Point((int)centers[0].x, (int)centers[0].y), 2,cv::Scalar(30,144,255), 4,8, 0);
        // ////////if (count==50)
        //  ////////imwrite( "final.jpg", imgOriginal+drawing+immgLines); 
        // // Return value of circle center and radius
        // *xpos = (int)centers[0].x;
        // *ypos = (int)centers[0].y;
        // *radi = (int)radius;
        //  //cout <<"Radius: " << radius;
        
    }
    //////count++;
    //show the thresholded image
    imshow("Thresholded Image", imgThresholded); 
}

