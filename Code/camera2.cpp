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
    int iLowH1 = 100;
    int iHighH1 = 130;
    int iLowS1 = 100; 
    int iHighS1 = 255;
    int iLowV1 = 49;
    int iHighV1 = 255;
    int iLowH2 = 0;
    int iHighH2 = 50;
    int iLowS2 = 120; 
    int iHighS2 = 255;
    int iLowV2 = 100;
    int iHighV2 = 255;
    int iLowH3 = 50;
    int iHighH3 = 90;
    int iLowS3 = 50; 
    int iHighS3 = 255;
    int iLowV3 = 100;
    int iHighV3 = 255;

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
                        int next_c = c+1;
                        if (next_c >=approx.size())
                        {
                            next_c=0;
                        }
                        cv::line(imgOriginal, approx[c], approx[next_c], color, 2, 8, 0);
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
                        int next_c = c+1;
                        if (next_c >=approx.size())
                        {
                            next_c=0;
                        }
                        cv::line(imgOriginal, approx[c], approx[next_c], color, 2, 8, 0);
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
      
                        int next_c = c+1;
                        if (next_c >=approx.size())
                        {
                            next_c=0;
                        }
                        cv::line(imgOriginal, approx[c], approx[next_c], color, 2, 8, 0);
                }
                int c = 0;
                cv::putText(imgOriginal, label, approx[c], 1, 1.0, color2, 2.0);
            }

        }
            if (cv::waitKey(10) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            {
                while(1)
                {
                    if (cv::waitKey(30) == 27) 
                    break;
                }
            }
    }
    imshow("Thresholded Image", imgThresholded); 
}