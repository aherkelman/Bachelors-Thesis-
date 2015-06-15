void rgbImage(cv::Mat imgOriginal ,int* xpos, int* ypos, int* radi)
{
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
        iLowH = 100;
        iHighH = 130;

        iLowS = 100; 
        iHighS = 255;

        iLowV = 49;
        iHighV = 255;
    }
    else if (object[0] == 50)
    {
        iLowH = 0;
        iHighH = 10;

        iLowS = 100; 
        iHighS = 255;

        iLowV = 49;
        iHighV = 255;
    }
    else if (object[0]==51)
    {
        iLowH = 50;
        iHighH = 90;

        iLowS = 50; 
        iHighS = 255;

        iLowV = 100;
        iHighV = 255;
    }
    else
    {
        iLowH = 80;
        iHighH = 179;

        iLowS = 0; 
        iHighS = 255;

        iLowV = 0;
        iHighV = 255;
    }
; 
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
     
    // the cv::Size function sets the size of objects removed. Increase to get rid +of large objects
    // decrease to leave in smaller objects.
    erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)) ); 

    //morphological closing (removes small holes from the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

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

        // approximate contours
        std::vector<std::vector<cv::Point> > contours_pol( contours.size() );
        for( int i = 0; i < contours.size(); i++ ) 
        {
            approxPolyDP(cv::Mat(contours[i]), contours_pol[i], 5, true );
        }
    
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
        // Return value of circle center and radius
        *xpos = (int)centers[0].x;
        *ypos = (int)centers[0].y;
        *radi = (int)radius;        
    }
    //show the thresholded image
    imshow("Thresholded Image", imgThresholded); 
}