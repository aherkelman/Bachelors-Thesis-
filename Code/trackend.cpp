// This code reads in color and depth data from creative sense3D using soft kinetic's
// DepthSenseSDK. (should also work with softkinetics DS311 camera). The color data is 
// then analyzed using OpenCV using blob detection to locate and track a red object. 
// The coorisponding depth coordinate is then found and displayed along with the x y.

//NOTE WEIRD ERROR RUNNING WHEN IT SAYS IT HAS SOMETHING TO DO WITH BOOST THEN IT MEANS A SERIAL PORT
// IS NOT CONNECTED. TO FIX CONNECT A SERIAL PORT. THAT IS ALL.

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <time.h>

#include <vector>
#include <exception>

#include "cv.h"
#include "highgui.h"
#include "math.h"

#include <DepthSense.hxx>

#include "DepthSenseGrabberCV.hxx"
#include "ConversionTools.hxx"

#include <boost/asio.hpp>

#include <iostream>
#include <string> 
#include "openCvFunctions.h"
#include "filters.h"
#include "communication.h"


using namespace DepthSense;
using namespace std;

///Object selected by program input
char* object;

bool usingUSB30Flag = 1;
bool exportJPGFlag = 1;

bool dispColorRawFlag = 1;
bool dispDepthRawFlag = 1;
bool dispColorSyncFlag = 1;
bool dispDepthSyncFlag = 1;

bool saveColorRawFlag = 1;
bool saveDepthRawFlag = 1;
bool saveColorSyncFlag = 1;
bool saveDepthSyncFlag = 1;

int frameRateDepth = 30;
int frameRateColor = 30;

// Location variables
int xpos;
int ypos;
float meterzpos = 0;
float meterypos = 0;
float meterxpos = 0;

FrameFormat frameFormatDepth = FRAME_FORMAT_QVGA; const int widthDepth = 320, heightDepth = 240; // Depth QVGA

FrameFormat frameFormatColor = FRAME_FORMAT_VGA; const int widthColor = 640, heightColor= 480; // Color VGA

// Variables for storing raw data.
const int nPixelsColorRaw = 3*widthColor*heightColor;
const int nPixelsDepthRaw = widthDepth*heightDepth;
const int nPixelsColorSync = 3*widthDepth*heightDepth;
const int nPixelsDepthSync = widthColor*heightColor;
uint8_t pixelsColorRaw[nPixelsColorRaw];
uint16_t pixelsDepthRaw[nPixelsDepthRaw];
uint8_t pixelsColorSync[nPixelsColorSync];
uint16_t pixelsDepthSync[nPixelsDepthSync];

const uint16_t noDepthDefault = 0;
const uint16_t noDepthThreshold = 1000;

uint8_t noDepthBGR[3] = {255,255,255};

// Varibles used for uv mapping.
int colorPixelInd, colorPixelRow, colorPixelCol;
UV uv;
float u,v;
int countColor, countDepth; // DS data index


int timeStamp;                                                                                         

int divideDepthBrightnessCV = 6;

unsigned int frameCount;

// Open CV vars
IplImage
*g_depthRawImage=NULL,
*g_colorRawImage=NULL, // initialized in main, used in CBs
*g_depthSyncImage=NULL, // initialized in main, used in CBs
*g_colorSyncImage=NULL, // initialized in main, used in CBs
*g_emptyImage=NULL; // initialized in main, used in CBs

CvSize g_szDepthRaw=cvSize(widthDepth,heightDepth), // QVGA
g_szColorRaw=cvSize(widthColor,heightColor); //VGA

CvSize g_szDepthSync = g_szColorRaw, g_szColorSync = g_szDepthRaw;
bool g_saveFrameFlag=false;

// Create color and depth node variables.
Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;

// Counters for depth and color streams.
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

//Low pass filter variables
struct filtdata Xcoord;
struct filtdata Ycoord;
struct filtdata Zcoord;

// This function used the DepthsenseSDK's uv map and translates it to a color pixel coordinate. 
void uvToColorPixelInd(UV uv, int widthColor, int heightColor, int* colorPixelInd, int* colorPixelRow, int* colorPixelCol) {
    if(uv.u > 0.001 && uv.u < 0.999 && uv.v > 0.001 && uv.v < 0.999) 
    {
        *colorPixelRow = (int) (uv.v * ((float) heightColor));
        *colorPixelCol = (int) (uv.u * ((float) widthColor));
        *colorPixelInd = (*colorPixelRow)*widthColor + *colorPixelCol;
    }
    else
        *colorPixelInd = -1;
}

// Stores color data when a new frame is recieved.
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{

    timeStamp = (int) (((float)(1000*clock()))/CLOCKS_PER_SEC);

    countColor = 0;

    if (data.colorMap!=0)// just in case !
        for (int i=0; i<heightColor; i++)
        {
            for (int j=0; j<widthColor; j++)
            {
                pixelsDepthSync[countColor] = noDepthDefault;
                pixelsColorRaw[3*countColor] = data.colorMap[3*countColor+2];
                pixelsColorRaw[3*countColor+1] = data.colorMap[3*countColor+1];
                pixelsColorRaw[3*countColor+2] = data.colorMap[3*countColor];
                if (dispColorRawFlag || (saveColorRawFlag && exportJPGFlag)) cvSet2D(g_colorRawImage,i,j,cvScalar(pixelsColorRaw[3*countColor+2],pixelsColorRaw[3*countColor+1],pixelsColorRaw[3*countColor])); //BGR format
                if (dispDepthSyncFlag || (saveDepthSyncFlag && exportJPGFlag)) cvSet2D(g_depthSyncImage,i,j,cvScalar(pixelsDepthSync[countColor]));
                countColor++;
            }
        }
    g_cFrames++;

}


/*----------------------------------------------------------------------------*/
// New depth sample event handler
// Activates when a new depth fram is recieved. 
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
    int ypos;
    int xpos;
    int radi;
    int iLastX;
    int iLastY;

    //timeStamp = (int) (((float)(1000*clock()))/CLOCKS_PER_SEC); 
    //cout <<timeStamp<<"\n";
    countDepth = 0;

    // Stores color frame into cv file.
    cv::Mat rawRGB;
    rawRGB = g_colorRawImage;
    cv::Mat immgLines = cv::Mat::zeros(rawRGB.size(), CV_8UC3 );;
    rgbImage(rawRGB, &xpos, &ypos, &radi);
    //rgbImage(rawRGB, &xpos, &ypos, &radi);
    radi = (int) radi/3;
    meterxpos = 0;
    meterypos = 0;
    meterzpos = 0;
    int pointincircle = 0;
    int ymin, ymax, xmin, xmax;
    ymin = ypos - radi;
    ymax = ypos + radi;
    xmin = xpos - radi;
    xmax = xpos + radi;

    // Gets depth image data.
    if (data.depthMap!=0)// just in case !
    {
        for (int i=0; i<heightDepth; i++)
        {
            for (int j=0; j<widthDepth; j++)
            {
                //gets uv depthmap
                uv = data.uvMap[countDepth];
                if (data.depthMap[countDepth] < noDepthThreshold)
                    pixelsDepthRaw[countDepth] = data.depthMap[countDepth];
                else
                    pixelsDepthRaw[countDepth] = noDepthDefault;
                // converts uv coordinate to color pixel index.
                uvToColorPixelInd(uv, widthColor, heightColor, &colorPixelInd, &colorPixelRow, &colorPixelCol);
            
                // compares uv maped coordinate to location of red blob, if equal stores the x,y,z coordinates
                // of the selected location.

                if ((colorPixelRow >= ymin)&&(colorPixelRow <= ymax)&&(colorPixelCol >= xmin)&&(colorPixelCol <= xmax))
                {
                    //if(data.verticesFloatingPoint[countDepth].x != 0 || data.verticesFloatingPoint[countDepth].y != 0 )
                    if((data.verticesFloatingPoint[countDepth].z != -2) && (data.depthMap[countDepth]< noDepthThreshold))
                    {
                        Xcoord.x = Xcoord.x + data.verticesFloatingPoint[countDepth].x*39.3701;
                        Ycoord.x = Ycoord.x  + data.verticesFloatingPoint[countDepth].y*39.3701;
                        Zcoord.x = Zcoord.x + data.verticesFloatingPoint[countDepth].z*39.3701;
                        //meterzpos = meterzpos + data.depthMap[countDepth];
                        pointincircle++;
                    }
                }
                
                // The following commented code is to calculate color to depth and depth to color maps. 

                // if (colorPixelInd == -1) {
                //     pixelsDepthSync[colorPixelInd] = noDepthDefault;
                //     pixelsColorSync[3*countDepth] = noDepthBGR[2];
                //     pixelsColorSync[3*countDepth+1] = noDepthBGR[1];
                //     pixelsColorSync[3*countDepth+2] = noDepthBGR[0];
                // }
                // else {
                //     pixelsDepthSync[colorPixelInd] = data.depthMap[countDepth];
                //     pixelsColorSync[3*countDepth] = pixelsColorRaw[3*colorPixelInd];
                //     pixelsColorSync[3*countDepth+1] = pixelsColorRaw[3*colorPixelInd+1];
                //     pixelsColorSync[3*countDepth+2] = pixelsColorRaw[3*colorPixelInd+2];
                // }
                // if (dispDepthSyncFlag || (saveDepthSyncFlag && exportJPGFlag)) cvSet2D(g_depthSyncImage,colorPixelRow,colorPixelCol,cvScalar(pixelsDepthSync[colorPixelInd]/divideDepthBrightnessCV));
                // if (dispColorSyncFlag || (saveColorSyncFlag && exportJPGFlag)) cvSet2D(g_colorSyncImage,i,j,cvScalar(pixelsColorSync[3*countDepth+2],pixelsColorSync[3*countDepth+1],pixelsColorSync[3*countDepth])); //BGR format
                
                //creates displayable depth map.
                if (dispDepthRawFlag || (saveDepthRawFlag && exportJPGFlag)) cvSet2D(g_depthRawImage,i,j,cvScalar(pixelsDepthRaw[countDepth]/divideDepthBrightnessCV));
                countDepth++;
            }
        }
    }
    g_dFrames++;
   
    /* OpenCV display - this will slow stuff down, should be in thread*/
    
    // Low pass filter coordinates
    if (g_dFrames == 0)
    {
        Xcoord.xp = Xcoord.xpp = Xcoord.yp = Xcoord.ypp = Xcoord.x;
        Ycoord.xp = Ycoord.xpp = Ycoord.yp = Ycoord.ypp  = Ycoord.x;
        Zcoord.xp = Zcoord.xpp = Zcoord.yp = Zcoord.ypp = Zcoord.x;
    }

    
    // Takes average of x y and z coordinates in red object.
    //cout <<"points: "<< pointincircle << " ";
    if (pointincircle == 0)
    {
        Xcoord.x = Xcoord.xp;
        Ycoord.x = Ycoord.xp;
        Zcoord.x = Zcoord.xp;
    }
    else
    {
        Xcoord.x = Xcoord.x/pointincircle;
        Ycoord.x = Ycoord.x/pointincircle;
        Zcoord.x = Zcoord.x/pointincircle;
    }  

    // Smooth large data jumps.
    if (((abs(Xcoord.x-Xcoord.xp)>2) || (abs(Ycoord.x-Ycoord.xp)>2) || (abs(Zcoord.x-Zcoord.xp)>2)) && (g_dFrames>5))
    {
        Xcoord.x = (Xcoord.x + Xcoord.xp)/2;
        Ycoord.x = (Ycoord.x + Ycoord.xp)/2;
        Zcoord.x = (Zcoord.x + Zcoord.xp)/2;
    }

    Xcoord.y = lpf(Xcoord);
    Ycoord.y = lpf(Ycoord);
    Zcoord.y = lpf(Zcoord);

    updateFilter(&Xcoord);
    updateFilter(&Ycoord);
    updateFilter(&Zcoord);

    // Displays x, y, and z coordinates of the red object's center.
    cout <<Xcoord.y << " " << Ycoord.y << " " << Zcoord.y << " " << radi << " \n";
    //cout << sizeof(int)<<"\n";

    //more serial attemps
    //sendData(Xcoord.y, Ycoord.y, Zcoord.y);
   
    cv::Scalar color = cv::Scalar(255,0,0);
    if (xpos >= 0 && ypos >= 0)
    {
        // Draw center of circle
        circle(immgLines, cv::Point(xpos, ypos), 2,cv::Scalar(30,144,255), 4,8, 0);
        rectangle(immgLines, cv::Point(xpos+radi,ypos+radi), cv::Point(xpos-radi,ypos-radi), color, 2, 6, 0);
    }


    rawRGB = rawRGB + immgLines;
    //Display images
    //cvNamedWindow("Name", CV_WINDOW_NORMAL);
    //cvSetWindowProperty("Name", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    if (dispColorRawFlag) imshow("Name", rawRGB);
    if (dispColorRawFlag) imshow("Raw Color",rawRGB);
    if (dispDepthRawFlag) cvShowImage("Raw Depth",g_depthRawImage);

    // Following code is for displaying the synconized images that can be calculated above.

    //if (dispDepthSyncFlag) cvShowImage("Synchronized Depth",g_depthSyncImage);
    //if (dispColorSyncFlag) cvShowImage("Synchronized Color",g_colorSyncImage);
    //if (dispColorRawFlag+dispColorSyncFlag+dispDepthRawFlag+dispDepthSyncFlag == 0)  cvShowImage("Empty",g_emptyImage);

    //Allow OpenCV to shut down the program
    char key = cvWaitKey(10);

    if (key==27)
    {
        //printf("Quitting main loop from OpenCV\n");
        g_context.quit();
    }
    else if (key=='W' || key=='w') g_saveFrameFlag = !g_saveFrameFlag;
}

/*----------------------------------------------------------------------------*/
// Enables data capture from the depth node. 

void configureDepthNode()
{
    g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);
    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = frameFormatDepth;
    config.framerate = frameRateDepth;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    g_dnode.setEnableDepthMap(true);
    g_dnode.setEnableUvMap(true);
    g_dnode.setEnableVerticesFloatingPoint(true);

    try
    {
        g_context.requestControl(g_dnode,0);

        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("DEPTH Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("DEPTH Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("DEPTH IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("DEPTH Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("DEPTH Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("DEPTH Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("DEPTH TimeoutException\n");
    }

}

/*----------------------------------------------------------------------------*/
// Allows data capture for the color node.

void configureColorNode()
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = frameFormatColor;
    config.compression = COMPRESSION_TYPE_MJPEG; // can also be COMPRESSION_TYPE_YUY2
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = frameRateColor;

    g_cnode.setEnableColorMap(true);

    try
    {
        g_context.requestControl(g_cnode,0);

        g_cnode.setConfiguration(config);
        g_cnode.setBrightness(0);
        g_cnode.setContrast(5);
        g_cnode.setSaturation(5);
        g_cnode.setHue(0);
        g_cnode.setGamma(3);
        g_cnode.setWhiteBalance(4650);
        g_cnode.setSharpness(5);
        g_cnode.setWhiteBalanceAuto(true);
    }
    catch (ArgumentException& e)
    {
        printf("COLOR Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("COLOR Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("COLOR IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("COLOR Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("COLOR Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("COLOR Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("COLOR TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
//Runs either depth or color configuration.
void configureNode(Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }

}

/*----------------------------------------------------------------------------*/
// Runs configure node when node is connected.
void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
// Disconnects the color or audio node.
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
        g_cnode.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
        g_dnode.unset();
    printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
// Connects to nodes when a device is connected.
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

/*----------------------------------------------------------------------------*/
// Return device not found if device is disconnected.
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
    if (argc == 2)
    {
        object = argv[1];
    }
    int i=0;
    //while(i<1000){
    //printf("Object %d\n",object[0]);
    i++;
    std::cout.setf( std::ios_base::unitbuf );
    //}
    //cout <<"Object \r\n";
    //while(1)
     //   printf("hi python\n");
    // if(object[0] == 49)
    // printf("Its a one!\n");
    // if(object[0] == 50)
    // printf("Its a two!\n");
    // if(object[0] == 51)
    // printf("Its a three!\n");
    g_context = Context::create("localhost");

    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    // Get the list of currently connected devices
    vector<Device> da = g_context.getDevices();

    // We are only interested in the first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;

        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

        vector<Node> na = da[0].getNodes();

        printf("Found %lu nodes\n",na.size());

        for (int n = 0; n < (int)na.size(); n++)
            configureNode(na[n]);
    }

    /* Some OpenCV init; make windows and buffers to display the data */

    // VGA format color image
    g_colorRawImage=cvCreateImage(g_szColorRaw,IPL_DEPTH_8U,3);
    if (g_colorRawImage==NULL)
    {
        //printf("Unable to create color image buffer\n");
        exit(0);
    }
   
    // QVGA format depth image
    g_depthRawImage=cvCreateImage(g_szDepthRaw,IPL_DEPTH_8U,1);
    if (g_depthRawImage==NULL)
    {
        //printf("Unable to create depth image buffer\n");
        exit(0);
    }

    // QVGA format depth color image
    g_depthSyncImage=cvCreateImage(g_szDepthSync,IPL_DEPTH_8U,1);
    if (g_depthSyncImage==NULL)
    {
        //printf("Unable to create depth color image buffer\n");
        exit(0);
    }

    // QVGA format depth color image
    g_colorSyncImage=cvCreateImage(g_szColorSync,IPL_DEPTH_8U,3);
    if (g_colorSyncImage==NULL)
    {
        //printf("Unable to create color depth image buffer\n");
        exit(0);
    }

    // Empty image
    g_emptyImage=cvCreateImage(g_szColorSync,IPL_DEPTH_8U,1);
    if (g_emptyImage==NULL)
    {
       // printf("Unable to create empty image buffer\n");
        exit(0);
    }
    //printf("dml@Fordham version of DS ConsoleDemo. June 2013.\n");
    //printf("Updated Feb. 2014 (thp@pham.in).\n");
    //printf("Click onto in image for commands. ESC to exit.\n");
    //printf("Use \'W\' or \'w\' to toggle frame dumping.\n");
    g_context.startNodes();

    g_context.run();

    g_context.stopNodes();

    if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
    if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
    // if (g_anode.isSet()) g_context.unregisterNode(g_anode);

    if (g_pProjHelper)
        delete g_pProjHelper;

    return 0;
}
// ConsoleDemo initial modification due to Damian Lyons
// Code originally retrieved from SoftKinetic forum
// http://www.softkinetic.com/Support/Forum/tabid/110/forumid/32/threadid/1450/scope/posts/language/en-US/Default.aspx

// DepthSense 325 parameters and conversion fix - Tu-Hoa Pham (thp@pham.in)

////////////////////////////////////////////////////////////////////////////////
// SoftKinetic DepthSense SDK
//
// COPYRIGHT AND CONFIDENTIALITY NOTICE - SOFTKINETIC CONFIDENTIAL
// INFORMATION
//
// All rights reserved to SOFTKINETIC SENSORS NV (a
// company incorporated and existing under the laws of Belgium, with
// its principal place of business at Boulevard de la Plainelaan 15,
// 1050 Brussels (Belgium), registered with the Crossroads bank for
// enterprises under company number 0811 341 454 - "Softkinetic
// Sensors").
//
// The source code of the SoftKinetic DepthSense Camera Drivers is
// proprietary and confidential information of Softkinetic Sensors NV.
//
// For any question about terms and conditions, please contact:
// info@softkinetic.com Copyright (c) 2002-2012 Softkinetic Sensors NV
////////////////////////////////////////////////////////////////////////////////

// Some OpenCV mods added below for viewing and saving - Damian Lyons, dlyons@fordham.edu