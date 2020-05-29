#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
using namespace std;
using namespace cv; 

Mat raw=imread("task_4_images/raw_images/dusseldorf_000114_000019_leftImg8bit.png",0);
Mat rawgb(raw.rows,raw.cols,CV_8UC1,Scalar(0,0,0));
Mat seg=imread("task_4_images/segmented_images/dusseldorf_000114_000019_gtFine_color.png",0);
Mat canny;
Mat hough(raw.rows,raw.cols,CV_8UC3,Scalar(0,0,0)); //Prints lines detected  
Mat houghfine(raw.rows,raw.cols,CV_8UC3,Scalar(0,0,0)); //Prints single best fitting line

void prewitt(int th, void *c){         //void call(int, void *c) is compatible with createTrackbar
    Mat a(raw.rows,raw.cols,CV_8UC1,Scalar(0));
    for (int i=0;i<raw.rows;i++){
        for (int j=0;j<raw.cols;j++){
            float g=0, gx=0, gy=0;
            //Gx
            for (int x=i-1;x<=i+1 && x>=0 && x<raw.rows;x++){
                gx+=(rawgb.at<uchar>(x,j-1)-rawgb.at<uchar>(x,j+1))/3;
            }
            //Gy
            for (int y=j-1;y<=j+1 && y>=0 && y<raw.cols;y++){
                gy+=(rawgb.at<uchar>(i-1,y)-rawgb.at<uchar>(i+1,y))/3;
            }
            g=sqrt((gx*gx)+(gy*gy));
            if (g>=th){
                a.at<uchar>(i,j)=255;
            }
        }   
    }
    imshow("Win",a);    //This image is just for setting threshold
}

void Gaussian_blur(int i,int j){
    int c=1,sum=0;
    for (int x=i-1;x<=i+1;x++){
        for (int y=j-1;y<=j+1;y++){
            if (c==1||c==3||c==7||c==9){
                sum+=0.0625*raw.at<uchar>(x,y);
            }else if (c==2||c==4||c==6||c==8){
                sum+=0.125*raw.at<uchar>(x,y);
            }else{
                sum+=0.25*raw.at<uchar>(x,y);
            }
        }
    }
    rawgb.at<uchar>(i,j)=sum;
}

int main(){

    //Applying Gaussian blur
    for(int i=0;i<raw.rows;i++){
        for(int j=0;j<raw.cols;j++){
            Gaussian_blur(i,j);
        }
    }
   
    // //For fine tuning our threshold
    // int th=0;
    // namedWindow("Win",WINDOW_NORMAL);
    // createTrackbar("threshold","Win",&th,255,prewitt);
    
    Canny(rawgb,canny,9,17,3);
    // imshow("img",canny);

    for(int i=0;i<raw.rows;i++){        //Using given image for segmenting out the road
        for(int j=0;j<raw.cols;j++){
            if(seg.at<uchar>(i,j)==0)
                canny.at<uchar>(i,j)=0;
        }
    }

    //Applying hough lines on the segmented image
    vector<Vec4i> lines;
    HoughLinesP(canny, lines, 1, CV_PI/180, 80, 30, 10 );
    cvtColor(raw, hough, cv::COLOR_GRAY2BGR);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line(hough, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }

    Vec4d regline_left,regline_right;
    vector<Point> left_line,right_line;
    int x1,y1,x2,y2;
    for( size_t i = 0; i < lines.size(); i++ ){
        x1=lines[i][0]; //start point
        y1=lines[i][1];
        x2=lines[i][2]; //end point
        y2=lines[i][3];
        
        if(x1<raw.cols/2){
            left_line.push_back(Point(x1,y1));
            left_line.push_back(Point(x2,y2));
        }else{
            right_line.push_back(Point(x1,y1));
            right_line.push_back(Point(x2,y2));
        }
    }
    cout<<left_line.size()<<"|"<<right_line.size()<<endl;

    //Drawing best fit line
    //right side
    double right_m,right_c;
    if(right_line.size()>0){
        fitLine(right_line,regline_right,CV_DIST_L2,0,0.01,0.01);
        right_m=regline_right[1]/regline_right[0];
        right_c=regline_right[3]-right_m*regline_right[3];
    
        for(int j = 0; j < right_line.size()-1; j++ )
        {   int x1_right=right_line[j].x;
            int y1_right=right_m*right_line[j].x+right_c;
            int x2_right=right_line[j+1].x;
            int y2_right=right_m*right_line[j+1].x+right_c; 
            line(houghfine, Point(x1,y1), Point(x2,y2), Scalar(0,0,255), 3, 8 );
        }
    }

    //left side
    double left_m,left_c;
    if(left_line.size()>0){
        fitLine(left_line,regline_left,CV_DIST_L2,0,0.01,0.01);
        left_m=regline_left[1]/regline_left[0];
        left_c=regline_left[3]-left_m*regline_left[3];
    }

    for( int i = 0; i < left_line.size()-1; i++ )
    {   int x1_left=left_line[i].x;
        int y1_left=left_m*left_line[i].x+left_c;
        int x2_left=left_line[i+1].x;
        int y2_left=left_m*left_line[i+1].x+left_c; 
        line(houghfine, Point(x1,y1), Point(x2,y2), Scalar(0,0,255), 3, 8 );
    }
    
    imshow("img",houghfine);

    //Counting no. of lanes
    int c=1,jtemp=0;
    for(int j=0;j<hough.cols;j++){
        if(hough.at<Vec3b>(223,j)[2]==255){
            if(abs(jtemp-j)>30)
                c++;
        jtemp=j;
        }
    }
    cout<<"No. of lanes="<<c<<endl;

    namedWindow("Window",WINDOW_NORMAL);
    imshow("Window",hough);
    waitKey();

}