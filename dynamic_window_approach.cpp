#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
using namespace std;
using namespace cv; 

Mat image=imread("Test1.png",1);
Mat img=imread("Test1.png",0);
//Globally defined parameters
float at=2;     //m/s
float ar=0.5;   //rad/s
int t_s=250;   //time-slice (msec)
double dt;
vector <float> vd,wd,vr,wr; //Vr = Va intersection Vd
float vf,wf;
double min_dist=INFINITY;

//**************************************************#######################################################################*************************************
//Storing v_dynamic
float vdynamic(float vi,float wi){ //stored in vd
    vd.push_back(vi-at*t_s);
    vd.push_back(vi+at*t_s);
    wd.push_back(wi-ar*t_s);
    wd.push_back(wi-ar*t_s);
}

//Finding distance of nearest obstacle
float obsdist(int xi,int yi,float theta){   
    for(float vi=vd[0];vi<=vd[1];vi+=0.5){
        for(float wi=wd[0];wi<=wd[1];wi+=0.5){
            if(wi)
                continue;
            float rad=vi/wi;
            float cx=-xi-rad*sin(theta);
            float cy=-yi+rad*cos(theta);
            for(int i=0;i<img.rows;i++){
                for(int j=0;j<img.cols;j++){
                    if(img.at<uchar>(i,j)!=255)
                        continue;
                    double deltatx=(asin(sin(theta)-(i-xi/rad))-theta)/wi;
                    double deltaty=(acos(cos(theta)+(j-yi/rad))-theta)/wi;
                    if(deltatx>0.25 || deltaty>0.25 || deltatx<0 || deltaty<0)
                        continue;
                        if(rad*acos((pow(cx-i,2)+pow(cy-j,2))/2*rad*rad)<min_dist){
                            min_dist=rad*acos((pow(cx-i,2)+pow(cy-j,2))/2*rad*rad);
                            dt=min(deltatx,deltaty);
                        }
                }
            }
        }
    }
}

//Checking v_admissible and storing in vr, wr
float vadmissible(){ //vr storing intersection of vd and va
    for(float vi=vd[0];vi<=vd[1];vi+=0.5){
        for(float wi=wd[0];wi<=wd[1];wi+=0.5){
            if(vi<=sqrt(2*at*min_dist) && wi<=sqrt(2*ar*min_dist)){
                vr.push_back(vi);
                wr.push_back(wi);
            }
        }
    }
}

//Maximizing the objective function
float objective(int xi,int yi,float theta,int xn,int yn){
    double obj=0,heading;
    double m=yn-yi/(xn-xi);
    double angle=atan(abs(m-tan(theta))/(1+m*tan(theta)));
    heading=M_PI-angle;
    for(int i=0;i<vr.size();i++){
        for(int j=0;j<wr.size();j++){
            double temp=0.8*heading+0.1*min_dist+0.1*vr[i];
            if(temp>obj){
                obj=temp;
                //Final velocities set for the path
                vf=vr[i];
                wf=vr[j];
            }
        }
    }
}

//*************************************************######################################################################************************************

//Class definition
class PathPlanner{
private:
    int x0,y0;
    int xn,yn;
    int xi,yi;
    Mat obstacle_map;
    float path_vector;
    float vi;
    float wi;
    // Mat planning_method;
    // float path_length;

public:
    PathPlanner(int start[],int end[],float v,float w,Mat map,float orient){
        //Initialized the original state of the bot
        x0=start[0];y0=start[1];
        xn=end[0];yn=end[1];
        obstacle_map=map;
        path_vector=orient;
        v=v;
        w=w;
        xi=x0;yi=y0;//initial value
        while(1){
            if(xi==xn && yi==yn)
                break;
            getPath(xi,yi,vi,wi,path_vector);
            cout<<"first\n";
        }
    }
    void getPath(int xi,int yi,float vi,float wi,float theta);
};

//Gets called recurisively
void PathPlanner::getPath(int xi,int yi,float vi,float wi,float theta){
    int time=0;
    while(1){
        vdynamic(vi,wi); //dynamic velocities selected
        obsdist(xi,yi,theta); //Closest obstacle distance found
        vadmissible(); //Set containing final velocities for the objective function created
        objective(xi,yi,theta,xn,yn); //Final v and w for next t seconds set
        //Updates the next position acc to new v and w for next dt seconds
        xi=xi+vf/wf*(sin(theta)-sin(theta+wf*dt));
        yi=yi-vf/wf*(cos(theta)-cos(theta+wf*dt));
        theta=theta+wf*dt;
        time+=dt;

        //Path printing
        img.at<uchar>(xi,yi)=127;
        if(xi==xn && yi==yn)
            break;
        if(time>=t_s)
            break;
        waitKey(dt);
        }
    }

//Main function
int main(){
    int start[2]={0},end[2]={0},cs=0,ce=0;
    
    //Initializing start and end points
    for(int i=0;i<img.rows;i++){
        for(int j=0;j<img.cols;j++){
            if(img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==255 && img.at<Vec3b>(i,j)[2]==0){
                start[0]=i;
                start[1]=j;
            }
            if(img.at<Vec3b>(i,j)[0]==0 && img.at<Vec3b>(i,j)[1]==0 && img.at<Vec3b>(i,j)[2]==255){
                end[0]=i;
                end[1]=j;
            }
        }
    }
    PathPlanner dwa(start,end,5,5,img,45*M_PI/180); //(x0,y0,xn,yn,vi,wi,image,theta_i) set
    namedWindow("Win",WINDOW_NORMAL);
    imshow("Win",img);
    waitKey(0);
    return 0;
}