#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
#include <iostream>
#define ROWS 400
#define COLS 640
// #define ROWS 600
// #define COLS 1000
#define length 30
#define breadth 10

using namespace std;
using namespace cv;
Mat image=imread("Test1.png",1);
// Mat image=imread("Test2.png",1);
Mat img(image.rows,image.cols,CV_8UC1,Scalar(0));

//struct to store all info of the node
struct Nodes{
    int row;
    int col;
    int parent_r;
    int parent_c;
    float theta;
    float heu; //for storing cost + heuristic
};
Nodes node[ROWS][COLS],current;

//Min heap for heuristic values and storing nodes in priority queue
struct myComparator{ 
    bool operator()(Nodes const& n1, Nodes const& n2){ 
        return n1.heu > n2.heu; 
    } 
}; 

//Cost function
float cost(int s0,int s1,int i0,int j0,int i1,int j1,int pr,int pc){
    int dx1,dx2,dy1,dy2,cross;
    dx1=i1-i0;
    dy1=j1-j0;
    dx2=s0-i0;
    dy2=s1-j0;
    cross=abs(dx1*dy2-dx2*dy1);
    return cross*0.001;
}
    

//Heuristic
float heuristic(int s0,int s1,int i0,int j0,int i1,int j1,int pr,int pc){
    float heuristic,D=1,D2=sqrt(2);
    int dx=abs(i1-i0);
    int dy=abs(j1-j0);
    heuristic=D*(dx+dy)+(D2-2*D)*min(dx,dy);
    return (heuristic+cost(s0,s1,i0,j0,i1,j1,pr,pc));
}

//Printing elements of priority queue
void showpq(priority_queue <Nodes, vector<Nodes>, myComparator> gq) 
{ 
    priority_queue <Nodes, vector<Nodes>, myComparator> g = gq; 
    while (!g.empty()) 
    { 
        cout << g.top().row<<"|"<<g.top().col<<"|"<<g.top().heu<<"\n"; 
        g.pop(); 
    } 
    cout << '\n'; 
} 

int main(){

    priority_queue <Nodes, vector<Nodes>, myComparator> open; 
    stack <Nodes> closed,dr; //LIFO
    int end[2]={0},start[2]={0};
    int stsum=0,endsum=0,cs=0,ce=0;
    bool isVisited[ROWS][COLS]; 
    memset(isVisited, false, sizeof(isVisited));

    for (int i=0;i<image.rows;i++){
        for (int j=0;j<image.cols;j++){
            
            if(image.at<Vec3b>(i,j)[0]==0 && image.at<Vec3b>(i,j)[1]==255 && image.at<Vec3b>(i,j)[2]==0){
                //Initializing starting node and pushing its heuristic in open list
                 start[0]+=i;
                 start[1]+=j;
                 cs++;
            }else if(image.at<Vec3b>(i,j)[0]==0 && image.at<Vec3b>(i,j)[1]==0 && image.at<Vec3b>(i,j)[2]==255){
                //Initializing end node
                end[0]+=i;
                end[1]+=j;
                ce++;
            }
                
            //Setting all distances to infinity at first
            node[i][j].row=i;
            node[i][j].col=j;
            node[i][j].heu=FLT_MAX;
            node[i][j].parent_r=-1;
            node[i][j].parent_c=-1;
            node[i][j].theta=0;
        
            
            //Creating a binary equivalent of nodes for simplicity
            if(image.at<Vec3b>(i,j)[0]>=127 && image.at<Vec3b>(i,j)[1]>=127 && image.at<Vec3b>(i,j)[2]>=127)
                img.at<uchar>(i,j)=255;    
            else 
                img.at<uchar>(i,j)=0;  
        }
    }

    start[0]/=cs;
    start[1]/=cs;
    node[start[0]][start[1]].row=start[0];
    node[start[0]][start[1]].col=start[1];
    node[start[0]][start[1]].parent_r=start[2];
    node[start[0]][start[1]].parent_c=start[1];
    node[start[0]][start[1]].heu=0;
    open.push(node[start[0]][start[1]]);

    end[0]/=ce;
    end[1]/=ce;
    node[end[0]][end[1]].row=end[0];
    node[end[0]][end[1]].col=end[1];
    node[end[0]][end[1]].parent_r=-1;   //to be set later
    node[end[0]][end[1]].parent_c=-1;
    node[end[0]][end[1]].heu=0;
    /*-----------------------------------------------------------A-star Algorithm----------------------------------------------------------------*/
    bool foundDest=false; 
    while(!open.empty() && !foundDest){
        current=open.top();
        closed.push(current);
        open.pop();
        isVisited[current.row][current.col]=true;
        //Checking neighbours of the current node, ie, (x,y)
        for(int x=current.row-1;x<=current.row+1 && x<image.rows && x>=0;x++){
            for(int y=current.col-1;y<=current.col+1 && y<image.cols && y>=0;y++){
                
                if(x!=current.row || y!=current.col && isVisited[x][y]==false){
                    if(x==end[0] && y==end[1]){
                        foundDest=true;
                        node[x][y].parent_r=current.row;
                        node[x][y].parent_c=current.col;
                        closed.push(node[x][y]);
                        isVisited[x][y]=true;
                        cout<<"Destination reached"<<endl;
                        break;
                    }

                    //2-D bot w rotation
                    bool found=false;
                    for(int i=x-breadth/2;i<=x+breadth/2 && i>=0 && i<image.rows;i++){
                        for(int j=y-length/2;j<=y+length/2 && j>=0 && j<image.cols;j++){
                            for(int k=0;k<=90;k+=15){ //k in degrees
                                
                            }
                        }
                    }

                    //Push/Update condition
                    if(heuristic(start[0],start[1],end[0],end[1],x,y,current.row,current.col) < node[x][y].heu){
                        node[x][y].row=x;
                        node[x][y].col=y;
                        node[x][y].heu=heuristic(start[0],start[1],end[0],end[1],x,y,current.row,current.col);
                        node[x][y].parent_r=current.row;
                        node[x][y].parent_c=current.col;
                        open.push(node[x][y]);
                    }
                }            
            }
           if(foundDest)
            break; 
        }
        if(!foundDest && open.empty() && closed.top().row!=end[0] && closed.top().col!=end[1])
            cout<<"Failed!"<<"\n";
    }
    cout<<closed.top().row<<"|"<<closed.top().col<<"\n";
    /*--------------------------------------------------------------------------------------------------------------------------------------------*/

    //Storing path points
    dr.push(closed.top());
    while(1){
        int temp_r=dr.top().parent_r;
        int temp_c=dr.top().parent_c;
        dr.push(node[temp_r][temp_c]);
        if(temp_r==start[0] && temp_c==start[1]){
           cout<<"Over\n";
           break;
        }
    }

    // stack <Nodes> show=dr;
    // while(!show.empty()){
    //     cout<<show.top().parent_r<<"|"<<show.top().parent_c<<"\n";
    //     show.pop();
    // }

    //Tracing out the path
    namedWindow("Win",WINDOW_NORMAL);
    while(!dr.empty()){
        Mat image1=image.clone();
        for(int i=dr.top().row-breadth/2;i<=dr.top().row+breadth/2 && i>=0 && i<image.rows;i++){
            for(int j=dr.top().col-length/2;j<=dr.top().col+length/2 && j>=0 && j<image.cols;j++){
            image1.at<Vec3b>(i,j)[0]=255;
            image1.at<Vec3b>(i,j)[1]=255;
            image1.at<Vec3b>(i,j)[2]=0;
        }
    }
    imshow("Win",image1);
    waitKey(10);
    dr.pop();
    }
    waitKey(0);
}

