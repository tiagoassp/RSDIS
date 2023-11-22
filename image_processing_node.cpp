#include "std_msgs/Float32.h"
#include <sstream>

#include "ros/ros.h"
#include <unistd.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#define MICROSECONDS 1000000

double angular, color = 0;

int stop = 0, canStop = 1, pulse = 0, azul = 0;

using namespace cv;
using namespace std;

ros::Publisher pub_image;
ros::Publisher pub_error;

void image_callback(const sensor_msgs::CompressedImageConstPtr& msg) {

        double time = ros::Time::now().toSec();

        //descomprimir imagem
        Mat image_decode;
        image_decode = cv::imdecode(msg->data,cv::IMREAD_COLOR);

        // usar apenas a metade inferior da imagem da raspicam para processamento de imagem 
        Rect Rec(0,154,410,154);
        Mat image_decode_rec =  image_decode(Rec);

        // Gaussian Blur
        Mat image_gaussian_blur;
        GaussianBlur(image_decode_rec, image_gaussian_blur, Size(5,5), 1.5);

        //Threshhold para apenas detetar preto sem as restantes cores da pista
        Mat img_RGB;
	cvtColor(image_gaussian_blur, img_RGB, COLOR_BGR2RGB);
        inRange(img_RGB,Scalar(0,0,0),Scalar(60,60,60),img_RGB); //black

        Mat red;
	cvtColor(image_gaussian_blur, red, COLOR_BGR2HSV);
        inRange(red, Scalar(160,100,20),Scalar(180,255,255), red); //red

        Mat img_blue_RGB;
        cvtColor(image_gaussian_blur, img_blue_RGB, COLOR_BGR2HSV);
        inRange(img_blue_RGB, Scalar(100,150,0),Scalar(140,255,255), img_blue_RGB); //blue

        //Função Canny para limites de transição de cor preta
        Mat edges;
        Canny(img_RGB, edges, 50, 200);

        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        geometry_msgs::Twist velocity;

        // sistema de linhas para detetar extremidades da linha com 4 pontos no total

        int num = 0;
        unsigned int pontos_linha_130[2];

        //Percorrer matriz edges na linha do pixel 100, guardando os pontos de transiçao de cor

        int linha=130;

        for(int coluna = 20; coluna<390; coluna++)// restringe as margens por uma largura de 20 pixeis de modo a nao detetar linhas paralelas
        {
                if (edges.at<uchar>(linha,coluna) != 0 && num == 0)
                {
                        pontos_linha_130[num] = coluna;
                        num++;
                        stop = 0;
                }

                if (edges.at<uchar>(linha,coluna) != 0 && num == 1 && coluna>(pontos_linha_130[num-1]+20))
                {
                        pontos_linha_130[num] = coluna;
                        num++;
                        stop = 0;
                }

        }

        for(int coluna = 20; coluna<390; coluna++)// restringe as margens por uma largura de 40 pixeis de modo a nao detetar linhas paralelas
        {
                if(red.at<uchar>(linha,coluna) != 0 && canStop==1)
                {
                        stop = 1;
                        break;
                }
                if(img_blue_RGB.at<uchar>(linha,coluna) != 0 && canStop==1)
                { 
                        stop = 2;
                        break;
                }

        }

        float distance = (pontos_linha_130[0] + pontos_linha_130[1]); 
        float media1 = (pontos_linha_130[0] + pontos_linha_130[1])/2;
        float e_x1;

        circle(edges, Point(pontos_linha_130[0],130), 3, (0,255,255), -1);
        circle(edges, Point(pontos_linha_130[1],130), 3, (0,255,255), -1);

        circle(edges, Point(pontos_linha_130[0],130), 3, (0,255,255), -1);

        circle(edges, Point(media1,130), 3, (0,255,255), -1);

        e_x1 = 205 - media1; //pixeis em x1

        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,edges);
        img_bridge.toImageMsg(img_msg);

        pub_image.publish(img_msg);


        // Set Boundaries
        if(pontos_linha_130[0] > 410) {
                pontos_linha_130[0] = 0;
        }
        if (pontos_linha_130[1] > 410) {
                pontos_linha_130[1] = 410;
        }

        if(stop == 1 && canStop==1) 
        {
                ROS_INFO("I see RED\n");
                velocity.linear.x = 10;
                pub_error.publish(velocity);
                canStop = 0;
                color = ros::Time::now().toSec();

        }
        else if (stop == 2 && canStop) 
        {
                color = ros::Time::now().toSec();
                ROS_INFO("I see BLUE\n");      
                velocity.linear.x = 10;
                pub_error.publish(velocity);
                canStop = 0;
                color = ros::Time::now().toSec();
        } 
        else 
        {      
                velocity.linear.x = 0;
                velocity.angular.z = e_x1;
                pub_error.publish(velocity);
                //canStop=1;
        }

        if(time >= (color+1.5)) 
        {
                canStop = 1;
                color = 0;
        }

}


int main(int argc, char **argv)
{

        ros::init(argc, argv, "image_processing");

        ros::NodeHandle n;

        ros::Subscriber sub_Cam = n.subscribe<sensor_msgs::CompressedImage>("/raspicam_node/image/compressed",1,image_callback);
        
        //ros::Subscriber odometrySubscriber = nh.subscribe("/your_robot/odom", 10, odometryCallback);
        
        pub_image = n.advertise<sensor_msgs::Image>("/Publisher_Image",1);
        pub_error = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        
        //pub_error = n.advertise<geometry_msgs::Twist>("/myPose", 1000);
        ros::spin();
        return 0;
}
