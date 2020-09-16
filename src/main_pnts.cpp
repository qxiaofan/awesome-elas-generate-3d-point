#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/common/common.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/visualization/pcl_visualizer.h>

#include "StereoEfficientLargeScale.h"
#include<opencv2/opencv.hpp>

const double camera_scale = 1000;

void disp2pcd(cv::Mat disp,std::string cloud_save_path);


int main(int argc, char **argv)
{
        std::string imageName_L = argv[1];
        std::string imageName_R = argv[2];
        std::cout<<"imageName_L == "<<imageName_L<<std::endl;
        std::cout<<"imageName_R == "<<imageName_R<<std::endl;
        cv::Mat leftim = cv::imread(imageName_L);
        cv::Mat rightim = cv::imread(imageName_R);

        if(leftim.data == nullptr || rightim.data == nullptr)
        {
            std::cerr<<"image input is empty,please check!!!";
            return 0;
        }

        cv::namedWindow("left_image",1);
        cv::namedWindow("right_image",1);
        cv::imshow("left_image",leftim);
        cv::imshow("right_image",rightim);
        cv::waitKey(100);

        Mat dest;
        int minDisparity = atoi(argv[3]);
        int maxDisparity = atoi(argv[4]);
        std::cout<<"minDisparity == "<<minDisparity<<std::endl;
        std::cout<<"maxDisparity == "<<maxDisparity<<std::endl;
        StereoEfficientLargeScale elas(minDisparity,maxDisparity);

        //elas(leftim,rightim,dest,100);
        elas(leftim,rightim,dest,100);

        Mat disp;
        dest.convertTo(disp,CV_8U,1.0/8);

        cv::Mat disp_color;//伪彩色
        applyColorMap(disp,disp_color,cv::COLORMAP_JET);

        cv::namedWindow("disp",1);
        imshow("disp",disp);

        cv::namedWindow("disp_color",1);
        cv::imshow("disp_color",disp_color);

        std::string disp_map_path = argv[1]; 	disp_map_path += ".d.bmp";
            std::string disp_color_map_path = argv[1]; disp_color_map_path += ".c.bmp";
        //cv::imwrite(disp_map_path,disp);
        //cv::imwrite(disp_color_map_path,disp_color);
        waitKey(300);

        //将视差图结合基线与相机的标定参数，转换为点云数据保存到本地
        std::string cloud_save_path = argv[1];
        cloud_save_path +=".p.pcd";
        disp2pcd(disp,cloud_save_path);
    

	return 0;
}


//left:
//width: 640, height: 400
//distortion_model: KANNALA_BRANDT
//D: -0.14889299848373000,-0.04364167748312290,0.03435832570726938,-0.01064062776854229,0.00000000000000000,
//K: 443.79692612349492720,0.00000000000000000,293.49938427510551264,0.00000000000000000,443.41192442172251731,202.02520621228134701,0.00000000000000000,0.00000000000000000,1.00000000000000000,
//R: 0.99995247102313090,-0.00212421717650776,-0.00951542937136942,0.00214685787750808,0.99999488754678378,0.00236978512716580,0.00951034678591017,-0.00239010076820710,0.99995191920528326,
//P: 442.05185684172238325,0.00000000000000000,158.16831588745117188,0.00000000000000000,0.00000000000000000,442.05185684172238325,216.55125045776367188,0.00000000000000000,0.00000000000000000,0.00000000000000000,1.00000000000000000,0.00000000000000000,

//right:
//width: 640, height: 400
//distortion_model: KANNALA_BRANDT
//D: -0.14697665095661802,-0.07324936449923376,0.10467077488904218,-0.05803606929814879,0.00000000000000000,
//K: 440.90950841155984108,0.00000000000000000,324.59837182329016514,0.00000000000000000,440.69178926172230604,204.82011592645241649,0.00000000000000000,0.00000000000000000,1.00000000000000000,
//R: 0.99986835144176900,-0.00297108675530597,-0.01595156508344964,0.00293362984971600,0.99999288594079405,-0.00237105118181846,0.01595849620183366,0.00232394304887038,0.99986995438791015,
//P: 442.05185684172238325,0.00000000000000000,158.16831588745117188,-35322.48634352167573525,0.00000000000000000,442.05185684172238325,216.55125045776367188,0.00000000000000000,0.00000000000000000,0.00000000000000000,1.00000000000000000,0.00000000000000000,

void disp2pcd(cv::Mat disp,std::string cloud_save_path)
{
   if(disp.data == nullptr)
   {
     std::cout<<"disp is empty! Please check!"<<std::endl;
     return;
   }
   cv::Size s;
   s.height = disp.rows;
   s.width = disp.cols;
   
   #if 0
   //left camera paras
   double fx_1 = 443.79692612349492720;
   double fy_1 = 443.41192442172251731;
   double u_1 = 293.49938427510551264;
   double v_1 = 202.02520621228134701;
   
   //right camera paras
   double fx_2 = 440.90950841155984108;
   double fy_2 = 440.69178926172230604;
   double u_2 = 324.59837182329016514;
   double v_2 = 204.82011592645241649;
   #endif
   
   #if 1
   //left camera paras
   double fx_1 = 442.05185684172238325;
   double fy_1 = 442.05185684172238325;
   double u_1 = 158.16831588745117188;
   double v_1 = 216.55125045776367188;
   
   //right camera paras
   double fx_2 =  442.05185684172238325;
   double fy_2 = 442.05185684172238325;
   double u_2 = 158.16831588745117188;
   double v_2 = 216.55125045776367188;
   #endif
   
   double base = 80.0/camera_scale;

   cv::Mat K1 = (cv::Mat_<double>(3,3)<< fx_1, 0, u_1, 0, fy_1, v_1, 0, 0, 1);
   cv::Mat K2 = (cv::Mat_<double>(3,3)<< fx_2, 0, u_2, 0, fy_2, v_2, 0, 0, 1);
   cv::Mat distort1 = (cv::Mat_<double>(5,1)<<0,0,0,0,0);
 
  cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat t = (cv::Mat_<double>(3, 1) << -base, 0, 0);
      
  cv::Mat R1 = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat R2 = cv::Mat::eye(3, 3, CV_64FC1); 
  cv::Mat P1 = cv::Mat::eye(3, 4, CV_64FC1); 
  cv::Mat P2 = cv::Mat::eye(3, 4, CV_64FC1); 
  cv::Mat Q_ = cv::Mat::eye(4, 4, CV_64FC1);
  //cv::stereoRectify(K1, distort1, K2, distort1, s, R, t, R1, R2, P1, P2, Q_,CV_CALIB_ZERO_DISPARITY);

  Q_ = (cv::Mat_<double>(4,4)<<1,0,0, -u_1,
                                       0,1,0, -v_1,
                                       0,0,0, fx_1,
                                       0,0, 1.0/base,0);

  cv::Mat image3D(s,CV_32FC3);
  cv::reprojectImageTo3D(disp,image3D,Q_,false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ p;
  cloud->width = (s.height)*(s.width);
  cloud->height = 1;
  cloud->points.reserve(cloud->width*cloud->height);
  //form 40 to width - 40: to remove outlire on the border
  std::cout<<" s.height == "<<s.height<<std::endl;
  std::cout<<" s.width == "<<s.width<<std::endl;

  for(int v = 40 ; v < s.height - 40; v++)
  {
    for(int u = 40; u < s.width - 40; u++)
    {
       cv::Vec3f pv = image3D.ptr<cv::Vec3f>(v)[u];
       p.x = pv[0];
       p.y = pv[1];
       p.z = pv[2];
       if(p.z < 2)
       {
         cloud->points.push_back(p);
       }
    }
  }
  cloud->width = cloud->points.size();

  std::cout<<"origin cloud points == "<<cloud->points.size()<<std::endl;

  pcl::io::savePCDFileASCII(cloud_save_path,*cloud);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green

  viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");

  while(!viewer->wasStopped())
  {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100));
  }
}


