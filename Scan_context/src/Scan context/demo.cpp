#include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include "Scancontext.h"
#include <pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree_flann.h>

#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>


using namespace std;
typedef pcl::PointXYZ  PointType;
float filter_size = 0.5;
const float revisit_thres=4;

/*Please replace the dataset folder path with the path in your computer. KITTI's 00, 02, 05, 06, 07, 08 have loops*/
const string seq = "00";
const string RESULT_PATH = "/home/zgy/ROS/Scan_context/sc_loop_"+seq +".txt";
const string dataset_folder = "/media/zgy/zgy/loop_closure/"+seq +"/velodyne/";
const string Gtpose_path="/home/zgy/ROS/Scan_context/poses/"+seq +".txt";
const string GTloop_save_path="/home/zgy/ROS/Scan_context/gt_loop/gt_loop_"+seq +".txt";

vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file;
    lidar_data_file.open(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    if(!lidar_data_file)
    {
        cout << "Read End..." << endl;
        exit(-1);
    }

    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr read_poses_data(const string& pose_path)
{
    std::ifstream pose_ifs(pose_path);
    std::string line;
    pcl::PointCloud<pcl::PointXYZI>::Ptr GTposes(new pcl::PointCloud<pcl::PointXYZI>);

    int index = 1;
    while(getline(pose_ifs, line)) 
    {   
        
        if(line.empty()) break;
        stringstream ss(line);
        float token;
        vector<float> tokens;
       while(ss>>token)
        {
            tokens.push_back(token);
       }
        pcl::PointXYZI curr_poses;
        curr_poses.x = tokens[3];
        curr_poses.y = 0;
        curr_poses.z = tokens[11];
        curr_poses.intensity = index++;
       
        GTposes->push_back(curr_poses);
        
    }
    return GTposes;
}
std::vector<vector<int>> calculate_GT_loop(pcl::PointCloud<pcl::PointXYZI>::Ptr & GTposes)
{
    int num_series=GTposes->points.size();
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(GTposes);
    std::vector<vector<int>> GT_loop(num_series+1);
    for(int i = 0; i < num_series; i++)
    {
        
        std::vector<int> pt_index;
        std::vector<float> pt_dis;
        std::vector<int> loop_frame_id;

        int query_frame_id = GTposes->points[i].intensity;
        kdtree.radiusSearch(GTposes->points[i],revisit_thres,pt_index,pt_dis);

        for(int j = 0; j < (int)pt_index.size(); j++)
        {  
          int history_frame=GTposes->points[pt_index[j]].intensity;
           
            if(history_frame == query_frame_id) continue;
               loop_frame_id.push_back(history_frame);
        }
        sort(loop_frame_id.begin(), loop_frame_id.end());
        GT_loop[query_frame_id] = loop_frame_id;
        
    }
    
    std::ofstream save_GTloop(GTloop_save_path);

    for(int i =1; i < (int)GT_loop.size(); i++)
    {
        save_GTloop << i << " ";
        for(int j = 0; j < (int)GT_loop[i].size(); j++)
        {
            save_GTloop << GT_loop[i][j] << " ";
        }
        
        save_GTloop << endl;
    }
    
    return GT_loop;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"Scan_context");
    ros::NodeHandle nh;  
    
    SCManager  SC;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(filter_size, filter_size, filter_size);

    pcl::PointCloud<PointType>::Ptr current_cloudDS(new pcl::PointCloud<pcl::PointXYZ>()); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr GT_poses=read_poses_data(Gtpose_path);
    auto gt = calculate_GT_loop(GT_poses); 
   
    vector<int>::iterator it;
    size_t cloudInd = 1;
    ros::Rate LiDAR_rate(100); //LiDAR frequency 10Hz
    while (ros::ok())
   {    
        
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << std::setfill('0') << std::setw(5) << cloudInd << ".bin";
        cout<<lidar_data_path.str()<<endl;
        vector<float> lidar_data = read_lidar_data(lidar_data_path.str());

        std::ofstream foutC(RESULT_PATH, std::ios::app);
        foutC.setf(std::ios::fixed, std::ios::floatfield);
        foutC.precision(5);

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(std::size_t i = 0; i < lidar_data.size(); i += 4)
        {            
            pcl::PointXYZ point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
    
            current_cloud->push_back(point);
        }
           
          current_cloudDS->clear();
          downSizeFilter.setInputCloud(current_cloud);
          downSizeFilter.filter(*current_cloudDS);
        
          SC.makeAndSaveScancontextAndKeys(*current_cloudDS);
           
          if (cloudInd>300)
        { 
    
         auto detectResult= SC.detectLoopClosureID() ; 
         
        it=std::find(gt[cloudInd].begin(),gt[cloudInd].end(),detectResult.first);
        if(it!=gt[cloudInd].end())
        {
         
         foutC << cloudInd << " " << detectResult.first << " "<< detectResult.second<< " "<< 1 <<endl;
     
        }
        else 
        {
            foutC << cloudInd << " " << detectResult.first << " "<< detectResult.second<< " "<< 0 <<endl;
        }

         foutC.close();
         } 
        
         cloudInd ++;
         
        ros::spinOnce();
        LiDAR_rate.sleep();
    }

    return 0;
}

