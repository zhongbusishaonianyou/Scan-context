### usage

#### 1. Requirements

1. cmake
2. PCL
3. python
4. Eigen3

#### 2. Run
```
cd ~/catkin_ws/src
git clone https://github.com/zhongbusishaonianyou/Scan-context.git
cd ..
catkin_make 
source devel/setup.bash
rosrun scan_context scan_context 
```
#### 3.visualize pr curve

1. please open draw.py file and revise  sequence 

2. python draw.py

3. you will see  following images:

   |                                                    KITTI 00  |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![Figure_7](https://github.com/user-attachments/assets/5fdb3525-878b-4e82-bdda-61850cc30eff) |![Figure_8](https://github.com/user-attachments/assets/f7295f69-de7c-4416-be48-60c8a3890901)|

   |                                                    KITTI 02  |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![Figure_5](https://github.com/user-attachments/assets/fd18bbb7-08ce-49ab-8c9b-26ac31eefcff) | ![Figure_6](https://github.com/user-attachments/assets/9978df92-eaa4-44be-90ad-127685333aa0)|

   |                                                    KITTI 05  |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![Figure_1](https://github.com/user-attachments/assets/d16ac402-90b3-47c6-a6ac-c1ccd9144050) | ![Figure_2](https://github.com/user-attachments/assets/19c19818-d2f6-48f8-9da8-c78f3c828638)|

   |                                                     KITTI 08 |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   |  ![Figure_3](https://github.com/user-attachments/assets/b86b1178-8a88-444b-bbf5-cb1ca4c823f7)| ![Figure_4](https://github.com/user-attachments/assets/316643e4-8ac2-4e79-9b88-551de6d80db0)| 

   

#### 4. cite

```
@INPROCEEDINGS { gkim-2018-iros,
  author = {Kim, Giseop and Kim, Ayoung},
  title = { Scan Context: Egocentric Spatial Descriptor for Place Recognition within {3D} Point Cloud Map },
  booktitle = { Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems },
  year = { 2018 },
  month = { Oct. },
  address = { Madrid }
}

```
