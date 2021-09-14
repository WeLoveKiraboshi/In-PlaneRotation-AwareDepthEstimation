# In-Plane Rotation-Aware Monocular Depth Estimation using SLAM
**Authors:** [Yuki Saito], [Ryo Hachiuma], [Hideo Saito]  


This is a repository of Paper: "In-Plane Rotation-Aware Monocular Depth Estimation using SLAM", "Training-free Approach to Improve the Accuracy of Monocular Depth Estimation with In-Plane Rotation"".

This script is composed of ORB-SLAM2 part, DepthEstimation part, and DenseReconstruction part.


<a href="http://www.hvrl.ics.keio.ac.jp/saito_y/site/" target="_blank"><img src="http://hvrl.ics.keio.ac.jp/saito_y/site/FCV2020.png" 
alt="ORB-SLAM2" width="916" height="197" border="30" /></a>



### Related Publications:

[Our Paper1] Yuki Saito, Ryo Hachiuma, and Hideo Saito. **In-Plane Rotation-Aware Monocular Depth Estimation using SLAM**. *International Workshop on Frontiers of Computer Vision(IW-FCV 2020),* pp. 305-317, 2020. **[PDF](https://link.springer.com/chapter/10.1007%2F978-981-15-4818-5_23)**.

[Our Paper2] Yuki Saito, Ryo Hachiuma, Masahiro Yamaguchi, and Hideo Saito,  **Training-free Approach to Improve the Accuracy of Monocular Depth Estimation with In-Plane Rotation**. *IEICE Transactions on Information and Systems*, 2021. **[pdf](https://search.ieice.org/bin/index.php?category=D&lang=J&curr=1)**


[ORB-SLAM2] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[Dense Depth] Ibraheem Alhashim and Peter Wonka. **High Quality Monocular Depth Estimation via Transfer Learning**. *ArXiv pe-prints abs/1812.11941,* 2018. **[PDF](https://arxiv.org/pdf/1812.11941.pdf)**

[CNN-MonoFusion] Wang, Jiafang and Liu, Haiwei and Cong, Lin and Xiahou, Zuoxin and Wang, Liming. **CNN-MonoFusion: Online Monocular Dense Reconstruction Using Learned Depth from Single View**, ISMAR, pp.57--62, 2018, **[pdf](https://ieeexplore.ieee.org/document/8699273)**
# 1. License

If you use our scripts  in an academic work, please cite:

    @article{In-PlaneRotationAwareMonoDepth2020,
      title={In-Plane Rotation-Aware Monocular Depth Estimation Using SLAM},
      author={Yuki Saito, Ryo Hachiuna, and Hideo Saito},
      journal={International Conference on Frontiers of Computer Vision},
      pages={305--317},
      publisher={Springer Singapore},
      year={2020}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

if you use DenseReconstruction Module in an academic work, please cite:

    @INPROCEEDINGS{WhangCNNMonoFusion,
    author={Wang, Jiafang and Liu, Haiwei and Cong, Lin and Xiahou, Zuoxin and Wang, Liming},
    booktitle={2018 IEEE International Symposium on Mixed and Augmented Reality Adjunct (ISMAR-Adjunct)},
    title={CNN-MonoFusion: Online Monocular Dense Reconstruction Using Learned Depth from Single View},
    year={2018},
    volume={},
    number={},
    pages={57--62},
    doi={10.1109/ISMAR-Adjunct.2018.00034}
    }



# 2. Prerequisites
We have tested the library in **Ubuntu **18.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

For other related libaries, please search in official page of ORB-SLAM2 **[Git](https://github.com/raulmur/ORB_SLAM2)**


# 3. How to Run

## Monocular Depth Estimation Module

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `XXX.yaml` to TUMX.yaml, or OurDataset.yaml for each sequence respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/RGB-D/XXX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
```


## Dense Reconstruction Module

1. Execute the following command. Change `XXXXX.yaml` to TUM1.yaml, or OurDataset.yaml for each sequence respectively. ALl yaml files are summarized in Examples/rgbd_monodepth folder. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/rgbd_monodepth/rgbd_monodepth Vocabulary/ORBvoc.txt Examples/Monocular/XXX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
```
