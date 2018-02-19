//
// Created by 顾仕鑫 on 1/27/18.
//

#ifndef LANDMARKS_TRIANGULATION_GREEDY_PROJECTION_H
#define LANDMARKS_TRIANGULATION_GREEDY_PROJECTION_H

#include "general_hd.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

class Greedy_projection {
public:
    pcl::PolygonMesh triangles_first_;
    void triangulation(std::string pcdFilePath);
};


#endif //LANDMARKS_TRIANGULATION_GREEDY_PROJECTION_H
