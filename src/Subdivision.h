//
// Created by 顾仕鑫 on 1/28/18.
//

#ifndef LANDMARKS_TRIANGULATION_SUBDIVISION_H
#define LANDMARKS_TRIANGULATION_SUBDIVISION_H

#include <cmath>
#include "general_hd.h"
#include "greedy_projection.h"


struct sphericalCoordinate{
    int id_;
    float theta_;
    float phi_;
    float r_;

};

class Subdivision {
public:
    void triangles_subdivision(pcl::PolygonMesh triangles_first, std::string vtkSavePath,
                               std::unordered_map<std::string, std::vector<float>> face_landmarks);
private:
    sphericalCoordinate transfer2sphericalCoordinate(float x, float y, float z, int id);
    void calculate4triangles(sphericalCoordinate sC1,
                             sphericalCoordinate sC2,
                             sphericalCoordinate sC3,
                             int curId,
                             std::vector<pcl::Vertices> &triangulation,
                             std::unordered_map<int, std::vector<float>> &landmarks_sequence);
    std::vector<float> transferFromSpherical(sphericalCoordinate sC);
    void saveToVTK(std::string vtkSavePath, int numOfPoints,
                   std::unordered_map<int, std::vector<float>> landmarks_sequence,
                   std::vector<pcl::Vertices> triangulation);
};


#endif //LANDMARKS_TRIANGULATION_SUBDIVISION_H
