//
// Created by 顾仕鑫 on 1/28/18.
//

#include "Subdivision.h"

void Subdivision::triangles_subdivision(pcl::PolygonMesh triangles_first, std::string vtkSavePath,
                                        std::unordered_map<std::string, std::vector<float>> face_landmarks) {
    std::vector<pcl::Vertices> triangulation(triangles_first.polygons.begin(),
                                             triangles_first.polygons.end());
    std::unordered_map<int, std::vector<float>> landmarks_sequence;
    int num = face_landmarks.size(), no = 0;
    for(auto face : face_landmarks) {
        landmarks_sequence[no] = face.second;
        no++;
    }
    int thirdSubdivisionStartPoint = no, thirdSubdivisionStartTri = triangulation.size(),
            secondSubdivisionStartPoint = no, secondSubdivisionEndTri = triangulation.size();
    for(int i = 0; i < secondSubdivisionEndTri; ++i){
        int ploy1 = triangulation[i].vertices[0];
        float x1 = landmarks_sequence[ploy1][0];
        float y1 = landmarks_sequence[ploy1][1];
        float z1 = landmarks_sequence[ploy1][2];
        sphericalCoordinate cur1 = transfer2sphericalCoordinate(x1, y1, z1, ploy1);
        int ploy2 = triangulation[i].vertices[1];
        float x2 = landmarks_sequence[ploy2][0];
        float y2 = landmarks_sequence[ploy2][1];
        float z2 = landmarks_sequence[ploy2][2];
        sphericalCoordinate cur2 = transfer2sphericalCoordinate(x2, y2, z2, ploy2);
        int ploy3 = triangulation[i].vertices[2];
        float x3 = landmarks_sequence[ploy3][0];
        float y3 = landmarks_sequence[ploy3][1];
        float z3 = landmarks_sequence[ploy3][2];
        sphericalCoordinate cur3 = transfer2sphericalCoordinate(x3, y3, z3, ploy3);

        calculate4triangles(cur1, cur2, cur3, secondSubdivisionStartPoint, triangulation, landmarks_sequence);
        secondSubdivisionStartPoint += 4;
    }
    // Start thirdSubdivision
    int thirdSubdivisionEndTri = triangulation.size();
    for (int i = thirdSubdivisionStartTri; i < thirdSubdivisionEndTri; ++i){
        int ploy1 = triangulation[i].vertices[0];
        float x1 = landmarks_sequence[ploy1][0];
        float y1 = landmarks_sequence[ploy1][1];
        float z1 = landmarks_sequence[ploy1][2];
        sphericalCoordinate cur1 = transfer2sphericalCoordinate(x1, y1, z1, ploy1);
        int ploy2 = triangulation[i].vertices[1];
        float x2 = landmarks_sequence[ploy2][0];
        float y2 = landmarks_sequence[ploy2][1];
        float z2 = landmarks_sequence[ploy2][2];
        sphericalCoordinate cur2 = transfer2sphericalCoordinate(x2, y2, z2, ploy2);
        int ploy3 = triangulation[i].vertices[2];
        float x3 = landmarks_sequence[ploy3][0];
        float y3 = landmarks_sequence[ploy3][1];
        float z3 = landmarks_sequence[ploy3][2];
        sphericalCoordinate cur3 = transfer2sphericalCoordinate(x3, y3, z3, ploy3);

        calculate4triangles(cur1, cur2, cur3, secondSubdivisionStartPoint, triangulation, landmarks_sequence);
        secondSubdivisionStartPoint += 4;
    }

    // saveToVtk
    int numOfPoints = landmarks_sequence.size();
    saveToVTK(vtkSavePath, numOfPoints, landmarks_sequence, triangulation);

}

sphericalCoordinate Subdivision::transfer2sphericalCoordinate(float x, float y, float z, int id) {
    sphericalCoordinate curCoordinate;
    curCoordinate.r_ = sqrt(x*x + y*y + z*z);
    curCoordinate.theta_ = atan2(z, x);
    curCoordinate.phi_ = atan2(y, sqrt(x*x + z*z));
    curCoordinate.id_ = id;
    return curCoordinate;
}

// call each time, curId += 4
void Subdivision::calculate4triangles(sphericalCoordinate sC1,
                                      sphericalCoordinate sC2,
                                      sphericalCoordinate sC3,
                                      int curId,
                                      std::vector<pcl::Vertices> &triangulation,
                                      std::unordered_map<int, std::vector<float>> &landmarks_sequence) {
    // calculate center point:
    float thetaC = (sC1.theta_ + sC2.theta_ + sC3.theta_)/3;
    float phiC = (sC1.phi_ + sC2.phi_ + sC3.phi_)/3;
    float rC = (sC1.r_ + sC2.r_ + sC3.r_)/3;
    sphericalCoordinate newSC_C;
    newSC_C.r_ = rC;
    newSC_C.phi_ = phiC;
    newSC_C.theta_ = thetaC;
    newSC_C.id_ = curId;

    // calculate 3 middle points:
    float theta1 = (sC1.theta_ + sC2.theta_)/2;
    float phi1 = (sC1.phi_ + sC2.phi_)/2;
    float r1 = (sC1.r_ + sC2.r_)/2;
    sphericalCoordinate newSC1;
    newSC1.r_ = r1;
    newSC1.phi_ = phi1;
    newSC1.theta_ = theta1;
    newSC1.id_ = curId + 1;

    float theta2 = (sC3.theta_ + sC2.theta_)/2;
    float phi2 = (sC3.phi_ + sC2.phi_)/2;
    float r2 = (sC3.r_ + sC2.r_)/2;
    sphericalCoordinate newSC2;
    newSC2.r_ = r2;
    newSC2.phi_ = phi2;
    newSC2.theta_ = theta2;
    newSC2.id_ = curId + 2;

    float theta3 = (sC1.theta_ + sC3.theta_)/2;
    float phi3 = (sC1.phi_ + sC3.phi_)/2;
    float r3 = (sC1.r_ + sC3.r_)/2;
    sphericalCoordinate newSC3;
    newSC3.r_ = r3;
    newSC3.phi_ = phi3;
    newSC3.theta_ = theta3;
    newSC3.id_ = curId + 3;

    landmarks_sequence[newSC_C.id_] = transferFromSpherical(newSC_C);
    landmarks_sequence[newSC1.id_] = transferFromSpherical(newSC1);
    landmarks_sequence[newSC2.id_] = transferFromSpherical(newSC2);
    landmarks_sequence[newSC3.id_] = transferFromSpherical(newSC3);

    pcl::Vertices v1;
    v1.vertices.push_back(newSC_C.id_);
    v1.vertices.push_back(newSC1.id_);
    v1.vertices.push_back(sC1.id_);

    pcl::Vertices v2;
    v2.vertices.push_back(newSC_C.id_);
    v2.vertices.push_back(newSC1.id_);
    v2.vertices.push_back(sC2.id_);

    pcl::Vertices v3;
    v3.vertices.push_back(newSC_C.id_);
    v3.vertices.push_back(newSC2.id_);
    v3.vertices.push_back(sC2.id_);

    pcl::Vertices v4;
    v4.vertices.push_back(newSC_C.id_);
    v4.vertices.push_back(newSC2.id_);
    v4.vertices.push_back(sC3.id_);

    pcl::Vertices v5;
    v5.vertices.push_back(newSC_C.id_);
    v5.vertices.push_back(newSC3.id_);
    v5.vertices.push_back(sC1.id_);

    pcl::Vertices v6;
    v6.vertices.push_back(newSC_C.id_);
    v6.vertices.push_back(newSC3.id_);
    v6.vertices.push_back(sC3.id_);

    triangulation.push_back(v1);
    triangulation.push_back(v2);
    triangulation.push_back(v3);
    triangulation.push_back(v4);
    triangulation.push_back(v5);
    triangulation.push_back(v6);
}

// return x, y, z
std::vector<float> Subdivision::transferFromSpherical(sphericalCoordinate sC) {
    float x = sC.r_ * cos(sC.phi_) * cos(sC.theta_);
    float y = sC.r_ * sin(sC.phi_);
    float z = sC.r_ * cos(sC.phi_) * sin(sC.theta_);
    std::vector<float> point({x, y, z});
    return point;
}

void Subdivision::saveToVTK(std::string vtkSavePath, int numOfPoints,
                            std::unordered_map<int, std::vector<float>> landmarks_sequence,
                            std::vector<pcl::Vertices> triangulation) {
    std::ofstream outputFile(vtkSavePath);
    if (outputFile.is_open()) {
        outputFile << "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS "
                   << numOfPoints << " float" << '\n';
        for(auto landmark: landmarks_sequence) {
            outputFile << landmark.second[0] << " " << landmark.second[1] << " " << landmark.second[2] << "\n";
        }
        outputFile << "\nVERTICES " << numOfPoints << " " << 2*numOfPoints << "\n";
        for (int i = 0; i < numOfPoints; ++i) {
            outputFile << "1 " << i << "\n";
        }

        // compute the correct number of polygons
        int triangle_size = triangulation.size();
        int correct_number = triangle_size;
        for (int i = 0; i < triangle_size; ++i) {
            correct_number += triangulation[i].vertices.size();
        }
        outputFile << "\nPOLYGONS " << triangle_size << " " << correct_number << '\n';

        for (int i = 0; i < triangle_size; ++i) {
            outputFile << triangulation[i].vertices.size() << " ";
            int j = 0;
            for (j = 0; j < triangulation[i].vertices.size() - 1; ++j){
                outputFile << triangulation[i].vertices[j] << " ";
            }
            outputFile << triangulation[i].vertices[j] << "\n";
        }
    }
    outputFile.close();
}