//
// Created by 顾仕鑫 on 1/27/18.
//

#include "Txt2pcd.h"

void Txt2pcd::writeToPCD(std::string pcdSavePath, std::unordered_map<std::string,
        std::vector<float>> face_landmarks) {
    std::ofstream outputFile(pcdSavePath);
    int num = face_landmarks.size();
    if (outputFile.is_open()) {
        outputFile << "# .PCD v0.7 - Point Cloud Data file format\n" ;
        outputFile << "VERSION 0.7\n" ;
        outputFile << "FIELDS x y z\n" ;
        outputFile << "SIZE 4 4 4\n";
        outputFile << "TYPE F F F\n";
        outputFile << "COUNT 1 1 1\n";
        outputFile << "WIDTH " << num << "\n";
        outputFile << "HEIGHT 1\n";
        outputFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
        outputFile << "POINTS " << num << "\n";
        outputFile << "DATA ascii\n";
        for(auto it : face_landmarks){
            outputFile << it.second[X] << " " << it.second[Y] << " " << it.second[Z] << "\n";
        }
    }
    //outputFile.close();
}