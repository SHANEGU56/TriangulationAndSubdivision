//
// Created by 顾仕鑫 on 1/26/18.
//

#include "point_collection.h"

void Point_collection::readFromFile(std::string filePath) {
    std::ifstream infile(filePath);
    std::string line;
    if(infile.is_open()) {
        while(std::getline(infile, line)) {
            std::istringstream iss(line);
            std::string name;
            float x, y, z;
            iss >> name >> x >> y >> z;
            std::vector<float> cur({x, y ,z});
            face_landmarks[name] =  cur;
        }
    }
    infile.close();
}