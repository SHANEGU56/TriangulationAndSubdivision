//
// Created by 顾仕鑫 on 1/26/18.
//

#ifndef LANDMARKS_TRIANGULATION_POINT_COLLECTION_H
#define LANDMARKS_TRIANGULATION_POINT_COLLECTION_H

#include "general_hd.h"

class Point_collection {
public:
    std::unordered_map<std::string, std::vector<float>> face_landmarks;
    void readFromFile(std::string filePath);
};


#endif //LANDMARKS_TRIANGULATION_POINT_COLLECTION_H
