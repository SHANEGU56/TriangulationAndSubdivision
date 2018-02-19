//
// Created by 顾仕鑫 on 1/27/18.
//

#ifndef LANDMARKS_TRIANGULATION_TXT2PCD_H
#define LANDMARKS_TRIANGULATION_TXT2PCD_H

#include "general_hd.h"

const int X = 0, Y = 1, Z = 2;

class Txt2pcd {
public:
    void writeToPCD(std::string pcdSavePath, std::unordered_map<std::string,
            std::vector<float>> face_landmarks);
};


#endif //LANDMARKS_TRIANGULATION_TXT2PCD_H
