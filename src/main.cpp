#include <iostream>
#include "point_collection.h"
#include "greedy_projection.h"
#include "Txt2pcd.h"
#include "Subdivision.h"

using namespace std;

int main() {
    Point_collection pc;
    Greedy_projection gp;
    Txt2pcd t2p;
    Subdivision sd;
    pc.readFromFile("/Users/gushixin/Desktop/Model.txt");
    cout << "Read Successfully" << endl;
    t2p.writeToPCD("/Users/gushixin/Desktop/Model.pcd", pc.face_landmarks);

    gp.triangulation("/Users/gushixin/Desktop/Model.pcd");
    sd.triangles_subdivision(gp.triangles_first_, "/Users/gushixin/Desktop/Model.vtk", pc.face_landmarks);
    return 0;
}