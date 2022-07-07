#pragma once
#include "headfiles.h"

using namespace std;
using namespace cv;

/**@brife 执行过程中保存轨迹到列表中
* @param rotationsVector       旋转矩阵列表的引用
* @param translationsVector    平移向量列表的引用
* @param rotation       旋转矩阵
* @param translation    平移向量
* @param origin         原点是相机("camera")还是标记("marker")
*/
void saveTrack(vector<vector<Mat>>& rotationsVector, vector<vector<Vec3d>>& translationsVector,
    vector<Mat> rotation, vector<Vec3d> translation, string origin);

/**@brife 结束以后利用vtk插值曲线绘制marker 3D轨迹
* @param rotationsVector       旋转矩阵列表
* @param translationsVector    平移向量列表
*/
void drawTracks(vector<vector<Mat>> rotationsVector, vector<vector<Vec3d>> translationsVector);
