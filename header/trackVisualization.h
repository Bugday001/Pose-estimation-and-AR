#pragma once
#include "headfiles.h"

using namespace std;
using namespace cv;

/**@brife ִ�й����б���켣���б���
* @param rotationsVector       ��ת�����б������
* @param translationsVector    ƽ�������б������
* @param rotation       ��ת����
* @param translation    ƽ������
* @param origin         ԭ�������("camera")���Ǳ��("marker")
*/
void saveTrack(vector<vector<Mat>>& rotationsVector, vector<vector<Vec3d>>& translationsVector,
    vector<Mat> rotation, vector<Vec3d> translation, string origin);

/**@brife �����Ժ�����vtk��ֵ���߻���marker 3D�켣
* @param rotationsVector       ��ת�����б�
* @param translationsVector    ƽ�������б�
*/
void drawTracks(vector<vector<Mat>> rotationsVector, vector<vector<Vec3d>> translationsVector);
