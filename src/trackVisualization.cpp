#include <iostream>
#include <windows.h>
#include <cstring>
#include <opencv2/viz.hpp>
#include <opencv2/opencv.hpp>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCubeAxesActor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricSpline.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGlyph3DMapper.h>
#include <vtkSphereSource.h>
#include <vtkNamedColors.h>
#include <vtkFollower.h>
#include <vtkVectorText.h>
#include <vtkAxes.h>
#include <vtkCubeSource.h>
#include <vtkArrowSource.h>

#include "vtkAutoInit.h" 

//#define DEBUG

#define POINT_RADIUS 0.00   // 控制点半径，设置成0则不打点
#define LINE_WIDTH 1.0      // 曲线宽度，设置成0则不划线

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

using namespace std;
using namespace cv;

/**@brife 执行过程中保存轨迹到列表中
* @param rotationsVector       旋转矩阵列表的引用
* @param translationsVector    平移向量列表的引用
* @param rotation              旋转矩阵
* @param translation           平移向量
* @param origin                原点是相机("camera")还是标记("marker")
*/
void saveTrack(vector<vector<Mat>> &rotationsVector, vector<vector<Vec3d>> &translationsVector, 
    vector<Mat> rotation, vector<Vec3d> translation, string origin) {

    double x_ratio = 5.0; // x轴缩放率
    double y_ratio = 5.0; // y轴缩放率
    double z_ratio = 2.0; // z轴缩放率

    // 如果以相机为原点
    if (origin == "camera") {
        // 由于给出的参数是相机在标志坐标系当中的变换矩阵，
        // 要求逆得到标志在相机坐标系当中的变换矩阵
        for (int i = 0; i < rotation.size(); i++) {
            invert(rotation[i], rotation[i]); // 求逆
            Mat temp = (-1) * rotation[i] * translation[i]; // 由分块矩阵求逆法则求出
            double* p_temp = (double*)temp.data;
            translation[i][0] = *(p_temp + 0) * x_ratio;
            translation[i][1] = *(p_temp + 1) * y_ratio;
            translation[i][2] = *(p_temp + 2) * z_ratio;
            
        }
    }
    else {
        for (int i = 0; i < translation.size(); i++) {
            translation[i][0] *= x_ratio;
            translation[i][1] *= y_ratio;
            translation[i][2] *= z_ratio;
        }
    }
#ifdef DEBUG
    for (int i = 0; i < translation.size(); i++) {
        cout << translation[i] << endl;
    }
#endif
    rotationsVector.push_back(rotation);
    translationsVector.push_back(translation);
};

/**@brife 结束以后利用vtk插值曲线绘制marker 3D轨迹
* @param rotationsVector       旋转矩阵列表
* @param translationsVector    平移向量列表
*/
void drawTracks(vector<vector<Mat>> rotationsVector, vector<vector<Vec3d>> translationsVector) {

    vtkSmartPointer<vtkNamedColors> colors =
        vtkSmartPointer<vtkNamedColors>::New();

    // 新建window render
    vtkSmartPointer<vtkRenderer> render =
        vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
        vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(render);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    render->SetBackground(colors->GetColor3d("Black").GetData());

    vtkSmartPointer<vtkPoints> vtk_points_list = vtkSmartPointer<vtkPoints>::New();
    // 增加控制点
    double shrink_rate = 0.8;   // 缩小比率，从而让点出现在坐标系内部
#ifdef DEBUG
    cout << "Control points:" << endl;
#endif
    for (int i = 0; i < translationsVector.size(); i++) {

        // 将Vec3d转化为double数组
        double control_point[3];
        for (int m = 0; m < 3; m++) {
            control_point[m] = translationsVector[i][0][m] * shrink_rate;    
        }
#ifdef DEBUG
        cout << "[";
        for (int m = 0; m < 3; m++) {
            cout << control_point[m] << ",";
        }
        cout << "]" << endl;
#endif
        // 将控制点存储到控制点序列当中
        vtk_points_list->InsertNextPoint(control_point);
    }
    vtkSmartPointer<vtkParametricSpline> spline =
        vtkSmartPointer<vtkParametricSpline>::New();
    spline->SetPoints(vtk_points_list);

    vtkSmartPointer<vtkParametricFunctionSource> functionSource =
        vtkSmartPointer<vtkParametricFunctionSource>::New();
    functionSource->SetParametricFunction(spline);
    functionSource->Update();

    vtkSmartPointer<vtkSphereSource> sphere =
        vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetPhiResolution(50);
    sphere->SetThetaResolution(50);
    sphere->SetRadius(POINT_RADIUS);    // 点半径

    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(functionSource->GetOutputPort());

    vtkSmartPointer<vtkActor> trackActor =
        vtkSmartPointer<vtkActor>::New();
    trackActor->SetMapper(mapper);
    trackActor->GetProperty()->SetColor(colors->GetColor3d("Skyblue").GetData());
    trackActor->GetProperty()->SetLineWidth(LINE_WIDTH);    // 线宽

    vtkSmartPointer<vtkPolyData> polyData =
        vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtk_points_list);

    vtkSmartPointer<vtkGlyph3DMapper> pointMapper =
        vtkSmartPointer<vtkGlyph3DMapper>::New();
    pointMapper->SetInputData(polyData);
    pointMapper->SetSourceConnection(sphere->GetOutputPort());

    vtkSmartPointer<vtkActor> pointActor =
        vtkSmartPointer<vtkActor>::New();
    pointActor->SetMapper(pointMapper);
    pointActor->GetProperty()->SetColor(colors->GetColor3d("Lavender").GetData());  // 点颜色

    // 绘制标记位置

    // 绘制原点文字注释
    vtkSmartPointer<vtkVectorText> atext = vtkSmartPointer<vtkVectorText>::New();
    atext->SetText("Marker");   //文字

    vtkSmartPointer<vtkPolyDataMapper> textMapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    textMapper->SetInputConnection(atext->GetOutputPort());

    vtkSmartPointer<vtkFollower> textActor =// vtkVectorActor vtkFollwer;
        vtkSmartPointer<vtkFollower>::New();
    textActor->SetMapper(textMapper);

    textActor->SetScale(0.06, 0.06, 0.06);
    textActor->AddPosition(0, 0.12, 0);
    textActor->GetProperty()->SetColor(colors->GetColor3d("Gold").GetData());  // 文字颜色

    // 绘制marker示意图
    vtkNew<vtkCubeSource> cube;
    cube->SetXLength(0.1);
    cube->SetYLength(0.1);
    cube->SetZLength(0.01);

    vtkNew<vtkPolyDataMapper> cubeMapper;
    cubeMapper->SetInputConnection(cube->GetOutputPort());

    vtkNew<vtkActor> cubeActor;
    cubeActor->SetMapper(cubeMapper);
    cubeActor->AddPosition(0, 0, 0);
    cubeActor->GetProperty()->SetColor(colors->GetColor3d("Gold").GetData());  // 立方体颜色


    // 构建3D坐标轴Acter
    vtkSmartPointer<vtkCubeAxesActor> cubeAxesActor = vtkSmartPointer<vtkCubeAxesActor>::New();
    cubeAxesActor->SetCamera(render->GetActiveCamera());
    double AxisRange = 20000; // 坐标轴范围
    cubeAxesActor->SetXAxisRange(-AxisRange, AxisRange);
    cubeAxesActor->SetYAxisRange(-AxisRange, AxisRange);
    cubeAxesActor->SetZAxisRange(-AxisRange, AxisRange);
    cubeAxesActor->SetScreenSize(6);
    cubeAxesActor->DrawXGridlinesOn();
    cubeAxesActor->DrawYGridlinesOn();
    cubeAxesActor->DrawZGridlinesOn();
    cubeAxesActor->SetGridLineLocation(1);
    //cubeAxesActor->XAxisMinorTickVisibilityOff();
    //cubeAxesActor->YAxisMinorTickVisibilityOff();
    //cubeAxesActor->ZAxisMinorTickVisibilityOff();

    // 向Render里添加Acter
    render->AddActor(cubeAxesActor);
    render->AddViewProp(textActor);
    render->AddActor(cubeActor);
    //render->AddActor(axesActor);
    render->AddActor(trackActor);
    render->AddActor(pointActor);


    render->ResetCamera();
    render->ResetCameraClippingRange();

    renderWindow->SetSize(800, 800);
    renderWindow->Render();
    renderWindowInteractor->Start();
}
