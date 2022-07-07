#pragma once
#include "headfiles.h"
#include <opencv2/aruco.hpp>
#include "PoseEstimation.h"
#include "visualizexd.h"
#include "ReadModel.h"
#include "trackVisualization.h"
#include "detectMarkers.h"
#include "CalibrationChessboard.h"

int detectPoseShow(ReadModelFile Model);