// infer_process.h

// for Jetson
#include <cuda_runtime_api.h>
#include "NvInfer.h"

#include <assert.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
const int INPUT_H = 360;
const int INPUT_W = 640;
const int INPUT_C = 3;
const int OUTPUT_H = 5;
const int OUTPUT_W = 10;
const int OUTPUT_C = 5;

const float anchor_size[2] = {360.0/5.0, 640.0/7.0};
// change the ENGINE_FILENAME if model is stored somewhere else
const string ENGINE_FILENAME = "/home/nvidia/f1tenth_ws/src/racecar_camera/src/model_14_1.trt";
const float encode_factor = 2.0;
const float confi_threshold = 0.6;
const float voting_iou_threshold = 0.7;

struct result_3d {
    float data[OUTPUT_C][OUTPUT_H][OUTPUT_W] = {0.0};
};