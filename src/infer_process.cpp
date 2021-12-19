// infer_process.cpp
// inference processes
#include "infer_process.h"
using namespace nvinfer1;

#define CHECK(status)                             \
    do                                            \
    {                                             \
        auto ret = (status);                      \
        if (ret != 0)                             \
        {                                         \
            std::cout << "Cuda failure: " << ret; \
            abort();                              \
        }                                         \
    } while (0)

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger
{
public:

    Logger(): Logger(Severity::kWARNING) {}

    Logger(Severity severity): reportableSeverity(severity) {}

    void log(Severity severity, const char* msg) noexcept override
    {
        // suppress messages with severity enum value greater than the reportable
        if (severity > reportableSeverity) return;

        switch (severity)
        {
        case Severity::kINTERNAL_ERROR: std::cerr << "INTERNAL_ERROR: "; break;
        case Severity::kERROR: std::cerr << "ERROR: "; break;
        case Severity::kWARNING: std::cerr << "WARNING: "; break;
        case Severity::kINFO: std::cerr << "INFO: "; break;
        default: std::cerr << "UNKNOWN: "; break;
        }
        std::cerr << msg << std::endl;
    }

    Severity reportableSeverity{Severity::kWARNING};
};

IExecutionContext* context;
static Logger gLogger;
ICudaEngine* engine;
IRuntime* runtime;
// float anchor_info[4];
float prev_box[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

void Mat_to_CHW(vector<float> &data, cv::Mat &frame)
// from https://oldpan.me/archives/tensorrt-code-toturial-1
{
    assert(!data.empty() && !frame.empty());
    unsigned int volChl = INPUT_H * INPUT_W;

    for(int c = 0; c < INPUT_C; ++c)
    {
        for (unsigned j = 0; j < volChl; ++j)
            data[c*volChl + j] = float(frame.data[j * INPUT_C + c]) / 255.0;
    }

    return;
}

void setupEngine(string ENGINE_FILENAME)
{
    // load pre-generated engine
    std::vector<char> trtModelStream_;
    size_t size{ 0 };
    std::ifstream file(ENGINE_FILENAME, std::ios::binary);
    if (file.good())
    {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream_.resize(size);
        file.read(trtModelStream_.data(), size);
        file.close();
    }
    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream_.data(), size, nullptr);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
}

void destroyEngine()
{
    // destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
}

void doInference(vector<float> &input, vector<float> &output, int batchSize)
{
    const ICudaEngine& engine = context->getEngine();
    // input and output buffer pointers that we pass to the engine - the engine requires exactly IEngine::getNbBindings(),
    // of these, but in this case we know that there is exactly one input and one output.
    assert(engine.getNbBindings() == 2);
    void* buffers[2];

    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // note that indices are guaranteed to be less than IEngine::getNbBindings()
    int inputIndex, outputIndex;
    for (int b = 0; b < engine.getNbBindings(); ++b)
    {
        if (engine.bindingIsInput(b))
            inputIndex = b;
        else
            outputIndex = b;
    }

    // create GPU buffers and a stream
    CHECK(cudaMalloc(&buffers[inputIndex], batchSize * INPUT_C * INPUT_H * INPUT_W * sizeof(float)));
    CHECK(cudaMalloc(&buffers[outputIndex], batchSize * OUTPUT_C * OUTPUT_H * OUTPUT_W * sizeof(float)));

    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));

    // DMA the input to the GPU,  execute the batch asynchronously, and DMA it back:
    CHECK(cudaMemcpyAsync(buffers[inputIndex], input.data(), batchSize * INPUT_C * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context->enqueue(batchSize, buffers, stream, nullptr);
    CHECK(cudaMemcpyAsync(output.data(), buffers[outputIndex], batchSize * OUTPUT_C * OUTPUT_H * OUTPUT_W * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    // release the stream and the buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFree(buffers[inputIndex]));
    CHECK(cudaFree(buffers[outputIndex]));
}

// void infer(cv::Mat color_frame, result_3d* result_temp, int batchSize)
// {   
//     cout << "In infer()!" << endl;
//     float color_data_input[INPUT_C * INPUT_H * INPUT_W];
//     Mat_to_CHW(&color_data_input, color_frame);
//     cout << "Mat converted to CHW!" << endl;
//     float result[OUTPUT_C * OUTPUT_H * OUTPUT_W];
//     // float result_3d[OUTPUT_C][OUTPUT_H][OUTPUT_W];
//     doInference(&color_data_input, &result, 1);
//     cout << "Inference Done!\n";

//     for (int i = 0; i < OUTPUT_C; i++) {
//         for (int j = 0; j < OUTPUT_H; j++) {
//             float* row = result + ((i * OUTPUT_H) + j) * OUTPUT_W;
//             for (int k = 0; k < OUTPUT_W; k++) {
//                 (*result_temp).data[i][j][k] = row[k];
//             }
// //            cout << result_3d[i][j][0] << " " << result_3d[i][j][1] << " " << result_3d[i][j][2] << " " << result_3d[i][j][3] << " " << result_3d[i][j][4] << " "
// //                 << result_3d[i][j][5] << " " << result_3d[i][j][6] << " " << result_3d[i][j][7] << " " << result_3d[i][j][8] << " " << result_3d[i][j][9] << endl;
//         }
// //        cout << " " << endl;
//     }
// }

void infer_dummy(cv::Mat& color_frame, result_3d* result_temp, int batchSize)
{   
    vector<float> color_data_input;
    color_data_input.resize(INPUT_C * INPUT_H * INPUT_W);
    Mat_to_CHW(color_data_input, color_frame);

    vector<float> result;
    result.resize(OUTPUT_C * OUTPUT_H * OUTPUT_W);
    doInference(color_data_input, result, 1);

    for (int i = 0; i < OUTPUT_C; i++) {
        for (int j = 0; j < OUTPUT_H; j++) {
            float* row = result.data() + ((i * OUTPUT_H) + j) * OUTPUT_W;
            for (int k = 0; k < OUTPUT_W; k++) {
                (*result_temp).data[i][j][k] = row[k];
            }
        //    cout << (*result_temp).data[i][j][0] << " " << (*result_temp).data[i][j][1] << " " << (*result_temp).data[i][j][2] << " " << (*result_temp).data[i][j][3] << " " << (*result_temp).data[i][j][4] << " "
        //         << (*result_temp).data[i][j][5] << " " << (*result_temp).data[i][j][6] << " " << (*result_temp).data[i][j][7] << " " << (*result_temp).data[i][j][8] << " " << (*result_temp).data[i][j][9] << endl;
        }
//        cout << " " << endl;
    }
    return;
}

void anchor_cell(int cell_indx, int cell_indy, float (*anchor_info)[4]){
    float stride_0 = 640.0 / 10.0;
    float stride_1 = 360.0 / 5.0;
    float c_x = stride_0 / 2.0 + (float)cell_indx * stride_0;
    float c_y = stride_1 / 2.0 + (float)cell_indy * stride_1;
    float w = anchor_size[1];
    float h = anchor_size[0];
    
    (*anchor_info)[0] = c_x - w/2;
    (*anchor_info)[1] = c_y - h/2;
    (*anchor_info)[2] = c_x + w/2;
    (*anchor_info)[3] = c_y + h/2;
}

bool frame_filter(vector<float> select_box) 
{
    if (*max_element(prev_box, prev_box+5) == 0) {
        return true;
    } else {
        float diff_x = fabs((select_box[1] + select_box[3]) / 2.0 - (prev_box[1] + prev_box[3]) / 2.0);
        float diff_y = fabs((select_box[2] + select_box[4]) / 2.0 - (prev_box[2] + prev_box[4]) / 2.0);
        // cout << diff_x << " " << diff_y << endl;
        if (diff_x > 50 || diff_y > 50) {
            // cout << "miss" << endl;
            return false;
        } else {
            return true;
        }
    }
}

void result_to_box(result_3d* output, float threshold, vector<vector<float>> &boxes_probs)
{
    float final_dim[2] = {5, 10};
    for (int ind_row = 0; ind_row < final_dim[0]; ind_row++) {
        for (int ind_col = 0; ind_col < final_dim[1]; ind_col++) {
            
            float anchor_info[4];
            anchor_cell(ind_col, ind_row, &anchor_info);
            // cout << anchor_info[0] << endl;
            if (output->data[0][ind_row][ind_col] >= threshold) {
                float c_x = anchor_info[0] + (float)anchor_size[1]/2.0 + output->data[1][ind_row][ind_col];
                float c_y = anchor_info[1] + (float)anchor_size[0]/2.0 + output->data[2][ind_row][ind_col];
                float w = 1/(1+exp(-output->data[3][ind_row][ind_col])) * anchor_size[1] * encode_factor;
                float h = 1/(1+exp(-output->data[4][ind_row][ind_col])) * anchor_size[0] * encode_factor;
                float x1 = c_x - w/2.0;
                float y1 = c_y - h/2.0;
                float x2 = c_x + w/2.0;
                float y2 = c_y + h/2.0;
//                cout << output[0][ind_row][ind_col] << endl;
//                cout << (int)c_x << "  " << (int)c_y << "  " << (int)w << "  " << (int)h << "  " << endl;
                x1 = (x1 > 0) ? x1 : 0;
                x1 = (x1 < 640) ? x1 : 640;
                x2 = (x2 > 0) ? x2 : 0;
                x2 = (x2 < 640) ? x2 : 640;
                y1 = (y1 > 0) ? y1 : 0;
                y1 = (y1 < 360) ? y1 : 360;
                y2 = (y2 > 0) ? y2 : 0;
                y2 = (y2 < 360) ? y2 : 360;
                if (x1 >= 0 && x1 <= 640 &&
                    x2 >= 0 && x2 <= 640 && 
                    y1 >= 0 && y1 <= 360 &&
                    y2 >= 0 && y2 <= 360){
                        vector<float> box_cell;
                        float box[5] = {output->data[0][ind_row][ind_col], x1, y1, x2, y2};
                        box_cell.assign(box, box+5);
                        boxes_probs.push_back(box_cell);
                    }
//                cout << (int)x1 << "  " << (int)y1 << "  " << (int)x2 << "  " << (int)y2 << "  " << endl;
            }
        }
    }
}

float IoU(vector<float> a, vector<float> b)
{
    float inter_w = max((float)0, (min(a[2], b[2]) - max(a[0], b[0])));
    float inter_h = max((float)0, (min(a[3], b[3]) - max(a[1], b[1])));
    float inter_ab = inter_w * inter_h;
    float area_a = (a[3] - a[1]) * (a[2] - a[0]);
    float area_b = (b[3] - b[1]) * (b[2] - b[0]);
    float union_ab = area_a + area_b - inter_ab;
    float iou = (union_ab != 0) ? inter_ab / union_ab : (float)0;
    return iou;
}

int voting_suppression(vector<vector<float>>result_box, float voting_threshold)
{
    int result_box_length = (int)result_box.size();
    int votes[result_box_length];
    for (int ind = 0; ind < result_box_length; ind++) {
        votes[ind] = 0;
        for (int ind2 = 0; ind2 < result_box_length; ind2++) {
            vector<float> box1 = result_box[ind];
            box1.erase(box1.begin());
            vector<float> box2 = result_box[ind2];
            box2.erase(box2.begin());
            if (IoU(box1, box2) >= voting_threshold){
                votes[ind] += 1;
            }
        }
    }
    return max_element(votes, votes+result_box_length) - votes;
}

void post_process(result_3d* result_temp, vector<float> &select_box)
{
    // vector<float> select_box = {0.0, 0.0, 0.0, 0.0, 0.0};
    // run threholding and voting suppresion
    vector<vector<float>> boxes_probs;
    result_to_box(result_temp, confi_threshold, boxes_probs);
    if ((int)boxes_probs.size() != 0) {
        int select_ind = voting_suppression(boxes_probs, voting_iou_threshold);
        for (int ind = 1; ind < 5; ind++){
            select_box[ind] = (select_box[ind] + prev_box[ind]) / 2.0;
        }
        // for (int ind = 1; ind < 5; ind++){
        //         // select_box[ind] = boxes_probs[select_ind][ind];
        //         cout << boxes_probs[select_ind][ind] << endl;
        //     }
        if (frame_filter(select_box)){
            select_box = boxes_probs[select_ind];
            select_box[0] = 1.0;
        }
        for (int ind = 1; ind < 5; ind++){
            prev_box[ind] = select_box[ind];
        }
    }
}