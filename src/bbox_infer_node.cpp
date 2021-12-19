// bbox_infer_node.cpp

#include "bbox_infer_node.h"
#include "infer_process.h"
extern void setupEngine(string ENGINE_FILENAME);
extern void infer(cv::Mat& color_frame, result_3d* result_temp, int batchSize);
extern void Mat_to_CHW(vector<float> &data, cv::Mat& frame);
extern void destroyEngine();
extern void post_process(result_3d* result_temp, vector<float> &select_box);

static const std::string OPENCV_WINDOW = "Image window";

BBoxInfer::BBoxInfer(const ros::NodeHandle& nh) : nh_(nh), it_(nh)
{
    setupEngine(ENGINE_FILENAME);
    cv::namedWindow(OPENCV_WINDOW);
    // color_sub = it_.subscribe("/usb_cam/image_raw", 1, &BBoxInfer::colorCallback, this);
    color_sub = it_.subscribe("/zed2/zed_node/rgb/image_rect_color", 
        1, &BBoxInfer::colorCallback, this, image_transport::TransportHints("compressed"));
    box_center_pub = nh_.advertise<geometry_msgs::Point32>("box_center", 1000);
}

BBoxInfer::~BBoxInfer()
{
    cv::destroyWindow(OPENCV_WINDOW);
    destroyEngine();
}

void BBoxInfer::colorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat color_frame = cv_ptr->image;
    cv::resize(color_frame, color_frame, cv::Size(640, 360));
    cv::Mat label_color_frame = color_frame.clone();
    cv::resize(label_color_frame, label_color_frame, cv::Size(640, 360));
    int rows = label_color_frame.rows;
    int cols = label_color_frame.cols;

    // run inference
    result_3d result_temp;
    infer(color_frame, &result_temp, 1);
    
    // run threholding and voting suppresion
    vector<float> select_box = {0.0, 0.0, 0.0, 0.0, 0.0};
    post_process(&result_temp, select_box);
    if (select_box[0] == 1.0) {
        cv::rectangle(label_color_frame, cv::Point((int)select_box[1], (int)select_box[2]), cv::Point((int)select_box[3], (int)select_box[4]), cv::Scalar(0, 255, 0));
        geometry_msgs::Point32 box_center;
        box_center.x = (select_box[1] + select_box[3]) / 2 / INPUT_W;
        box_center.y = (select_box[2] + select_box[4]) / 2 / INPUT_H;
        box_center_pub.publish(box_center);
    }

    cout << select_box[0] << endl;
    cout << cv::Point((int)select_box[1], (int)select_box[2]) << endl;
    cout << cv::Point((int)select_box[3], (int)select_box[4]) << endl;
    cv::cvtColor(label_color_frame, label_color_frame, CV_RGB2BGR);
    cv::imshow(OPENCV_WINDOW, label_color_frame);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "bbox_infer");
    ros::NodeHandle nh;
    BBoxInfer yolo(nh);
    ros::spin();    
}
