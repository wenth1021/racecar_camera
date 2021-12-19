// bbox_infer_node.h

#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Point32.h>

class BBoxInfer
{
    public:
        BBoxInfer(const ros::NodeHandle& nh);
        ~BBoxInfer();

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber color_sub;
        ros::Publisher box_center_pub;
        void colorCallback(const sensor_msgs::ImageConstPtr& msg);
};
