#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

extern "C" {
#include "devmem/devmem.h"
#include "fpgaint_poll/fpgaint_poll.h"
}

#define IMG_ADDR 0x3FF00000
#define IMG_WIDTH 752
#define IMG_HEIGHT 480
#define IMG_SPAN (IMG_WIDTH*IMG_HEIGHT)


class CameraNode {
public:
    CameraNode(ros::NodeHandle& nh) : node(nh) {
        ROS_INFO("[CameraNode]: Initializing camera node...");

        fpgaint_poll = FPGAINT_POLL_NEW(0);
        if (!fpgaint_poll_init(&fpgaint_poll)) {
            ROS_ERROR("[CameraNode]: File to initialize FPGA interrupt poll!");
            return;
        }

        if ((ram_image_fd = devmem_map(IMG_ADDR, IMG_SPAN, (void**) &ram_image)) <= 0) {
            ROS_ERROR("[CameraNode]: Mapping of 0x%08X failed with code %d\r\n!", IMG_ADDR, ram_image_fd);
            return;
        }

        image_transport::ImageTransport it(nh);
        pub = it.advertise("frames", 1);
    };

    ~CameraNode() {
        ROS_INFO("[CameraNode]: Deinitializing camera node...");

        fpgaint_poll_deinit(&fpgaint_poll);

        if (ram_image_fd > 0 && (ram_image_fd = devmem_unmap(ram_image_fd, (void*) ram_image, IMG_SPAN))) {
            ROS_ERROR("[CameraNode]: Unmapping of 0x%08X failed with code %d\r\n!", IMG_ADDR, ram_image_fd);
        }
    };

    void run() {
        sensor_msgs::ImagePtr msg;
        cv::Mat ram_mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, ram_image);
        unsigned int cnt;

        ROS_INFO("[CameraNode]: Running camera node...");

        while (ros::ok()) {
            if ((cnt = fpgaint_poll_wait(&fpgaint_poll, 1000)) > 0) {
                if (cnt > 1) {
                    ROS_WARN("[CameraNode]: Missed %d frames (FPGA interrupts)!", cnt-1);
                }
                msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ram_mat.clone()).toImageMsg();
                pub.publish(msg);
            }
        }
    };
private:
    ros::NodeHandle& node;
    image_transport::Publisher pub;

    fpgaint_poll_t fpgaint_poll;
    int ram_image_fd;
    uint8_t* ram_image;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");

    ros::NodeHandle nh("~");
    CameraNode cn(nh);
    ROS_INFO("[CameraNode]: Camera node initialized.");

    cn.run();

    ros::spin();
    return 0;
}
