#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_srvs/SetBool.h>
#include <mrs_msgs/SetInt.h>

extern "C" {
#include "devmem/devmem.h"
#include "fpgaint_poll/fpgaint_poll.h"
#include "i2c/i2c.h"
#include "mt9v034.h"
}


class CameraNode {
public:
    CameraNode(ros::NodeHandle& nh) : node_(nh) {
        ROS_INFO("[CameraNode]: Initializing camera node...");

        nh.param<std::string>("i2c_dev", i2c_dev_, "/dev/i2c-1");
        nh.param<int32_t>("frame_width", frame_width_, MT9V034_MAX_WIDTH);
        nh.param<int32_t>("frame_height", frame_height_, MT9V034_MAX_HEIGHT);
        nh.param<int32_t>("exposure_us", exposure_us_, 100);
        nh.param<bool>("auto_exposure", auto_exposure_, false);
        nh.param<int32_t>("frame_ram_addr", ram_frame_addr__, 0x3FC00000);
        nh.param<bool>("frame_vflip", frame_vflip_, false);
        nh.param<bool>("frame_hflip", frame_hflip_, false);
        nh.param<int32_t>("fpgaint_num", fpgaint_num_, 0);

        serv_set_exposure_us_ = nh.advertiseService("set_exposure_us", &CameraNode::callbackSetExposureUs, this);
        serv_set_auto_exposure_ = nh.advertiseService("set_auto_exposure", &CameraNode::callbackSetAutoExposure, this);
        serv_set_frame_vflip_ = nh.advertiseService("set_frame_vflip", &CameraNode::callbackSetFrameVFlip, this);
        serv_set_frame_hflip_ = nh.advertiseService("set_frame_hflip", &CameraNode::callbackSetFrameHFlip, this);

        i2c_handle_ = (i2c_handle_t) { .dev_path = (const char*) i2c_dev_.c_str(), .handle = -1 };
        if (i2c_init(&i2c_handle_) != 0) {
            ROS_ERROR("[CameraNode]: Failed to open I2C device '%s'!", i2c_handle_.dev_path);
            return;
        }

        i2c_set_addr(&i2c_handle_, MT9V034_SLV_ADDR >> 1);

        uint16_t chip_version;
        i2c_read_reg_u16(&i2c_handle_, MTV_CHIP_VERSION_REG, &chip_version);
	    if (chip_version != MTV_CHIP_VERSION_REG_VAL) {
            ROS_ERROR("[CameraNode]: Failed to communicate with MT9V034 sensor over I2C! (Got chip id: 0x%04X)", chip_version);
            i2c_deinit(&i2c_handle_);
            return;
        }

        fpgaint_poll_ = FPGAINT_POLL_NEW(fpgaint_num_);
        if (!fpgaint_poll_init(&fpgaint_poll_)) {
            ROS_ERROR("[CameraNode]: File to initialize FPGA interrupt poll!");
            return;
        }

        if ((ram_image_fd_ = devmem_map(ram_frame_addr__, frame_width_*frame_height_, (void**) &ram_image_)) <= 0) {
            ROS_ERROR("[CameraNode]: Mapping of 0x%08X failed with code %d\r\n!", ram_frame_addr__, ram_image_fd_);
            return;
        }

        /*if (setFrameSize(frame_width_, frame_height_)) {
            ROS_WARN("[CameraNode]: Failed to configure camera: frame width and height!");
        }*/
        if (setVerticalFlip(frame_vflip_)) {
            ROS_WARN("[CameraNode]: Failed to configure camera: vertical flip!");
        }
        if (setHorizontalFlip(frame_hflip_)) {
            ROS_WARN("[CameraNode]: Failed to configure camera: horizontal flip!");
        }
        if (setAutoExposure(auto_exposure_, exposure_us_)) {
            ROS_WARN("[CameraNode]: Failed to configure camera: exposure!");
        }

        image_transport::ImageTransport it(nh);
        pub_frames_ = it.advertise("frames", 1);

        initialized_ = true;

        ROS_INFO("[CameraNode]: Camera node initialized.");
    };

    ~CameraNode() {
        ROS_INFO("[CameraNode]: Deinitializing camera node...");

        i2c_deinit(&i2c_handle_);

        fpgaint_poll_deinit(&fpgaint_poll_);

        if (ram_image_fd_ > 0 && (ram_image_fd_ = devmem_unmap(ram_image_fd_, (void*) ram_image_, frame_width_*frame_height_))) {
            ROS_ERROR("[CameraNode]: Unmapping of 0x%08X failed with code %d\r\n!", ram_frame_addr__, ram_image_fd_);
        }
    };

    void run() {
        if (!initialized_) return;

        sensor_msgs::ImagePtr msg;
        cv::Mat ram_mat(frame_height_, frame_width_, CV_8UC1, ram_image_);
        uint32_t cnt;

        ROS_INFO("[CameraNode]: Running camera node...");

        while (ros::ok()) {
            if ((cnt = fpgaint_poll_wait(&fpgaint_poll_, 20)) > 0) {
                if (cnt > 1) {
                    ROS_WARN("[CameraNode]: Missed %d frames (FPGA interrupts)!", cnt-1);
                }
                msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ram_mat.clone()).toImageMsg();
                pub_frames_.publish(msg);
            }
            ros::spinOnce();
        }
    };
private:
    bool initialized_ = false;

    ros::NodeHandle& node_;
    image_transport::Publisher pub_frames_;
    ros::ServiceServer serv_set_exposure_us_;
    ros::ServiceServer serv_set_auto_exposure_;
    ros::ServiceServer serv_set_frame_vflip_;
    ros::ServiceServer serv_set_frame_hflip_;

    fpgaint_poll_t fpgaint_poll_;
    i2c_handle_t i2c_handle_;
    int32_t ram_image_fd_;
    uint8_t* ram_image_;

    int32_t ram_frame_addr__;
    std::string i2c_dev_;
    int32_t frame_width_, frame_height_;
    int32_t exposure_us_;
    bool auto_exposure_;
    bool frame_vflip_, frame_hflip_;
    int32_t fpgaint_num_;

    bool callbackSetAutoExposure(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[CameraNode]: Service callbackSetAutoExposure - not initialized!");
            return true;
        }

        auto_exposure_ = req.data;
        if (setAutoExposure(auto_exposure_, exposure_us_)) {
            res.message = std::string("Failed to set auto exposure to: "+ std::string(auto_exposure_ ? "true" : "false") + "!").c_str();
            res.success = false;
            ROS_ERROR_STREAM("[CameraNode]: " << res.message);
        } else {
            res.message = std::string("Setting auto exposure to: "+ std::string(auto_exposure_ ? "true" : "false")).c_str();
            res.success = true;
            ROS_INFO_STREAM("[CameraNode]: " << res.message);
        }

        return true;
    };

    bool callbackSetFrameVFlip(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[CameraNode]: Service callbackSetFrameVFlip - not initialized!");
            return true;
        }

        frame_vflip_ = req.data;
        if (setVerticalFlip(frame_vflip_)) {
            res.message = std::string("Failed to set vertical flip to: "+ std::string(frame_vflip_ ? "true" : "false") + "!").c_str();
            res.success = false;
            ROS_ERROR_STREAM("[CameraNode]: " << res.message);
        } else {
            res.message = std::string("Setting vertical flip to: "+ std::string(frame_vflip_ ? "true" : "false")).c_str();
            res.success = true;
            ROS_INFO_STREAM("[CameraNode]: " << res.message);
        }

        return true;
    };

    bool callbackSetFrameHFlip(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[CameraNode]: Service callbackSetFrameHFlip - not initialized!");
            return true;
        }

        frame_hflip_ = req.data;
        if (setHorizontalFlip(frame_hflip_)) {
            res.message = std::string("Failed to set horizontal flip to: "+ std::string(frame_hflip_ ? "true" : "false") + "!").c_str();
            res.success = false;
            ROS_ERROR_STREAM("[CameraNode]: " << res.message);
        } else {
            res.message = std::string("Setting horizontal flip to: "+ std::string(frame_hflip_ ? "true" : "false")).c_str();
            res.success = true;
            ROS_INFO_STREAM("[CameraNode]: " << res.message);
        }

        return true;
    };

    bool callbackSetExposureUs(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[CameraNode]: Service callbackSetExposureUs - not initialized!");
            return true;
        }

        exposure_us_ = req.value;
        if (setAutoExposure(auto_exposure_, exposure_us_)) {
            res.message = std::string("Failed to set exposure time to: "+ std::to_string((int32_t) (exposure_us_)) + " us!").c_str();
            res.success = false;
            ROS_ERROR_STREAM("[CameraNode]: " << res.message);
        } else {
            res.message = std::string("Setting exposure time to: "+ std::to_string((int32_t) (exposure_us_)) + " us").c_str();
            res.success = true;
            ROS_INFO_STREAM("[CameraNode]: " << res.message);
        }

        return true;
    };

    /*bool setFrameSize(uint16_t width, uint16_t height) {
        // TODO: fix this function, when calling it with the default resolution (752x480), the FPS drops to about 45 FPS - why?
        // also, changing frame size during runtime must be reflected in the DCMI interface design and changes must be propagated to the FPGA fabric (eg. through PIO)
        if ((width > MT9V034_MAX_WIDTH) || (height > MT9V034_MAX_HEIGHT)) {
            return true;
        }

        uint16_t read_mode;
        if (i2c_read_reg_u16(&i2c_handle_, MT9V034_READ_MODE, &read_mode) != 0) {
            return true;
        }

        int32_t read_mode_mul = 1;
        read_mode &= 0xFFF0;

        if ((width <= (MT9V034_MAX_WIDTH / 4)) && (height <= (MT9V034_MAX_HEIGHT / 4))) {
            read_mode_mul = 4;
            read_mode |= MT9V034_READ_MODE_COL_BIN_4 | MT9V034_READ_MODE_ROW_BIN_4;
        } else if ((width <= (MT9V034_MAX_WIDTH / 2)) && (height <= (MT9V034_MAX_HEIGHT / 2))) {
            read_mode_mul = 2;
            read_mode |= MT9V034_READ_MODE_COL_BIN_2 | MT9V034_READ_MODE_ROW_BIN_2;
        }

        int32_t ret = 0;
        int32_t val;

        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_COL_START, ((MT9V034_MAX_WIDTH - (width * read_mode_mul)) / 2) + MT9V034_COL_START_MIN);
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_ROW_START, ((MT9V034_MAX_HEIGHT - (height * read_mode_mul)) / 2) + MT9V034_ROW_START_MIN);
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_WINDOW_WIDTH, width * read_mode_mul);
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_WINDOW_HEIGHT, height * read_mode_mul);

        val = width * read_mode_mul;
        if (val > MT9V034_MAX_HEIGHT) val = MT9V034_MAX_HEIGHT;
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_HORIZONTAL_BLANKING,
        MT9V034_HORIZONTAL_BLANKING_DEF + (MT9V034_MAX_WIDTH - val));

        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_READ_MODE, read_mode);
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_PIXEL_COUNT, (width * height) / 8);

        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_PIXEL_CLOCK, (read_mode_mul == 1) ? MT9V034_PIXEL_CLOCK_INV_PXL_CLK : 0);

        return (ret != 0);
    };*/

    bool setVerticalFlip(bool enable) {
        uint16_t read_mode;
        int32_t ret = i2c_read_reg_u16(&i2c_handle_, MT9V034_READ_MODE, &read_mode);
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_READ_MODE, (read_mode & (~MT9V034_READ_MODE_ROW_FLIP)) | ((!enable) ? MT9V034_READ_MODE_ROW_FLIP : 0));

        return (ret != 0);
    };

    bool setHorizontalFlip(bool enable) {
        uint16_t read_mode;
        int32_t ret = i2c_read_reg_u16(&i2c_handle_, MT9V034_READ_MODE, &read_mode);
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_READ_MODE, (read_mode & (~MT9V034_READ_MODE_COL_FLIP)) | ((!enable) ? MT9V034_READ_MODE_COL_FLIP : 0));

        return (ret != 0);
    };

    bool setAutoExposure(bool enable, int32_t exposure_us) {
        uint16_t reg, row_time_0, row_time_1;
        int32_t ret = i2c_read_reg_u16(&i2c_handle_, MT9V034_AEC_AGC_ENABLE, &reg);
        ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_AEC_AGC_ENABLE, (reg & (~MT9V034_AEC_ENABLE)) | ((enable != 0) ? MT9V034_AEC_ENABLE : 0));

        if ((enable == 0) && (exposure_us >= 0)) {
            ret |= i2c_read_reg_u16(&i2c_handle_, MT9V034_WINDOW_WIDTH, &row_time_0);
            ret |= i2c_read_reg_u16(&i2c_handle_, MT9V034_HORIZONTAL_BLANKING, &row_time_1);

            int32_t exposure = MICROSECOND_CLKS / 2; 
            if (exposure_us < exposure) exposure = exposure_us;
            exposure *= (MT9V034_XCLK_FREQ / MICROSECOND_CLKS);
            int32_t row_time = row_time_0 + row_time_1;
            int32_t coarse_time = exposure / row_time;
            int32_t fine_time = exposure % row_time;

            ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_TOTAL_SHUTTER_WIDTH, coarse_time);
            ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_FINE_SHUTTER_WIDTH_TOTAL, fine_time);
        } else if ((enable != 0) && (exposure_us >= 0)) {
            ret |= i2c_read_reg_u16(&i2c_handle_, MT9V034_WINDOW_WIDTH, &row_time_0);
            ret |= i2c_read_reg_u16(&i2c_handle_, MT9V034_HORIZONTAL_BLANKING, &row_time_1);

            int32_t exposure = MICROSECOND_CLKS / 2;
            if (exposure_us < exposure) exposure = exposure_us;
            exposure *= (MT9V034_XCLK_FREQ / MICROSECOND_CLKS);
            int32_t row_time = row_time_0 + row_time_1;
            int32_t coarse_time = exposure / row_time;

            ret |= i2c_write_reg_u16(&i2c_handle_, MT9V034_MAX_EXPOSE, coarse_time);
        }

        return (ret != 0);
    };
};


int32_t main(int32_t argc, char** argv) {
    ros::init(argc, argv, "camera_node");

    ros::NodeHandle nh("~");
    CameraNode cn(nh);

    cn.run();

    ros::spin();
    return 0;
}
