#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_msgs/ImagePointsWithFloatStamped.h>

extern "C" {
#include "devmem/devmem.h"
#include "fpgaint_poll/fpgaint_poll.h"
}

#define GET_MASKED_VALUE(val, bits, pos) (((val) >> (pos)) & ((1 << (bits)) - 1))

class DetectorNode {
public:
    DetectorNode(ros::NodeHandle& nh) : node_(nh) {
        ROS_INFO("[DetectorNode]: Initializing detector node...");

        double concentrate_dist, marker_sun_dist;

        nh.param<int>("frame_width", frame_width_, 752);
        nh.param<int>("frame_height", frame_height_, 480);
        nh.param<int>("detections_ram_addr", ram_detections_addr_, 0x3FE00000);
        nh.param<int>("fpgaint_num", fpgaint_num_, 3);
        nh.param<double>("concentrate_dist_px", concentrate_dist, 5);
        nh.param<double>("marker_sun_dist_px", marker_sun_dist, 8);

        concentrate_dist_sq_ = pow(concentrate_dist, 2);
        marker_sun_dist_sq_ = pow(marker_sun_dist, 2);

        fpgaint_poll_ = FPGAINT_POLL_NEW(fpgaint_num_);
        if (!fpgaint_poll_init(&fpgaint_poll_)) {
            ROS_ERROR("[DetectorNode]: File to initialize FPGA interrupt poll!");
            return;
        }

        if ((ram_detections_fd_ = devmem_map(ram_detections_addr_, sizeof(uint32_t)*frame_width_*frame_height_, (void**) &ram_detections_)) <= 0) {
            ROS_ERROR("[DetectorNode]: Mapping of 0x%08X failed with code %d\r\n!", ram_detections_addr_, ram_detections_fd_);
            return;
        }

        pub_sun_pts_ = nh.advertise<mrs_msgs::ImagePointsWithFloatStamped>("sun_pts", 1);
        pub_markers_ = nh.advertise<mrs_msgs::ImagePointsWithFloatStamped>("markers", 1);

        initialized_ = true;

        ROS_INFO("[DetectorNode]: Detector node initialized_.");
    };

    ~DetectorNode() {
        ROS_INFO("[DetectorNode]: Deinitializing detector node...");

        fpgaint_poll_deinit(&fpgaint_poll_);

        if (ram_detections_fd_ > 0 && (ram_detections_fd_ = devmem_unmap(ram_detections_fd_, (void*) ram_detections_, sizeof(uint32_t)*frame_width_*frame_height_))) {
            ROS_ERROR("[DetectorNode]: Unmapping of 0x%08X failed with code %d\r\n!", ram_detections_addr_, ram_detections_fd_);
        }
    };

    void run() {
        if (!initialized_) return;

        unsigned int cnt;

        ROS_INFO("[DetectorNode]: Running detector node...");

        while (ros::ok()) {
            if ((cnt = fpgaint_poll_wait(&fpgaint_poll_, 20)) > 0) {
                if (cnt > 1) {
                    ROS_WARN("[DetectorNode]: Missed %d detection results (FPGA interrupts)!", cnt-1);
                }
                processDetectionResults();
            }
            ros::spinOnce();
        }
    };

    void processDetectionResults() {
        mrs_msgs::ImagePointsWithFloatStamped msg_sun_pts;
        mrs_msgs::ImagePointsWithFloatStamped msg_markers;

        std::vector<mrs_msgs::Point2DWithFloat> markers;
        std::vector<mrs_msgs::Point2DWithFloat> sun_pts;

        int i;
        uint32_t val;
        bool sun_pot, marker_pot;
        uint32_t det_y, det_x;

        for (i = 0; i < frame_width_*frame_height_; i++) {
            val = ram_detections_[i];
            if (val == 0) {
                break;
            }

            sun_pot = GET_MASKED_VALUE(val, sun_pot_bits_, sun_pot_pos_);
            marker_pot = GET_MASKED_VALUE(val, marker_pot_bits_, marker_pot_pos_);
            det_y = GET_MASKED_VALUE(val, det_y_bits_, det_y_pos_);
            det_x = GET_MASKED_VALUE(val, det_x_bits_, det_x_pos_);

            mrs_msgs::Point2DWithFloat point;
            point.x = (double) det_x;
            point.y = (double) det_y;
            point.value = (double) 1.0;

            if (sun_pot) {
                sun_pts.push_back(point);
            } else if (marker_pot) {
                markers.push_back(point);
            } else {
                ROS_WARN("[DetectorNode] Received detection result (x=%d, y=%d) which is neither marker or sun point?!", det_x, det_y);
            }
        }

        concentrateMarkers(markers);
        filterMarkersBySun(markers, sun_pts);

        msg_markers.stamp = msg_sun_pts.stamp = ros::Time::now();
        msg_markers.image_width = msg_sun_pts.image_width = frame_width_;
        msg_markers.image_height = msg_sun_pts.image_height = frame_height_;

        msg_markers.points = markers;
        msg_sun_pts.points = sun_pts;

        pub_markers_.publish(msg_markers);
        pub_sun_pts_.publish(msg_sun_pts);
    };

    void concentrateMarkers(std::vector<mrs_msgs::Point2DWithFloat>& markers) {
        unsigned int i, j;
        int j_found;

        for (i = 0; i < markers.size(); i++) {
            j_found = -1;
            for (j = 0; j < i; j++) {
                if (markers[j].value < 1) {
                    continue;
                }
                if (pow(markers[i].x - (markers[j].x / markers[j].value), 2) + pow(markers[i].y - (markers[j].y / markers[j].value), 2) <= concentrate_dist_sq_) {
                    j_found = j;
                    break;
                }
            }
            if (j_found >= 0) {
                markers[j_found].x += markers[i].x;
                markers[j_found].y += markers[i].y;
                markers[j_found].value += 1;
                markers[i].value = 0;
            }
        }
    };

    void filterMarkersBySun(std::vector<mrs_msgs::Point2DWithFloat>& markers, std::vector<mrs_msgs::Point2DWithFloat>& sun_pts) {
        auto marker_ptr = markers.begin();
        while (marker_ptr != markers.end()) {
            bool erased = false;
            if ((*marker_ptr).value < 1) {
                markers.erase(marker_ptr);
                erased = true;
            } else {
                (*marker_ptr).x = round((*marker_ptr).x / (*marker_ptr).value);
                (*marker_ptr).y = round((*marker_ptr).y / (*marker_ptr).value);
                (*marker_ptr).value = 1;

                auto sun_pt_ptr = sun_pts.begin();
                while (sun_pt_ptr != sun_pts.end() && !erased) {
                    if (pow((*marker_ptr).x - (*sun_pt_ptr).x, 2) + pow((*marker_ptr).y - (*sun_pt_ptr).y, 2) <= marker_sun_dist_sq_) {
                        markers.erase(marker_ptr);
                        erased = true;
                    }
                    ++sun_pt_ptr;
                }
            }
            if (!erased) {
                ++marker_ptr;
            }
        }
    }
private:
    bool initialized_ = false;

    ros::NodeHandle& node_;
    ros::Publisher pub_sun_pts_;
    ros::Publisher pub_markers_;

    fpgaint_poll_t fpgaint_poll_;
    int ram_detections_fd_;
    uint32_t* ram_detections_;

    int ram_detections_addr_;
    int frame_width_, frame_height_;
    int fpgaint_num_;
    double concentrate_dist_sq_;
    double marker_sun_dist_sq_;

    static constexpr uint32_t det_x_pos_ = 0;
    static constexpr uint32_t det_y_pos_ = 15;
    static constexpr uint32_t marker_pot_pos_ = 30;
    static constexpr uint32_t sun_pot_pos_ = 31;

    static constexpr uint32_t det_x_bits_ = 15;
    static constexpr uint32_t det_y_bits_ = 15;
    static constexpr uint32_t marker_pot_bits_ = 1;
    static constexpr uint32_t sun_pot_bits_ = 1;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "detector_node");

    ros::NodeHandle nh("~");
    DetectorNode dn(nh);

    dn.run();

    ros::spin();
    return 0;
}
