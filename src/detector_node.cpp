#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_msgs/SetInt.h>
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

        double concentrate_dist;
        int32_t ram_detections_addr_signed, control_fast_addr_signed;
        int32_t fast_threshold, fast_threshold_diff, fast_threshold_sun; 

        nh.param<int32_t>("frame_width", frame_width_, 752);
        nh.param<int32_t>("frame_height", frame_height_, 480);
        nh.param<int32_t>("detections_ram_addr", ram_detections_addr_signed, (int32_t) 0x3FE00000);
        nh.param<int32_t>("control_fast_addr", control_fast_addr_signed, (int32_t) 0xFF200010);
        nh.param<int32_t>("fpgaint_num", fpgaint_num_, 3);
        nh.param<double>("concentrate_dist_px", concentrate_dist, 5);
        nh.param<double>("marker_sun_dist_px", marker_sun_dist_, 8);
        nh.param<int32_t>("fast_threshold", fast_threshold, 120);
        nh.param<int32_t>("fast_threshold_diff", fast_threshold_diff, 60);
        nh.param<int32_t>("fast_threshold_sun", fast_threshold_sun, 240);
        nh.param<int32_t>("max_markers_count", max_markers_count_, 30);
        nh.param<int32_t>("max_sun_pts_count", max_sun_pts_count_, 6000);

        ram_detections_addr_ = (uint32_t) ram_detections_addr_signed;
        control_fast_addr_ = (uint32_t) control_fast_addr_signed;
        concentrate_dist_sq_ = pow(concentrate_dist, 2);
        marker_sun_dist_sq_ = pow(marker_sun_dist_, 2);
        fast_threshold_ = (uint8_t) fast_threshold;
        fast_threshold_diff_ = (uint8_t) fast_threshold_diff;
        fast_threshold_sun_ = (uint8_t) fast_threshold_sun;

        serv_set_fast_threshold_ = nh.advertiseService("set_fast_threshold", &DetectorNode::callbackSetFASTThreshold, this);
        serv_set_fast_threshold_diff_ = nh.advertiseService("set_fast_threshold_diff", &DetectorNode::callbackSetFASTThresholdDiff, this);
        serv_set_fast_threshold_sun_ = nh.advertiseService("set_fast_threshold_sun", &DetectorNode::callbackSetFASTThresholdSun, this);
        serv_set_concentrate_dist_px_ = nh.advertiseService("set_concentrate_dist_px", &DetectorNode::callbackSetConcentrateDistPx, this);
        serv_set_marker_sun_dist_px_ = nh.advertiseService("set_marker_sun_dist_px", &DetectorNode::callbackSetMarkerSunDistPx, this);
        serv_set_max_markers_count_ = nh.advertiseService("set_max_markers_count", &DetectorNode::callbackSetMaxMarkersCount, this);
        serv_set_max_sun_pts_count_px_ = nh.advertiseService("set_max_sun_pts_count", &DetectorNode::callbackSetMaxSunPtsCount, this);

        fpgaint_poll_ = FPGAINT_POLL_NEW(fpgaint_num_);
        if (!fpgaint_poll_init(&fpgaint_poll_)) {
            ROS_ERROR("[DetectorNode]: File to initialize FPGA interrupt poll!");
            return;
        }

        if ((ram_detections_fd_ = devmem_map(ram_detections_addr_, sizeof(uint32_t)*frame_width_*frame_height_, (void**) &ram_detections_)) <= 0) {
            ROS_ERROR("[DetectorNode]: Mapping of 0x%08X failed with code %d\r\n!", ram_detections_addr_, ram_detections_fd_);
            return;
        }

        if ((control_fast_fd_ = devmem_map(control_fast_addr_, sizeof(uint32_t), (void**) &control_fast_)) <= 0) {
            ROS_ERROR("[DetectorNode]: Mapping of 0x%08X failed with code %d\r\n!", control_fast_addr_, control_fast_fd_);
            return;
        }

        setFASTThresholds();

        pub_sun_pts_ = nh.advertise<mrs_msgs::ImagePointsWithFloatStamped>("sun_pts", 1);
        pub_markers_ = nh.advertise<mrs_msgs::ImagePointsWithFloatStamped>("markers", 1);

        initialized_ = true;

        ROS_INFO("[DetectorNode]: Detector node initialized.");
    };

    ~DetectorNode() {
        ROS_INFO("[DetectorNode]: Deinitializing detector node...");

        fpgaint_poll_deinit(&fpgaint_poll_);

        if (ram_detections_fd_ > 0 && (ram_detections_fd_ = devmem_unmap(ram_detections_fd_, (void*) ram_detections_, sizeof(uint32_t)*frame_width_*frame_height_))) {
            ROS_ERROR("[DetectorNode]: Unmapping of 0x%08X failed with code %d\r\n!", ram_detections_addr_, ram_detections_fd_);
        }

        if (control_fast_fd_ > 0 && (control_fast_fd_ = devmem_unmap(control_fast_fd_, (void*) control_fast_, sizeof(uint32_t)))) {
            ROS_ERROR("[DetectorNode]: Unmapping of 0x%08X failed with code %d\r\n!", control_fast_addr_, control_fast_fd_);
        }
    };

    void run() {
        if (!initialized_) return;

        uint32_t cnt;

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
private:
    bool initialized_ = false;

    ros::NodeHandle& node_;
    ros::Publisher pub_sun_pts_;
    ros::Publisher pub_markers_;

    ros::ServiceServer serv_set_fast_threshold_;
    ros::ServiceServer serv_set_fast_threshold_diff_;
    ros::ServiceServer serv_set_fast_threshold_sun_;
    ros::ServiceServer serv_set_concentrate_dist_px_;
    ros::ServiceServer serv_set_marker_sun_dist_px_;
    ros::ServiceServer serv_set_max_markers_count_;
    ros::ServiceServer serv_set_max_sun_pts_count_px_;

    fpgaint_poll_t fpgaint_poll_;
    int32_t ram_detections_fd_, control_fast_fd_;
    uint32_t* ram_detections_;
    uint32_t* control_fast_;

    uint32_t ram_detections_addr_, control_fast_addr_;
    int32_t frame_width_, frame_height_;
    int32_t fpgaint_num_;
    double concentrate_dist_sq_;
    double marker_sun_dist_, marker_sun_dist_sq_;
    uint8_t fast_threshold_, fast_threshold_diff_, fast_threshold_sun_;
    int32_t max_markers_count_, max_sun_pts_count_;

    static constexpr uint32_t det_x_pos_ = 0;
    static constexpr uint32_t det_y_pos_ = 15;
    static constexpr uint32_t marker_pot_pos_ = 30;
    static constexpr uint32_t sun_pot_pos_ = 31;

    static constexpr uint32_t det_x_bits_ = 15;
    static constexpr uint32_t det_y_bits_ = 15;
    static constexpr uint32_t marker_pot_bits_ = 1;
    static constexpr uint32_t sun_pot_bits_ = 1;


    bool callbackSetFASTThreshold(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[DetectorNode]: Service callbackSetFASTThreshold - not initialized!");
            return true;
        }

        fast_threshold_ = (uint8_t) req.value;
        if (setFASTThresholds()) {
            res.message = std::string("Failed to set fast threshold to: "+ std::to_string((int32_t) (fast_threshold_)) + "!").c_str();
            res.success = false;
            ROS_ERROR_STREAM("[DetectorNode]: " << res.message);
        } else {
            res.message = std::string("Setting fast threshold to: "+ std::to_string((int32_t) (fast_threshold_))).c_str();
            res.success = true;
            ROS_INFO_STREAM("[DetectorNode]: " << res.message);
        }

        return true;
    };

    bool callbackSetFASTThresholdDiff(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[DetectorNode]: Service callbackSetFASTThresholdDiff - not initialized!");
            return true;
        }

        fast_threshold_diff_ = (uint8_t) req.value;
        if (setFASTThresholds()) {
            res.message = std::string("Failed to set differential fast threshold to: "+ std::to_string((int32_t) (fast_threshold_diff_)) + "!").c_str();
            res.success = false;
            ROS_ERROR_STREAM("[DetectorNode]: " << res.message);
        } else {
            res.message = std::string("Setting differential fast threshold to: "+ std::to_string((int32_t) (fast_threshold_diff_))).c_str();
            res.success = true;
            ROS_INFO_STREAM("[DetectorNode]: " << res.message);
        }

        return true;
    };

    bool callbackSetFASTThresholdSun(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[DetectorNode]: Service callbackSetFASTThresholdSun - not initialized!");
            return true;
        }

        fast_threshold_sun_ = (uint8_t) req.value;
        if (setFASTThresholds()) {
            res.message = std::string("Failed to set sun fast threshold to: "+ std::to_string((int32_t) (fast_threshold_sun_)) + "!").c_str();
            res.success = false;
            ROS_ERROR_STREAM("[DetectorNode]: " << res.message);
        } else {
            res.message = std::string("Setting sun fast threshold to: "+ std::to_string((int32_t) (fast_threshold_sun_))).c_str();
            res.success = true;
            ROS_INFO_STREAM("[DetectorNode]: " << res.message);
        }

        return true;
    };

    bool callbackSetConcentrateDistPx(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[DetectorNode]: Service callbackSetConcentrateDistPx - not initialized!");
            return true;
        }

        int32_t dist_px = (uint8_t) req.value;
        concentrate_dist_sq_ = (double) (dist_px * dist_px);
        res.message = std::string("Setting marker concentrating distance to: "+ std::to_string((int32_t) (dist_px)) + " px").c_str();
        res.success = true;
        ROS_INFO_STREAM("[DetectorNode]: " << res.message);

        return true;
    };

    bool callbackSetMarkerSunDistPx(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[DetectorNode]: Service callbackSetMarkerSunDistPx - not initialized!");
            return true;
        }

        marker_sun_dist_ = (double) (req.value);
        marker_sun_dist_sq_ = pow(marker_sun_dist_, 2);
        res.message = std::string("Setting marker-sun filtering distance to: "+ std::to_string((int32_t) (marker_sun_dist_)) + " px").c_str();
        res.success = true;
        ROS_INFO_STREAM("[DetectorNode]: " << res.message);

        return true;
    };

    bool callbackSetMaxMarkersCount(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[DetectorNode]: Service callbackSetMaxMarkersCount - not initialized!");
            return true;
        }

        max_markers_count_ = req.value;
        res.message = std::string("Setting maximum markers count to: "+ std::to_string((int32_t) (max_markers_count_))).c_str();
        res.success = true;
        ROS_INFO_STREAM("[DetectorNode]: " << res.message);

        return true;
    };

    bool callbackSetMaxSunPtsCount(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res) {
        if (!initialized_) {
            ROS_ERROR("[DetectorNode]: Service callbackSetMaxSunPtsCount - not initialized!");
            return true;
        }

        max_sun_pts_count_ = req.value;
        res.message = std::string("Setting maximum sun points count to: "+ std::to_string((int32_t) (max_sun_pts_count_))).c_str();
        res.success = true;
        ROS_INFO_STREAM("[DetectorNode]: " << res.message);

        return true;
    };


    bool setFASTThresholds() {
        control_fast_[0] = ((uint32_t) fast_threshold_) | (((uint32_t) fast_threshold_diff_) << 8) | (((uint32_t) fast_threshold_sun_) << 16);
        return false;
    };

    void processDetectionResults() {
        mrs_msgs::ImagePointsWithFloatStamped msg_sun_pts;
        mrs_msgs::ImagePointsWithFloatStamped msg_markers;

        std::vector<mrs_msgs::Point2DWithFloat> markers;
        std::vector<mrs_msgs::Point2DWithFloat> sun_pts;

        int32_t i;
        uint32_t val;
        bool sun_pot, marker_pot;
        uint32_t det_y, det_x;
        uint32_t markers_raw_count = 0, sun_pts_count = 0;

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
                if (sun_pts_count < (uint32_t) max_sun_pts_count_) {
                    filterNewPoints(markers, sun_pts, point, true);
                }
                sun_pts_count++;
            } else if (marker_pot) {
                if (markers.size() < (uint32_t) max_markers_count_) {
                    filterNewPoints(markers, sun_pts, point, false);
                }
                markers_raw_count++;
            } else {
                ROS_WARN("[DetectorNode] Received detection result (x=%d, y=%d) which is neither marker or sun point?!", det_x, det_y);
            }
        }

        if (sun_pts_count >= (uint32_t) max_sun_pts_count_) {
            ROS_WARN("[DetectorNode] Maximum count of sun points (%d >= %d) reached! Skipping the rest...", sun_pts_count, max_sun_pts_count_);
        }
        
        if (markers.size() == (uint32_t) max_markers_count_) {
            ROS_WARN("[DetectorNode] Maximum count of concentrated markers (= %d) reached (%d raw markers in total)! Skipping the rest...", max_markers_count_, markers_raw_count);
        }

        for (auto it = markers.begin(); it != markers.end(); it++) {
            (*it).x = round((*it).x / (*it).value);
            (*it).y = round((*it).y / (*it).value);
            (*it).value = 1.0;
        }

        msg_markers.stamp = msg_sun_pts.stamp = ros::Time::now();
        msg_markers.image_width = msg_sun_pts.image_width = frame_width_;
        msg_markers.image_height = msg_sun_pts.image_height = frame_height_;

        msg_markers.points = markers;
        msg_sun_pts.points = sun_pts;

        pub_markers_.publish(msg_markers);
        pub_sun_pts_.publish(msg_sun_pts);
    };

    bool filterNewPoints(std::vector<mrs_msgs::Point2DWithFloat>& markers, std::vector<mrs_msgs::Point2DWithFloat>& sun_pts, mrs_msgs::Point2DWithFloat point, bool is_sun_pt) {
        bool new_point = true;
        double abs_dist_x, abs_dist_y;

        auto it = markers.end();
        if (!is_sun_pt) {
            // if the point is a marker, try to find an another marker close enough to be merged with
            while (it != markers.begin()) {
                --it;
                abs_dist_x = point.x - ((*it).x / (*it).value);
                abs_dist_y = point.y - ((*it).y / (*it).value);
                if (pow(abs_dist_x, 2) + pow(abs_dist_y, 2) <= concentrate_dist_sq_) {
                    new_point = false;
                    break;
                }
            }
        }
        if (new_point) {
            // if no close marker is found / the point is a sun point
            if (is_sun_pt) {
                // if the new point is a sun point
                it = markers.end();
                while (it != markers.begin()) {
                    --it;
                    abs_dist_y = abs(((*it).y / (*it).value) - point.y);
                    if (abs_dist_y > marker_sun_dist_) {
                        break; // too far away, no need to check the markers anymore
                    }
                    abs_dist_x = abs(((*it).x / (*it).value) - point.x);
                    if (abs_dist_x > marker_sun_dist_) {
                        continue;
                    }
                    if (pow(abs_dist_x, 2) + pow(abs_dist_y, 2) <= marker_sun_dist_sq_) {
                        markers.erase(it); // marker is close to the new sun point, remove the marker
                    }
                }
                sun_pts.push_back(point);
            } else {
                // if the new point is a marker
                it = sun_pts.end();
                while (it != sun_pts.begin()) {
                    --it;
                    abs_dist_y = abs((*it).y - point.y);
                    if (abs_dist_y > marker_sun_dist_) {
                        break; // too far away, no need to check the sun points anymore
                    }
                    abs_dist_x = abs((*it).x - point.x);
                    if (abs_dist_x > marker_sun_dist_) {
                        continue;
                    }
                    if (pow(abs_dist_x, 2) + pow(abs_dist_y, 2) <= marker_sun_dist_sq_) {
                        new_point = false; // sun point is close to the new marker, skip the new marker
                        break;
                    }
                }
                if (new_point) {
                    markers.push_back(point);
                }
            }
        } else {
            // close marker found, merge them
            (*it).x += point.x;
            (*it).y += point.y;
            (*it).value += 1;
        }

        return new_point;
    };
};


int32_t main(int32_t argc, char** argv) {
    ros::init(argc, argv, "detector_node");

    ros::NodeHandle nh("~");
    DetectorNode dn(nh);

    dn.run();

    ros::spin();
    return 0;
}
