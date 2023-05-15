//
// Created by xc on 2021/4/6.
//

#include <std_msgs/Bool.h>
#include "luvi-mapping/uwb_calibration.h"

namespace LIRO{
    UwbCalibration::UwbCalibration():nh_("~") {
        nh_.getParam("/data_dir_path", data_dir_path_);
        nh_.getParam("/uwb_anchor_number", anchor_number_);
        msgs_flag = false;
        uwb_data_path_ = data_dir_path_ + "/uwb_data.txt";
        save_uwb_path_ = data_dir_path_ + "/uwb_settled_postions.txt";
        sub_uwb_data_flag_ = nh_.subscribe<std_msgs::Bool>("/liro/data_flag",2,&UwbCalibration::UwbDataFlagCallback,this);
        pub_anchor_initial_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/anchor_initial",2);
        pub_anchor_optimized_ = nh_.advertise<sensor_msgs::PointCloud2>("/liro/anchor_settled",2);
        pub_anchor_measurments_ = nh_.advertise<visualization_msgs::MarkerArray>("/liro/uwb_measurments",1);
        pub_anchor_outliners_measurments_ = nh_.advertise<visualization_msgs::MarkerArray>("/liro/uwb_outliner_measurments",1);
        for(int i=0;i<anchor_number_;++i){
            front_dis_[i] = 100;
            mid_dis_[i] = 100;
            back_dis_[i] = 100;
        }
        std::thread* command(new std::thread(&UwbCalibration::Command,this));
    };

    UwbCalibration::~UwbCalibration() {}

    void UwbCalibration::Run() {
        if(msgs_flag){
            LoadData();
            if(uwb_data_.empty())
                return;
            GuassUwbInitial();
            Optimization();
            AnchorsVisualization();
            AnchorMeasurmentsVisualization();
            std::map<int,std::vector<std::array<double,4>>> empty_map;
            uwb_data_.swap(empty_map);
            uwb_data_.clear();
            msgs_flag = false;
        }
    }

    void UwbCalibration::UwbDataFlagCallback(std_msgs::Bool flag) {
        if(flag.data)
            msgs_flag = true;
    }

    void UwbCalibration::Command() {
        while(1){
            char c = std::getchar();
            if('u'==c){
                std::cout << "get command u ..."<<std::endl;
                SaveDatas();
            }
            std::chrono::microseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }

    bool UwbCalibration::LoadData() {
        std::FILE* uwb_ifile;
        uwb_ifile = std::fopen(uwb_data_path_.c_str(),"r");
        if(uwb_ifile == NULL){
            std::cout << "! Open [ " << uwb_data_path_ << " ] failed ..." << std::endl;
            return false;
        }
        int anchor_idx, kf_idx;
        double px, py, pz, dis;
        while(std::fscanf(uwb_ifile,"%d %d %lf %lf %lf %lf", &anchor_idx, &kf_idx, &px, &py, &pz, &dis) != EOF){
            if(!uwb_data_.count(anchor_idx)){
                std::vector<std::array<double,4>> uwb_p_dis;
                uwb_p_dis.push_back({px,py,pz,dis});
                uwb_data_[anchor_idx] = uwb_p_dis;
            }
            else{
                uwb_data_[anchor_idx].push_back({px,py,pz,dis});
            }
        }
        std::fclose(uwb_ifile);
        ROS_ERROR("load uwb data done ...");
        return true;
    }

    void UwbCalibration::GuassUwbInitial() {
            for(auto data : uwb_data_){
                if(data.second.size() < 30){
                    continue;
                }
                double mid_dis = 1000;
                if(mid_dis_[data.first] >1.0){
                    for(auto dis : data.second){ // to find the point which has min distance
                        if(dis.back() < mid_dis){
                            mid_dis = dis.back();
                        }
                    }
                    mid_dis_[data.first] = mid_dis + 0.1;
                    front_dis_[data.first] = mid_dis + 0.5;
                    back_dis_[data.first] = mid_dis + 0.6;
                }
                ROS_ERROR_STREAM("mid dis :  " << mid_dis_[data.first] <<"\n");
                std::vector<std::array<double,4>>::iterator front_iter, mid_iter,back_iter;
                std::array<double,4> front_data, mide_data, back_data;
                bool valid_mid = false, valid_front = false, valid_end = false;
                for(auto it = data.second.begin(); it != data.second.end(); ++it){
                    if(it->back() < mid_dis_[data.first]){
                        mide_data = *it;
                        mid_iter = it;
                        valid_mid = true;
                        break;
                    }
                }
                if (!valid_mid)
                    continue;
                for(front_iter = mid_iter; front_iter != data.second.begin(); --front_iter){
                    if(front_iter->back() > front_dis_[data.first]){
                        front_data = * front_iter;
                        valid_front = true;
                        break;
                    }
                }
                if(!valid_front)
                    continue;
                for(back_iter = mid_iter; back_iter != data.second.end(); ++back_iter){
                    if(back_iter->back() > back_dis_[data.first]){
                        back_data = * back_iter;
                        valid_end = true;
                        break;
                    }
                }
                if(!valid_end)
                    continue;
                Circle circle1(front_data[0],front_data[1],front_data.back());
                Circle circle2(mide_data[0],mide_data[1],mide_data.back());
                Circle circle3(back_data[0],back_data[1],back_data.back());
                std::vector<Point> p12 = circle1.FindIntersectionPoints(circle2);
                std::vector<Point> p13 = circle1.FindIntersectionPoints(circle3);
                std::vector<Point> p23 = circle2.FindIntersectionPoints(circle3);
                ROS_ERROR_STREAM("anchor " << data.first << " ---> p12.size :"<< p12.size() <<"  p13.size :"<< p13.size() <<"  p23.size :"<< p23.size() <<"\n");
                if(p12.size()<2|| p13.size()<2|| p23.size()<2){
                    continue;
                }
                std::vector<Point> three_in_part,three_in_anthor_part;
                three_in_part.push_back(p12.at(0));
                three_in_anthor_part.push_back(p12.at(1));
                // classify the intersecting points into 2 parts
                if(Distance(p12.at(0),p13.at(0)) < Distance(p12.at(0),p13.at(1))){
                    three_in_part.push_back(p13.at(0));
                    three_in_anthor_part.push_back(p13.at(1));
                }
                else{
                    three_in_part.push_back(p13.at(1));
                    three_in_anthor_part.push_back(p13.at(0));

                }
                if(Distance(p12.at(0),p23.at(0)) < Distance(p12.at(0),p23.at(1))){
                    three_in_part.push_back(p23.at(0));
                    three_in_anthor_part.push_back(p23.at(1));
                }
                else{
                    three_in_part.push_back(p23.at(1));
                    three_in_anthor_part.push_back(p23.at(0));

                }

                double d1 = Distance(three_in_part[0],three_in_part[1]) +
                            Distance(three_in_part[1],three_in_part[2]) + Distance(three_in_part[2],three_in_part[0]);
                double d2 = Distance(three_in_anthor_part[0],three_in_anthor_part[1]) +
                            Distance(three_in_anthor_part[1],three_in_anthor_part[2]) + Distance(three_in_anthor_part[2],three_in_anthor_part[0]);
                Point p1(front_data[0],front_data[1]);
                Point p2(mide_data[0],mide_data[1]);
                Point p3(back_data[0],back_data[1]);
                bool is_curve = IsCurve(p1, p2, p3);
                ROS_ERROR_STREAM("anchor "<<data.first <<" : is curve  " << is_curve <<"\n" );
                double x1, y1, z1, x2, y2, z2;
                x1  = (three_in_part.at(0).get_x_coord() + three_in_part.at(1).get_x_coord() + three_in_part.at(2).get_x_coord())/3;
                y1  = (three_in_part.at(0).get_y_coord() + three_in_part.at(1).get_y_coord() + three_in_part.at(2).get_y_coord())/3;
                z1 = mide_data.at(2) + 0.4;

                x2 = (three_in_anthor_part.at(0).get_x_coord() + three_in_anthor_part.at(1).get_x_coord() + three_in_anthor_part.at(2).get_x_coord())/3;
                y2 = (three_in_anthor_part.at(0).get_y_coord() + three_in_anthor_part.at(1).get_y_coord() + three_in_anthor_part.at(2).get_y_coord())/3;
                z2 = mide_data.at(2) + 0.4 ;

                if(d1 > d2){
                    if(is_curve)
                        uwb_initial_[data.first] = {x2, y2, z2};
                    else
                        uwb_initial_[data.first] = {x1, y1, z1};
                }
                else{
                    if(is_curve)
                        uwb_initial_[data.first] = {x1, y1, z1};
                    else
                        uwb_initial_[data.first] = {x2, y2, z2};
                }
        }
    }

    /***
     * input: p1 p2 p3
     * function:
     * return: ture: curve   false: straight line
     ***/
    bool UwbCalibration::IsCurve(LIRO::Point &p1, LIRO::Point &p2, LIRO::Point &p3) {
        bool is_curve = false;
        double x12_dif = std::abs(p1.get_x_coord() - p2.get_x_coord());
        double x23_dif = std::abs(p2.get_x_coord() - p3.get_x_coord());
        if(x12_dif < 0.1 || x23_dif < 0.1){
            if(x12_dif <0.1 && x23_dif < 0.1)  // straight line
                return is_curve;
            else{
                is_curve = true;
                return is_curve;
            }
        }
        else{
            double dis12 = std::abs(std::sqrt(std::pow(p2.get_x_coord() - p1.get_x_coord(),2) +
                    std::pow(p2.get_y_coord() - p1.get_y_coord(),2)));
            double dis23 = std::abs(std::sqrt(std::pow(p3.get_x_coord() - p2.get_x_coord(),2) +
                                              std::pow(p3.get_y_coord() - p2.get_y_coord(),2)));
            double p12_product_p23 = (p2.get_x_coord()-p1.get_x_coord()) * (p3.get_x_coord() - p2.get_x_coord()) +
                    (p2.get_y_coord() - p1.get_y_coord()) * (p3.get_y_coord() -p2.get_y_coord());
            double cos_a = p12_product_p23 / (dis12 * dis23);
            ROS_ERROR_STREAM("dis12: " <<dis12 <<"  dis23: " <<dis23 <<"  p12_product_p23: " <<p12_product_p23 <<" cosa: " <<cos_a <<"\n");
            if(cos_a > 0.9){ // cos25
                return is_curve;
            }
        }
        return true;

    }

    void UwbCalibration::SaveDatas() {
        std::FILE* anchor_p_file;
        anchor_p_file =  std::fopen(save_uwb_path_.c_str(),"w");
        std::cout << "uwb_optimized size : " << uwb_optimized_.size() <<"\n";
        for( auto uwb : uwb_optimized_){
            std::fprintf(anchor_p_file,"%d %lf %lf %lf\n", uwb.first,
                         uwb.second.at(0), uwb.second.at(1), uwb.second.at(2));
        }
        std::fclose(anchor_p_file);
        ROS_ERROR_STREAM("uwb settled [" << uwb_optimized_.size() <<" ] positions are save to [ " << save_uwb_path_.c_str() <<"] ... \n");

    }


    double UwbCalibration::Distance(LIRO::Point p, LIRO::Point q) {
        return (std::abs(std::sqrt(std::pow(p.get_x_coord()-q.get_x_coord(),2) + std::pow(p.get_y_coord()-q.get_y_coord(),2))));
    }

    void UwbCalibration::Optimization() {
        for(auto data : uwb_data_){
            if(uwb_initial_.count(data.first) ==0)
                continue;
            double anchor_p[3] ={uwb_initial_.at(data.first).at(0), uwb_initial_.at(data.first).at(1), uwb_initial_.at(data.first).at(2), };
            ceres::Problem::Options options;
            ceres::Problem problem(options);
            ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
            double error;
            for(auto it : data.second){
//                error = std::abs(std::sqrt(std::pow(it.at(0) - uwb_initial_.at(data.first).at(0), 2) +
//                                           std::pow(it.at(1) - uwb_initial_.at(data.first).at(1), 2) +
//                                           std::pow(it.at(2) - uwb_initial_.at(data.first).at(2), 2)) - it.back());
                if(uwb_optimized_.count(data.first)==1)
                    error = std::abs(std::sqrt(std::pow(it.at(0) - uwb_optimized_.at(data.first).at(0), 2) +
                                                          std::pow(it.at(1) - uwb_optimized_.at(data.first).at(1), 2) +
                                                          std::pow(it.at(2) - uwb_optimized_.at(data.first).at(2), 2)) - it.back());
                else
                    error = std::abs(std::sqrt(std::pow(it.at(0) - uwb_initial_.at(data.first).at(0), 2) +
                                               std::pow(it.at(1) - uwb_initial_.at(data.first).at(1), 2) +
                                               std::pow(it.at(2) - uwb_initial_.at(data.first).at(2), 2)) - it.back());

                if(error > 0.7) // remove bad data
                    continue;
                ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<UWBRangeCost,1,3>(
                        new UWBRangeCost(it.at(0),it.at(1),it.at(2), it.at(3)));
                problem.AddResidualBlock(cost_function, loss_function,anchor_p);
            }
            ceres::Solver::Options solver_options;
            solver_options.linear_solver_type = ceres::DENSE_QR;
            solver_options.minimizer_progress_to_stdout = false;
            solver_options.max_num_iterations = 100;
            ceres::Solver::Summary summary;
            ceres::Solve(solver_options, &problem, &summary);
            std::cout << summary.BriefReport() <<std::endl;
            uwb_optimized_[data.first] = {anchor_p[0], anchor_p[1], anchor_p[2]};
        }
    }

    void UwbCalibration::AnchorsVisualization() {
        if(uwb_initial_.empty() || uwb_optimized_.empty())
            return;
        pcl::PointXYZI initial, settled;
        for(auto anchor : uwb_initial_){
            initial.x = uwb_initial_.at(anchor.first).at(0);
            initial.y = uwb_initial_.at(anchor.first).at(1);
            initial.z = uwb_initial_.at(anchor.first).at(2);
            anchors_initial_p_.push_back(initial);
        }
        for(auto anchor : uwb_optimized_){
            settled.x = uwb_optimized_.at(anchor.first).at(0);
            settled.y = uwb_optimized_.at(anchor.first).at(1);
            settled.z = uwb_optimized_.at(anchor.first).at(2);
            anchors_settled_p_.push_back(settled);
        }
        sensor_msgs::PointCloud2 anchor_initial_msgs, anchor_settled_msgs;
        pcl::toROSMsg(anchors_initial_p_,anchor_initial_msgs);
        anchor_initial_msgs.header.frame_id = "camera_init";
        anchor_initial_msgs.header.stamp = ros::Time::now();
        pub_anchor_initial_.publish(anchor_initial_msgs);

        pcl::toROSMsg(anchors_settled_p_,anchor_settled_msgs);
        anchor_settled_msgs.header.frame_id = "camera_init";
        anchor_settled_msgs.header.stamp = ros::Time::now();
        pub_anchor_optimized_.publish(anchor_settled_msgs);

        anchors_initial_p_.clear();
        anchors_settled_p_.clear();
    }


    void UwbCalibration::AnchorMeasurmentsVisualization() {
        uwb_measurments_msgs_.markers.resize(0);
        for(auto data : uwb_data_){
            if(uwb_optimized_.count(data.first)==1){
                visualization_msgs::Marker line_model, line_model_outliner;
                line_model.type = visualization_msgs::Marker::LINE_LIST;
                line_model.action = visualization_msgs::Marker::ADD;
                line_model.header.frame_id = "camera_init";
                line_model.header.stamp = ros::Time::now();
                line_model.color.a = 1.0;
                line_model.color.g = 0.9;
                line_model.id = data.first;
                line_model.scale.x = 0.02;

                line_model_outliner.type = visualization_msgs::Marker::LINE_LIST;
                line_model_outliner.action = visualization_msgs::Marker::ADD;
                line_model_outliner.header.frame_id = "camera_init";
                line_model_outliner.header.stamp = ros::Time::now();
                line_model_outliner.color.a = 1.0;
                line_model_outliner.color.r = 1.0;
                line_model_outliner.id = data.first + 100;
                line_model_outliner.scale.x = 0.02;

                geometry_msgs::Point anchor_p, kf_p;
                anchor_p.x = uwb_optimized_.at(data.first).at(0);
                anchor_p.y = uwb_optimized_.at(data.first).at(1);
                anchor_p.z = uwb_optimized_.at(data.first).at(2);
                for(auto it : data.second){
                    kf_p.x = it.at(0);
                    kf_p.y = it.at(1);
                    kf_p.z = it.at(2);
                    double error = std::abs(std::sqrt(std::pow(it.at(0) - uwb_optimized_.at(data.first).at(0), 2) +
                                                      std::pow(it.at(1) - uwb_optimized_.at(data.first).at(1), 2) +
                                                      std::pow(it.at(2) - uwb_optimized_.at(data.first).at(2), 2)) - it.back());
                    if(error > 0.7) { // outliners are marked red;
                        line_model_outliner.points.push_back(kf_p);
                        line_model_outliner.points.push_back(anchor_p);
                        uwb_measurments_outliners_msgs_.markers.push_back(line_model_outliner);
                    }
                    else{
                        line_model.points.push_back(kf_p);
                        line_model.points.push_back(anchor_p);
                        uwb_measurments_msgs_.markers.push_back(line_model);
                    };

                }
            }

        }
        pub_anchor_measurments_.publish(uwb_measurments_msgs_);
        pub_anchor_outliners_measurments_.publish(uwb_measurments_outliners_msgs_);
    }
}

int main(int argc, char** argv){
    ros::init(argc,argv,"anchor_optimization");
    LIRO::UwbCalibration uwb_opt;
    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        uwb_opt.Run();
        rate.sleep();
    }

}

