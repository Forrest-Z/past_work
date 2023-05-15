
#include "lateral_stanley.hpp"



namespace GCT{
    Control::Control():nh_("~"){

        nh_.getParam("/stanley/k_v", K_V);
        nh_.getParam("/stanley/alpha", ALPHA);
        nh_.getParam("/tractor/wheelbase", WHEELBASE);

//        nh_.getParam("/k_v", K_V);
//        nh_.getParam("/alpha", ALPHA);
//        nh_.getParam("/wheelbase", WHEELBASE);

        sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/localization/Imu_incremental", 10, &Control::OdomCallback, this);
        sub_path_ = nh_.subscribe<nav_msgs::Path>("/GlobalPath", 10, &Control::PathCallback, this);

        pub_cmd_delta_ = nh_.advertise<geometry_msgs::Twist>("/cmd_delta", 100);
        pub_cross_error_ = nh_.advertise<geometry_msgs::Twist>("/cross_error", 100);

    }

    Control::~Control() {}

    void Control::PathCallback(const nav_msgs::PathConstPtr &path_msg) {

        mux_.lock();
        gp_v_.clear();
        Eigen::Vector3d Point;
        for (int i = 0; i < path_msg->poses.size(); i++)
        {
           Point(0) = path_msg->poses[i].pose.position.x;
           Point(1) = path_msg->poses[i].pose.position.y;
           Point(2) = path_msg->poses[i].pose.position.z;
           gp_v_.push_back(Point);
        }
//        std::cout <<"PathCallback---> :gp_vec size: " << gp_v_.size() << std::endl;

        mux_.unlock();
    }

    void Control::OdomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {

        //std::cout <<"x,y,z:" << odom_msg->pose.pose.position.x << "m "<<odom_msg->pose.pose.position.y <<"m "<<odom_msg->pose.pose.position.z<<"m " <<std::endl;
        if(gp_v_.empty())
            return;
        mux_.lock();
        bot_.x = odom_msg->pose.pose.position.x;
        bot_.y = odom_msg->pose.pose.position.y;

        bot_.siny = 2.0 * (odom_msg->pose.pose.orientation.w * odom_msg->pose.pose.orientation.z + odom_msg->pose.pose.orientation.x * odom_msg->pose.pose.orientation.y);
        bot_.cosy = 1.0 - 2.0 * (odom_msg->pose.pose.orientation.y * odom_msg->pose.pose.orientation.y + odom_msg->pose.pose.orientation.z * odom_msg->pose.pose.orientation.z);
        bot_.bot_vel = fabs(odom_msg->twist.twist.linear.x * cos(bot_.bot_theta) + odom_msg->twist.twist.linear.y * sin(bot_.bot_theta));


        double v_begin_x, v_begin_y, v_end_x, v_end_y;
        bot_.bot_theta = atan2(bot_.siny, bot_.cosy);

        bot_.bot_f_x = bot_.x + WHEELBASE * 0.5 * cos(normalize_angle(bot_.bot_theta));
        bot_.bot_f_y = bot_.y + WHEELBASE * 0.5 * sin(normalize_angle(bot_.bot_theta));
        bot_.x = bot_.bot_f_x;
        bot_.y = bot_.bot_f_y;

        geometry_msgs::Twist cross_err;

        m = m + 1;
        ep_sum = ep_sum + ep;
        ep_avg = ep_sum / m;

        double min_y,min_x,min_dis;
        int min_id;
        find_nearest_in_global_path(gp_v_, bot_.x, bot_.y, min_x, min_y, min_dis, min_id);
        int min_index = min_id + 1;
//        steer_path = atan2(x_p.poses[cp+1].pose.position.y - x_p.poses[cp].pose.position.y, x_p.poses[cp+1].pose.position.x - x_p.poses[cp].pose.position.x );
        double steer_path = atan2(gp_v_[min_index+1].y() - gp_v_[min_index].y(), gp_v_[min_index+1].x() - gp_v_[min_index].x());
        double path_yaw = normalize_angle(steer_path);
        double steer_err = normalize_angle(steer_path - bot_.bot_theta);

        std::cout <<"steer_path(not normalized):  " << steer_path * 180 / 3.14159 << "deg "<<std::endl;
        std::cout <<"bot_theta (not normalized):  " << bot_.bot_theta * 180 / 3.14159 << "deg " << std::endl;
        std::cout <<"steer_err: (steer_path - bot_.bot_theta_normalized) " << steer_err * 180 / 3.14159  << "deg "<< std::endl;
	std::cout <<"  " << std::endl;
        geometry_msgs::Twist cmd;



        ep = min_dis;
        /*     cross2 = [(x - data1.poses[cp].pose.position.x), (y - data1.poses[cp].pose.position.y)];
               cross = [math.sin(path_yaw), -math.cos(path_yaw)];*/
//      cross_dot = np.dot(cross2,cross);
        double cross_dot = (bot_.x - gp_v_[min_index].x()) * sin(path_yaw) + (bot_.y - gp_v_[min_index].y()) * (-cos(path_yaw));
        ep = sign(cross_dot) * ep;
 	std::cout <<"x,y:                   " << "(" <<odom_msg->pose.pose.position.x << "m"<<", "<<odom_msg->pose.pose.position.y <<"m "<<") "<<std::endl;
        std::cout <<"path_min_x,path_min_y: " << "(" <<min_x << "m"<<", "<<min_y <<"m "<<") "<<std::endl;
        std::cout <<"later_error(ep):  " << ep << "m "<<std::endl;

        cross_err.linear.x = ep;
        cross_err.angular.x = ep_max;
        cross_err.angular.y = ep_avg;

        double tar_x,tar_y,w_cos,w_sin;
/*      siny = 2.0 * (x_p.poses[cp].pose.orientation.w * x_p.poses[cp].pose.orientation.z + x_p.poses[cp].pose.orientation.x * x_p.poses[cp].pose.orientation.y);
        cosy = 1.0 - 2.0 * (x_p.poses[cp].pose.orientation.y * x_p.poses[cp].pose.orientation.y + x_p.poses[cp].pose.orientation.z * x_p.poses[cp].pose.orientation.z);*/
        tar_x = gp_v_[min_index].x();
        tar_y = gp_v_[min_index].y();
        v_begin_x = bot_.bot_f_x;
        v_begin_y = bot_.bot_f_y;
        v_end_x = v_begin_x + cos(normalize_angle(bot_.bot_theta));
        v_end_y = v_begin_y + sin(normalize_angle(bot_.bot_theta));

        /*v_vec = np.array([v_end_x - v_begin_x, v_end_y - v_begin_y, 0.0]);
        w_vec = np.array([tar_x - v_begin_x,  tar_y - v_begin_y, 0.0]);

        w_cos = np.dot(w_vec, v_vec) / (np.linalg.norm(w_vec) * np.linalg.norm(v_vec));*/
        w_cos = ((tar_x - v_begin_x) * (v_end_x - v_begin_x) + (tar_y - v_begin_y) * (v_end_y - v_begin_y)) / sqrt(pow((tar_x - v_begin_x),2) + pow((tar_y - v_begin_y),2)) * sqrt(pow((v_end_x - v_begin_x),2) + pow((v_end_y - v_begin_y),2));
        w_sin = sqrt(1- w_cos * w_cos);
        double ep_lat = ep *  w_sin;

        double tan_1 = 0;
        double delta = 0;
        if (bot_.bot_vel < 0.1)
            {
		tan_1 = 0.0;
		std::cout <<"(bot_v<0.01),   " << "bot_v:  " << bot_.bot_vel << "m/s "<<std::endl;
	    }
        else
            {
	    tan_1 = atan(K_V * ep_lat /bot_.bot_vel);
	    std::cout <<"(bot_v > = 0.01),   " << "bot_v:  " << bot_.bot_vel << "m/s "<<std::endl;
	    }

	std::cout <<"k_v:    (for calculating tan_1):  " << K_V <<std::endl;
        std::cout <<"ep_lat: (for calculating tan_1):  " << ep << "m "<<std::endl;
        std::cout <<"bot_v:  (for calculating tan_1):  " << bot_.bot_vel << "m/s "<<std::endl;
        std::cout <<"tan_1(angle_according_to_lateral_ep):  " << tan_1 * 180 /3.14159 << "deg "<< std::endl;
        delta = (1*steer_err + tan_1);

        delta = delta * 180 / 3.1415926;
        std::cout <<"delta_stanley_original:  " << delta << "deg "<<std::endl;
        delta = min(47,max(-47,delta));
        std::cout <<"delta_stanley_after_MaxProcessing:  " << delta <<"deg "<<std::endl;
	std::cout <<"    " << std::endl;
	std::cout <<"    " << std::endl;
        cmd.angular.z = delta;
        cross_err.linear.y = steer_err;
        cross_err.linear.z = 0;

        pub_cmd_delta_.publish(cmd);
        pub_cross_error_.publish(cross_err);

        mux_.unlock();
    }

    double Control::normalize_angle(double angle)
    {
        if (angle > 3.1415926)
            angle -= 2.0 * 3.1415926;

        if (angle < -3.1415926)
            angle += 2.0 * 3.1415926;

        return angle;
    }

    void Control::find_nearest_in_global_path(std::vector<Eigen::Vector3d> gp_vec, double x, double y, double& min_x, double& min_y, double& min_dis, int& min_id)
    {
    //std::cout << " " << gp_vec.front().x()<<" "<< gp_vec.front().y()<< " "<< x<< " "<< y<< std::endl;
    min_dis = calc_dis(gp_vec.front().x(), gp_vec.front().y(), x, y);

    int i = 0;
//    std::cout <<"find_nearest_in_global_path()---> :gp_vec size: " << gp_vec.size() << std::endl;
    for(auto point : gp_vec)
    {
//        std::cout << "1.1" << std::endl;
        double dis = calc_dis(point.x(), point.y(), x, y);
//        std::cout << "1.2" << std::endl;
        if (dis < min_dis)
        {
//            std::cout << "1.3" << std::endl;
            min_dis = dis;
            min_x = point.x();
            min_y = point.y();
            min_id = i;
//            std::cout << "1.4" << std::endl;
        }
        i++;

    }
//        std::cout << "min_id: "<< min_id << std::endl;

        return;
    }

    void Control::Run() 
        {

        }

}



int main(int argc, char** argv){
    ros::init(argc,argv,"path_tracking");
    GCT::Control CON;
    ros::Rate rate(200);
    while(ros::ok()){
        ros::spinOnce();
        CON.Run();
        rate.sleep();
    }
    return 0;
}
