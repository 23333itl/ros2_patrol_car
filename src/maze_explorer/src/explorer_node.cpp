#include<memory>
#include<chrono>
#include<functional>
#include<cstdint>
#include<cmath>
#include<vector>
#include<array>
#include<utility>


#include"rclcpp/rclcpp.hpp"
#include"nav_msgs/msg/occupancy_grid.hpp"
#include"tf2/exceptions.hpp"
#include"tf2_ros/buffer.h"
#include"tf2_ros/transform_listener.hpp"

using namespace std::chrono_literals;

constexpr std::array<std::pair<int,int>,8> kDirs8={
    {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}}
};


//订阅slam 话题
// 主循环： 定时器驱动的状态机:1 等待nav2上线 2 空闲 3 正在导航 4 探索完成
//看门狗来判断是否长时间没有任何可以探索的地方 存图
//自由可走区域和未知区域的交界 前沿



class ExplorerNode :public rclcpp::Node{
public: 
    ExplorerNode():Node("maze_explorer"){
        timer_=create_wall_timer(2s,std::bind(&ExplorerNode::OnTimer,this));
        map_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            map_=msg;
                }
            );
        tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(get_logger(),"探索节点已启动");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    

private:
    int countFronttiers(nav_msgs::msg::OccupancyGrid& m)const {
        const int w=m.info.width,h=m.info.height;
        int c=0;
        for(int x=0;x<w;x++){
            for(int y=0;y<h;y++){
                if(!isFree(m,x,y)) continue;
                for(const auto& [dx,dy]:kDirs8){
                    const int nx=x+dx,ny=y+dy;
                    if(nx<0||nx>w||ny<0||ny>h) continue;
                    if(m.data[ny*w+nx]<0){
                        c++;
                        break;
                    }
                }
            }
        }
        return c;
    }
    //判断一个区域是否可走
    bool isFree(nav_msgs::msg::OccupancyGrid&  m,int x,int y) const{
        const int8_t v=m.data[y*m.info.width+x];
        return v>0&&v<50;
    }
    bool robotPose(double &x,double &y){
        try{
        const auto t=tf_buffer_->lookupTransform(
            "map","base_footprint",tf2::TimePointZero,200ms
        );
        x=t.transform.translation.x;
        y=t.transform.translation.y;
        return true;
        }
        catch(const tf2::TransformException& e){
            return false;
        }
    }

    int count_free(nav_msgs::msg::OccupancyGrid&  m) const{
        int c=0;
        for(const int8_t v:m.data){
            if(v>=0&&v<50){
                c++;
            }
        }
        return c;
    }

    void OnTimer(){
        if(!map_){
            RCLCPP_INFO(get_logger(),"等待地图");
            return;
        }
        double rx=0.0,ry=0.0;
        if(!robotPose(rx,ry)){
            RCLCPP_WARN(get_logger(),"查不到TF");
            return;
        }
        RCLCPP_INFO(get_logger(),"地图: %ux%u  可走区域面积: %d  前沿格子数: %d 机器人坐标(%.2f,%.2f)",
        map_->info.width,map_->info.height,count_free(*map_),countFronttiers(*map_),rx,ry);
        //RCLCPP_INFO(get_logger(),"ticker");
    }
};


int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ExplorerNode>());
    rclcpp::shutdown();
    return 0;
}
