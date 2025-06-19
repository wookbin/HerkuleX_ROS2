//herkulex_pantilt_node

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <memory>
#include <time.h>

#include "herkulex.h"
#include "herkulex/msg/msg_herkulex_ram.hpp" //MSG
#include "herkulex/msg/herkulex_info_ram.hpp" //MSG_Array
#include "herkulex/srv/herkulex_register_command.hpp" //SRV
#include "herkulex/srv/herkulex_position_move.hpp" //SRV
#include "herkulex/srv/pan_angle_move.hpp" //SRV
#include "herkulex/srv/tilt_angle_move.hpp" //SRV


using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

/********************************************************************************/
std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}
/********************************************************************************/

int m_iTotal_Axis = 2; //{1,2}
double m_dPan_angle = 0.0;
double m_dTilt_angle = 0.0;

double m_dGear_ratio = -5.0; // 5:1 gear ratio


class HerkuleX_PANTILT : public rclcpp::Node 
{
    public:
        HerkuleX_PANTILT() : Node("herkulex_pantilt_node")
        {
            printf("[HerkuleX] herkulex_pantilt_node init!\n");

            //Publish/////////////////////////////////////////////////////////////////////////////////////////////
            // Joint State Publisher
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
            // TF Broadcaster
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            //Subscribe///////////////////////////////////////////////////////////////////////////////////////////
            RAM_Array_sub_1 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "Info_RAM_ID_1", 10, std::bind(&HerkuleX_PANTILT::RAM_SubCallback1, this, _1));
            RAM_Array_sub_2 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "Info_RAM_ID_2", 10, std::bind(&HerkuleX_PANTILT::RAM_SubCallback2, this, _1));
 
            //Client list///////////////////////////////////////////////////////////////////////////////////////
		    Register_client = this->create_client<herkulex::srv::HerkulexRegisterCommand>("Register_cmd");
            PositionMove_client = this->create_client<herkulex::srv::HerkulexPositionMove>("PositionMove_cmd");
            //Service list///////////////////////////////////////////////////////////////////////////////////////
            PanAngleMove_service = create_service<herkulex::srv::PanAngleMove>(
                "PanMove_cmd", 
            std::bind(&HerkuleX_PANTILT::PanMove_Command, this, std::placeholders::_1, std::placeholders::_2));
            
            TiltAngleMove_service = create_service<herkulex::srv::TiltAngleMove>(
                "TiltMove_cmd", 
            std::bind(&HerkuleX_PANTILT::TiltMove_Command, this, std::placeholders::_1, std::placeholders::_2));
            

            //Timer
            read_timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&HerkuleX_PANTILT::RegisterReadCallback, this));
        }
        
        //add function//

        void HerkulexRegister_Callback(rclcpp::Client<herkulex::srv::HerkulexRegisterCommand>::SharedFuture future)
        {
            //RCLCPP_INFO(this->get_logger(), "response HerkulexRegister_Callback");
        }
        void HerkulexPositionMove_Callback(rclcpp::Client<herkulex::srv::HerkulexPositionMove>::SharedFuture future)
        {
            //RCLCPP_INFO(this->get_logger(), "response AngleMove_Callback");
        }

        void RAM_SubCallback1(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dPan_angle = Get_CountToRadian(6, RAM_msg.calibratedposition);
            //printf("[RAM_SubCallback1] m_dPan_angle: %.3f \n", m_dPan_angle);
        }
        void RAM_SubCallback2(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dTilt_angle = Get_CountToRadian(6, RAM_msg.calibratedposition);
            //printf("[RAM_SubCallback2] m_dTilt_angle: %.3f \n", m_dTilt_angle);
        }

        int Set_RadianToCount(int iModel_num, double dTarget_rad)
        {
            int m_iTarget_count = 0;
            double m_dTarget_deg = (dTarget_rad * m_dGear_ratio * 180.0)/M_PI;
            
            switch (iModel_num)
			{
				case 0:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.325) + 512; //0101
					break;
				case 1:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0102
					break;
				case 2:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.325) + 512; //0201
					break;
				case 3:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.163) + 1024; //0401
					break;
				case 4:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0402
					break;
				case 5:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.163) + 1024; //0601
					break;
				case 6:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0602
					break;
			}

            return m_iTarget_count;
        }
        
        int Set_DegreeToCount(int iModel_num, double dTarget_deg)
        {
            int m_iTarget_count = 0;
            double m_dTarget_deg = dTarget_deg * m_dGear_ratio;
            switch (iModel_num)
			{
				case 0:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.325) + 512; //0101
					break;
				case 1:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0102
					break;
				case 2:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.325) + 512; //0201
					break;
				case 3:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.163) + 1024; //0401
					break;
				case 4:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0402
					break;
				case 5:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.163) + 1024; //0601
					break;
				case 6:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0602
					break;
			}

            return m_iTarget_count;
        }

        double Get_CountToRadian(int iModel_num, int iTarget_count)
        {
            double dTarget_deg = 0.0;
            double dTarget_rad = 0.0;
            //Count -> Degree
            switch (iModel_num)
			{
				case 0:
                    dTarget_deg = (iTarget_count - 512.0) * 0.325; //0101
					break;
				case 1:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0102
					break;
				case 2:
                    dTarget_deg = (iTarget_count - 512.0) * 0.325; //0201
					break;
				case 3:
                    dTarget_deg = (iTarget_count - 1024.0) * 0.163; //0401
					break;
				case 4:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0402
					break;
				case 5:
                    dTarget_deg = (iTarget_count - 1024.0) * 0.163; //0601
					break;
				case 6:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0602
					break;
			}

            //Degree -> Radian , include gear ratio
            dTarget_rad = (dTarget_deg * M_PI)/(180.0 * m_dGear_ratio);

            return dTarget_rad;
        }
        
        bool PanMove_Command(const std::shared_ptr<herkulex::srv::PanAngleMove::Request>  req, 
                             const std::shared_ptr<herkulex::srv::PanAngleMove::Response> rep)
        {
			bool bResult = false;
			int m_target_position = Set_DegreeToCount(6, req->target_angle);
			/*target_angle, playtime*/
			auto request = std::make_shared<herkulex::srv::HerkulexPositionMove::Request>();
			request->id = 1;
			request->led = 1; 
			request->playtime= req->playtime;
			request->targetposition= m_target_position; 
			request->jogmode= 0;
			// Call the service
			auto result = PositionMove_client->async_send_request(request);
			
			bResult = true;
			return bResult;
		}
		
		bool TiltMove_Command(const std::shared_ptr<herkulex::srv::TiltAngleMove::Request>  req, 
                              const std::shared_ptr<herkulex::srv::TiltAngleMove::Response> rep)
        {
			bool bResult = false;
			int m_target_position = Set_DegreeToCount(6, req->target_angle);
			/*target_angle, playtime*/
			auto request = std::make_shared<herkulex::srv::HerkulexPositionMove::Request>();
			request->id = 2;
			request->led = 1; 
			request->playtime= req->playtime;
			request->targetposition= m_target_position; 
			request->jogmode= 0;
			// Call the service
			auto result = PositionMove_client->async_send_request(request);
			
			bResult = true;
			return bResult;
			
		}
        
    
    private:

        //function//////////////////////////////////////////////////////////////////////////////////////////////

        void publishJointStateAndTF() 
        {
            // Publish Joint State
            auto joint_state = sensor_msgs::msg::JointState();
            joint_state.header.stamp = this->now();
            
            joint_state.name.push_back("joint1");
            joint_state.name.push_back("joint2");
            
            joint_state.position.push_back(m_dPan_angle);
            joint_state.position.push_back(m_dTilt_angle);
            
            joint_state_pub_->publish(joint_state);

        }

        void RegisterReadCallback() 
        {
            //to do...
            auto request = std::make_shared<herkulex::srv::HerkulexRegisterCommand::Request>();
			
			for (int i=0; i< m_iTotal_Axis; i++)
			{
				request->command = "RAM_RegisterData_Read";
				request->model_num = 6; //DRS-0602
				request->id = i+1; 
				request->addr = RAM_CALIBRATED_POSITION;
				request->value = 1;
				// Call the service
				auto result = Register_client->async_send_request(request);
				usleep(1000);
			}
            
            publishJointStateAndTF();

        }
        

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        //// timer//
        rclcpp::TimerBase::SharedPtr read_timer_;
        //// Client_Service/////////////////////////////////////////////////////////////////////////////////
	    rclcpp::Client<herkulex::srv::HerkulexRegisterCommand>::SharedPtr Register_client;
        rclcpp::Client<herkulex::srv::HerkulexPositionMove>::SharedPtr PositionMove_client;
        //// Service////////////////////////////////////////////////////////////////////////////////////////
        rclcpp::Service<herkulex::srv::PanAngleMove>::SharedPtr PanAngleMove_service;
        rclcpp::Service<herkulex::srv::TiltAngleMove>::SharedPtr TiltAngleMove_service;
        //// Publish
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        //// Subscribe
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_1;
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_2;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);

	std::signal(SIGINT, signal_handler);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto node = std::make_shared<HerkuleX_PANTILT>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
