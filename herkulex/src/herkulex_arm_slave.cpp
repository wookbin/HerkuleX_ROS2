//herkulex_arm_slave node

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
#include "herkulex/srv/herkulex_angle_move.hpp" //SRV
#include "herkulex/srv/herkulex_sjog_move.hpp" //SRV


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

int m_iTotal_Axis = 7;
double m_dJoint_angle[7] = {0.0, };
int m_iTarget_Position[7] = {0, };


class HerkuleX_Arm_Slave : public rclcpp::Node 
{
    public:
        HerkuleX_Arm_Slave() : Node("herkulex_arm_slave_node")
        {
            printf("[HerkuleX] herkulex_arm_slave_node init!\n");

            //Publish/////////////////////////////////////////////////////////////////////////////////////////////
            // Joint State Publisher
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
            // TF Broadcaster
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            //Subscribe///////////////////////////////////////////////////////////////////////////////////////////
            RAM_Array_sub_1 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "slave/Info_RAM_ID_1", 10, std::bind(&HerkuleX_Arm_Slave::RAM_SubCallback1, this, _1));
            RAM_Array_sub_2 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "slave/Info_RAM_ID_2", 10, std::bind(&HerkuleX_Arm_Slave::RAM_SubCallback2, this, _1));
            RAM_Array_sub_3 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "slave/Info_RAM_ID_3", 10, std::bind(&HerkuleX_Arm_Slave::RAM_SubCallback3, this, _1));
            RAM_Array_sub_4 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "slave/Info_RAM_ID_4", 10, std::bind(&HerkuleX_Arm_Slave::RAM_SubCallback4, this, _1));
            RAM_Array_sub_5 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "slave/Info_RAM_ID_5", 10, std::bind(&HerkuleX_Arm_Slave::RAM_SubCallback5, this, _1));
            RAM_Array_sub_6 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "slave/Info_RAM_ID_6", 10, std::bind(&HerkuleX_Arm_Slave::RAM_SubCallback6, this, _1));
            RAM_Array_sub_7 = this->create_subscription<herkulex::msg::HerkulexInfoRam>(
                "slave/Info_RAM_ID_7", 10, std::bind(&HerkuleX_Arm_Slave::RAM_SubCallback7, this, _1));
            
            //Client list///////////////////////////////////////////////////////////////////////////////////////
		    Register_client = this->create_client<herkulex::srv::HerkulexRegisterCommand>("slave/Register_cmd");
            AngleMove_client = this->create_client<herkulex::srv::HerkulexAngleMove>("slave/AngleMove_cmd");
            SjogMove_client = this->create_client<herkulex::srv::HerkulexSjogMove>("master/SjogMove_cmd");
            
            //Timer
            read_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&HerkuleX_Arm_Slave::RegisterReadCallback, this));
        }

        void HerkulexRegister_Callback(rclcpp::Client<herkulex::srv::HerkulexRegisterCommand>::SharedFuture future)
        {
            //RCLCPP_INFO(this->get_logger(), "response HerkulexRegister_Callback");

        }
        void AngleMove_Callback(rclcpp::Client<herkulex::srv::HerkulexAngleMove>::SharedFuture future)
        {
            //RCLCPP_INFO(this->get_logger(), "response AngleMove_Callback");

        }

        void RAM_SubCallback1(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dJoint_angle[0] = Get_CountToRadian(0, RAM_msg.calibratedposition);
            m_iTarget_Position[0] = RAM_msg.calibratedposition;
        }

        void RAM_SubCallback2(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dJoint_angle[1] = Get_CountToRadian(0, RAM_msg.calibratedposition);
            m_iTarget_Position[1] = RAM_msg.calibratedposition;
        }

        void RAM_SubCallback3(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dJoint_angle[2] = Get_CountToRadian(0, RAM_msg.calibratedposition);
            m_iTarget_Position[2] = RAM_msg.calibratedposition;
        }

        void RAM_SubCallback4(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dJoint_angle[3] = Get_CountToRadian(0, RAM_msg.calibratedposition);
            m_iTarget_Position[3] = RAM_msg.calibratedposition;
        }

        void RAM_SubCallback5(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dJoint_angle[4] = Get_CountToRadian(0, RAM_msg.calibratedposition);
            m_iTarget_Position[4] = RAM_msg.calibratedposition;
        }

        void RAM_SubCallback6(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dJoint_angle[5] = Get_CountToRadian(0, RAM_msg.calibratedposition);
            m_iTarget_Position[5] = RAM_msg.calibratedposition;
        }

        void RAM_SubCallback7(const herkulex::msg::HerkulexInfoRam::SharedPtr msg) 
        {
            const herkulex::msg::MsgHerkulexRam &RAM_msg = msg->pherkulex_ram[0];
            m_dJoint_angle[6] = Get_CountToRadian(0, RAM_msg.calibratedposition);
            m_iTarget_Position[6] = RAM_msg.calibratedposition;
        }



        int Set_RadianToCount(int iModel_num, double dTarget_rad)
        {
            int m_iTarget_count = 0;
            double m_dTarget_deg = (dTarget_rad * 180.0)/M_PI;
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

            //Degree -> Radian
            dTarget_rad = (dTarget_deg * M_PI)/180.0;

            return dTarget_rad;
        }

        void ServoOff()
        {
            //to do...
            auto request = std::make_shared<herkulex::srv::HerkulexRegisterCommand::Request>();

            for(int i=0; i<m_iTotal_Axis; i++)
            {
                request->command = "SERVO_OFF";
                request->model_num = 0;
                request->id = i+1; 
                request->addr = RAM_TORQUE_CONTROL;
                request->value = 0;
                // Call the service
                auto result = Register_client->async_send_request(request);
                usleep(10000);
            }

        }
        
    
    private:

        //function//////////////////////////////////////////////////////////////////////////////////////////////

        // void publishJointStateAndTF() 
        // {
        //     // Publish Joint State
        //     auto joint_state = sensor_msgs::msg::JointState();
        //     joint_state.header.stamp = this->now();

        //     joint_state.name.push_back("joint1");
        //     joint_state.name.push_back("joint2");
        //     joint_state.name.push_back("joint3");
        //     joint_state.name.push_back("joint4");
        //     joint_state.name.push_back("joint5");
        //     joint_state.name.push_back("joint6");
        //     joint_state.name.push_back("joint7");

        //     joint_state.position.push_back(m_dJoint_angle[0]);
        //     joint_state.position.push_back(m_dJoint_angle[1]);
        //     joint_state.position.push_back(m_dJoint_angle[2]);
        //     joint_state.position.push_back(m_dJoint_angle[3]);
        //     joint_state.position.push_back(m_dJoint_angle[4]);
        //     joint_state.position.push_back(m_dJoint_angle[5]);
        //     joint_state.position.push_back(m_dJoint_angle[6]);

        //     joint_state_pub_->publish(joint_state);

        // }

        void RegisterReadCallback() 
        {
            //position read//
            auto request = std::make_shared<herkulex::srv::HerkulexRegisterCommand::Request>();

            for(int i=0; i<m_iTotal_Axis; i++)
            {
                request->command = "RAM_RegisterData_Read";
                request->model_num = 0; //DRS-0101
                request->id = i+1; 
                request->addr = RAM_CALIBRATED_POSITION;
                request->value = 1;
                // Call the service
                auto result = Register_client->async_send_request(request);
                usleep(1000);
            }
            //publishJointStateAndTF();

            //Master Arm move Command// //m_iTarget_Position[0] ~ [6]
            auto request2 = std::make_shared<herkulex::srv::HerkulexSjogMove::Request>();
            for(int j=0; j<m_iTotal_Axis; j++)
            {
                request2->id_arr.push_back(j+1);
                request2->led_arr.push_back(1);
                request2->targetposition_arr.push_back(m_iTarget_Position[j]);
            }
            request2->playtime = 30;
            request2->total_axis = 7;
            
            auto result2 = SjogMove_client->async_send_request(request2);

            /*
            int8[]  id_arr
            int8[]  led_arr
            int16[] targetposition_arr
            int16 playtime
            int16 total_axis
            ---
            string command 
            bool command_result
            */

            
        }


        /////////////////////////////////////////////////////////////////////////////////////////////////////
        //// timer//
        rclcpp::TimerBase::SharedPtr read_timer_;
        //// Client_Service/////////////////////////////////////////////////////////////////////////////////
	    rclcpp::Client<herkulex::srv::HerkulexRegisterCommand>::SharedPtr Register_client;
        rclcpp::Client<herkulex::srv::HerkulexAngleMove>::SharedPtr AngleMove_client;
        rclcpp::Client<herkulex::srv::HerkulexSjogMove>::SharedPtr SjogMove_client;
        //// Publish
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

        //// Subscribe
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_1;
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_2;
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_3;
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_4;
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_5;
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_6;
        rclcpp::Subscription<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_sub_7;


        //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);

	std::signal(SIGINT, signal_handler);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto node = std::make_shared<HerkuleX_Arm_Slave>();
    sleep(1);
    node->ServoOff();
    printf("[Arm_Slave]: Servo Off Call !!");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
