#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <csignal>
#include <fcntl.h>
#include <libserial/SerialPort.h>

class ImuXG6000Node : public rclcpp::Node
{
    public:
        ImuXG6000Node(): Node("imu_xg6000_node")
        {
            this->declare_parameter<std::string>("port_name", "/dev/ttyIMU");
            this->declare_parameter<int>("baudrate", 38400);
            this->declare_parameter<std::string>("frame_id", "imu_link");
            this->declare_parameter<double>("rate", 50.0);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<uint32_t>();

            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            try
            {
                ser_.Open(port_name);
            }
            catch(LibSerial::OpenFailed &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "Exceptions: \033[0;91m%s\033[0m", e.what());
                RCLCPP_ERROR(rclcpp::get_logger("MinibotSystemHardware"), "\033[0;91mFailed to open port\033[0m [\033[0;92m%s\033[0m]...", port_name.c_str());
                exit(-1);
            }

            auto ser_baudrate = LibSerial::BaudRate::BAUD_9600;
            switch(baudrate)
            {
                case 9600:   ser_baudrate = LibSerial::BaudRate::BAUD_9600;   break;
                case 19200:  ser_baudrate = LibSerial::BaudRate::BAUD_19200;  break;
                case 38400:  ser_baudrate = LibSerial::BaudRate::BAUD_38400;  break;
                case 57600:  ser_baudrate = LibSerial::BaudRate::BAUD_57600;  break;
                case 115200: ser_baudrate = LibSerial::BaudRate::BAUD_115200; break;
            }
            ser_.SetBaudRate(ser_baudrate);
            ser_.FlushIOBuffers();
            rclcpp::sleep_for(std::chrono::milliseconds(100));


            pub_imu_msg_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", rclcpp::SystemDefaultsQoS());
            imu_msg_.header.frame_id = this->get_parameter("frame_id").get_parameter_value().get<std::string>();
            imu_msg_.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
            imu_msg_.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
            imu_msg_.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};


            enable_thread_ = true;
            thread_sensor_ = std::make_shared<std::thread>(std::bind(&ImuXG6000Node::thread_sensor_func, this));

            // auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            // timer_ = this->create_wall_timer(period, std::bind(&ImuXG6000Node::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }
        ~ImuXG6000Node()
        {
            enable_thread_ = false;
            thread_sensor_->join();

            if(ser_.IsOpen())
            {
                ser_.Close();
            }
        }

    private:
        // void timer_callback()
        // {

        // }

        void thread_sensor_func(void)
        {
            uint8_t current_state = 0;
            uint8_t current_index = 0;
            bool is_stream_started = false;

            ser_.Write("$MIB,RESET*87\r");
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            while(rclcpp::ok() && enable_thread_)
            {
                if(current_state == 0 && ser_.GetNumberOfBytesAvailable() >= 1)
                {
                    char header_byte = 0;
                    ser_.ReadByte(header_byte, 100);

                    if(header_byte == 0xA6)
                    {
                        current_state = 1;
                    }
                }
                else if(current_state == 1 && ser_.GetNumberOfBytesAvailable() >= 1)
                {
                    char header_byte = 0;
                    ser_.ReadByte(header_byte, 100);

                    if(header_byte == 0xA6)
                    {
                        current_state = 2;
                    }
                }
                else if(current_state == 2 && ser_.GetNumberOfBytesAvailable() >= 1)
                {
                    char index_byte = 0;
                    ser_.ReadByte(index_byte, 100);

                    if(!is_stream_started && index_byte == 1)
                    {
                        current_state = 3;
                        is_stream_started = true;
                        RCLCPP_INFO(this->get_logger(), "start streaming...");
                        current_index = 1;
                    }
                    else if(is_stream_started)
                    {
                        current_state = 3;
                        current_index = index_byte;
                    }
                    else
                    {
                        current_state = 0;
                        // RCLCPP_INFO(this->get_logger(), "waste index %d...", index_byte);
                    }
                }
                else if(current_state == 3 && ser_.GetNumberOfBytesAvailable() >= 23)
                {
                    std::vector<uint8_t> recv_buf(24, 0);
                    ser_.Read(recv_buf, 23, 100);


                    double r = (int16_t)((recv_buf[1] << 8) + recv_buf[0]) / 100.0 * M_PI / 180.0;
                    double p = (int16_t)((recv_buf[3] << 8) + recv_buf[2]) / 100.0 * M_PI / 180.0;
                    double y = (int16_t)((recv_buf[5] << 8) + recv_buf[4]) / 100.0 * M_PI / 180.0;

                    tf2::Quaternion q;
                    q.setRPY(r, -p, -y);

                    imu_msg_.orientation.x = q.getX();
                    imu_msg_.orientation.y = q.getY();
                    imu_msg_.orientation.z = q.getZ();
                    imu_msg_.orientation.w = q.getW();

                    imu_msg_.angular_velocity.x = (int16_t)((recv_buf[7] << 8) + recv_buf[6]) / 100.0 * M_PI / 180.0;
                    imu_msg_.angular_velocity.y = (int16_t)((recv_buf[9] << 8) + recv_buf[8]) / 100.0 * M_PI / 180.0;
                    imu_msg_.angular_velocity.z = (int16_t)((recv_buf[11] << 8) + recv_buf[10]) / 100.0 * M_PI / 180.0;

                    imu_msg_.linear_acceleration.x = (int16_t)((recv_buf[13] << 8) + recv_buf[12]) / 1000.0 * 9.80665;
                    imu_msg_.linear_acceleration.y = (int16_t)((recv_buf[15] << 8) + recv_buf[14]) / 1000.0 * 9.80665;
                    imu_msg_.linear_acceleration.z = (int16_t)((recv_buf[17] << 8) + recv_buf[16]) / 1000.0 * 9.80665;

                    imu_msg_.header.stamp = this->now();
                    pub_imu_msg_->publish(imu_msg_);

                    // printf("%d = ", current_index);
                    // for(size_t i = 0; i < 23; i++)
                    // {
                    //     printf("%2X ", recv_buf[i]);
                    // }
                    // printf("\r\n");

                    current_state = 0;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

    private:
        std::shared_ptr<std::thread> thread_sensor_;
        bool enable_thread_;
        rclcpp::TimerBase::SharedPtr timer_;
        LibSerial::SerialPort ser_;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> pub_imu_msg_;
        sensor_msgs::msg::Imu imu_msg_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuXG6000Node>());
    rclcpp::shutdown();
    return 0;
}
