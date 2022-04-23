/*
Reference : https://hotnews8.net/programming/tricky-code/c-code03

*/

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <signal.h>

#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_SPACE 0x20
#define KEYCODE_Q     0x71

class KeyboardReader
{
public:
    KeyboardReader():
        kfd(0)
    {
        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }
    
    void readOne(char * c)
    {
        int rc = read(kfd, c, 1);
        if (rc < 0)
        {
        throw std::runtime_error("read failed");
        }
    }

    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    int kfd;
    struct termios cooked;
};

KeyboardReader input;

void quit(int sig)
{
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}


class MotorCmd
{
public:
    MotorCmd();
    void keyLoop(); 

private:
    ros::NodeHandle nh_;
    int motor_cmd;
    ros::Publisher pub;
  
};

MotorCmd::MotorCmd():
    motor_cmd(0)
{
    pub = nh_.advertise<std_msgs::Int16>("motor_ctrl", 1);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "motor_cmd_key");
    MotorCmd motor_cmd;

    signal(SIGINT,quit);

    motor_cmd.keyLoop();
    quit(0);
    
    return 0;
}

void MotorCmd::keyLoop()
{
    char c;
    bool dirty=false;

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle. 'q' to quit.");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    for(;;)
    {
        // get the next event from the keyboard  
        try
        {
            input.readOne(&c);
        }
        catch (const std::runtime_error &)
        {
            perror("read():");
            return;
        }

        ROS_DEBUG("value: 0x%02X\n", c);
    
        switch(c)
        {
        case KEYCODE_UP:
            ROS_DEBUG("UP");
            if (motor_cmd<100)
            {
                motor_cmd = motor_cmd + 5;
            }
            dirty = true;
            break;
        case KEYCODE_DOWN:
            ROS_DEBUG("DOWN");
            if (motor_cmd>-100)
            {
                motor_cmd = motor_cmd - 5;
            }
            dirty = true;
            break;
        case KEYCODE_SPACE:
            ROS_DEBUG("SPACE");
            motor_cmd = 0;
            dirty = true;
            break;
        case KEYCODE_Q:
            ROS_DEBUG("quit");
            return;
        }
    

        std_msgs::Int16 motor_cmd_msg;
        motor_cmd_msg.data = motor_cmd;
        if(dirty ==true)
        {
            pub.publish(motor_cmd_msg);    
            dirty=false;
        }
    }

    return;
}