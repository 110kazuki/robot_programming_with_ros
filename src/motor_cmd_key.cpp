/*
Reference : https://hotnews8.net/programming/tricky-code/c-code03

*/

#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "cpp_minimum_node");
    ros::NodeHandle n;

    while (1) {
        if (kbhit()) {
            printf("'%c'を押しました。\n", getchar());
            break;
        }
    }

    return 0;
}