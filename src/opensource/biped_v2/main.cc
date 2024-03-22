#include <HighlyDynamicRobot.h>

int main(int argc, char *argv[])
{
    // sched_process(0);
    HighlyDynamic::HighlyDynamicRobot robot;
    robot.doMain(argc, argv);

    return 0;
}
