#include <iostream>
#include <UnOrderSortingServer.h>

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "UnOrderSortingNode");
    ros::NodeHandle n;
    ros::AsyncSpinner as(3);
    as.start();

    UnOrderSortingServer unOrderSortingServer(&n);
    unOrderSortingServer.start();
    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();
    return 0;
}
