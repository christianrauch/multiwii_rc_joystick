#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <dirent.h>

#include <map>
#include <string>
#include <cstring>
#include <iostream>

#include <libevdev/libevdev.h>

#include <ros/console.h>
#include <ros/ros.h>

std::map<std::string, std::string> getEventList() {
    static const std::string event_dir = "/dev/input";
    static const std::string event_file = "event";

    std::map<std::string, std::string> evdevices;

    DIR *dinp = opendir(event_dir.c_str());
    struct dirent *dir;
    while ((dir = readdir(dinp)) != NULL) {
        if(std::strncmp(dir->d_name, event_file.c_str(), event_file.size())!=0)
            continue;
        const std::string evpath = event_dir+"/"+std::string(dir->d_name);
        const int fd = open(evpath.c_str(), O_RDONLY|O_NONBLOCK);
        if(fd<0)
            continue;
        char evdev_name[256];
        ioctl(fd, EVIOCGNAME(sizeof(evdev_name)), evdev_name);
        evdevices[evpath] = std::string(evdev_name);
        close(fd);
    }
    free(dir);
    closedir(dinp);

    return evdevices;
}

int main(int argc, char **argv) {
    // get list of event devices
    const std::map<std::string, std::string> evdevs = getEventList();
    if(evdevs.size()==0) {
        ROS_ERROR_STREAM("no devices available");
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "rc_joystick");
    ros::NodeHandle n("~");

    // read parameters
    std::string device;
    n.param<std::string>("device", device, "");

    if(device.size()==0) {
        // use first available device
        device = evdevs.begin()->first;
    }
    else {
        if(evdevs.count(device)==0) {
            // no such evdev device
            ROS_ERROR_STREAM("device "+device+" is not an event device");
            std::cout << "Available devices:" << std::endl;
            for(const auto &p : evdevs) {
                std::cout << p.second << " (" << p.first << ")" << std::endl;
            }
            exit(EXIT_FAILURE);
        }
        else {
            ROS_INFO_STREAM("using "+evdevs.at(device));
        }
    }

    struct libevdev *dev = NULL;
    int fd = open(evdevs.begin()->first.c_str(), O_RDONLY|O_NONBLOCK);
    if(libevdev_new_from_fd(fd, &dev) < 0) {
        ROS_ERROR_STREAM("libevdev error");
        close(fd);
        return EXIT_FAILURE;
    }
    ROS_INFO_STREAM("opened: "<<libevdev_get_name(dev));

    //...

    libevdev_free(dev);
    close(fd);
}
