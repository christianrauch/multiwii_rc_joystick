#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <dirent.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <mavros_msgs/OverrideRCIn.h>


static const uint16_t rcmin = 1000;
static const uint16_t rcmax = 2000;

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
    ros::Rate rate(100);
    ros::Publisher pub = n.advertise<mavros_msgs::OverrideRCIn>("rc/override/raw", 1);

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
    }

    struct libevdev *dev = NULL;
    int fd = open(device.c_str(), O_RDONLY);
    if(libevdev_new_from_fd(fd, &dev) < 0) {
        ROS_ERROR_STREAM("libevdev error");
        close(fd);
        return EXIT_FAILURE;
    }
    ROS_INFO_STREAM("device: "<<libevdev_get_name(dev));
    ROS_INFO_STREAM("firmware: "<<libevdev_get_id_version(dev));

    // check if device is joystick
    if(!libevdev_has_event_type(dev, EV_ABS)) {
        ROS_ERROR_STREAM("device is not a joystick");
        libevdev_free(dev);
        close(fd);
        return EXIT_FAILURE;
    }

    const int max_evcode = libevdev_event_type_get_max(EV_ABS);
    if(max_evcode<0) {
        ROS_ERROR_STREAM("invalid type");
        libevdev_free(dev);
        close(fd);
        return EXIT_FAILURE;
    }

    // get minimum and maximum value ranges
    std::vector<int> axmin, axmax;
    for (uint icode(0); icode < uint(max_evcode); icode++) {
        if(libevdev_has_event_code(dev, EV_ABS, icode)) {
            axmin.push_back(libevdev_get_abs_minimum(dev, icode));
            axmax.push_back(libevdev_get_abs_maximum(dev, icode));
        }
    }

    uint naxes;
    if(axmin.size()==axmax.size()) {
        naxes = uint(axmin.size());
    }
    else {
        throw std::runtime_error("amount of axes do not match");
    }

    mavros_msgs::OverrideRCIn msg;

    while(n.ok()) {
        struct input_event ev;
        int rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if(rc==LIBEVDEV_READ_STATUS_SUCCESS) {
            for(uint iaxis(0); iaxis<naxes; iaxis++) {
                int axval = libevdev_get_event_value(dev, EV_ABS, iaxis);
                msg.channels[iaxis] = rcmin + ((axval-axmin[iaxis])*(rcmax-rcmin)) / (axmax[iaxis]-axmin[iaxis]);
            }
            pub.publish(msg);
        }
        else if(rc==LIBEVDEV_READ_STATUS_SYNC) {
            ROS_ERROR_STREAM("out of sync");
        }

        rate.sleep();
    }

    libevdev_free(dev);
    close(fd);
}
