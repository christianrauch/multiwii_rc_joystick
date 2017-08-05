#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <dirent.h>

#include <thread>
#include <mutex>

#include <ros/console.h>
#include <ros/ros.h>

#include <mavros_msgs/OverrideRCIn.h>


class AxesEventReader {
private:
    static const std::string event_dir;
    static const std::string event_file;

    static const uint16_t rcmin;
    static const uint16_t rcmax;

    ros::NodeHandle n;
    ros::Rate *rate;
    ros::Publisher pub;

    int fd;
    struct libevdev *dev = NULL;

    std::vector<int> axmin, axmax;

    mavros_msgs::OverrideRCIn msg;

    std::thread ev_block_thread;
    std::mutex msg_mutex;

public:
    AxesEventReader() {
        n = ros::NodeHandle("~");
        rate = new ros::Rate(100);
        pub = n.advertise<mavros_msgs::OverrideRCIn>("rc/override/raw", 1);

        if(connect()) {
            // start event thread
            start();
            // start main loop
            loop();
        }
        else {
            ROS_ERROR_STREAM("could not connect to any joystick");
        }
    }

    ~AxesEventReader() {
        libevdev_free(dev);
        close(fd);
        delete rate;
    }

    bool connect() {
        // get list of event devices
        const std::map<std::string, std::string> evdevs = AxesEventReader::getEventList();
        if(evdevs.size()==0) {
            ROS_ERROR_STREAM("no devices available");
            return false;
        }

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
                return false;
            }
        }

        fd = open(device.c_str(), O_RDONLY);
        if(libevdev_new_from_fd(fd, &dev) < 0) {
            ROS_ERROR_STREAM("libevdev error");
            close(fd);
            return false;
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
        for (uint icode(0); icode < uint(max_evcode); icode++) {
            if(libevdev_has_event_code(dev, EV_ABS, icode)) {
                axmin.push_back(libevdev_get_abs_minimum(dev, icode));
                axmax.push_back(libevdev_get_abs_maximum(dev, icode));
            }
        }

        if(axmin.size()!=axmax.size()) {
            ROS_ERROR_STREAM("amount of axes do not match");
            return false;
        }

        return true;
    }

    void start() {
        ev_block_thread = std::thread([this](){
            struct input_event ev;
            while(n.ok()) {
                int rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_BLOCKING, &ev);
                if(n.ok() && rc==LIBEVDEV_READ_STATUS_SUCCESS) {
                    if(ev.type==EV_ABS) {
                        int axval = ev.value;
                        msg_mutex.lock();
                        // map [min,max] to [1000,2000]
                        msg.channels[ev.code] = rcmin + ((axval-axmin[ev.code])*(rcmax-rcmin)) / (axmax[ev.code]-axmin[ev.code]);
                        msg_mutex.unlock();
                        pub.publish(msg);
                    }
                }
                else if(rc==LIBEVDEV_READ_STATUS_SYNC) {
                    ROS_ERROR_STREAM("out of sync");
                }
                else if(rc==-ENODEV){
                    ROS_ERROR_STREAM("device disconnected, shutting down");
                    ros::shutdown();
                }
                else {
                    ROS_ERROR_STREAM("unknown return value: " << rc);
                }
            }
        });
        ev_block_thread.detach();
    }

    void loop() {
        while(n.ok()) {
            msg_mutex.lock();
            for(uint iaxis(0); iaxis<msg.channels.size(); iaxis++) {
                int axval = libevdev_get_event_value(dev, EV_ABS, iaxis);
                msg.channels[iaxis] = rcmin + ((axval-axmin[iaxis])*(rcmax-rcmin)) / (axmax[iaxis]-axmin[iaxis]);
            }
            pub.publish(msg);
            msg_mutex.unlock();

            rate->sleep();
        }
    }

    static
    std::map<std::string, std::string> getEventList() {
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
};

const std::string AxesEventReader::event_dir = "/dev/input";
const std::string AxesEventReader::event_file = "event";

const uint16_t AxesEventReader::rcmin = 1000;
const uint16_t AxesEventReader::rcmax = 2000;



int main(int argc, char **argv) {
    ros::init(argc, argv, "rc_joystick");
    AxesEventReader();
}
