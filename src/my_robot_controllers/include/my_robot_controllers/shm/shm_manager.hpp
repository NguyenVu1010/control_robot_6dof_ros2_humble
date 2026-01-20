#ifndef MY_ROBOT_CONTROLLERS__SHM__SHM_MANAGER_HPP_
#define MY_ROBOT_CONTROLLERS__SHM__SHM_MANAGER_HPP_

#include "shm_msgs.hpp"
#include <string>

namespace my_robot_controllers {
namespace shm {

class ShmManager {
public:
    ShmManager(const std::string& name, bool is_server);
    ~ShmManager();

    bool init();
    void close_shm();
    RobotData* get();

private:
    std::string shm_name_;
    bool is_server_;
    int shm_fd_;
    RobotData* data_ptr_;
};

} // namespace shm
} // namespace my_robot_controllers

#endif