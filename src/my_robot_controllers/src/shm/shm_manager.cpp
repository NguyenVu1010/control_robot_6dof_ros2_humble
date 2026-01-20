#include "my_robot_controllers/shm/shm_manager.hpp" // Include đúng đường dẫn nội bộ

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace my_robot_controllers {
namespace shm {

ShmManager::ShmManager(const std::string& name, bool is_server)
    : shm_name_(name), is_server_(is_server), shm_fd_(-1), data_ptr_(nullptr) {}

ShmManager::~ShmManager() { close_shm(); }

bool ShmManager::init() {
    int flags = O_RDWR;
    if (is_server_) flags |= O_CREAT;
    
    shm_fd_ = shm_open(shm_name_.c_str(), flags, 0666);
    if (shm_fd_ == -1) {
        std::cerr << "[Shm] Failed to shm_open" << std::endl;
        return false;
    }

    if (is_server_) {
        if (ftruncate(shm_fd_, sizeof(RobotData)) == -1) return false;
    }

    void* ptr = mmap(0, sizeof(RobotData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (ptr == MAP_FAILED) return false;

    data_ptr_ = static_cast<RobotData*>(ptr);

    if (is_server_ && !data_ptr_->sys_ready) {
        memset(ptr, 0, sizeof(RobotData));
        data_ptr_->sys_ready = true;
    }
    return true;
}

void ShmManager::close_shm() {
    if (data_ptr_) { munmap(data_ptr_, sizeof(RobotData)); data_ptr_ = nullptr; }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        if (is_server_) shm_unlink(shm_name_.c_str());
        shm_fd_ = -1;
    }
}

RobotData* ShmManager::get() { return data_ptr_; }

} // namespace shm
} // namespace my_robot_controllers