#include <thread>

#include "io/dm_imu/dm_imu.hpp"

int main()
{
  io::DM_IMU imu;

  std::this_thread::sleep_for(std::chrono::seconds(5));

  return 0;
}