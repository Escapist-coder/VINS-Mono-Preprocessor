# VinsSyncData.msg
std_msgs/Header header       # 与图像帧完全一致的时间戳
float64 td                   # 当前估算的时间延迟
sensor_msgs/Imu[] imu_data   # 两帧之间的 IMU 数据包
