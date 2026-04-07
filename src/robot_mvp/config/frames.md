# Robot Frame Convention

## Main frames

- map
- odom
- base_link
- base_footprint
- laser
- imu_link
- camera_link

## TF tree

map
└── odom
    └── base_link
        ├── base_footprint
        ├── laser
        ├── imu_link
        └── camera_link

## Notes

- `base_link` = center of robot body
- `base_footprint` = ground projection of robot
- `laser` = LiDAR frame
- `imu_link` = IMU frame
- `camera_link` = RGB camera frame

## Topic assumptions

- LiDAR publishes to `/scan`
- IMU publishes to `/imu/data`
- Camera publishes to `/camera/image_raw`
- Final motor command topic is `/cmd_vel`