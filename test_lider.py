from rplidar import RPLidar

PORT = "/dev/ttyUSB0"  # Change if needed

lidar = RPLidar(PORT)

try:
    print("LiDAR Info:")
    print(lidar.get_info())

    print("\nHealth:")
    print(lidar.get_health())

    print("\nStarting scan...\n")

    for i, scan in enumerate(lidar.iter_scans()):
        print(f"Scan {i+1}: {len(scan)} points")

        # Print first 10 measurements
        for measurement in scan[:10]:
            quality, angle, distance = measurement
            print(
                f"Quality={quality:2d}  "
                f"Angle={angle:7.2f}°  "
                f"Distance={distance:8.2f} mm"
            )

        print("-" * 50)

        if i >= 4:   # Stop after 5 scans
            break

finally:
    lidar.stop()
    lidar.disconnect()
from rplidar import RPLidar

PORT = "/dev/ttyUSB0"  # Change if needed

lidar = RPLidar(PORT)

try:
    print("LiDAR Info:")
    print(lidar.get_info())

    print("\nHealth:")
    print(lidar.get_health())

    print("\nStarting scan...\n")

    for i, scan in enumerate(lidar.iter_scans()):
        print(f"Scan {i+1}: {len(scan)} points")

        # Print first 10 measurements
        for measurement in scan[:10]:
            quality, angle, distance = measurement
            print(
                f"Quality={quality:2d}  "
                f"Angle={angle:7.2f}°  "
                f"Distance={distance:8.2f} mm"
            )

        print("-" * 50)

        if i >= 4:   # Stop after 5 scans
            break

finally:
    lidar.stop()
    lidar.disconnect()
