 
import rosbag
 
import roslib; # roslib.load_manifest(PKG)import rospy
 
import cv2

import os

import argparse

from tqdm import tqdm

import numpy as np
 
from cv_bridge import CvBridge
 
from sensor_msgs.msg import Image

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def save_pointcloud_to_bin(filename, pc_msg):
    ## Get data from pcd (x, y, z, intensity)
    np_x = (np.array(list(pc2.read_points(pc_msg, skip_nans=True, field_names=("x"))), dtype=np.float32)).astype(np.float32)
    np_y = (np.array(list(pc2.read_points(pc_msg, skip_nans=True, field_names=("y"))), dtype=np.float32)).astype(np.float32)
    np_z = (np.array(list(pc2.read_points(pc_msg, skip_nans=True, field_names=("z"))), dtype=np.float32)).astype(np.float32)
    np_i = (np.array(list(pc2.read_points(pc_msg, skip_nans=True, field_names=("intensity"))), dtype=np.float32)).astype(np.float32)/256
    max_i = np_i[:, 1]
    np_i = max_i.reshape(-1, 1)
    
    ## Stack all data    
    points_32 = np.transpose(np.vstack((np_x, np_y, np_z, np_i)))

    ## Save bin file                                    
    points_32.tofile(filename)
    
    return

def extract_velo(out_path: str, bag, rostopic: str):
    bag_data = bag.read_messages(rostopic)

    if not os.path.exists(out_path):
    # Create the directory
        os.makedirs(out_path)
    data_path = os.path.join(out_path, 'data')
    if not os.path.exists(data_path):
    # Create the directory
        os.makedirs(data_path)
        
    timestr_file_path = out_path + '/timestamps.txt'

    with open(timestr_file_path, 'w') as timestr_file:
        for topic, msg, t in tqdm(bag_data, desc="Processing pointcloud", unit="pc"):

            timestr = "%.6f" % msg.header.stamp.to_sec()
            timestr_file.write(f"{timestr}\n")

            # %.6f表示小数点后带有6位，可根据精确度需要修改； 
            bin_name = timestr + ".bin" # bin命名：时间戳.bin
            save_pointcloud_to_bin(os.path.join(data_path, bin_name), msg) # 保存；
    
    return        
            
def extract_ublox_fix (out_path: str, bag):
    bag_data = bag.read_messages('/ublox/fix')

    if not os.path.exists(out_path):
    # Create the directory
        os.makedirs(out_path)
    data_path = os.path.join(out_path, 'data')
    if not os.path.exists(data_path):
    # Create the directory
        os.makedirs(data_path)
        
    timestr_file_path = os.path.join(out_path, 'timestamps.txt') 
    dataformat_file_path = os.path.join(out_path, 'dataformat.txt') 
    
    with open(dataformat_file_path, 'w') as dataformat_file:
        dataformat_file.write(f"{'rostopic: /ublox/fix'}\n")
        dataformat_file.write(f"{'latitude'} {'longitude'} {'altitude'}")
        
    with open(timestr_file_path, 'w') as timestr_file:
        for topic, msg, t in tqdm(bag_data, desc="Processing files", unit="file"):

            timestr = "%.6f" % msg.header.stamp.to_sec()
            timestr_file.write(f"{timestr}\n")

            # %.6f表示小数点后带有6位，可根据精确度需要修改； 
            file_name = timestr + ".txt" # bin命名：时间戳.bin
            with open(os.path.join(data_path, file_name), 'w') as file: # 保存； 
                file.write(f"{msg.latitude} {msg.longitude} {msg.altitude}")
    return

def extract_ublox_fix_velocity (out_path: str, bag):
    bag_data = bag.read_messages('/ublox/fix_velocity')

    if not os.path.exists(out_path):
    # Create the directory
        os.makedirs(out_path)
    data_path = os.path.join(out_path, 'data')
    if not os.path.exists(data_path):
    # Create the directory
        os.makedirs(data_path)
        
    timestr_file_path = os.path.join(out_path, 'timestamps.txt') 
    dataformat_file_path = os.path.join(out_path, 'dataformat.txt') 
    
    with open(dataformat_file_path, 'w') as dataformat_file:
        dataformat_file.write(f"{'rostopic: /ublox/fix_velocity'}\n")
        dataformat_file.write(f"{'twist.twist.linear.x'} {'twist.twist.linear.y'} {'twist.twist.linear.z'} {'twist.twist.angular.x'} {'twist.twist.angular.y'} {'twist.twist.angular.z'}")
        
    with open(timestr_file_path, 'w') as timestr_file:
        for topic, msg, t in tqdm(bag_data, desc="Processing files", unit="file"):

            timestr = "%.6f" % msg.header.stamp.to_sec()
            timestr_file.write(f"{timestr}\n")

            # %.6f表示小数点后带有6位，可根据精确度需要修改； 
            file_name = timestr + ".txt" # bin命名：时间戳.bin
            with open(os.path.join(data_path, file_name), 'w') as file: # 保存； 
                file.write(f"{msg.twist.twist.linear.x} {msg.twist.twist.linear.y} {msg.twist.twist.linear.z} {msg.twist.twist.angular.x} {msg.twist.twist.angular.y} {msg.twist.twist.angular.z}")
    return

def extract_imu (out_path: str, bag):
    bag_data = bag.read_messages('/handsfree/imu')

    if not os.path.exists(out_path):
    # Create the directory
        os.makedirs(out_path)
    data_path = os.path.join(out_path, 'data')
    if not os.path.exists(data_path):
    # Create the directory
        os.makedirs(data_path)
        
    timestr_file_path = os.path.join(out_path, 'timestamps.txt') 
    dataformat_file_path = os.path.join(out_path, 'dataformat.txt') 
    
    with open(dataformat_file_path, 'w') as dataformat_file:
        dataformat_file.write(f"{'rostopic: /handsfree/imu'}\n")
        dataformat_file.write(f"{'orientation.x'} {'orientation.y'} {'orientation.z'} {'orientation.w'} {'angular_velocity.x'} {'angular_velocity.y'} {'angular_velocity.z'} {'linear_acceleration.x'} {'linear_acceleration.y'} {'linear_acceleration.z'}")
        
    with open(timestr_file_path, 'w') as timestr_file:
        for topic, msg, t in tqdm(bag_data, desc="Processing files", unit="file"):

            timestr = "%.6f" % msg.header.stamp.to_sec()
            timestr_file.write(f"{timestr}\n")

            # %.6f表示小数点后带有6位，可根据精确度需要修改； 
            file_name = timestr + ".txt" # bin命名：时间戳.bin
            with open(os.path.join(data_path, file_name), 'w') as file: # 保存； 
                file.write(f"{msg.orientation.x} {msg.orientation.y} {msg.orientation.z} {msg.orientation.w} {msg.angular_velocity.x} {msg.angular_velocity.y} {msg.angular_velocity.z} {msg.linear_acceleration.x} {msg.linear_acceleration.y} {msg.linear_acceleration.z}")
    return

def extract_image(out_path: str, bag, rostopic: str, show_image: bool = True):
    bridge = CvBridge()
    #此处的节点一次只可以是一个
    bag_data = bag.read_messages(rostopic)

    if not os.path.exists(out_path):
    # Create the directory
        os.makedirs(out_path)
    data_path = os.path.join(out_path, 'data')
    if not os.path.exists(data_path):
    # Create the directory
        os.makedirs(data_path)
        
    timestr_file_path = out_path + '/timestamps.txt'

    with open(timestr_file_path, 'w') as timestr_file:
        for topic, msg, t in tqdm(bag_data, desc="Processing images", unit="image"):

            cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            if show_image:
                cv2.imshow("Image window", cv_image)
                cv2.waitKey(3)

            # imshow可有可无只是为了检验是否在提取图像，并展示
            timestr = "%.6f" % msg.header.stamp.to_sec()
            timestr_file.write(f"{timestr}\n")

            # %.6f表示小数点后带有6位，可根据精确度需要修改； 
            image_name = timestr + ".png" # 图像命名：时间戳.png 
            cv2.imwrite(os.path.join(data_path, image_name), cv_image) # 保存； 
    
    return
	    


def main():
    # Create argument parser
    parser = argparse.ArgumentParser(description="Input of extractor.")

    # Define arguments
    parser.add_argument('-b', '--bag', type=str, help='path to bag')
    parser.add_argument('-o', '--outpath', type=str, help='output path')
    parser.add_argument('-s', '--show_image', type=bool, help='show image or not')

    # Parse command-line arguments
    args = parser.parse_args()

    print("===Reading the rosbag file===")
    bag = rosbag.Bag(args.bag, "r")
    
    if not os.path.exists(args.outpath):
    # Create the directory
        os.makedirs(args.outpath)
    
    print("\n===Start extract GNSS Ublox fix messages!===")
    out_path = os.path.join(args.outpath, 'ublox_fix')
    extract_ublox_fix(out_path, bag)
    print("\n===Start extract GNSS Ublox fix velocity messages!===")
    out_path = os.path.join(args.outpath, 'ublox_fix_velocity')
    extract_ublox_fix_velocity(out_path, bag)
    print("\n===Start extract imu!===")
    out_path = os.path.join(args.outpath, 'imu')
    extract_imu(out_path, bag)
    print("\n===Start extract velodyne pointcloud!===")
    out_path = os.path.join(args.outpath, 'velodyne_points')
    extract_velo(out_path, bag, '/velodyne_points')
    print("\n===Start extract color image!===")
    out_path = os.path.join(args.outpath, 'image_color')
    extract_image(out_path, bag, '/camera/color/image_raw/compressed', args.show_image)
    print("\n===Start extract left image!===")
    out_path = os.path.join(args.outpath, 'image_left')
    extract_image(out_path, bag, '/camera/left/image_raw/compressed', args.show_image)
    print("\n===Start extract right image!===")
    out_path = os.path.join(args.outpath, 'image_right')
    extract_image(out_path, bag, '/camera/right/image_raw/compressed', args.show_image)
    print("\n===Start extract third image!===")
    out_path = os.path.join(args.outpath, 'image_third')
    extract_image(out_path, bag, '/camera/third/image_raw/compressed', args.show_image)
    print("\n===Start extract fourth image!===")
    out_path = os.path.join(args.outpath, 'image_fourth')
    extract_image(out_path, bag, '/camera/fourth/image_raw/compressed', args.show_image)
    print("\n===Start extract fifth image!===")
    out_path = os.path.join(args.outpath, 'image_fifth')
    extract_image(out_path, bag, '/camera/fifth/image_raw/compressed', args.show_image)
    print("\n===Start extract sixth image!===")
    out_path = os.path.join(args.outpath, 'image_sixth')
    extract_image(out_path, bag, '/camera/sixth/image_raw/compressed', args.show_image)
    
    
    
    

if __name__ == "__main__":
    main()
