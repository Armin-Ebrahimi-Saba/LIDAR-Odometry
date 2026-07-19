import rosbag
import csv
import bisect
import os

def main():
    print("Loading Ground Truth CSV...")
    gt_times = []
    gt_data = []
    with open('/workspace/data/ground_truth_gnss/xtrack_global_position_t12.csv', 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            gt_times.append(float(row['timestamp']))
            gt_data.append((float(row['lat']), float(row['lon']), float(row['alt'])))

    print("Patching bag...")
    in_bag = rosbag.Bag('/workspace/bags/lio_sam_ready.bag', 'r')
    out_bag = rosbag.Bag('/workspace/bags/lio_sam_ready_patched.bag', 'w')
    
    count = 0
    for topic, msg, t in in_bag.read_messages():
        if topic == '/gps/fix':
            # Find closest GT
            msg_time_sec = msg.header.stamp.to_sec()
            idx = bisect.bisect_left(gt_times, msg_time_sec)
            
            if idx == 0:
                closest_idx = 0
            elif idx == len(gt_times):
                closest_idx = len(gt_times) - 1
            else:
                before = msg_time_sec - gt_times[idx - 1]
                after = gt_times[idx] - msg_time_sec
                if before < after:
                    closest_idx = idx - 1
                else:
                    closest_idx = idx
            
            lat, lon, alt = gt_data[closest_idx]
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            
            # Write with correct data
            out_bag.write(topic, msg, t)
            count += 1
        else:
            out_bag.write(topic, msg, t)
            
    in_bag.close()
    out_bag.close()
    
    # Replace old bag with patched bag
    os.rename('/workspace/bags/lio_sam_ready.bag', '/workspace/bags/lio_sam_ready.bag.bak')
    os.rename('/workspace/bags/lio_sam_ready_patched.bag', '/workspace/bags/lio_sam_ready.bag')
    
    print("Patched {} GPS messages.".format(count))

if __name__ == '__main__':
    main()
