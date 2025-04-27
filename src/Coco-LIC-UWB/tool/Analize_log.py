import re
import matplotlib.pyplot as plt

# I0115 17:15:12.231796 24899 odometry_manager.cpp:199] [Update time]: 23.2281 ms.
def parse_update_time(line):
    match = re.search(
        r'\d{2}:(\d{2}):(\d+\.\d+).*?\[Update time\]:\s+(\d+\.\d+)', line)

    if match:
        time_min = match.group(1)
        time_sec = match.group(2)
        time_cost = match.group(3)
        # print(time_min, time_sec, time_cost)

        timestamp = float(time_min) * 60 + float(time_sec)
        # print(time_min, time_sec, timestamp, time_cost)

        return timestamp, time_cost

    return None, None


def extract_content(log_file_path, keyword):
    with open(log_file_path, 'r') as file:
        lines = file.readlines()
    t1_vals = []
    t2_vals = []
    cnt = []
    i = 0
    for line in lines:
        time1, time2 = parse_update_time(line)
        if time1 and time2:
            t1_vals.append(float(time1))
            t2_vals.append(float(time2))
            i = i + 1
            cnt.append(i)
            mean_time2 = sum(t2_vals) / len(t2_vals) if t2_vals else 0
    plt.plot(t1_vals, t2_vals, marker='o')
    plt.axhline(y=mean_time2, color='r', linestyle='--',
                label=f'Mean Time: {mean_time2:.2f} ms')
    plt.xlabel("timestamp")
    plt.ylabel("time cusumed")
    plt.legend()
    plt.show()



if __name__ == "__main__":
    # Update this path to your log file
    log_file_path = '/home/mint/ws_uav_setup/src/Coco-LIC-UWB/log/odometry_node.INFO'
    # Update this keyword to search for different keywords
    keyword = '[Update time]'
    content = extract_content(log_file_path, keyword)
