
import os.path as osp
import subprocess
import shlex
import json
import os

def generate_sync_commands(topics_conf, queue_sizes: list, sync_errors: list, path):
    cmds = []
    enum = 0
    cfg_dicts = []
    for conf_list in topics_conf:
        for sync_error in sync_errors:
            for queue_size in queue_sizes:
                enum += 1
                dataset_path = osp.join(path, f"dataset{enum}")
                topic_string = "".join(f"{entry[0]} " for entry in conf_list)
                exporter_string = "".join(f"{entry[0]} {entry[1]} --dir {dataset_path}" for entry in conf_list)
                cmds.append(f"-f sync -t {topic_string} --slop {sync_error} --queue-size {queue_size} -c {exporter_string}")
                cfg_dict = {"topics": topic_string,
                            "allowed_sync_error": sync_error,
                            "queue_size": queue_size
                            }
                cfg_dicts.append(cfg_dict)
    return cmds, cfg_dicts


if __name__ == "__main__":
   
    path_to_store = "/home/lars/Data_IAC/Generated_Datasets/"

    allowed_sync_errors = [0.01, 0.02, 0.05, 0.1, 0.5, 1, 2]
    topics_conf = [[["/camera/front_left_center/camera/image", "image"],
                    ["/luminar_front_points", "pcd"]],
                   [["/camera/front_right_center/camera/image", "image"],
                    ["/luminar_front_points", "pcd"]],
                   [["/camera/front_right_center/camera/image", "image"],
                    ["/camera/front_right_center/camera/image", "image"],
                    ["/luminar_front_points", "pcd"]]]
    queue_sizes = [1, 2, 5]

    generated_cmds, cfg_dicts = generate_sync_commands(topics_conf, queue_sizes,
                                            allowed_sync_errors, path_to_store)
    for generated_cmd, cfg_dict in zip(generated_cmds, cfg_dicts):
        cmd = []
        cmd.extend(["ros2", "bag", "export"])
        cmd.extend(shlex.split(generated_cmd))
        path = cmd[-1]
        json_obj = json.dumps(cfg_dict)
        dir = os.path.dirname(path)
        if not os.path.exists(dir):
            os.mkdir(dir)
        if not os.path.exists(path):
            os.mkdir(path)
        with open(f"{path}/simple_metadata.json", "w+") as f:
            f.write(json_obj)
        print(cmd)
        process = subprocess.Popen(cmd,
                                   stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        break