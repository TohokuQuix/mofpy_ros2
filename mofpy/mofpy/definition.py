import yaml
import copy
import rclpy.logging


class Definitions:
    definitions = dict()

    def __init__(self):
        pass

    @staticmethod
    def parse(filepaths):
        # node.get_logger().info(str(filepaths))
        for filepath in filepaths:
            with open(filepath, 'r') as yml:
                definition = yaml.safe_load(yml)
                Definitions.definitions = merge_dicts(Definitions.definitions, definition)

    @staticmethod
    def get(key : str, default_val=None):
        keys = key.split('/')

        d = copy.deepcopy(Definitions.definitions)
        if keys in d.items():
            return default_val
        for k in keys:
            if isinstance(d, dict) and k in d:
                d = d[k]
            else:
                raise KeyError(f"Key '{k}' not found in the dictionary.")

        return d


def merge_dicts(d1, d2):
    merged = copy.deepcopy(d1)  # d1の内容をコピー
    for key, value in d2.items():
        if key in merged:
            # 両方が辞書なら、再帰的にマージ
            if isinstance(merged[key], dict) and isinstance(value, dict):
                merged[key] = merge_dicts(merged[key], value)
            else:
                # d2 の値で上書き
                merged[key] = value
        else:
            merged[key] = value
    # rclpy.logging.get_logger("merge_dicts").info(f"merge: {str(merged)}, {str(d2)}")
    return merged
