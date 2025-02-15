import copy

import yaml


class Definitions:
    definitions = {}

    def __init__(self):
        pass

    @staticmethod
    def parse(filepaths):
        for filepath in filepaths:
            with open(filepath, "r") as yml:
                definition = yaml.safe_load(yml)
                Definitions.definitions = Definitions.__merge_dicts__(
                    Definitions.definitions, definition
                )

    @staticmethod
    def get(key: str, default_val=None):
        keys = key.split("/")

        d = copy.deepcopy(Definitions.definitions)
        if keys in d.items():
            return default_val
        for k in keys:
            if isinstance(d, dict) and k in d:
                d = d[k]
            else:
                raise KeyError(f"Key '{k}' not found in the dictionary.")

        return d

    @staticmethod
    def __merge_dicts__(d1, d2):
        merged = copy.deepcopy(d1)
        for key, value in d2.items():
            if key in merged:
                # If both are dictionaries, recursively merge
                if isinstance(merged[key], dict) and isinstance(value, dict):
                    merged[key] = Definitions.__merge_dicts__(merged[key], value)
                else:
                    merged[key] = value
            else:
                merged[key] = value
        return merged
