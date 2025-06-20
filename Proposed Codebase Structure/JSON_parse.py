# Config file

import json
from typing import Dict

def load_config(json_file_path: str) -> Dict:
    """
    Loads configuration data from a JSON file.

    :param json_file_path: Path to the JSON configuration file.
    :return: Dictionary containing configuration data.
    """
    try:
        with open(json_file_path, 'r') as file:
            config = json.load(file)
        return config
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error loading config: {e}")
        return {}









