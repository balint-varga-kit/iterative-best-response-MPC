



"""
This script is used to parse a YAML configuration file and extract a subset of parameters starting from the first occurrence of a specific key.

The script performs the following steps:
1. Imports the necessary modules: os and yaml.
2. Specifies the name of the YAML configuration file to be parsed.
3. Retrieves the directory path of the script.
4. Constructs the absolute path of the YAML configuration file.
5. Opens the YAML configuration file and loads its contents into the 'params' variable.
6. Finds the index of the first occurrence of a key ending with '_node' in the 'params' dictionary.
7. Creates a new dictionary, 'new_params', containing only the key-value pairs starting from the first occurrence of the key found in step 6.
8. Saves the modified YAML configuration to a new file named 'new_config.yaml' in the same directory as the script.
"""


# Import YAML config files:
import os
import yaml

def anchor_and_alias_yaml_support(input_file_path=None, output_file_path="None"):
    """
    Extract parameters starting from the first node and save them to a new YAML file.

    Basically the YAML file is interpreted as a big dictionary, and when we use anchors and aliases,
    and use the default yaml parses to read the file, than on the top level, first we see the global 
    coordinates, than the nodes. We want to cut the global coordinates and keep only the nodes. So
    we find the index of the first node, and create a new dictionary from there. We only do this,
    if the node has parameters on the first place, otherwise, we leave the key out.

    Args:
        input_file (str): Path to the input YAML file.
        output_file (str): Path to the output YAML file.

    Returns:
        None
    """

    with open(input_file_path, 'r') as f:
        params = yaml.safe_load(f)

    # Find the index of the first node
    first_node_index = next((i for i, key in enumerate(params) if key.endswith('_node')),None)

    # Create a new dictionary starting from the first node
    new_params = {k: params[k] for k in list(params.keys())[first_node_index:] if params[k]['ros__parameters'] not in [0, None, {}, 'null']}

    # Save the modified YAML to a new file
    with open(output_file_path, 'w') as f:
        yaml.dump(new_params, f)