"""
This script generates a YAML string documenting the available parameters for the config of each input type.
It is intended for documentation generation purposes, allowing users to understand the configuration options
available when creating a FrameGrabber object. The script writes the schema of each model to a output
string, which we inject into the README.md file, providing a reference for developers and users.
"""

import yaml
from framegrab.config import (
    InputTypes,
    FrameGrabberConfig
)
def write_schema_to_readme(models, readme_file):
    with open(readme_file, "r") as f:
        lines = f.readlines()

    with open(readme_file, "w") as f:
        in_config_schema_section = False
        for line in lines:
            if "##### Config Schema" in line:
                in_config_schema_section = True
                f.write(line)
                f.write("```yaml\n")
                for model in models:
                    schema = model.model_json_schema()
                    yaml.dump({f"{model.__name__}": schema}, f, default_flow_style=False)
                    f.write("\n")
                f.write("```\n")
                in_config_schema_section = False
            elif not in_config_schema_section:
                f.write(line)


if __name__ == "__main__":
    models = [FrameGrabberConfig.get_class_for_input_type(input_type.value) for input_type in InputTypes]
    output_file = "README.md"
    write_schema_to_readme(models, output_file)
    print(f"Schema written to {output_file}")
