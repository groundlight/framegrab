"""
This script generates an HTML table documenting the available parameters for the config of each input type.
It is intended for documentation generation purposes, allowing users to understand the configuration options
available when creating a FrameGrabber object.
"""

from typing import Any
from pydantic.fields import FieldInfo
from framegrab.config import InputTypes, FrameGrabberConfig

CONFIG_TABLE_START = "<!-- start configuration table -->"
CONFIG_TABLE_END = "<!-- end configuration table -->"

# Table cell symbols
REQUIRED_SYMBOL = "🔴"
OPTIONAL_SYMBOL = "✅"
NOT_SUPPORTED_SYMBOL = "-"

def get_python_type(field_info: FieldInfo) -> str:
    """Extract Python type from pydantic field, unwrapping nested generics."""
    annotation = field_info.annotation
    
    # Unwrap nested generic types (e.g., OptionsField[Optional[int]] -> int)
    while hasattr(annotation, '__origin__'):
        args = getattr(annotation, '__args__', ())
        types = [arg for arg in args if arg is not type(None)]
        if not types:
            break
        annotation = types[0]
    
    type_name = getattr(annotation, '__name__', str(annotation))
    type_map = {'str': 'string', 'int': 'int', 'float': 'float', 'bool': 'bool', 'dict': 'dict'}
    return type_map.get(type_name, type_name)


def add_config_field(config_map: dict, path: str, input_type: InputTypes, 
                     availability: str, datatype: str = 'string') -> None:
    """Add a configuration field to the config map."""
    if path not in config_map:
        config_map[path] = {'types': {}, 'datatype': datatype}
    config_map[path]['types'][input_type] = availability


def get_config_rows(models: list[type[FrameGrabberConfig]]) -> list[tuple[str, dict[str, Any]]]:
    """Extract configuration rows from models."""
    config_map = {}
    required_id_fields = {"rtsp_url", "hls_url", "youtube_url", "filename"}
    
    for model in models:
        input_type = model.get_input_type()
        
        # Add common fields
        add_config_field(config_map, "name", input_type, "optional")
        add_config_field(config_map, "input_type", input_type, "required")
        
        # Add ID field
        id_field = FrameGrabberConfig.get_input_type_to_id_dict().get(input_type)
        if id_field:
            availability = "required" if id_field in required_id_fields else "optional"
            add_config_field(config_map, f"id.{id_field}", input_type, availability)
        
        # Add options fields
        for fname, field_info in model.model_fields.items():
            if fname in ["name", "input_type"] or fname == id_field:
                continue
            
            opt_key = (field_info.json_schema_extra or {}).get("options_key")
            if opt_key:
                path = f"options.{opt_key}"
                availability = "required" if field_info.is_required() else "optional"
                datatype = get_python_type(field_info)
                add_config_field(config_map, path, input_type, availability, datatype)
    
    # Sort all items, but put input_type first
    sorted_items = sorted(config_map.items())
    input_type_item = next((item for item in sorted_items if item[0] == "input_type"), None)
    other_items = [item for item in sorted_items if item[0] != "input_type"]
    
    return [input_type_item] + other_items if input_type_item else other_items


def format_cell(value: str) -> str:
    """Format a table cell value with appropriate styling."""
    if value == "required":
        return REQUIRED_SYMBOL
    elif value == "optional":
        return OPTIONAL_SYMBOL
    return NOT_SUPPORTED_SYMBOL


def generate_html_table(models: list[type[FrameGrabberConfig]]) -> str:
    """Generate HTML table from models."""
    config_rows = get_config_rows(models)
    input_types = [model.get_input_type() for model in models]
    
    # Build header
    header_cells = ['Configuration Name', 'Type'] + [it.value for it in input_types]
    headers = '\n'.join(f'      <th>{cell}</th>' for cell in header_cells)
    
    # Build rows
    rows = []
    for config_path, config_info in config_rows:
        cells = [config_path, config_info.get('datatype', '-')]
        type_map = config_info.get('types', {})
        cells.extend(format_cell(type_map.get(it, '-')) for it in input_types)
        row = '\n'.join(f'      <td>{cell}</td>' for cell in cells)
        rows.append(f'    <tr>\n{row}\n    </tr>')
    
    legend = f"""<table>
  <tr>
    <td><strong>Legend:</strong></td>
    <td>{REQUIRED_SYMBOL} = required</td>
    <td>{OPTIONAL_SYMBOL} = optional</td>
    <td>{NOT_SUPPORTED_SYMBOL} = not supported</td>
  </tr>
</table>"""
    
    return f"""<div style="overflow-x: auto;">

<table>
  <thead>
    <tr>
{headers}
    </tr>
  </thead>
  <tbody>
{chr(10).join(rows)}
  </tbody>
</table>

</div>

{legend}"""

def write_schema_to_readme(models: list[type[FrameGrabberConfig]], readme_file: str) -> None:
    """Replace the config schema section in README with HTML table."""
    with open(readme_file, "r") as f:
        lines = f.readlines()
    
    html_table = generate_html_table(models)
    
    with open(readme_file, "w") as f:
        in_section = False
        for line in lines:
            if CONFIG_TABLE_START in line:
                in_section = True
                f.write(line)
                f.write("\n")
                f.write(html_table)
                f.write("\n\n")
            elif in_section and CONFIG_TABLE_END in line:
                in_section = False
                f.write(line)
            elif not in_section:
                f.write(line)


if __name__ == "__main__":
    models = [FrameGrabberConfig.get_class_for_input_type(input_type) for input_type in InputTypes]
    write_schema_to_readme(models, "README.md")
    print("Schema written to README.md")
