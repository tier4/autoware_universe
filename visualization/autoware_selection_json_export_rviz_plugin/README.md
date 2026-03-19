# autoware_selection_json_export_rviz_plugin

RViz2 panel plugin. For objects selected with the **Select** tool in the 3D view, it can show, copy to the clipboard, or save to a file as **JSON**:

- `picked`: picking handles, `pixel_count`, and `extra_handles`
- `property_tree`: property tree aligned with the Selection panel (name, description, children, value)

## Build

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select autoware_selection_json_export_rviz_plugin
source install/setup.bash
```

## Usage

1. Start `rviz2`.
2. **Panels → Add New Panel →** choose **Selection JSON Export** (or `autoware_selection_json_export_rviz_plugin::SelectionJsonExportPanel`; the label may vary by environment).
3. In RViz **Interact**, pick the **Select** tool and select an object in the 3D view.
4. In the panel, press **Refresh JSON**, then **Copy** or **Save to file...**

## Notes on the JSON

- Properties are produced by recursively walking RViz’s internal `Property` tree. Depending on the property type, `value` may be stringified.
- Recursion depth is capped at 256 levels for safety.

## License

Apache License 2.0
