# Dynamic Output Names Feature

**Date**: 2025-12-09  
**Critical Feature**: Filters now use output names from YAML configuration instead of hardcoded names

---

## Problem

Previously, filters had hardcoded output names in their implementations:
- `OrganizeFilter` always output to `"organized"`
- `TransformFilter` always output to `"transformed"`
- `CropBoxFilter` always output to `"filtered"`
- etc.

This caused **runtime errors** when YAML configuration specified different output names:
```
Error: Missing required node output: cropbox from node cropbox for node distortion
```

The YAML said output name should be `"cropbox"`, but the filter was outputting to `"filtered"`.

---

## Solution

### 1. Updated Filter Interface

Added `output_names` parameter to the `process()` method:

```cpp
virtual void process(
  const TypedInputs & inputs,
  std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context,
  const std::vector<std::string> & output_names) = 0;  // NEW!
```

### 2. DAG Executor Passes Output Names

The DAG executor now passes the configured output names to each filter:

```cpp
// From dag_executor.cpp
node.filter->process(typed_inputs, node_outputs, context_, node.config.outputs);
```

The `node.config.outputs` comes from the YAML configuration.

### 3. Filters Use Dynamic Output Names

All filters now use `output_names[0]` instead of hardcoded strings:

**Before:**
```cpp
outputs["filtered"] = ...;  // Hardcoded!
```

**After:**
```cpp
outputs[output_names[0]] = ...;  // Dynamic from YAML!
```

---

## Example

### YAML Configuration

```yaml
- id: "cropbox"
  type: "CropBoxFilter"
  outputs:
    - name: "my_custom_output_name"  # User can name it anything!
```

### Filter Implementation

```cpp
void CropBoxFilter::process(..., const std::vector<std::string> & output_names) {
  // ...process data...
  outputs[output_names[0]] = processed_data;  // Uses "my_custom_output_name"
}
```

### Result

The output will be stored as `"cropbox.my_custom_output_name"` in the DAG, and can be referenced by downstream nodes:

```yaml
- id: "next_node"
  inputs:
    - source: "my_custom_output_name"  # Matches the configured name
      from_node: "cropbox"
```

---

## Files Modified

### Core Interface
- `include/.../dag/filter_interface.hpp` - Updated `IFilter::process()` signature

### DAG Executor
- `src/dag/dag_executor.cpp` - Pass `node.config.outputs` to filters

### All Filter Implementations
- `src/dag/filters/organize_filter.cpp`
- `src/dag/filters/transform_filter.cpp`
- `src/dag/filters/cropbox_filter.cpp`
- `src/dag/filters/ring_outlier_filter.cpp`
- `src/dag/filters/distortion_filter.cpp`
- `src/dag/filters/finalize_filter.cpp`

### All Filter Headers
- `include/.../dag/filters/*.hpp` - Updated process signatures

### Tests
- `test/test_dag_executor.cpp` - Updated mock filter

---

## Benefits

1. **Flexibility**: Users can name outputs anything they want in YAML
2. **Consistency**: Output names in YAML now match what filters actually produce
3. **No Runtime Errors**: Eliminates "missing output" errors caused by name mismatches
4. **Better UX**: YAML configuration is the source of truth for naming

---

## Migration Notes

### For Filter Developers

When creating new filters, always use `output_names` parameter:

```cpp
void MyFilter::process(
  const TypedInputs & inputs,
  std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context,
  const std::vector<std::string> & output_names)  // Don't forget this!
{
  // Process data...
  
  // Use dynamic output name
  if (!output_names.empty()) {
    outputs[output_names[0]] = result;
  }
  
  // For filters with multiple outputs:
  if (output_names.size() > 1) {
    outputs[output_names[1]] = secondary_result;
  }
}
```

### For YAML Configuration Writers

Output names in YAML now directly map to filter outputs:

```yaml
nodes:
  - id: "my_filter"
    type: "SomeFilter"
    outputs:
      - name: "primary"      # Filter will output to "primary"
      - name: "secondary"    # Filter will output to "secondary" (if it produces multiple outputs)
```

---

## Testing

✅ All tests pass with dynamic output names
✅ Build successful
✅ Runtime errors eliminated

---

## Related Features

- **TypedInputs**: DAG executor handles type identification
- **Dynamic Input Names**: Filters accept any input name (source from YAML)
- **Dynamic Output Names**: Filters produce outputs with names from YAML (this feature)

Together, these features make the DAG system fully configurable via YAML without code changes.

