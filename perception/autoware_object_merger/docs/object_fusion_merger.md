# object_fusion_merger

## Purpose

`object_fusion_merger` associates two detected-object streams and keeps the main stream as the base output.
For each matched pair, it expands the main object's shape so that the union of the main and sub regions is enclosed by the main shape type.

The node is stateless across frames and reuses the existing association logic from `autoware_object_merger::DataAssociation`.

## Inputs / Outputs

### Input

| Name                | Type                                             | Description           |
| ------------------- | ------------------------------------------------ | --------------------- |
| `input/main_object` | `autoware_perception_msgs::msg::DetectedObjects` | Main detected objects |
| `input/sub_object`  | `autoware_perception_msgs::msg::DetectedObjects` | Sub detected objects  |

### Output

| Name            | Type                                             | Description                                                       |
| --------------- | ------------------------------------------------ | ----------------------------------------------------------------- |
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | Main-based detected objects after association and shape expansion |

## Processing Flow

1. Synchronize the two input topics with `ApproximateTime`.
2. Transform both object lists into `base_link_frame_id`.
3. Build the association score matrix using the existing distance, angle, and IoU gates.
4. Solve one-to-one assignment with the same GNN solver used by `object_association_merger`.
5. For each matched pair, copy the main object and expand only its shape so that the main/sub union is enclosed.
6. Keep unmatched main objects unchanged.
7. Discard unmatched sub objects.

## Shape Update Rule

Only the shape is updated for matched pairs.
All other fields remain main-object values.

- Pose is kept from the main object.
- Orientation is kept from the main object.
- Twist is kept from the main object.
- Classification is kept from the main object.
- Existence probability is kept from the main object.

The enclosing shape is generated using the main object's shape type.

- `BOUNDING_BOX`: enlarge local x/y extents around the main pose to cover the union footprint.
- `CYLINDER`: enlarge radius to cover the farthest union point.
- `POLYGON`: build a convex hull from the union footprint points in the main local frame.

The height dimension is updated so the output encloses the full z-extent of both objects.

### Notes Per Shape Type

- `BOUNDING_BOX` main objects remain axis-aligned in the main-object local frame. The box can grow conservatively when the sub object is laterally offset from the main pose.
- `CYLINDER` main objects are expanded isotropically in x/y because the enclosing radius is computed from the farthest union point. The resulting diameter is identical for `dimensions.x` and `dimensions.y`.
- `POLYGON` main objects keep polygon output and rebuild the footprint from the convex hull of the combined main/sub footprint points. Concavities in the original input footprints are not preserved.

## Association Preconditions

Shape expansion happens only when the main/sub pair passes the existing `DataAssociation` gates.
In practice, this means the pair must satisfy all of the following:

- the class pair is allowed by `can_assign_matrix`
- the center distance is within `max_dist_matrix`
- the yaw difference is within `max_rad_matrix` when that gate is enabled
- the 2D IoU is above `min_iou_matrix`

If a pair does not pass those gates, the main object is treated as unmatched and remains unchanged, and the sub object is discarded.

## Parameter

- `shape_fusion_policy: 0`
  This implementation supports only `union_enclosing_main_shape`.

## Default Behavior

With the default configuration:

- matched main/sub pairs produce one output object based on the main object
- unmatched main objects remain in the output
- unmatched sub objects are not emitted

## Test Coverage

The current focused test suite covers the following cases:

- matched `BOUNDING_BOX` pair expands the main box while keeping main pose and metadata
- unmatched main objects remain in the output and unmatched sub objects are dropped
- `BOUNDING_BOX` main object can enclose a larger sub `POLYGON`
- `CYLINDER` main object expands to cover the union footprint
- `POLYGON` main object rebuilds its footprint from the union convex hull

## Known Limits

- There is no temporal fusion across messages.
- The node does not maintain track IDs or track existence over time.
- The output pose is always the main object pose, so the enclosing shape may become conservative when the sub object center is offset.
- Because matching depends on the existing association gates, some geometrically plausible pairs may still remain unmatched if their class-specific IoU threshold is not met.

## Intended Usage

This node is intended for cases where the main detector should stay authoritative, while the sub detector is used only to expand the geometric extent of matched objects.
