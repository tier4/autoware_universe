## map and debug
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from shapely.geometry import MultiLineString, LineString

# Helper function to extract X/Y coordinates
def get_xy_from_linestring(linestring):
    x = [point.x for point in linestring]
    y = [point.y for point in linestring]
    return x, y

def plot_lanelet(ax, lanelet, color='black', linewidth=1):
    # Plot left boundary
    left_x, left_y = get_xy_from_linestring(lanelet.leftBound)
    ax.plot(left_x, left_y, color=color, linewidth=linewidth)
    
    # Plot right boundary
    right_x, right_y = get_xy_from_linestring(lanelet.rightBound)
    return ax.plot(right_x, right_y, color=color, linewidth=linewidth)

def plot_line_string(ax, linestring, color='black', linewidth=1):
    x, y = get_xy_from_linestring(linestring)
    return ax.plot(x, y, color=color, linewidth=linewidth)

def plot_line_string_list(ax, linestring_list, color='black', linewidth=1):
    lines = []
    for linestring in linestring_list:
        ln = plot_line_string(ax, linestring, color=color, linewidth=linewidth)
        lines.append(ln)
    return lines

def plot_shapely_line(ax, line: LineString, color='blue', linewidth=1):
    """
    Plots a single Shapely LineString on a given matplotlib axis.
    """
    if not line or line.is_empty:
        return None
    # .xy returns two separate arrays for x and y coordinates
    x, y = line.xy
    return ax.plot(x, y, color=color, linewidth=linewidth)

def plot_shapely_multilinestring(ax, multi_line: MultiLineString, linewidth=2, use_distinct_colors=True):
    """
    Plots a Shapely MultiLineString on a given matplotlib axis.
    
    If use_distinct_colors is True, each component LineString will have a
    different color from the 'viridis' colormap. Otherwise, 'blue' is used.
    """
    if not multi_line or multi_line.is_empty:
        return []
        
    lines = []
    # .geoms gives access to the individual geometries (LineStrings)
    num_lines = len(multi_line.geoms)
    
    if use_distinct_colors and num_lines > 0:
        # Get a list of colors from the 'viridis' colormap
        colors = cm.viridis(np.linspace(0, 1, num_lines))
    else:
        # Use a single default color
        colors = ['blue'] * num_lines

    for line, color in zip(multi_line.geoms, colors):
        if isinstance(line, LineString):
            plotted_line = plot_shapely_line(ax, line, color=color, linewidth=linewidth)
            if plotted_line:
                lines.append(plotted_line)
                
    return lines

def debug_map(lanelet_map):
        
    print("=============== MAP DEBUG INFO START ===============")
    
    # === 1. Console Output ===
    
    # Count the different elements
    try:
        num_points = len(lanelet_map.pointLayer)
        num_linestrings = len(lanelet_map.lineStringLayer)
        num_lanelets = len(lanelet_map.laneletLayer)
        num_reg_elements = len(lanelet_map.regulatoryElementLayer)

        print(f"  Points (Points):             {num_points}")
        print(f"  LineStrings (LineStrings):   {num_linestrings}")
        print(f"  Lanelets:                    {num_lanelets}")
        print(f"  Regulatory Elements:         {num_reg_elements}")

    except AttributeError as e:
        print(f"Error accessing basic map layers: {e}")
        print("The lanelet_map object seems incomplete or invalid.")
        print("=============== MAP DEBUG INFO END ===============")
        return

    # === 2. Matplotlib Plot ===
    
    print("Creating Matplotlib plot of the map...")
    
    fig, ax = plt.subplots(figsize=(30, 15))

    # Iterate over all lanelets and draw them
    for lanelet in lanelet_map.laneletLayer:

        # 1. Draw boundaries
        left_x, left_y = get_xy_from_linestring(lanelet.leftBound)
        right_x, right_y = get_xy_from_linestring(lanelet.rightBound)
        
        # Plot bounds in black
        ax.plot(left_x, left_y, color='black', linewidth=1)
        ax.plot(right_x, right_y, color='black', linewidth=1)
        
        # 2. Draw centerline (optional, but helpful)
        center_x, center_y = get_xy_from_linestring(lanelet.centerline)
        ax.plot(center_x, center_y, color='gray', linestyle='--', linewidth=0.5)
        
        # 3. Write ID in the middle of the lanelet
        if center_x:
            mid_index = len(center_x) // 2
            mid_x = center_x[mid_index]
            mid_y = center_y[mid_index]
            
            # Add text label for the ID
            ax.text(mid_x, mid_y, 
                    str(lanelet.id), 
                    color='blue', 
                    fontsize=8, 
                    ha='center',  # horizontal alignment
                    va='center',  # vertical alignment
                    bbox=dict(facecolor='white', alpha=0.5, pad=0.1, edgecolor='none'))

    ax.set_aspect('equal', adjustable='box')
    ax.set_title(f"Lanelet Map Visualization (With IDs)")
    ax.set_xlabel("X (Meters)")
    ax.set_ylabel("Y (Meters)")
    plt.grid(True)

    print("=============== MAP DEBUG INFO END ===============")

    return ax