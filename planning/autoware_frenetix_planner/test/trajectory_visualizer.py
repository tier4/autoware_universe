import tkinter as tk
from tkinter import filedialog, ttk
import json
import os
import glob
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib import cm

class TrajectoryVisualizer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Interactive Trajectory Visualizer")
        self.geometry("1400x950")

        self.data_files = []
        self.current_file_index = -1
        self.loaded_data = {}
        self.plotted_lines = {} # Maps plot line artist to its trajectory data
        self.feasible_lines = [] # Stores feasible trajectory line artists in order
        self.selected_line = None
        self.border_line = None # Artist for the black border of the selected line
        self.selected_traj_index = tk.IntVar()
        self.current_file_index_var = tk.IntVar()

        # --- Create GUI Layout ---
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.grid_rowconfigure(2, weight=1) # Changed row index for plot
        main_frame.grid_columnconfigure(0, weight=1)

        # -- Top Control Bar
        top_bar_frame = ttk.Frame(main_frame)
        top_bar_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        self.load_button = ttk.Button(top_bar_frame, text="Load Directory", command=self.load_directory)
        self.load_button.pack(side=tk.LEFT, padx=(0, 10))

        # -- File Navigation Section
        file_nav_frame = ttk.LabelFrame(main_frame, text="File Navigation", padding="10")
        file_nav_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        file_nav_frame.columnconfigure(1, weight=1)

        self.prev_button = ttk.Button(file_nav_frame, text="<<", command=self.prev_file, state=tk.DISABLED)
        self.prev_button.grid(row=0, column=0, padx=(0, 5))
        
        self.file_slider = ttk.Scale(file_nav_frame, from_=0, to=0, orient=tk.HORIZONTAL, variable=self.current_file_index_var, command=self.on_file_slider_change, state=tk.DISABLED)
        self.file_slider.grid(row=0, column=1, sticky="ew")

        self.next_button = ttk.Button(file_nav_frame, text=">>", command=self.next_file, state=tk.DISABLED)
        self.next_button.grid(row=0, column=2, padx=(5, 0))

        self.file_label = ttk.Label(file_nav_frame, text="No data loaded", anchor="center")
        self.file_label.grid(row=0, column=3, padx=(10,0))


        # -- Matplotlib Canvas
        plot_container = ttk.Frame(main_frame)
        plot_container.grid(row=2, column=0, sticky="nsew") # Changed row index
        plot_container.rowconfigure(0, weight=1)
        plot_container.columnconfigure(0, weight=1)

        self.fig = Figure(figsize=(10, 7), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_container)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        
        toolbar = NavigationToolbar2Tk(self.canvas, plot_container, pack_toolbar=False)
        toolbar.update()
        toolbar.grid(row=1, column=0, sticky="ew")

        # -- Sidebar for Info and Controls
        sidebar_frame = ttk.Frame(main_frame, padding="10")
        sidebar_frame.grid(row=2, column=1, sticky="ns") # Changed row index
        main_frame.grid_columnconfigure(1, weight=0, minsize=300)

        # -- Trajectory Selector Section
        selector_frame = ttk.LabelFrame(sidebar_frame, text="Trajectory Selector", padding="10")
        selector_frame.pack(fill=tk.X, anchor="n")

        self.traj_slider = ttk.Scale(selector_frame, from_=0, to=199, orient=tk.HORIZONTAL, variable=self.selected_traj_index, command=self.on_slider_change, state=tk.DISABLED)
        self.traj_slider.pack(fill=tk.X, expand=True, pady=(0, 5))

        spinbox_frame = ttk.Frame(selector_frame)
        spinbox_frame.pack(fill=tk.X)
        
        # Validation command for integer-only input
        vcmd = (self.register(self._validate_int), '%P')
        
        self.traj_spinbox = ttk.Spinbox(spinbox_frame, from_=0, to=199, textvariable=self.selected_traj_index, command=self.on_spinbox_change, state=tk.DISABLED, width=5, validate="key", validatecommand=vcmd)
        self.traj_spinbox.pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        # -- Cost Information Section
        cost_frame = ttk.LabelFrame(sidebar_frame, text="Trajectory Costs", padding="10")
        cost_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0), anchor="n")
        
        self.cost_text = tk.Text(cost_frame, wrap=tk.WORD, state=tk.DISABLED)
        self.cost_text.pack(fill=tk.BOTH, expand=True)

        self.ax.set_title('Please load a data directory')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.fig.tight_layout()

    def _validate_int(self, P):
        """Internal validation function for integer-only input in the spinbox."""
        if P.isdigit() or P == "":
            return True
        return False

    def green_to_red_colormap(self):
        return matplotlib.colors.LinearSegmentedColormap.from_list('GreenToRed', ['green', 'yellow', 'red'])

    def load_directory(self):
        dir_path = filedialog.askdirectory(title="Select the directory containing trajectory data")
        if not dir_path: return
        self.data_files = sorted(glob.glob(os.path.join(dir_path, "plot_data_*.json")))
        if not self.data_files:
            self.file_label.config(text=f"No 'plot_data_*.json' files found.")
            self.current_file_index = -1
        else:
            self.current_file_index = 0
            self.file_slider.config(to=len(self.data_files)-1)
            self.plot_current_file()
        self.update_controls()

    def update_controls(self):
        num_files = len(self.data_files)
        has_files = num_files > 0
        
        self.file_slider.config(state=tk.NORMAL if has_files else tk.DISABLED)

        if num_files <= 1:
            self.prev_button.config(state=tk.DISABLED)
            self.next_button.config(state=tk.DISABLED)
        else:
            self.prev_button.config(state=tk.NORMAL if self.current_file_index > 0 else tk.DISABLED)
            self.next_button.config(state=tk.NORMAL if self.current_file_index < num_files - 1 else tk.DISABLED)

        if self.current_file_index != -1:
            filename = os.path.basename(self.data_files[self.current_file_index])
            self.file_label.config(text=f"{filename} ({self.current_file_index + 1}/{num_files})")
            self.current_file_index_var.set(self.current_file_index)
        else:
            self.file_label.config(text="No data loaded")

    def on_file_slider_change(self, event=None):
        new_index = self.current_file_index_var.get()
        if new_index != self.current_file_index:
            self.current_file_index = new_index
            self.plot_current_file()
            self.update_controls()

    def prev_file(self):
        if self.current_file_index > 0:
            self.current_file_index -= 1
            self.plot_current_file()
            self.update_controls()
            
    def next_file(self):
        if self.current_file_index < len(self.data_files) - 1:
            self.current_file_index += 1
            self.plot_current_file()
            self.update_controls()

    def plot_current_file(self):
        if self.current_file_index == -1 or not self.data_files: return
            
        filepath = self.data_files[self.current_file_index]
        with open(filepath, 'r') as f: self.loaded_data = json.load(f)
        
        self.ax.clear()
        self.plotted_lines.clear()
        self.feasible_lines.clear()
        self.selected_line = None
        self.border_line = None
        self.update_cost_display({})

        ref_path = np.array(self.loaded_data.get('reference_path', []))
        if ref_path.any(): self.ax.plot(ref_path[:, 0], ref_path[:, 1], 'g', label='Reference Path')
        
        # Feasible trajectories are assumed to be sorted from best (lowest cost) to worst
        feasible_trajectories = self.loaded_data.get('feasible_trajectories', [])
        norm = matplotlib.colors.Normalize(vmin=0, vmax=len(feasible_trajectories), clip=True)
        mapper = cm.ScalarMappable(norm=norm, cmap=self.green_to_red_colormap())

        for idx, traj in enumerate(feasible_trajectories):
            color = mapper.to_rgba(idx) # Best (idx 0) is green, worst is red
            line, = self.ax.plot(traj['cartesian']['x'], traj['cartesian']['y'], color=color, zorder=20, linewidth=1.0, alpha=1.0, picker=True, pickradius=3)
            self.plotted_lines[line] = traj
            self.feasible_lines.append(line)
        
        optimal_trajectory = self.loaded_data.get('optimal_trajectory', {})
        if optimal_trajectory:
            line, = self.ax.plot(optimal_trajectory['cartesian']['x'], optimal_trajectory['cartesian']['y'], '#0065bd', label='Optimal Trajectory', zorder=30, linewidth=3.5, alpha=1.0, picker=True, pickradius=5)
            self.plotted_lines[line] = optimal_trajectory

        obstacles = np.array(self.loaded_data.get('obstacle_positions', []))
        if obstacles.any(): self.ax.scatter(obstacles[:, 0], obstacles[:, 1], c='m', marker='x', label='Obstacles', zorder=5)

        self.ax.set_xlabel('X [m]'); self.ax.set_ylabel('Y [m]'); self.ax.set_title(f'Frenetix Debug Plot: {os.path.basename(filepath)}')
        self.ax.axis('equal'); self.ax.legend(); self.fig.tight_layout(); self.canvas.draw()
        self.update_selector_controls()
    
    def update_selector_controls(self):
        num_feasible = len(self.feasible_lines)
        if num_feasible > 0:
            self.traj_slider.config(state=tk.NORMAL, to=num_feasible - 1)
            self.traj_spinbox.config(state=tk.NORMAL, to=num_feasible - 1)
            self.selected_traj_index.set(0)
            self.select_trajectory_by_index(0)
        else:
            self.traj_slider.config(state=tk.DISABLED)
            self.traj_spinbox.config(state=tk.DISABLED)

    def on_pick(self, event):
        line = event.artist
        if line not in self.plotted_lines: return
        
        if line in self.feasible_lines:
            index = self.feasible_lines.index(line)
            self.selected_traj_index.set(index)
        
        self.highlight_line(line)

    def on_slider_change(self, event=None):
        self.select_trajectory_by_index(self.selected_traj_index.get())

    def on_spinbox_change(self):
        self.select_trajectory_by_index(self.selected_traj_index.get())

    def select_trajectory_by_index(self, index):
        if not self.feasible_lines or not (0 <= index < len(self.feasible_lines)):
            return
        line_to_select = self.feasible_lines[index]
        self.highlight_line(line_to_select)

    def highlight_line(self, line_to_highlight):
        # Reset previous selection
        if self.selected_line:
            self.selected_line.set_linewidth(getattr(self.selected_line, 'original_linewidth', 1.0))
            self.selected_line.set_zorder(getattr(self.selected_line, 'original_zorder', 20))
        if self.border_line:
            self.border_line.remove()
            self.border_line = None

        # Highlight new selection
        self.selected_line = line_to_highlight
        if not hasattr(self.selected_line, 'original_linewidth'):
             self.selected_line.original_linewidth = self.selected_line.get_linewidth()
             self.selected_line.original_zorder = self.selected_line.get_zorder()

        # Add black border by drawing a thicker black line underneath
        x_data, y_data = self.selected_line.get_data()
        border_width = self.selected_line.original_linewidth * 4.0 + 2
        self.border_line, = self.ax.plot(x_data, y_data, color='black', linewidth=border_width, zorder=39, alpha=1.0)

        # Apply highlight to the original line
        self.selected_line.set_linewidth(self.selected_line.original_linewidth * 4.0)
        self.selected_line.set_zorder(40) # Bring to front

        self.update_cost_display(self.plotted_lines.get(line_to_highlight, {}).get('costMap', {}))
        self.canvas.draw_idle()

    def update_cost_display(self, cost_map):
        self.cost_text.config(state=tk.NORMAL)
        self.cost_text.delete('1.0', tk.END)
        if not cost_map:
            self.cost_text.insert(tk.END, "No trajectory selected.")
        else:
            try:
                # The second value in the list is the weighted cost. Sum these for the total.
                total_cost = sum(v[1] for v in cost_map.values() if isinstance(v, list) and len(v) == 2)
                self.cost_text.insert(tk.END, f"TOTAL WEIGHTED COST:\n {total_cost:.4f}\n\n" + "-"*20 + "\n\n")
                
                # Sort items by the weighted cost (the second element in the list)
                sorted_costs = sorted(
                    cost_map.items(), 
                    key=lambda item: item[1][1] if isinstance(item[1], list) and len(item[1]) == 2 else 0, 
                    reverse=True
                )
                
                for key, value in sorted_costs:
                    if isinstance(value, list) and len(value) == 2:
                        unweighted = value[0]
                        weighted = value[1]
                        self.cost_text.insert(tk.END, f"{key}:\n  Unweighted: {unweighted:.4f}\n  Weighted:   {weighted:.4f}\n\n")
                    else:
                        # Fallback for old data format or unexpected values
                        self.cost_text.insert(tk.END, f"{key}:\n {str(value)}\n\n")

            except (TypeError, IndexError) as e:
                self.cost_text.delete('1.0', tk.END)
                self.cost_text.insert(tk.END, f"Error processing cost map:\n{e}\n\nData might be in an old format.\n\nRaw Data:\n{json.dumps(cost_map, indent=2)}")

        self.cost_text.config(state=tk.DISABLED)

if __name__ == '__main__':
    app = TrajectoryVisualizer()
    app.mainloop()

