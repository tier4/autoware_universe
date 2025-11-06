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

from autoware_frenetix_planner.frenetix_motion_planner import FrenetixMotionPlanner
from autoware_frenetix_planner.frenetix_motion_planner import CartesianState, CurvilinearState

class DummyLogger:
    """A dummy logger that mimics the ROS 2 logger and prints to the console."""
    def info(self, msg):
        print(f"[INFO] {msg}")
    def warn(self, msg):
        print(f"[WARN] {msg}")
    def debug(self, msg):
        print(f"[DEBUG] {msg}")
    def error(self, msg):
        print(f"[ERROR] {msg}")
    def get_name(self):
        return "dummy_logger"

class TrajectoryVisualizer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Interactive Trajectory Visualizer")
        self.geometry("1400x950")

        self.data_files = []
        self.current_file_index = -1
        self.loaded_data = {}
        self.current_mode = None
        self.plotted_lines = {} # Maps plot line artist to its trajectory data
        self.feasible_lines = [] # Stores feasible trajectory line artists in order
        self.selected_line = None
        self.border_line = None # Artist for the black border of the selected line
        self.selected_traj_index = tk.IntVar()
        self.current_file_index_var = tk.IntVar()

        # --- GUI Layout ---
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.grid_rowconfigure(2, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)

        # -- Top Control Bar
        top_bar_frame = ttk.Frame(main_frame)
        top_bar_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        self.load_button = ttk.Button(top_bar_frame, text="Load Directory", command=self.load_directory)
        self.load_button.pack(side=tk.LEFT, padx=(0, 10))
        self.mode_label = ttk.Label(top_bar_frame, text="Mode: N/A", font=("TkDefaultFont", 10, "bold"))
        self.mode_label.pack(side=tk.LEFT)

        # -- File Navigation
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
        plot_container.grid(row=2, column=0, sticky="nsew")
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
        sidebar_frame.grid(row=2, column=1, sticky="ns")
        main_frame.grid_columnconfigure(1, weight=0, minsize=350)

        # -- Trajectory Selector
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
        
        # -- Info Display (Tabs)
        self.notebook = ttk.Notebook(sidebar_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=(10, 0), anchor="n")
        
        self.cost_text = tk.Text(self.notebook, wrap=tk.WORD, state=tk.DISABLED)
        self.state_text = tk.Text(self.notebook, wrap=tk.WORD, state=tk.DISABLED)
        
        self.notebook.add(self.cost_text, text='Costs')
        self.notebook.add(self.state_text, text='State')

        self.ax.set_title('Please load a data directory')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.fig.tight_layout()

        # initialize planner
        # self.planner = FrenetixMotionPlanner(logger=DummyLogger(), logging_enabled=False)

    def _validate_int(self, P):
        """Internal validation function for integer-only input in the spinbox."""
        if P.isdigit() or P == "":
            return True
        return False

    def green_to_red_colormap(self):
        return matplotlib.colors.LinearSegmentedColormap.from_list('GreenToRed', ['green', 'yellow', 'red'])

    def _serialize_trajectory(self, trajectory):
        """Converts a Trajectory object into a dictionary for UI display, or returns it if already a dict."""
        if isinstance(trajectory, dict):
            return trajectory  # Already a dict, do nothing
        if trajectory is None:
            return None
        
        # return dict
        return {
            'cartesian': {
                'x': list(trajectory.cartesian.x),
                'y': list(trajectory.cartesian.y)
            },
            'costMap': trajectory.costMap
        }

    def load_directory(self):
        dir_path = filedialog.askdirectory(title="Select directory")
        if not dir_path: return
        self.data_files = sorted(glob.glob(os.path.join(dir_path, "plot_data_*.json")))
        if not self.data_files:
            self.file_label.config(text=f"No 'plot_data_*.json' files found.")
            self.current_file_index = -1
            self.current_mode = None
        else:
            self.current_file_index = 0
            # Auto-detect mode from first file
            with open(self.data_files[0], 'r') as f:
                first_data = json.load(f)
                self.current_mode = first_data.get("data_mode", "trajectories") # Default to trajectories for old data
            self.mode_label.config(text=f"Mode: {self.current_mode.capitalize()}")
            self.file_slider.config(to=len(self.data_files)-1)
            self.plot_current_file()
        self.update_controls()

    def update_controls(self):
        num_files = len(self.data_files)
        has_files = num_files > 0
        
        self.file_slider.config(state=tk.NORMAL if has_files else tk.DISABLED)
        
        if num_files <= 1:
            self.prev_button.config(state=tk.DISABLED); self.next_button.config(state=tk.DISABLED)
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
            self.current_file_index -= 1; self.plot_current_file(); self.update_controls()
            
    def next_file(self):
        if self.current_file_index < len(self.data_files) - 1:
            self.current_file_index += 1; self.plot_current_file(); self.update_controls()

    def plot_current_file(self):
        if self.current_file_index == -1 or not self.data_files: return
        filepath = self.data_files[self.current_file_index]
        with open(filepath, 'r') as f: self.loaded_data = json.load(f)
        
        self.ax.clear(); self.plotted_lines.clear(); self.feasible_lines.clear()
        self.selected_line = None; self.border_line = None
        self.update_cost_display({}); self.update_state_display({})

        # Load obstacle predictions
        obstacle_predictions = self.loaded_data.get('obstacle_predictions', [])

        if self.current_mode == "recompute":
            # Recalculate trajectories from saved inputs
            cart_state = self.loaded_data.get('cartesian_state', {})
            curv_state = self.loaded_data.get('curvilinear_state', {})
            ref_path = np.array(self.loaded_data.get('reference_path', []))
            obstacles = np.array(self.loaded_data.get('obstacle_positions', []))
            
            optimal, feasible, infeasible = self.recalculate_trajectories(cart_state, curv_state, ref_path, obstacles)
            
            self._plot_trajectories(optimal, feasible, infeasible, ref_path, obstacles)
            self.update_state_display(self.loaded_data)
        else: # "trajectories" mode
            optimal = self.loaded_data.get('optimal_trajectory')
            feasible = self.loaded_data.get('feasible_trajectories', [])
            infeasible = self.loaded_data.get('infeasible_trajectories', [])
            ref_path = np.array(self.loaded_data.get('reference_path', []))
            obstacles = np.array(self.loaded_data.get('obstacle_positions', []))
            self._plot_trajectories(optimal, feasible, infeasible, ref_path, obstacle_predictions)
        
        self.ax.set_xlabel('X [m]'); self.ax.set_ylabel('Y [m]'); self.ax.set_title(f'Debug Plot: {os.path.basename(filepath)}')
        self.ax.axis('equal'); self.ax.legend(); self.fig.tight_layout(); self.canvas.draw()
        self.update_selector_controls()

    def _plot_trajectories(self, optimal_trajectory, feasible_trajectories, infeasible_trajectories, reference_path, obstacle_predictions):
        if feasible_trajectories is None: feasible_trajectories = []
        if infeasible_trajectories is None: infeasible_trajectories = []
        if obstacle_predictions is None: obstacle_predictions = []
        if reference_path.any(): self.ax.plot(reference_path[:, 0], reference_path[:, 1], 'g', label='Reference Path')

        for traj in infeasible_trajectories:
            if traj:
                x_data = traj.cartesian.x if not isinstance(traj, dict) else traj['cartesian']['x']
                y_data = traj.cartesian.y if not isinstance(traj, dict) else traj['cartesian']['y']
                self.ax.plot(x_data, y_data, color='#808080', zorder=19, linewidth=0.8, alpha=0.4, picker=False)
        
        norm = matplotlib.colors.Normalize(vmin=0, vmax=len(feasible_trajectories), clip=True)
        mapper = cm.ScalarMappable(norm=norm, cmap=self.green_to_red_colormap())

        for idx, traj in enumerate(feasible_trajectories):
            color = mapper.to_rgba(idx)
            x_data = traj.cartesian.x if not isinstance(traj, dict) else traj['cartesian']['x']
            y_data = traj.cartesian.y if not isinstance(traj, dict) else traj['cartesian']['y']
            line, = self.ax.plot(x_data, y_data, color=color, zorder=20, linewidth=1.0, alpha=1.0, picker=True, pickradius=3)
            self.plotted_lines[line] = self._serialize_trajectory(traj)
            self.feasible_lines.append(line)
        
        if optimal_trajectory:
            x_data = optimal_trajectory.cartesian.x if not isinstance(optimal_trajectory, dict) else optimal_trajectory['cartesian']['x']
            y_data = optimal_trajectory.cartesian.y if not isinstance(optimal_trajectory, dict) else optimal_trajectory['cartesian']['y']
            line, = self.ax.plot(x_data, y_data, '#0065bd', label='Optimal Trajectory', zorder=30, linewidth=3.5, alpha=1.0, picker=True, pickradius=5)
            self.plotted_lines[line] = self._serialize_trajectory(optimal_trajectory)

        # if obstacles.any(): self.ax.scatter(obstacles[:, 0], obstacles[:, 1], c='m', marker='x', label='Obstacles', zorder=10000)

        # Plot predicted obstacle paths
        first_pred = True
        for obstacle in obstacle_predictions:
            path = obstacle.get('predicted_path', [])
            if path:
                path_x = [step['position'][0] for step in path]
                path_y = [step['position'][1] for step in path]
                
                label = 'Predicted Path' if first_pred else None
                self.ax.plot(path_x, path_y, '-o', color='black', zorder=10000, label=label, markersize=2)
                first_pred = False

    def update_selector_controls(self):
        num_feasible = len(self.feasible_lines)
        if num_feasible > 0:
            self.traj_slider.config(state=tk.NORMAL, to=num_feasible - 1)
            self.traj_spinbox.config(state=tk.NORMAL, to=num_feasible - 1)
            self.selected_traj_index.set(0)
            self.select_trajectory_by_index(0)
        else:
            self.traj_slider.config(state=tk.DISABLED); self.traj_spinbox.config(state=tk.DISABLED)

    def on_pick(self, event):
        line = event.artist
        if line not in self.plotted_lines: return
        if line in self.feasible_lines: self.selected_traj_index.set(self.feasible_lines.index(line))
        self.highlight_line(line)

    def on_slider_change(self, event=None):
        self.select_trajectory_by_index(self.selected_traj_index.get())

    def on_spinbox_change(self):
        self.select_trajectory_by_index(self.selected_traj_index.get())

    def select_trajectory_by_index(self, index):
        if not self.feasible_lines or not (0 <= index < len(self.feasible_lines)): return
        self.highlight_line(self.feasible_lines[index])

    def highlight_line(self, line_to_highlight):
        # Reset previous selection
        if self.selected_line:
            self.selected_line.set_linewidth(getattr(self.selected_line, 'original_linewidth', 1.0))
            self.selected_line.set_zorder(getattr(self.selected_line, 'original_zorder', 20))
        if self.border_line:
            self.border_line.remove(); self.border_line = None

        # Highlight new selection
        self.selected_line = line_to_highlight
        if not hasattr(self.selected_line, 'original_linewidth'):
             self.selected_line.original_linewidth = self.selected_line.get_linewidth()
             self.selected_line.original_zorder = self.selected_line.get_zorder()

        # Add black border by drawing a thicker black line underneath
        x_data, y_data = self.selected_line.get_data()
        border_width = self.selected_line.original_linewidth * 4.0 + 2
        self.border_line, = self.ax.plot(x_data, y_data, color='black', marker='o', markersize=border_width*1.5, linewidth=border_width, zorder=39, alpha=1.0)

        # Apply highlight to the original line
        self.selected_line.set_linewidth(self.selected_line.original_linewidth * 4.0)
        self.selected_line.set_zorder(40) # Bring to front

        self.update_cost_display(self.plotted_lines.get(line_to_highlight, {}).get('costMap', {}))
        self.canvas.draw_idle()

    def update_state_display(self, data):
        self.state_text.config(state=tk.NORMAL)
        self.state_text.delete('1.0', tk.END)
        cart_state = data.get("cartesian_state")
        curv_state = data.get("curvilinear_state")
        
        if not cart_state and not curv_state:
            self.state_text.insert(tk.END, "State information only available in 'recompute' mode.")
        else:
            if cart_state:
                self.state_text.insert(tk.END, "CARTESIAN STATE:\n" + json.dumps(cart_state, indent=2) + "\n\n")
            if curv_state:
                self.state_text.insert(tk.END, "CURVILINEAR STATE:\n" + json.dumps(curv_state, indent=2))
        self.state_text.config(state=tk.DISABLED)

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
                sorted_costs = sorted(cost_map.items(), key=lambda i: i[1][1] if isinstance(i[1], list) and len(i[1]) == 2 else 0, reverse=True)
                for key, value in sorted_costs:
                    if isinstance(value, list) and len(value) == 2:
                        self.cost_text.insert(tk.END, f"{key}:\n  Unweighted: {value[0]:.4f}\n  Weighted:   {value[1]:.4f}\n\n")
                    else:
                        self.cost_text.insert(tk.END, f"{key}:\n {str(value)}\n\n")

            except (TypeError, IndexError) as e:
                self.cost_text.delete('1.0', tk.END)
                self.cost_text.insert(tk.END, f"Error processing cost map:\n{e}\n\nRaw Data:\n{json.dumps(cost_map, indent=2)}")
        self.cost_text.config(state=tk.DISABLED)

    def recalculate_trajectories(self, cartesian_state, curvilinear_state, reference_path, obstacle_positions):

        # set reference path (sets path and creates curvilinear coordinate system)
        self.planner.set_reference_path(reference_path)

        # set cartesian state
        self.planner.cartesian_state = CartesianState(position=cartesian_state.get('position', [0.0, 0.0]),
                                                      orientation=cartesian_state.get('orientation', 0.0),
                                                      velocity=cartesian_state.get('velocity', 0.0),
                                                      acceleration=cartesian_state.get('acceleration', 0.0),
                                                      steering_angle=cartesian_state.get('steering_angle', 0.0),
                                                      )


        # set curvilinear state
        self.planner.curvilinear_state = CurvilinearState(s=curvilinear_state.get('s', 0.0),
                                                          s_dot=curvilinear_state.get('s_dot', 0.0),
                                                          s_ddot=curvilinear_state.get('s_ddot', 0.0),
                                                          d=curvilinear_state.get('d', 0.0),
                                                          d_dot=curvilinear_state.get('d_dot', 0.0),
                                                          d_ddot=curvilinear_state.get('d_ddot', 0.0)
                                                          )
        
        # set desired velocity
        self.planner.set_desired_velocity(desired_velocity=5, current_speed=self.planner.curvilinear_state.s_dot)

        # set current d position for lateral sampling
        self.planner.sampling_handler.set_d_sampling(self.planner.curvilinear_state.d)

        # set changing cost functions
        self.planner._trajectory_handler_set_changing_cost_functions()

        # Initialization of while loop
        optimal_trajectory = None
        feasible_trajectories = []
        infeasible_trajectories = []
        sampling_level = self.planner.params['sampling_min']
        max_sampling_level = self.planner.params['sampling_max']

        # sample until trajectory has been found or sampling sets are empty
        while optimal_trajectory is None and sampling_level < max_sampling_level:
            self.planner.handler.reset_Trajectories()

            # generate sampling matrix with current sampling level
            sampling_matrix = self.planner._generate_sampling_matrix(sampling_level)

            # generate trajectories
            self.planner.handler.generate_trajectories(sampling_matrix, False)

            self.planner.handler.evaluate_all_current_functions_concurrent(True)

            feasible_trajectories = []
            infeasible_trajectories = []
            for idx, trajectory in enumerate(self.planner.handler.get_sorted_trajectories()):
                # check if trajectory is feasible
                if trajectory.feasible:
                    feasible_trajectories.append(trajectory)
                elif trajectory.valid:
                    infeasible_trajectories.append(trajectory)

            if self.planner.curvilinear_state.s_dot < 1.0:
              optimal_trajectory = feasible_trajectories[0] if feasible_trajectories else infeasible_trajectories[0]
            else:
              optimal_trajectory = feasible_trajectories[0] if feasible_trajectories else None

        # increase sampling level (i.e., density) if no optimal trajectory could be found
        sampling_level += 1

        return optimal_trajectory, feasible_trajectories, infeasible_trajectories

if __name__ == '__main__':
    app = TrajectoryVisualizer()
    app.mainloop()

