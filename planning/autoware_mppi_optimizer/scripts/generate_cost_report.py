import glob
import os

import matplotlib.pyplot as plt
import pandas as pd


def generate_report():
    csv_files = glob.glob("/tmp/mppi_report_*.csv")
    if not csv_files:
        print("No report CSVs found. Did you run the tests with MPPI_REPORT=1 ?")
        return

    print("=" * 70)
    print(f"{'MPPI RAW COST SANDBOX REPORT':^70}")
    print("=" * 70)

    for f in csv_files:
        test_name = os.path.basename(f).replace("mppi_report_", "").replace(".csv", "")
        df = pd.read_csv(f)

        # --- Print Full Cost Table ---
        print(f"\n[ TEST CASE: {test_name.upper()} ]")
        print("-" * 70)
        print(f"{'Cost Component':<35} | {'Summed Raw Value':<20}")
        print("-" * 70)

        # Dynamically grab all exported cost columns
        cost_cols = [col for col in df.columns if col.startswith("cost_")]
        for col in cost_cols:
            display_name = col.replace("cost_", "").replace("_", " ").title()
            val = df[col].sum()
            print(f"{display_name:<35} | {val:10.4f}")

        # --- Generate Plots ---
        fig, axs = plt.subplots(1, 3, figsize=(15, 4))
        fig.suptitle(f"Trajectory Profile: {test_name}", fontsize=14)

        # 1. 2D Path
        axs[0].plot(df["x"], df["y"], marker="o", markersize=3, label="Ego Path")
        axs[0].axhline(0, color="red", linestyle="--", alpha=0.5, label="Reference")
        axs[0].set_title("2D Trajectory")
        axs[0].set_xlabel("X [m]")
        axs[0].set_ylabel("Y [m]")
        axs[0].legend()

        # 2. Velocity Profile
        axs[1].plot(df["step"], df["v"], color="orange", linewidth=2, label="Ego Velocity")
        axs[1].plot(
            df["step"], df["v_ref"], color="blue", linestyle="--", alpha=0.7, label="Reference"
        )
        axs[1].set_title("Velocity vs. Step")
        axs[1].set_xlabel("Horizon Step")
        axs[1].set_ylabel("Velocity [m/s]")
        axs[1].legend()

        # 3. Steering Command Profile
        axs[2].plot(df["step"], df["steer_cmd"], color="green", linewidth=2)
        axs[2].set_title("Steering Command vs. Step")
        axs[2].set_xlabel("Horizon Step")
        axs[2].set_ylabel("Steering Angle [rad]")

        plt.tight_layout()
        plot_path = f"/tmp/{test_name}_plot.png"
        plt.savefig(plot_path)
        print(f"\n=> Plot saved to {plot_path}")
        plt.close()


if __name__ == "__main__":
    generate_report()
