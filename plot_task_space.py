#!/usr/bin/env python3
import argparse
import csv
import yaml

import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D
import numpy as np


def read_path(path):
    t1, t2 = [], []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t1.append(float(row["theta1"]))
            t2.append(float(row["theta2"]))
    return np.array(t1), np.array(t2)


def read_obstacles(path):
    obs = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            obs.append((float(row["cx"]), float(row["cy"]), float(row["size"])))
    return obs


def forward_kinematics(t1, t2, l1, l2):
    """Returns the x and y coordinates of the base, elbow, and end-effector."""
    x0, y0 = 0.0, 0.0
    x1 = l1 * np.cos(t1)
    y1 = l1 * np.sin(t1)
    x2 = x1 + l2 * np.cos(t1 + t2)
    y2 = y1 + l2 * np.sin(t1 + t2)
    return np.array([x0, x1, x2]), np.array([y0, y1, y2])


def main():
    parser = argparse.ArgumentParser(description="Plot task space for arm planning")
    parser.add_argument("--config", required=True, help="Path to the YAML config file")
    parser.add_argument("--path", default="path_angles.csv", help="Path to path angles CSV")
    parser.add_argument("--obstacles", default="obstacles.csv", help="Path to obstacles CSV")
    args = parser.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    # Kinematics and environment config
    l1 = float(cfg.get("link1_length", 1.0))
    l2 = float(cfg.get("link2_length", 0.9))
    ws_min = float(cfg.get("workspace_min", -1.6))
    ws_max = float(cfg.get("workspace_max", 1.6))
    start_angles = cfg.get("start", [0.0, 0.0])
    goal_angles = cfg.get("goal", [0.0, 0.0])

    # Dynamic outputs: replace 'torus' with 'task' so they don't overwrite C-space plots
    output_pdf = cfg.get("output_pdf", "output.png").replace("torus", "task")
    if "task" not in output_pdf: output_pdf = "task_" + output_pdf
    
    output_video = cfg.get("output_video", "").replace("torus", "task")
    if output_video and "task" not in output_video: output_video = "task_" + output_video
    
    video_fps = int(cfg.get("video_fps", 30))

    # Read planner for dynamic legends and titles
    planner_type = str(cfg.get("planner", "ompl")).lower()
    if planner_type == "ceres":
        path_label = "Ceres Straight-Line EE path"
        title_text = "2-Link Planar Arm Task Space: Ceres Straight-Line"
    else:
        path_label = "RRT* EE path"
        title_text = "2-Link Planar Arm Task Space: RRT*"

    # Load data
    path_t1, path_t2 = read_path(args.path)
    obstacles = read_obstacles(args.obstacles)

    # Plot Setup
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor('black')
    ax.set_facecolor('black')
    
    # Calculate dynamic boundaries ensuring the full arm reach is strictly visible
    max_reach = l1 + l2
    plot_limit = max(abs(ws_min), abs(ws_max), max_reach) * 1.15  # 15% padding
    
    ax.set_xlim(-plot_limit, plot_limit)
    ax.set_ylim(-plot_limit, plot_limit)
    ax.set_aspect('equal')
    
    ax.set_title(title_text, pad=18, color="#d9d9d9", fontsize=14)
    ax.set_xlabel("x (m)", color="#d0d0d0")
    ax.set_ylabel("y (m)", color="#d0d0d0")
    ax.tick_params(colors="#c8c8c8")

    # Dark grid
    ax.grid(color='#333333', linestyle='-', linewidth=0.7)
    for spine in ax.spines.values():
        spine.set_color('#555555')

    # 1. Draw Obstacles
    for cx, cy, size in obstacles:
        rect = Rectangle((cx - size/2, cy - size/2), size, size,
                         facecolor='#4a4a4a', edgecolor='#222222', lw=2, zorder=1)
        ax.add_patch(rect)

    # 2. Draw Start and Goal Configurations
    sx, sy = forward_kinematics(start_angles[0], start_angles[1], l1, l2)
    gx, gy = forward_kinematics(goal_angles[0], goal_angles[1], l1, l2)
    
    ax.plot(sx, sy, color='#66ff99', lw=2.5, alpha=0.35, marker='o', markersize=6, zorder=2)
    ax.plot(gx, gy, color='#66ccff', lw=2.5, alpha=0.35, marker='o', markersize=6, zorder=2)

    # 3. Draw End-Effector Path
    ee_x = l1 * np.cos(path_t1) + l2 * np.cos(path_t1 + path_t2)
    ee_y = l1 * np.sin(path_t1) + l2 * np.sin(path_t1 + path_t2)
    ax.plot(ee_x, ee_y, color='#ffd34d', lw=2.5, zorder=3)

    # Base Marker
    ax.plot([0], [0], marker='s', color='#ffffff', markersize=8, zorder=10)

    # 4. Legend
    legend_handles = [
        Line2D([0], [0], color="#ffd34d", lw=3.0, label=path_label),
        Line2D([0], [0], color="#66ff99", lw=2.5, marker='o', label="Start Config"),
        Line2D([0], [0], color="#66ccff", lw=2.5, marker='o', label="Goal Config"),
        Line2D([0], [0], color="#e0e0e0", lw=4.0, marker='o', 
               markerfacecolor='#ff5555', markeredgecolor='black', markersize=8, label="Robot Arm"),
        Rectangle((0, 0), 1, 1, facecolor='#4a4a4a', edgecolor='#222222', label="Obstacles")
    ]
    legend = ax.legend(handles=legend_handles, loc="upper left", facecolor="#101010", edgecolor="#a0a0a0")
    for text in legend.get_texts():
        text.set_color("#d6d6d6")

    # 5. Animation objects
    moving_arm_line, = ax.plot([], [], color='#e0e0e0', lw=4.0, zorder=4)
    moving_arm_joints, = ax.plot([], [], marker='o', color='#ffffff',
                                 markerfacecolor='#ff5555', markeredgecolor='black',
                                 markersize=8, linestyle='', zorder=5)

    # Update function for the first frame / static image
    def draw_frame(idx):
        xs, ys = forward_kinematics(path_t1[idx], path_t2[idx], l1, l2)
        moving_arm_line.set_data(xs, ys)
        moving_arm_joints.set_data(xs, ys)
        return moving_arm_line, moving_arm_joints

    # Draw the final pose for the static image export
    draw_frame(len(path_t1) - 1)
    
    fig.tight_layout()
    fig.savefig(output_pdf, dpi=200) 
    print(f"Task Space visualization saved to {output_pdf}")

    # 6. Video Export
    if output_video:
        print(f"Rendering animation for {output_video}... this may take a moment.")
        frames = len(path_t1)
        
        def update(frame_idx):
            return draw_frame(frame_idx)

        anim = animation.FuncAnimation(
            fig, update, frames=frames, interval=1000.0 / float(video_fps), blit=True
        )

        video_path = output_video
        if video_path.lower().endswith(".mp4"):
            try:
                writer = animation.FFMpegWriter(fps=video_fps, bitrate=2200)
                anim.save(video_path, writer=writer, dpi=170)
                print(f"Video saved to {video_path}")
                return
            except Exception as exc:
                fallback = video_path.rsplit(".", 1)[0] + ".gif"
                print(f"FFmpeg MP4 export failed ({exc}). Falling back to {fallback}")
                video_path = fallback

        try:
            writer = animation.PillowWriter(fps=video_fps)
            anim.save(video_path, writer=writer, dpi=140)
            print(f"Video saved to {video_path}")
        except Exception as exc:
            print(f"Video export failed ({exc}). Image was still generated: {output_pdf}")


if __name__ == "__main__":
    main()
