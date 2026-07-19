#!/usr/bin/env python3
import argparse
import csv
import math
import yaml

import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.colors import LightSource
from matplotlib.lines import Line2D
import numpy as np


def read_collision_map(path):
    t1, t2, valid = [], [], []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t1.append(float(row["theta1"]))
            t2.append(float(row["theta2"]))
            valid.append(int(row["valid"]))
    return np.array(t1), np.array(t2), np.array(valid)


def read_path(path):
    t1, t2 = [], []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t1.append(float(row["theta1"]))
            t2.append(float(row["theta2"]))
    return np.array(t1), np.array(t2)


def torus_xyz(theta1, theta2, major, minor):
    x = (major + minor * np.cos(theta2)) * np.cos(theta1)
    y = (major + minor * np.cos(theta2)) * np.sin(theta1)
    z = minor * np.sin(theta2)
    return x, y, z


def main():
    parser = argparse.ArgumentParser(description="Plot torus PDF for arm planning")
    parser.add_argument("--config", required=True, help="Path to the YAML config file")
    parser.add_argument("--map", default="collision_map.csv", help="Path to collision map CSV")
    parser.add_argument("--path", default="path_angles.csv", help="Path to path angles CSV")
    parser.add_argument("--obstacles", default="obstacles.csv", help="Path to obstacles CSV")
    parser.add_argument("--torus-u-res", type=int, default=240)
    parser.add_argument("--torus-v-res", type=int, default=160)
    args = parser.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    
    # Read planner for dynamic legends and titles
    planner_type = str(cfg.get("planner", "ompl")).lower()
    if planner_type == "ceres":
        path_label = "Ceres Straight-Line path"
        title_text = "2-Link Planar Arm on T^2 with Ceres Straight-Line and FCL"
    else:
        path_label = "RRT* path"
        title_text = "2-Link Planar Arm on T^2 with RRT* and FCL"

    major = float(cfg.get("torus_major_radius", 2.0))
    minor = float(cfg.get("torus_minor_radius", 0.7))
    start_angles = cfg.get("start", [0.0, 0.0])
    goal_angles = cfg.get("goal", [0.0, 0.0])
    
    output_pdf = cfg.get("output_pdf", "output.pdf")
    output_video = cfg.get("output_video", "")
    video_frames = int(cfg.get("video_frames", 180))
    video_fps = int(cfg.get("video_fps", 30))
    
    elev = float(cfg.get("camera_elev", 40.0))
    azim = float(cfg.get("camera_azim", 0.0))
    
    print(f"Loaded Camera Configuration -> Elevation: {elev}°, Azimuth: {azim}°")

    map_t1, map_t2, map_valid = read_collision_map(args.map)
    path_t1, path_t2 = read_path(args.path)

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")

    fig.patch.set_facecolor("black")
    ax.set_facecolor("black")

    u_res = max(48, args.torus_u_res)
    v_res = max(32, args.torus_v_res)
    u = np.linspace(-math.pi, math.pi, u_res)
    v = np.linspace(-math.pi, math.pi, v_res)
    U, V = np.meshgrid(u, v)
    
    surf_x, surf_y, surf_z = torus_xyz(U, V, major, minor)

    base_rgba = np.array([31 / 255.0, 111 / 255.0, 191 / 255.0, 1.0])
    obstacle_rgba = np.array([74 / 255.0, 74 / 255.0, 74 / 255.0, 1.0])
    path_rgba = np.array([1.0, 211 / 255.0, 77 / 255.0, 1.0])
    start_rgba = np.array([102 / 255.0, 1.0, 153 / 255.0, 1.0])
    goal_rgba = np.array([102 / 255.0, 204 / 255.0, 1.0, 1.0])

    facecolors = np.tile(base_rgba, (V.shape[0], U.shape[1], 1))

    def blend_layer(base_colors, layer_color, weight):
        w = np.clip(weight, 0.0, 1.0)[..., None]
        return base_colors * (1.0 - w) + layer_color[None, None, :] * w

    # 1. Render Obstacles
    cell_count = map_valid.size
    grid_size = int(round(math.sqrt(cell_count)))
    if grid_size * grid_size == cell_count:
        valid_grid = map_valid.reshape(grid_size, grid_size).astype(float)
        obstacle_grid = 1.0 - valid_grid

        t1_norm = (U + math.pi) / (2.0 * math.pi)
        t2_norm = (V + math.pi) / (2.0 * math.pi)
        su = np.mod(t1_norm * grid_size, grid_size)
        sv = np.mod(t2_norm * grid_size, grid_size)
        iu0 = np.floor(su).astype(int)
        iv0 = np.floor(sv).astype(int)
        iu1 = (iu0 + 1) % grid_size
        iv1 = (iv0 + 1) % grid_size
        fu = su - iu0
        fv = sv - iv0

        o00 = obstacle_grid[iu0, iv0]
        o10 = obstacle_grid[iu1, iv0]
        o01 = obstacle_grid[iu0, iv1]
        o11 = obstacle_grid[iu1, iv1]

        obstacle_field = (
            (1.0 - fu) * (1.0 - fv) * o00
            + fu * (1.0 - fv) * o10
            + (1.0 - fu) * fv * o01
            + fu * fv * o11
        )
        
        obstacle_weight = (obstacle_field >= 0.5).astype(float)
        facecolors = blend_layer(facecolors, obstacle_rgba, obstacle_weight)

    # 2. Render Path and Markers
    U32 = U.astype(np.float32)
    V32 = V.astype(np.float32)

    def get_angular_dist2(t1, t2):
        du = np.abs(((U32 - t1 + np.pi) % (2.0 * np.pi)) - np.pi)
        dv = np.abs(((V32 - t2 + np.pi) % (2.0 * np.pi)) - np.pi)
        return du * du + dv * dv

    grid_step_u = (2.0 * math.pi) / max(1, (u_res - 1))

    path_min_dist2 = np.full(U.shape, np.inf, dtype=np.float32)
    path_chunk = 64
    for idx in range(0, len(path_t1), path_chunk):
        chunk_t1 = path_t1[idx:idx + path_chunk].astype(np.float32)
        chunk_t2 = path_t2[idx:idx + path_chunk].astype(np.float32)
        du = np.abs(((U32[None, :, :] - chunk_t1[:, None, None] + np.pi) % (2.0 * np.pi)) - np.pi)
        dv = np.abs(((V32[None, :, :] - chunk_t2[:, None, None] + np.pi) % (2.0 * np.pi)) - np.pi)
        dist2 = du * du + dv * dv
        path_min_dist2 = np.minimum(path_min_dist2, np.min(dist2, axis=0))

    # FIX: Increased multiplier from 0.5 to 1.5 to guarantee intersection with the discrete geometry faces
    path_radius = 1.5 * grid_step_u
    path_weight = (path_min_dist2 <= path_radius**2).astype(float)
    facecolors = blend_layer(facecolors, path_rgba, path_weight)

    marker_radius = 1.5 * grid_step_u
    s_dist2 = get_angular_dist2(start_angles[0], start_angles[1])
    g_dist2 = get_angular_dist2(goal_angles[0], goal_angles[1])
    
    start_weight = (s_dist2 <= marker_radius**2).astype(float)
    goal_weight = (g_dist2 <= marker_radius**2).astype(float)
    
    facecolors = blend_layer(facecolors, start_rgba, start_weight)
    facecolors = blend_layer(facecolors, goal_rgba, goal_weight)

    facecolors = np.clip(facecolors, 0.0, 1.0)

    # 3. Render Surface
    top_light = LightSource(azdeg=0.0, altdeg=90.0)
    ax.plot_surface(
        surf_x, surf_y, surf_z,
        facecolors=facecolors,
        rcount=v_res, ccount=u_res,
        alpha=1.0, linewidth=0,
        antialiased=True, shade=True, lightsource=top_light
    )

    # 4. Axes & Camera Configuration
    ax.set_title(title_text, pad=18, color="#d9d9d9")
    ax.set_xlabel("x", color="#d0d0d0")
    ax.set_ylabel("y", color="#d0d0d0")
    ax.set_zlabel("z", color="#d0d0d0")
    ax.tick_params(colors="#c8c8c8")

    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((0.0, 0.0, 0.0, 1.0))
        axis._axinfo["grid"]["color"] = (0.75, 0.75, 0.75, 0.45)
        axis._axinfo["grid"]["linewidth"] = 0.7

    limit_xy = major + minor
    limit_z = minor

    ax.set_xlim(-limit_xy, limit_xy)
    ax.set_ylim(-limit_xy, limit_xy)
    ax.set_zlim(-limit_z, limit_z)
    ax.set_box_aspect((1.0, 1.0, limit_z / limit_xy))
    ax.view_init(elev=elev, azim=azim)

    # Dynamically inject the label for the path line
    legend_handles = [
        Line2D([0], [0], color="#ffd34d", lw=4.0, label=path_label),
        Line2D([0], [0], marker="o", linestyle="", markersize=9,
               markerfacecolor="#66ff99", markeredgecolor="black", label="start"),
        Line2D([0], [0], marker="o", linestyle="", markersize=9,
               markerfacecolor="#66ccff", markeredgecolor="black", label="goal"),
    ]
    legend = ax.legend(handles=legend_handles, loc="upper left", facecolor="#101010", edgecolor="#a0a0a0")
    for text in legend.get_texts():
        text.set_color("#d6d6d6")

    fig.tight_layout()
    fig.savefig(output_pdf, dpi=200) 
    print(f"Visualization saved to {output_pdf}")

    if output_video:
        frames = max(24, video_frames)
        fps = max(10, video_fps)

        def update(frame_idx):
            current_azim = azim + (360.0 * frame_idx) / float(frames)
            ax.view_init(elev=elev, azim=current_azim)
            return ()

        print(f"Rendering {frames} frames for {output_video}... this may take a moment.")
        anim = animation.FuncAnimation(
            fig, update, frames=frames, interval=1000.0 / float(fps), blit=False
        )

        video_path = output_video
        if video_path.lower().endswith(".mp4"):
            try:
                writer = animation.FFMpegWriter(fps=fps, bitrate=2200)
                anim.save(video_path, writer=writer, dpi=170)
                print(f"Video saved to {video_path}")
                return
            except Exception as exc:
                fallback = video_path.rsplit(".", 1)[0] + ".gif"
                print(f"FFmpeg MP4 export failed ({exc}). Falling back to {fallback}")
                video_path = fallback

        try:
            writer = animation.PillowWriter(fps=fps)
            anim.save(video_path, writer=writer, dpi=140)
            print(f"Video saved to {video_path}")
        except Exception as exc:
            print(f"Video export failed ({exc}). PDF was still generated: {output_pdf}")


if __name__ == "__main__":
    main()
