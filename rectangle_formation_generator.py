#!/usr/bin/env python3
"""
Rectangle formation generator for 20 robots.

Generates JSON files for robots moving in a rectangular formation,
with fixed relative positions that rotate as a rigid body.

Specification: docs/rectangle_formation_spec.md
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ============================================================================
# Type Definitions
# ============================================================================

Point = Tuple[float, float]  # Pixel coordinates (x, y)
PointMM = Tuple[float, float]  # Physical coordinates in mm (x, y)
Heading = float  # Angle in degrees [0, 360)

# ============================================================================
# Field Constants (Pixel Coordinates)
# ============================================================================

FIELD_MIN_X_PX = 34.0
FIELD_MAX_X_PX = 949.0
FIELD_MIN_Y_PX = 35.0
FIELD_MAX_Y_PX = 898.0

FIELD_WIDTH_PX = FIELD_MAX_X_PX - FIELD_MIN_X_PX
FIELD_HEIGHT_PX = FIELD_MAX_Y_PX - FIELD_MIN_Y_PX

# ============================================================================
# Coordinate Conversion Constants
# ============================================================================

SCALE_X_MM_PER_PX = 1.377  # mm per pixel (X direction)
SCALE_Y_MM_PER_PX = 1.376  # mm per pixel (Y direction)

SCALE_X_PX_PER_MM = 1.0 / SCALE_X_MM_PER_PX
SCALE_Y_PX_PER_MM = 1.0 / SCALE_Y_MM_PER_PX

# Field bounds in mm
FIELD_MIN_X_MM = FIELD_MIN_X_PX * SCALE_X_MM_PER_PX
FIELD_MAX_X_MM = FIELD_MAX_X_PX * SCALE_X_MM_PER_PX
FIELD_MIN_Y_MM = FIELD_MIN_Y_PX * SCALE_Y_MM_PER_PX
FIELD_MAX_Y_MM = FIELD_MAX_Y_PX * SCALE_Y_MM_PER_PX

# ============================================================================
# Robot Physical Constants
# ============================================================================

# Robot dimensions (mm)
# Based on: 1260mm / 17.5 ≈ 72mm (long side), 1187mm / 36 ≈ 33mm (short side)
ROBOT_LENGTH_MM = 72.0  # Long side (when rotated 90° along X axis)
ROBOT_WIDTH_MM = 33.0   # Short side (when at 0° along Y axis)

# Diagonal length for rotation safety
ROBOT_DIAGONAL_MM = math.sqrt(ROBOT_LENGTH_MM**2 + ROBOT_WIDTH_MM**2)  # ≈ 79.2mm

# Grid pitch (center-to-center distance)
# Diagonal (79.2mm) + safety margin (20.8mm) = 100.0mm
# Increased for better stability and collision avoidance
# Column spacing (long side, 5 columns) is increased to avoid collisions during rotation
GRID_PITCH_COL_MM = 200.0  # Column spacing (X direction, long side)
GRID_PITCH_ROW_MM = 150.0  # Row spacing (Y direction, short side)

# ============================================================================
# Formation Constants
# ============================================================================

NUM_ROBOTS = 20
FORMATION_COLS = 5  # Local X axis (left-right)
FORMATION_ROWS = 4  # Local Y axis (forward-backward)

# Formation size in mm
FORMATION_WIDTH_MM = (FORMATION_COLS - 1) * GRID_PITCH_COL_MM  # Column spacing (X direction)
FORMATION_HEIGHT_MM = (FORMATION_ROWS - 1) * GRID_PITCH_ROW_MM  # Row spacing (Y direction)

# ============================================================================
# Waypoints (Group Center Coordinates in mm)
# ============================================================================

# Calculated to keep formation within field bounds
# Safety margin from field edges: 50 pixels ≈ 69mm
# This provides sufficient clearance for stable locomotion
WAYPOINT_SAFETY_MARGIN_PX = 50.0
WAYPOINT_SAFETY_MARGIN_X_MM = WAYPOINT_SAFETY_MARGIN_PX * SCALE_X_MM_PER_PX
WAYPOINT_SAFETY_MARGIN_Y_MM = WAYPOINT_SAFETY_MARGIN_PX * SCALE_Y_MM_PER_PX

WAYPOINTS_MM: Dict[str, PointMM] = {
    "TL": (
        FIELD_MIN_X_MM + FORMATION_WIDTH_MM / 2.0 + WAYPOINT_SAFETY_MARGIN_X_MM,
        FIELD_MIN_Y_MM + FORMATION_HEIGHT_MM / 2.0 + WAYPOINT_SAFETY_MARGIN_Y_MM,
    ),
    "TR": (
        FIELD_MAX_X_MM - FORMATION_WIDTH_MM / 2.0 - WAYPOINT_SAFETY_MARGIN_X_MM,
        FIELD_MIN_Y_MM + FORMATION_HEIGHT_MM / 2.0 + WAYPOINT_SAFETY_MARGIN_Y_MM,
    ),
    "BR": (
        FIELD_MAX_X_MM - FORMATION_WIDTH_MM / 2.0 - WAYPOINT_SAFETY_MARGIN_X_MM,
        FIELD_MAX_Y_MM - FORMATION_HEIGHT_MM / 2.0 - WAYPOINT_SAFETY_MARGIN_Y_MM,
    ),
    "BL": (
        FIELD_MIN_X_MM + FORMATION_WIDTH_MM / 2.0 + WAYPOINT_SAFETY_MARGIN_X_MM,
        FIELD_MAX_Y_MM - FORMATION_HEIGHT_MM / 2.0 - WAYPOINT_SAFETY_MARGIN_Y_MM,
    ),
}

# ============================================================================
# Speed Constants (Configurable)
# ============================================================================

# Movement speed (mm per second)
DEFAULT_MOVE_SPEED_MM_PER_SEC = 50.0

# Rotation speed (degrees per second)
DEFAULT_ROTATION_SPEED_DEG_PER_SEC = 45.0

# ============================================================================
# Coordinate Conversion Functions
# ============================================================================


def mm_to_px_x(mm: float) -> float:
    """Convert mm to pixel X coordinate."""
    return mm * SCALE_X_PX_PER_MM


def mm_to_px_y(mm: float) -> float:
    """Convert mm to pixel Y coordinate."""
    return mm * SCALE_Y_PX_PER_MM


def px_to_mm_x(px: float) -> float:
    """Convert pixel X coordinate to mm."""
    return px * SCALE_X_MM_PER_PX


def px_to_mm_y(px: float) -> float:
    """Convert pixel Y coordinate to mm."""
    return px * SCALE_Y_MM_PER_PX


def mm_to_px_point(mm_point: PointMM) -> Point:
    """Convert mm coordinates to pixel coordinates."""
    return (mm_to_px_x(mm_point[0]), mm_to_px_y(mm_point[1]))


def px_to_mm_point(px_point: Point) -> PointMM:
    """Convert pixel coordinates to mm coordinates."""
    return (px_to_mm_x(px_point[0]), px_to_mm_y(px_point[1]))


# ============================================================================
# Angle Functions
# ============================================================================


def normalize_heading(deg: float) -> float:
    """Normalize angle to [0, 360)."""
    return deg % 360.0


def robot_angle_to_math_angle(robot_angle_deg: float) -> float:
    """
    Convert robot angle to mathematical angle.
    
    Robot angle: 0° = up (-Y), 90° = right (+X), clockwise is positive
    Math angle: 0° = right (+X), 90° = up (-Y), counter-clockwise is positive
    
    Returns: Mathematical angle in degrees
    """
    # Robot 0° (up) = Math 270° (-90°)
    # Robot 90° (right) = Math 0°
    return normalize_heading(270.0 - robot_angle_deg)


def math_angle_to_robot_angle(math_angle_deg: float) -> float:
    """Convert mathematical angle to robot angle."""
    return normalize_heading(270.0 - math_angle_deg)


# ============================================================================
# Position Functions
# ============================================================================


def clamp_position_px(pos: Point) -> Point:
    """Clamp pixel position to field bounds."""
    x = max(FIELD_MIN_X_PX, min(FIELD_MAX_X_PX, pos[0]))
    y = max(FIELD_MIN_Y_PX, min(FIELD_MAX_Y_PX, pos[1]))
    return (x, y)


# ============================================================================
# Formation Functions
# ============================================================================


def get_local_offsets_mm() -> List[Tuple[int, float, float]]:
    """
    Calculate fixed local offsets for each robot (mm).
    
    Returns: List of (robot_id, offset_x_mm, offset_y_mm)
    where offsets are relative to formation center (0, 0).
    """
    offsets: List[Tuple[int, float, float]] = []
    
    # Center shift to make center at (0, 0)
    shift_x = (FORMATION_COLS - 1) * GRID_PITCH_COL_MM / 2.0
    shift_y = (FORMATION_ROWS - 1) * GRID_PITCH_ROW_MM / 2.0
    
    for row in range(FORMATION_ROWS):
        for col in range(FORMATION_COLS):
            robot_id = row * FORMATION_COLS + col
            local_x = (col * GRID_PITCH_COL_MM) - shift_x
            local_y = (row * GRID_PITCH_ROW_MM) - shift_y
            offsets.append((robot_id, local_x, local_y))
    
    return offsets


def rotate_local_offset(
    local_x_mm: float, local_y_mm: float, robot_angle_deg: float
) -> PointMM:
    """
    Rotate local offset by robot angle.
    
    Args:
        local_x_mm: Local X offset (mm)
        local_y_mm: Local Y offset (mm)
        robot_angle_deg: Robot heading angle (degrees)
    
    Returns:
        Rotated offset (x_mm, y_mm)
    """
    # Convert robot angle to mathematical angle
    math_angle_rad = math.radians(robot_angle_to_math_angle(robot_angle_deg))
    cos_a = math.cos(math_angle_rad)
    sin_a = math.sin(math_angle_rad)
    
    # Rotation matrix (counter-clockwise in math coordinates)
    # But we need clockwise rotation in image coordinates
    # For clockwise: x' = x*cos + y*sin, y' = -x*sin + y*cos
    rotated_x = local_x_mm * cos_a + local_y_mm * sin_a
    rotated_y = -local_x_mm * sin_a + local_y_mm * cos_a
    
    return (rotated_x, rotated_y)


def calculate_robot_positions(
    center_mm: PointMM, heading_deg: float
) -> Dict[int, Point]:
    """
    Calculate global positions (pixel) for all robots.
    
    Args:
        center_mm: Group center in mm (x, y)
        heading_deg: Robot heading angle (degrees)
    
    Returns:
        Dictionary mapping robot_id to pixel position (x, y)
    """
    local_offsets = get_local_offsets_mm()
    positions: Dict[int, Point] = {}
    
    for robot_id, local_x, local_y in local_offsets:
        # No rotation - use local offsets directly (fixed formation)
        # Add to center
        global_mm: PointMM = (
            center_mm[0] + local_x,
            center_mm[1] + local_y,
        )
        
        # Convert to pixel and clamp
        px_pos = mm_to_px_point(global_mm)
        positions[robot_id] = clamp_position_px(px_pos)
    
    return positions


# ============================================================================
# Movement Phases
# ============================================================================


@dataclass(frozen=True)
class MovementPhase:
    """Represents a phase in the rectangle movement sequence."""
    name: str
    waypoint: str
    heading_deg: float
    action_type: str  # "INIT", "MOVE_RIGHT", "TURN_DOWN", etc.


def generate_phases() -> List[MovementPhase]:
    """
    Generate the sequence of movement phases.
    
    Returns exactly one cycle of four-direction movement:
    TL → TR → BR → BL → TL (back to start)
    After MOVE_UP completes, the sequence ends.
    """
    # Movement directions and corresponding headings:
    # - Right (TL→TR): 90° (right)
    # - Down (TR→BR): 180° (down)
    # - Left (BR→BL): 270° (left)
    # - Up (BL→TL): 0° (up)
    # Sequence ends after MOVE_UP (returns to TL)
    return [
        MovementPhase("INIT", "TL", 90.0, "INIT"),  # Start facing right
        MovementPhase("MOVE_RIGHT", "TR", 90.0, "MOVE_RIGHT"),  # Move right, facing right
        MovementPhase("TURN_DOWN", "TR", 180.0, "TURN_DOWN"),  # Rotate in place: 90°→180°
        MovementPhase("MOVE_DOWN", "BR", 180.0, "MOVE_DOWN"),  # Move down, facing down
        MovementPhase("TURN_LEFT", "BR", 270.0, "TURN_LEFT"),  # Rotate in place: 180°→270°
        MovementPhase("MOVE_LEFT", "BL", 270.0, "MOVE_LEFT"),  # Move left, facing left
        MovementPhase("TURN_UP", "BL", 0.0, "TURN_UP"),  # Rotate in place: 270°→0°
        MovementPhase("MOVE_UP", "TL", 0.0, "MOVE_UP"),  # Move up, facing up (END: returns to TL)
    ]


# ============================================================================
# Frame Generation
# ============================================================================


def make_frame(
    time_sec: float,
    position_px: Point,
    heading_deg: float,
    use_position: bool,
    use_rotation: bool,
) -> Dict[str, object]:
    """Create a JSON frame."""
    # Round time to 0.5 second intervals
    TIME_INTERVAL_SEC = 0.5
    rounded_time = round(time_sec / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
    
    pos_clamped = clamp_position_px(position_px)
    frame: Dict[str, object] = {
        "time": round(rounded_time, 3),
        "x": round(pos_clamped[0], 3),
        "y": round(pos_clamped[1], 3),
        "rotation": round(normalize_heading(heading_deg), 3),
        "use_position": use_position,
        "use_rotation": use_rotation,
        "sound": None,
    }
    if use_position:
        frame["stop_distance"] = 20.0
    if use_rotation:
        frame["angle_tolerance"] = 5.0
    return frame


def calculate_move_duration(start_mm: PointMM, end_mm: PointMM, speed_mm_per_sec: float) -> float:
    """Calculate movement duration based on distance and speed."""
    dx = end_mm[0] - start_mm[0]
    dy = end_mm[1] - start_mm[1]
    distance_mm = math.sqrt(dx**2 + dy**2)
    return distance_mm / speed_mm_per_sec


def calculate_rotation_duration(start_angle_deg: float, end_angle_deg: float, speed_deg_per_sec: float) -> float:
    """Calculate rotation duration based on angle difference and speed."""
    # Calculate shortest angle difference
    diff = (end_angle_deg - start_angle_deg) % 360.0
    if diff > 180.0:
        diff = 360.0 - diff
    return abs(diff) / speed_deg_per_sec


def generate_robot_frames(
    start_time_sec: float,
    move_speed_mm_per_sec: float,
    rotation_speed_deg_per_sec: float,
) -> Dict[int, List[Dict[str, object]]]:
    """
    Generate frames for all robots.
    
    Args:
        start_time_sec: Start time for the sequence
        rotation_duration_sec: Duration for rotation phases
        move_duration_sec: Duration for movement phases
    
    Returns:
        Dictionary mapping robot_id to list of frames
    """
    phases = generate_phases()
    frames: Dict[int, List[Dict[str, object]]] = {
        rid: [] for rid in range(NUM_ROBOTS)
    }
    
    # Fixed timeline: 0.5 second intervals
    TIME_INTERVAL_SEC = 0.5
    
    current_time = start_time_sec
    prev_heading = None
    prev_center_mm = None
    
    for phase_idx, phase in enumerate(phases):
        waypoint_center_mm = WAYPOINTS_MM[phase.waypoint]
        robot_positions = calculate_robot_positions(
            waypoint_center_mm, phase.heading_deg
        )
        
        # Fixed duration: 0.5 seconds per phase
        if phase.action_type == "INIT":
            duration = 0.0
        else:
            duration = TIME_INTERVAL_SEC
        
        # Generate frames
        if phase.action_type == "INIT":
            # Initial setup: set position and heading, with rotation command
            init_time = round(current_time / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
            for robot_id in range(NUM_ROBOTS):
                if robot_id in robot_positions:
                    frames[robot_id].append(
                        make_frame(
                            init_time,
                            robot_positions[robot_id],
                            phase.heading_deg,  # Use phase heading
                            use_position=True,
                            use_rotation=True,  # First frame: also set rotation
                        )
                    )
            # Update prev_center_mm and prev_heading for next phase
            prev_center_mm = waypoint_center_mm
            prev_heading = phase.heading_deg
            current_time = init_time
        
        elif phase.action_type.startswith("TURN"):
            # Rotation: each robot rotates in place (position does NOT change)
            # Generate frames at 0.5 second intervals based on rotation speed
            
            # Position during rotation: same as before rotation (in-place rotation)
            if prev_center_mm is not None and prev_heading is not None:
                positions_during_rotation = calculate_robot_positions(
                    prev_center_mm, prev_heading
                )
            else:
                positions_during_rotation = robot_positions

            # Calculate rotation angle difference
            start_angle = prev_heading if prev_heading is not None else phase.heading_deg
            end_angle = phase.heading_deg
            angle_diff = (end_angle - start_angle + 180.0) % 360.0 - 180.0  # Shortest path
            
            # Calculate actual duration based on rotation speed
            actual_duration = abs(angle_diff) / rotation_speed_deg_per_sec if rotation_speed_deg_per_sec > 0 else 0.0
            
            # Generate frames at 0.5 second intervals
            rotation_start_time = round(current_time / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
            if phase_idx > 0 and rotation_start_time <= current_time:
                rotation_start_time += TIME_INTERVAL_SEC
            
            rotation_end_time = rotation_start_time + actual_duration
            rotation_end_time = round(rotation_end_time / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC

            for robot_id in range(NUM_ROBOTS):
                if robot_id not in robot_positions:
                    continue

                # Position during rotation: same position throughout (in-place rotation)
                pos_during = positions_during_rotation.get(robot_id, robot_positions[robot_id])

                # Generate frames at 0.5 second intervals
                t = rotation_start_time
                while t <= rotation_end_time + 0.001:  # Small epsilon for floating point
                    # Calculate rotation progress
                    elapsed = t - rotation_start_time
                    progress = min(elapsed / actual_duration, 1.0) if actual_duration > 0 else 1.0
                    
                    # Interpolate angle
                    current_angle = start_angle + angle_diff * progress
                    current_angle = (current_angle + 360.0) % 360.0
                    
                    # Determine flags
                    use_rot = (t < rotation_end_time - 0.001)  # Still rotating if not at end
                    
                    frames[robot_id].append(
                        make_frame(
                            t,
                            pos_during,  # Same position (rotation is in-place)
                            current_angle,
                            use_position=False,  # Position does not change during rotation
                            use_rotation=use_rot,
                        )
                    )
                    
                    t += TIME_INTERVAL_SEC
            
            # Update current_time for next phase
            current_time = rotation_end_time
        
        else:  # MOVE_*
            # Movement: move from current position to target position
            # Generate frames at 0.5 second intervals based on movement speed
            
            # Determine start and end positions
            if phase_idx > 0 and phases[phase_idx - 1].action_type.startswith("TURN"):
                # Previous phase was rotation - start from rotation end position
                if prev_center_mm is not None and prev_heading is not None:
                    positions_before_move = calculate_robot_positions(
                        prev_center_mm, prev_heading  # Position at rotation end (same as rotation start)
                    )
                else:
                    positions_before_move = robot_positions
                positions_after_move = robot_positions
            else:
                # Previous phase was movement or INIT
                if prev_center_mm is not None and prev_heading is not None:
                    positions_before_move = calculate_robot_positions(
                        prev_center_mm, prev_heading
                    )
                else:
                    # First movement after INIT - use INIT positions
                    positions_before_move = robot_positions
                positions_after_move = robot_positions
            
            # Calculate move start time
            if phase_idx > 0 and phases[phase_idx - 1].action_type.startswith("TURN"):
                # Previous phase was rotation - start after rotation end
                move_start_time = round((current_time + TIME_INTERVAL_SEC) / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
            else:
                # Previous phase was INIT or MOVE - start from current_time rounded to next 0.5s
                move_start_time = round((current_time + TIME_INTERVAL_SEC) / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
            
            for robot_id in range(NUM_ROBOTS):
                if robot_id not in robot_positions:
                    continue
                
                # Get start and end positions
                if phase_idx > 0 and phases[phase_idx - 1].action_type.startswith("TURN"):
                    # Previous phase was rotation - get position from rotation end frame
                    pos_start = None
                    # Try to get from last frame (rotation end)
                    for frame in reversed(frames[robot_id]):
                        if frame.get('time', 0) <= move_start_time + 0.001:
                            pos_start = (frame['x'], frame['y'])
                            break
                    # Fallback: use calculated position
                    if pos_start is None:
                        pos_start = positions_before_move.get(robot_id, robot_positions[robot_id])
                else:
                    # Previous phase was INIT or MOVE - get from last frame or calculated position
                    pos_start = None
                    # Try to get from last frame
                    for frame in reversed(frames[robot_id]):
                        if frame.get('time', 0) <= move_start_time + 0.001:
                            pos_start = (frame['x'], frame['y'])
                            break
                    # Fallback: use calculated position
                    if pos_start is None:
                        pos_start = positions_before_move.get(robot_id, robot_positions[robot_id])
                
                pos_end = positions_after_move[robot_id]
                
                # Calculate movement distance (in mm)
                dx_px = pos_end[0] - pos_start[0]
                dy_px = pos_end[1] - pos_start[1]
                dx_mm = px_to_mm_x(dx_px)
                dy_mm = px_to_mm_y(dy_px)
                distance_mm = math.sqrt(dx_mm**2 + dy_mm**2)
                
                # Calculate actual duration based on movement speed
                actual_duration = distance_mm / move_speed_mm_per_sec if move_speed_mm_per_sec > 0 else 0.0
                
                # Calculate move end time
                move_end_time = move_start_time + actual_duration
                move_end_time = round(move_end_time / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
                
                # Generate frames at 0.5 second intervals
                t = move_start_time
                while t <= move_end_time + 0.001:  # Small epsilon for floating point
                    # Calculate movement progress
                    elapsed = t - move_start_time
                    progress = min(elapsed / actual_duration, 1.0) if actual_duration > 0 else 1.0
                    
                    # Interpolate position
                    current_x = pos_start[0] + dx_px * progress
                    current_y = pos_start[1] + dy_px * progress
                    current_pos = (current_x, current_y)
                    
                    frames[robot_id].append(
                        make_frame(
                            t,
                            current_pos,
                            phase.heading_deg,  # Use phase heading (direction of movement)
                            use_position=True,
                            use_rotation=False,
                        )
                    )
                    
                    t += TIME_INTERVAL_SEC
        
        # Update for next iteration
        prev_heading = phase.heading_deg
        prev_center_mm = waypoint_center_mm
        
        # Update current_time based on actual phase duration
        if phase.action_type == "INIT":
            current_time = round((current_time + duration) / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
        elif phase.action_type.startswith("TURN"):
            # Rotation end time was already set
            # current_time is updated in the TURN block
            pass
        else:  # MOVE_*
            # Calculate from last frame time (should be same for all robots)
            if frames[0]:  # Check if any frames exist
                last_frame_time = max(f.get('time', 0) for f in frames[0])
                current_time = round(last_frame_time / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
            else:
                current_time = round((current_time + duration) / TIME_INTERVAL_SEC) * TIME_INTERVAL_SEC
    
    # Sort frames by time
    for robot_id in frames:
        frames[robot_id].sort(key=lambda f: f["time"])
    
    return frames


# ============================================================================
# Output Functions
# ============================================================================


def write_outputs(
    frames: Dict[int, List[Dict[str, object]]], output_dir: Path
) -> None:
    """Write JSON files for each robot and combined file."""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    set_entries: List[Dict[str, object]] = []
    
    for robot_id, frame_list in frames.items():
        payload = {"frames": frame_list}
        set_entries.append({"id": f"robot_{robot_id:02d}", "frames": frame_list})
        
        out_path = output_dir / f"robot_{robot_id:02d}.json"
        with out_path.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
    
    # Write combined file
    all_path = output_dir / "all_robots.json"
    with all_path.open("w", encoding="utf-8") as fh:
        json.dump({"sets": set_entries}, fh, ensure_ascii=False, indent=2)


# ============================================================================
# Main
# ============================================================================


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Generate rectangle formation JSON for 20 robots"
    )
    parser.add_argument(
        "--start-time",
        type=float,
        default=0.0,
        help="Start time for the sequence (seconds, default: 0.0)",
    )
    parser.add_argument(
        "--move-speed",
        type=float,
        default=DEFAULT_MOVE_SPEED_MM_PER_SEC,
        help=f"Movement speed (mm/sec, default: {DEFAULT_MOVE_SPEED_MM_PER_SEC})",
    )
    parser.add_argument(
        "--rotation-speed",
        type=float,
        default=DEFAULT_ROTATION_SPEED_DEG_PER_SEC,
        help=f"Rotation speed (deg/sec, default: {DEFAULT_ROTATION_SPEED_DEG_PER_SEC})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("rectangle_output"),
        help="Directory to write JSON files (default: rectangle_output)",
    )
    return parser.parse_args()


def main() -> None:
    """Main entry point."""
    args = parse_args()
    
    print("Generating rectangle formation JSON...")
    print(f"  Start time: {args.start_time}s")
    print(f"  Move speed: {args.move_speed}mm/sec")
    print(f"  Rotation speed: {args.rotation_speed}deg/sec")
    print(f"  Output directory: {args.output_dir}")
    
    frames = generate_robot_frames(
        start_time_sec=args.start_time,
        move_speed_mm_per_sec=args.move_speed,
        rotation_speed_deg_per_sec=args.rotation_speed,
    )
    
    write_outputs(frames, args.output_dir)
    
    # Calculate total duration
    phases = generate_phases()
    total_duration = 0.0
    prev_heading = None
    prev_center_mm = None
    
    for phase in phases:
        waypoint_center_mm = WAYPOINTS_MM[phase.waypoint]
        if phase.action_type == "INIT":
            duration = 0.0
        elif phase.action_type.startswith("TURN"):
            if prev_heading is not None:
                duration = calculate_rotation_duration(
                    prev_heading, phase.heading_deg, args.rotation_speed
                )
            else:
                duration = 0.0
        else:
            if prev_center_mm is not None:
                duration = calculate_move_duration(
                    prev_center_mm, waypoint_center_mm, args.move_speed
                )
            else:
                duration = 0.0
        total_duration += duration
        prev_heading = phase.heading_deg
        prev_center_mm = waypoint_center_mm
    
    print(f"\nGenerated JSON for {len(frames)} robots")
    print(f"Total sequence duration: {total_duration:.1f}s")
    print(f"Output directory: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
