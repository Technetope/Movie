#!/usr/bin/env python3
"""
指定のJSONファイルを時系列ごとに分けて合体するスクリプト。

複数のJSONファイルを読み込み、同じtime値を持つフレームを時系列ごとに統合します。

Usage:
  python tools/merge_by_timeline.py \
      --input rectangle_output/all_robots.json/all_robots.json \
      --input spiral_output/all_robots.json \
      --output merged_output/all_robots.json

  # または、時系列ごとに分割して出力
  python tools/merge_by_timeline.py \
      --input rectangle_output/all_robots.json/all_robots.json \
      --input spiral_output/all_robots.json \
      --output merged_output/ \
      --split-by-time
"""

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Set


def load_json_file(path: Path) -> Dict[str, List[Dict]]:
    """JSONファイルを読み込み、ロボットIDをキーとする辞書を返す。"""
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    
    robots: Dict[str, List[Dict]] = {}
    
    if "sets" in data:
        for entry in data["sets"]:
            robot_id = entry["id"]
            frames = entry.get("frames", [])
            robots[robot_id] = frames
    elif "frames" in data:
        # 単一ロボットの形式の場合
        robots["robot_00"] = data["frames"]
    else:
        raise ValueError(f"Unsupported JSON format in {path}")
    
    return robots


def get_time(frame: Dict) -> float:
    """フレームからtime値を取得。"""
    return float(frame.get("time", 0.0))


def merge_by_timeline(input_files: List[Path]) -> Dict[float, List[Dict]]:
    """
    複数のJSONファイルを読み込み、時系列（time）ごとにフレームを統合する。
    
    Returns:
        time値をキーとし、その時系列の全ロボットのフレームリストを値とする辞書
    """
    timeline_frames: Dict[float, List[Dict]] = defaultdict(list)
    all_robots: Dict[str, List[Dict]] = {}
    
    # すべてのJSONファイルからロボットデータを読み込む
    for input_file in input_files:
        robots = load_json_file(input_file)
        for robot_id, frames in robots.items():
            if robot_id in all_robots:
                # 既存のロボットIDがある場合は、フレームを追加
                all_robots[robot_id].extend(frames)
            else:
                all_robots[robot_id] = frames.copy()
    
    # 各ロボットのフレームを時系列ごとに分類
    for robot_id, frames in all_robots.items():
        for frame in frames:
            time = get_time(frame)
            # ロボットIDをフレームに追加（必要に応じて）
            frame_with_id = frame.copy()
            frame_with_id["robot_id"] = robot_id
            timeline_frames[time].append(frame_with_id)
    
    return timeline_frames


def create_merged_output(timeline_frames: Dict[float, List[Dict]], 
                        output_format: str = "sets") -> Dict:
    """
    時系列ごとに統合されたフレームから、出力用のJSON構造を作成する。
    
    Args:
        timeline_frames: time値をキーとするフレームリストの辞書
        output_format: "sets" (デフォルト) または "by_time"
    
    Returns:
        出力用のJSON構造
    """
    if output_format == "by_time":
        # 時系列ごとに分けた形式
        result = {
            "timelines": []
        }
        for time in sorted(timeline_frames.keys()):
            result["timelines"].append({
                "time": time,
                "frames": timeline_frames[time]
            })
        return result
    else:
        # デフォルト: sets形式（各ロボットごとに統合）
        robot_frames: Dict[str, List[Dict]] = defaultdict(list)
        
        # 各時系列のフレームをロボットごとに分類
        for time in sorted(timeline_frames.keys()):
            for frame in timeline_frames[time]:
                robot_id = frame.get("robot_id", "robot_00")
                # robot_idを削除して元のフレーム形式に戻す
                frame_clean = {k: v for k, v in frame.items() if k != "robot_id"}
                robot_frames[robot_id].append(frame_clean)
        
        # 各ロボットのフレームをtimeでソート
        for robot_id in robot_frames:
            robot_frames[robot_id].sort(key=get_time)
        
        # sets形式で出力
        sets = []
        for robot_id in sorted(robot_frames.keys()):
            sets.append({
                "id": robot_id,
                "frames": robot_frames[robot_id]
            })
        
        return {"sets": sets}


def split_by_timeline(timeline_frames: Dict[float, List[Dict]], 
                      output_dir: Path) -> None:
    """
    時系列ごとに分割して個別のJSONファイルとして出力する。
    
    Args:
        timeline_frames: time値をキーとするフレームリストの辞書
        output_dir: 出力ディレクトリ
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    
    for time in sorted(timeline_frames.keys()):
        frames = timeline_frames[time]
        output_file = output_dir / f"time_{time:.1f}.json"
        
        output_data = {
            "time": time,
            "frames": frames
        }
        
        with output_file.open("w", encoding="utf-8") as f:
            json.dump(output_data, f, ensure_ascii=False, indent=2)
        
        print(f"Wrote {len(frames)} frames at time {time} to {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="指定のJSONファイルを時系列ごとに分けて合体する"
    )
    parser.add_argument(
        "--input",
        type=Path,
        action="append",
        required=True,
        help="入力JSONファイル（複数指定可能）"
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="出力ファイルまたはディレクトリ"
    )
    parser.add_argument(
        "--split-by-time",
        action="store_true",
        help="時系列ごとに分割して個別ファイルとして出力"
    )
    parser.add_argument(
        "--format",
        choices=["sets", "by_time"],
        default="sets",
        help="出力形式: sets (デフォルト) または by_time"
    )
    
    args = parser.parse_args()
    
    # 入力ファイルの存在確認
    for input_file in args.input:
        if not input_file.exists():
            raise FileNotFoundError(f"Input file not found: {input_file}")
    
    # 時系列ごとにフレームを統合
    print(f"Loading {len(args.input)} JSON file(s)...")
    timeline_frames = merge_by_timeline(args.input)
    
    print(f"Found {len(timeline_frames)} unique time points")
    total_frames = sum(len(frames) for frames in timeline_frames.values())
    print(f"Total frames: {total_frames}")
    
    # 出力
    if args.split_by_time:
        # 時系列ごとに分割して出力
        split_by_timeline(timeline_frames, args.output)
        print(f"\nSplit output written to {args.output}")
    else:
        # 統合して1つのファイルとして出力
        output_data = create_merged_output(timeline_frames, args.format)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        
        with args.output.open("w", encoding="utf-8") as f:
            json.dump(output_data, f, ensure_ascii=False, indent=2)
        
        print(f"\nMerged output written to {args.output}")
        
        # 統計情報を表示
        if "sets" in output_data:
            print(f"Number of robots: {len(output_data['sets'])}")
            for robot_set in output_data["sets"]:
                print(f"  {robot_set['id']}: {len(robot_set['frames'])} frames")


if __name__ == "__main__":
    main()


