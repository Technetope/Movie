# 移動制御用JSONフォーマット
1フレームごとに「いつ」「どこへ」「どの向き」で動かすか、任意で効果音を鳴らすかを記述する。

## キーの意味
- `time` (float, sec): 動作を適用するタイムスタンプ
- `use_position` (bool): 位置指令を反映するか
- `use_rotation` (bool): 回転指令を反映するか
  - 位置と回転は排他的
- `position` (object): X/Y座標。`use_position` が true のときのみ有効
- `rotation` (float, deg): 向き。`use_rotation` が true のときのみ有効
- `sound` (string|null): このフレームで鳴らす効果音ID。鳴らさない場合は null

## JSON例
```json
{
  "frames": [
    {
      "time": 0.0,
      "use_position": true,
      "use_rotation": true,
      "position": { "x": 0.0, "y": 0.0 },
      "rotation": 0.0,
      "sound": "start_beep"
    },
    {
      "time": 0.5,
      "use_position": true,
      "use_rotation": false,
      "position": { "x": 100.0, "y": 0.0 },
      "rotation": 0.0,
      "sound": null
    },
    {
      "time": 1.0,
      "use_position": false,
      "use_rotation": true,
      "position": { "x": 100.0, "y": 0.0 },
      "rotation": 90.0,
      "sound": "turn_se"
    }
  ]
}
```
