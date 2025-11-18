# 移動制御用JSONフォーマット
1フレームごとに「いつ」「どこへ」「どの向き」で動かすか、任意で効果音を鳴らすかを記述する。

## キーの意味
- `time` (float, sec) もしくは `t`: 動作を適用するタイムスタンプ。
- `use_position`/`up` (bool): 位置指令を反映するか。
- `use_rotation`/`use_heading`/`ur`/`uh` (bool): 回転指令を反映するか。
  - 位置と回転は排他的ではないが、不要なら false にする。
- `x`, `y` (float) または `position.{x,y}`: 位置。`use_position` が true のときのみ必須。
- `rotation`/`angle`/`ang` (float, deg): 向き。`use_rotation` が true のときのみ必須。
- `stop_distance`/`sd` (float, mm): 位置停止距離（省略時 20.0）。
- `angle_tolerance`/`at` (float, deg): 角度許容誤差（省略時 5.0）。
- `sound` (string|null): このフレームで鳴らす効果音ID。鳴らさない場合は null/未指定。

## 埋め込みサウンドID（8kHz/16bit/mono）
- `aurora`
- `birds`
- `cicada`
- `crystal`
- `dolphin`
- `frog`
- `geyser`
- `insect`
- `reef`

## JSON例（単一セット）
```json
{
  "frames": [
    {
      "time": 0.0,
      "use_position": true,
      "use_rotation": false,
      "x": 0.0,
      "y": 0.0,
      "sound": "aurora"
    },
    {
      "time": 2.0,
      "use_position": true,
      "use_rotation": false,
      "x": 100.0,
      "y": 0.0,
      "sound": null
    },
    {
      "time": 4.0,
      "use_position": false,
      "use_rotation": true,
      "rotation": 90.0,
      "sound": "reef"
    }
  ]
}
```

## 複数セットをまとめて持たせる場合（推奨）
1つのJSONに複数のフレームセットを入れておき、クライアント側で送信対象を選ぶ場合は、次のように `sets` 配列を使ってください。各要素の `frames` は上記と同じフォーマットです。

```json
{
  "sets": [
    {
      "id": "toioA",
      "frames": [
        { "time": 0.0, "use_position": true, "x": 0.0, "y": 0.0, "sound": "aurora" },
        { "time": 2.0, "use_position": true, "x": 100.0, "y": 0.0 },
        { "time": 4.0, "use_rotation": true, "rotation": 90.0, "sound": "reef" }
      ]
    },
    {
      "id": "toioB",
      "frames": [
        { "time": 0.0, "use_position": true, "x": 10.0, "y": 10.0, "sound": "frog" },
        { "time": 3.0, "use_position": true, "x": 150.0, "y": 30.0 }
      ]
    }
  ]
}
```

運用例: ブラウザ側で `sets` を列挙し、選択したセットの `frames` をそのまま `timeline-load` のペイロード `{ "frames": [...] }` として送信します。M5側の受信仕様（`frames`）はそのままなので、既存のプロトコルを変える必要はありません。
