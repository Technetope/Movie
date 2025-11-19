# 251118実装前条件整理
ksk432（本人）

# 実装するもの
- json fileで構築されたスケジューラーの作成
    - json fileの中身
        - time(s), position(x,y,r), locomotion mode(move,rotate), sound
            - 時間は0.5秒感覚で定義する
            - positionはtoio playmatの絶対座標を条件とする
            - locomotion modeによって、回転運動をするべきか、並行運動をするべきかを決める（スクリプトは[jun]が作ってくれる）
                - 正確には回転運動のみとして制御するべきか通常運動として整理するか
                    - 通常運動はPIDを使って効果的に移動している
- json fileは30パターン用意しなければいけない
    - 言い換えると、30台のロボットをスケジューラーに基づいて動かす
- ロボットの動きに関しては[jun]が作成してくれる
    - schedulerのディレクトリには作らない

## 背景・前提
- やまなしメディアアートアワードに投稿を試みている、動画撮影用のtoio制御は事前に決めた方法でやるほうが良い
- 30台のtoioの座標・移動・音再生を時間を軸にして制御する
    - 時間のズレなどに関してはNTPサーバーを用いる
    - リアルタイムというよりは発火＋リードタイムで開始時刻を揃えて動かす感じ
        - そこら辺のスクリプトは[jun]が作ってくれる
- toioの仕様
    -  7.2cm（奥行き）×3.2cm（幅）×4cm（車輪込みの高さ）
     - toio cube 上向きが0


## 移動制御用JSONフォーマット
1フレームごとに「いつ」「どこへ」「どの向き」で動かすか、任意で効果音を鳴らすかを記述する。

## キーの意味
- `time` (float, sec): 動作を適用するタイムスタンプ
- `use_position` (bool): 位置指令を反映するか
- `use_rotation` (bool): 回転指令を反映するか
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


## Schedulerの大まかな作成プロセス
- 正直迷っている部分はある
    - まず30台分のjsonをいちいち書いているのはめんどくさいので、効率的にやりたい
        - 例えば、基本的な動きに関しては制御方針を書いてちょっとずらすなど
    - 途中で群のごちゃっとした制御をするのだけど、これをプログラミングするのはめんどくさい
        - 結局、なんかシミュレーションで自動生成したものを時系列的に置くのが効果的な気がしてきた。
        - technetope/technetopeのなかにsimulation branchが存在して、そこでp5.jsの制御を実はコーディングしている
            - https://github.com/Technetope/technetope/tree/6d4da3b6d7e49bf965c43a76acea2e2fb67400de/toio_control/archive/development_tools/swarm_control_p5js
    - 最後にtoioを再整列させるのだけれど、それについても結構めんどくさい印象を持っている
        - これもできればいい感じにコーディングしたい

## Schedulerの大まかな中身
- 序盤
    - 初期ポジション
        - 向きは全部下向き(つまり180°)
        - 等間隔で30台のtoioが6個×5行（横に並べる個数のほうが多い）
        - 間隔比率に関してはtoioが回転運動をしたときにぶつからないことが重要、回転移動時の遅れはほぼ考えない。
            - (AI推論して)          
    - toio全体の回転運動
        -  r=270°の回転運動（つまり90°半時計回りに回転）
        - 座標移動は行わない、向きのみ回転運動で変更する
            - 45 deg/secで回転
    - toio全体の並行運動
        - toio全体を40cm移動
        - 10cm/secで操作
    - これを長方形で動かす。
        - 縦は30cm(10cm/secで移動)
        - もとの場所（初期ポジションに戻る）
    - 135°回転する
        - 45°/sec
    - 中心に移動する
    　- これはtoioのプレイマットを中心に初期配置の間隔とかを変えないままで作られる長方形の中心の位置を想定している
    - 45°回転して、向きをr=180°に戻す
    - 回転運動から螺旋運動（中盤へ）
        - 最初はただの回転運動
        - だんだん螺旋運動というか広がっていく
            - もちろん広がりにも勾配が存在している。
                - 中心から遠くなっていく、最終的には螺旋みたいな感じのを作りたい
                    - ほぼ分散している感じ
- 中盤
    - 群ロボットの動作
        - 群ロボットの動作
            - 動作アルゴリズムはシミュレーション上の内容をなんとか取りたいが序盤と中盤の接続をどうするかが悩ましい
            -  群ロボットに関しては以下を参照したい。
                - technetope/technetopeのなかにsimulation branchが存在して、そこでp5.jsの制御を実はコーディングしている
                - https://github.com/Technetope/technetope/tree/6d4da3b6d7e49bf965c43a76acea2e2fb67400de/toio_control/archive/development_tools/swarm_control_p5js