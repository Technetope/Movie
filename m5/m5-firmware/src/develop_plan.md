## 目的
- タイムラインJSONの`sound`フィールドで指定された音を、埋め込みWAV（8kHz/16bit/mono）として再生できるようにする。
- ファイルシステムは使わず、`embedded_wav.h` でまとめた配列を参照する。

## 方針
- 出力先は M5StickC Plus2 + SPK HAT2（外部スピーカー）。M5Unified の config で外部スピーカを有効にする。
- `TimelineFrame` に `sound_id` を追加し、`protocol_handler` の `timeline-load` で `sound` を読み取る（null/未指定は空扱い）。
- `updateTimelinePlayback()` でフレーム適用時に `sound_id` があれば即再生する。未定義IDはログのみ。
- 再生中に次の音が来た場合は「後勝ち」で現在の音を停止し、新しい音に差し替える。
- 出力は `M5.Speaker.playWav(buffer, size, 1, 0, true)` を使用。音量は固定値で開始し、必要なら後日 `gain` フィールドを拡張する。

## 実装タスク
1) モデル/プロトコル拡張  
   - `TimelineFrame` に `std::string sound_id;` を追加。  
   - `protocol_handler::HandleMessage` の `timeline-load` で `sound` をパースしてフレームに格納。

2) 再生ヘルパー  
   - `audio` ヘルパー（クラス/名前空間）を追加し、`PlayById(const char* id)` / `Stop()` / `Loop()` を持たせる。  
   - `embedded_wav.h` の `kEmbeddedWavs` を線形検索し、ヒットしたら `playWav`。ミスならログ。

3) ループ統合  
   - `toio_controller::updateTimelinePlayback()` のフレーム適用部分で `PlayById(sound_id)` を呼ぶ。  
   - `loop()` で `audio.Loop()` を呼び、再生終了時の後処理を行う（必要最低限）。

4) クリーンアップ  
   - クライアント切断や `timeline-stop` 時に `Stop()` を呼んでスピーカを止める。

## 動作確認
- 埋め込み済みの音ID（例: `aurora`, `birds`, `cicada`, `crystal`, `dolphin`, `frog`, `geyser`, `insect`, `reef`）を `sound` に指定したフレームで再生されること。
- 未定義IDでエラーにならずログだけでスキップされること。
- 再生中に次の音が来た場合、後勝ちで切り替わること。

## オープンな決定
- 音量の固定値と、必要なら `gain` をJSONに追加するかどうか。  
- 既存のトーンスケジューラを併存させるか置き換えるか（優先順位）。
