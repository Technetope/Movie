# 移動制御JSONでtoioを動かす実装方針

方針（更新）
- WebでJSONをパース・バリデーションし、M5には整形済みのフレーム配列をそのまま送る。M5側のチェックは最小限にして軽量化。
- 再生開始時刻はWebが決めて送信（delay_ms付き）。M5は受信したタイムラインをメモリに保持し、ローカルタイマで自律再生する。

現状の把握
- ファーム: `goal-set` で use_position/use_heading を受け取り、位置→角度の二段制御。`goal-tuning` でVmax/Wmax/ゲイン/後退閾値を変更可。効果音対応なし。
- Webクライアント: 手動送信用UIと goal-tuning UI がある。タイムライン再生UIはあるが送信ロジック未実装。
- JSON仕様: `time/use_position/use_heading/position/angle/stop_distance/angle_tolerance` をサポート（効果音未実装）。

やること（優先順）
1. Web側タイムライン再生実装  
   - JSONをパース/検証し、短いキーに圧縮したフレーム配列を `timeline-load` で送る（例 `{t,up,uh,x,y,ang,sd,at}`）。  
   - `timeline-start(delay_ms?)` で再生開始を指示。停止は `timeline-stop`。  
   - エラー時は再生中断とログ表示。
2. M5側タイムライン再生ロジック追加  
   - 受信したフレーム配列を固定長バッファに格納（上限例: 64フレーム）。  
   - `timeline-start` で `start_ms = millis() + delay` をセットし、`loop()` で経過時間に応じてフレームを順次 `goal-set` 呼び出し。  
   - `timeline-stop` で再生フラグを落とし `goal-clear`。簡易バリデーション（フレーム数上限、time>=0）だけ行う。
3. チューニング利用/プリセット  
   - 再生前に必要なら `goal-tuning` を送信。UIにプリセットボタン（デフォルト/高速等）を追加するか再生前自動送信を検討。
4. 実機テストと安全措置  
   - ケース: 位置のみ、角度のみ、位置+角度の3パターン、`on_mat=false` での停止確認。  
   - 停止ボタンの挙動と invalid-goal 発生時のログ確認。
5. 拡張（任意）  
   - 効果音コマンドを追加して再生対応。  
   - ステータスに現在のチューニング値や再生状態を載せ、UIで表示。  
   - 再生進捗表示（現在フレーム/残り時間）。

送信ペイロード（案）
- timeline-load: `{ frames: [ { t, up, uh, x?, y?, ang?, sd?, at? } ] }`
- timeline-start: `{ delay_ms?: number }`（未指定なら即時）
- timeline-stop: 中断 + goal-clear
- goal-tuning: 必要に応じて事前送信

その他メモ
- M5の電源リセットでチューニング値やロード済みタイムラインは消える。永続化するなら別途保存。  
- 効果音フィールドは現状ダミー。必要ならプロトコル拡張が要る。
