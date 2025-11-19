# swarm_control_p5js の工夫分析

## 概要

`swarm_control_p5js`は、Boidモデルをベースにしながら、重なりを防ぎ、自然な動きを実現するための多層的な工夫が実装されています。単純なBoidモデルでは「重なりまくりで変な動きしかしない」問題を解決するために、以下の工夫が施されています。

## 主要な工夫

### 1. **確率的な群れ行動発動（Boidモデルの弱体化）**

**実装**: `boid_model.js` の `applyFlocking()`

```javascript
const shouldFlock = Math.random() < Params.boid.activationProbability; // 15%の確率
```

**工夫点**:
- 群れ行動を**15%の確率でしか発動しない**（`activationProbability: 0.15`）
- 残り85%の時間は自律性を維持
- これにより、Boidモデルの強すぎる相互作用を弱め、重なりを防ぐ

**効果**: 群としての協調性を保ちながら、過度な集約を防ぐ

---

### 2. **多層的な衝突回避システム**

#### 2.1 緊急停止モード
**実装**: `robot_agent.js` の `applyCollisionAvoidance()`

```javascript
if (dist < emergencyStopDist && dist > 0.001) { // 71mm以下
    isEmergencyStop = true;
    this.vx *= emergencyStopSpeed; // 15%に減速
    // 強力な反発力
}
```

**工夫点**:
- 71mm（toioの最長辺）以下で緊急停止
- 速度を15%に減速し、強力な反発力を適用
- 位置を直接分離（`overlap * 0.3`で弱めている）

#### 2.2 予測的衝突回避
**実装**: `predictive_avoidance.js`

```javascript
predictionTimes: [0.5, 1.0, 2.0] // 0.5秒、1秒、2秒先を予測
```

**工夫点**:
- 将来位置を複数時間点で予測（0.5秒、1秒、2秒先）
- 予測時間が長いほど不確実性を大きくする（`uncertaintyGrowth: 0.2`）
- 予測領域が重なる場合に反発力を計算

**効果**: 衝突を事前に防ぐ

#### 2.3 混雑度推定による回避
**実装**: `spatial_density_grid.js` + `robot_agent.js`

```javascript
const leastCrowded = densityGrid.getLeastCrowdedDirection(this.x, this.y, sampleRadius);
```

**工夫点**:
- 空間をグリッドに分割し、各方向の混雑度を推定
- 混雑度が低い方向に移動を促す
- 20%の能動的なロボット（fastカテゴリ）は混雑度回避を無視

**効果**: 局所的な混雑を分散させる

#### 2.4 重み付け合成
**実装**: `robot_agent.js` の `applyCollisionAvoidance()`

```javascript
const weightCrowd = 0.4;      // 混雑度回避: 40%
const weightPredictive = 0.3; // 予測回避: 30%
const weightCollision = 0.3;  // 衝突回避: 30%
```

**工夫点**:
- 3つの回避力を重み付けして合成
- 各力を正規化してから合成することで、力のバランスを保つ

---

### 3. **個体差の導入（速度分布と個体パラメータ）**

#### 3.1 速度分布
**実装**: `robot_agent.js` の `assignSpeedCategory()`

```javascript
// 20%速い、60%中程度、20%遅い
if (ratio < 0.2) return 'fast';
else if (ratio < 0.8) return 'moderate';
else return 'slow';
```

**工夫点**:
- 各ロボットに速度カテゴリを割り当て
- 速いロボット（20%）: `maxSpeed * 1.3-1.7`
- 中程度（60%）: `minSpeed + (maxSpeed - minSpeed) * 0.3` から `maxSpeed * 0.9`
- 遅い（20%）: `minSpeed * 0.7-1.0`

**効果**: 速度の多様性により、群全体が同時に同じ動きをしない

#### 3.2 個体パラメータの変動
**実装**: `robot_agent.js` のコンストラクタ

```javascript
this.speedBias = 0.8 + Math.random() * 0.4; // 個体差
this.orientationAlignment = ... // 個体差
this.homeostasisEnergy = ... // 個体差
```

**工夫点**:
- 各ロボットでパラメータをランダムに変動
- 速度誤差、向きの一致度、恒常性エネルギーなどが個体差を持つ
- これにより、同じ状況でも異なる反応をする

**効果**: 個体差により、群全体が均一に動くことを防ぐ

---

### 4. **自律性の強化（Boidモデルへの対抗）**

#### 4.1 自律性と相互作用のバランス
**実装**: `boid_model.js` + `sketch.js`

```javascript
// Boidモデル内
autonomyWeight: 0.6,         // 自律性: 60%
interactionWeight: 0.4,      // 相互作用: 40%

// sketch.js内
const combinedDirection = {
    x: prefDirection.x * 0.3 + autonomyDirection.x * 0.7, // 自律性を優先
    y: prefDirection.y * 0.3 + autonomyDirection.y * 0.7
};
```

**工夫点**:
- 自律性を60-70%に設定し、相互作用を40-30%に抑制
- ポテンシャル場の希望速度（30%）と自律性（70%）を合成
- これにより、Boidモデルの強すぎる相互作用を弱める

#### 4.2 相関ランダムウォーク
**実装**: `robot_agent.js` の `updateAutonomy()`

```javascript
this.currentHeading = ... // 現在の進行方向
this.headingPersistence = 5.0秒; // 方向を保持する時間
```

**工夫点**:
- 完全にランダムではなく、前の方向から小さな角度変化（`persistenceAngle: 0.25`）
- 方向を5秒間保持し、その後小さく変化
- 急旋回は3%の確率で±90度

**効果**: 自然な動きを実現し、局所最小値に留まることを防ぐ

#### 4.3 恒常性（Homeostasis）システム
**実装**: `robot_agent.js` の `updateHomeostasis()`

```javascript
this.homeostasisEnergy = 0.5-1.0; // エネルギー値
this.homeostasisTarget = 0.6-0.9;  // 目標値
```

**工夫点**:
- エネルギーが減衰し、目標値から外れると探索行動を促進
- 20%のロボットは低い恒常性（0.2-0.5）を持ち、人間の周りを動き回る
- 位置履歴を追跡し、静止していると強制的に移動

**効果**: 同じ場所に留まることを防ぎ、探索を促進

---

### 5. **物理的制約の反映**

#### 5.1 2輪ロボットの差動駆動
**実装**: `robot_agent.js` の `computeDifferentialDrive()`

```javascript
this.wheelBase = 70.0; // 車輪間距離（mm）
const leftWheelSpeed = linearVelocity - (angularVelocity * wheelBase) / 2;
const rightWheelSpeed = linearVelocity + (angularVelocity * wheelBase) / 2;
```

**工夫点**:
- 2輪ロボットの物理的制約を反映
- 速度ベクトルをロボットの向きに投影（前進/後進のみ）
- 角速度を30-60度/秒に制限

**効果**: 実際のロボットの動きに近づける

#### 5.2 向きベースの移動
**実装**: `robot_agent.js` の `updateOrientationBasedMovement()`

```javascript
const threshold = Math.PI * 0.8; // 144度以上の場合のみ回転
```

**工夫点**:
- 144度以上の角度差がある場合のみ回転
- それ以外は移動しながら向きを調整
- 20%以外のロボットは向きが合っていなくても移動

**効果**: 回転を最小限にし、動き続けることを優先

---

### 6. **局所最小値からの脱出**

#### 6.1 輪郭検出による脱出
**実装**: `robot_agent.js` の `updateAutonomy()`

```javascript
if (this.stuckTimer > this.stuckThreshold) { // 2秒間動かない
    const contours = this.clusterDetector.estimateContours(robots);
    const escapeDirection = this.clusterDetector.getEscapeDirection(this, contour);
}
```

**工夫点**:
- 2秒間20mm未満の移動で局所最小値と判定
- クラスタの輪郭を検出し、外側方向に脱出
- 輪郭検出が失敗した場合は完全にランダムな方向に変更

**効果**: デッドロック状態から脱出

#### 6.2 強制移動タイマー
**実装**: `robot_agent.js` の `updateHomeostasis()`

```javascript
if (this.stationaryTime > this.stationaryTimeThreshold) {
    this.forceMoveTimer = 2.0 + Math.random() * 2.0; // 2-4秒間強制移動
    this.homeostasisEnergy = 0.1; // エネルギーを急激に下げる
}
```

**工夫点**:
- 静止時間が閾値を超えると強制移動モードに入る
- 強制移動中は速度を200%に上げる
- ランダムな方向に移動を強制

**効果**: 静止状態を防ぐ

---

### 7. **蛇行（Serpentine）による自然な動き**

**実装**: `robot_agent.js` の `updateSerpentine()`

```javascript
this.serpentineTargetAngle = (Math.random() - 0.5) * 2 * Params.serpentine.angleChange;
// 2秒ごとに方向を変える
```

**工夫点**:
- 2秒ごとにランダムな方向変化（最大0.15ラジアン）
- 平滑化により急激な変化を防ぐ
- 30%の影響度で速度ベクトルに適用

**効果**: 直線的な動きを避け、自然な動きを実現

---

### 8. **空間ハッシュによる最適化**

**実装**: `boid_model.js` の `updateSpatialHash()`

```javascript
this.hashCellSize = 200.0; // セルサイズ
const key = `${ix},${iy}`; // ハッシュキー
```

**工夫点**:
- 空間を200mmのセルに分割
- 近傍検索を現在のセルと周囲8セルのみに限定
- O(n²)からO(n)に計算量を削減

**効果**: 計算コストを削減し、リアルタイム処理を可能に

---

### 9. **人間との相互作用**

#### 9.1 人間の足の間を通り抜ける
**実装**: `robot_agent.js` の `findPathAroundHumanFeet()`

```javascript
// 人間が立っている場合：20%のロボットが50cm以内で5台に限定
if (this.isLowHomeostasis && distToCenter < 500.0) {
    // 40-70cm範囲で探索
}
```

**工夫点**:
- 人間が立っている場合、低い恒常性のロボット（20%）が50cm以内で5台に限定
- 40-70cmの範囲で探索
- 人間が動いている場合は足を避ける

**効果**: 人間との自然な相互作用を実現

#### 9.2 ポテンシャル場による回避
**実装**: `spot_potential.js` + `sketch.js`

```javascript
const gradient = potentialField.getGradient(robot.x, robot.y);
let vxPref = -Params.potential.alpha * gradient.gx;
```

**工夫点**:
- 人間の位置を負のポテンシャルとして定義
- 勾配を計算し、反発方向に移動
- 自律性（70%）とポテンシャル場（30%）を合成

**効果**: 人間領域を自然に回避

---

### 10. **動的パラメータ変更（収束防止）**

**実装**: `robot_agent.js` の `updateDynamicParameters()`

```javascript
// エントロピーが低い場合（収束している）、パラメータを変更
if (avgEntropy < 0.3) {
    this.homeostasisTarget += variation;
    this.orientationAlignment += orientationVariation;
}
```

**工夫点**:
- 近傍ロボットの位置の分散度（エントロピー）を計算
- エントロピーが低い（収束している）場合、パラメータをランダムに変更
- エントロピーが高い（分散している）場合、パラメータを安定化

**効果**: 群全体が同じ場所に集約することを防ぐ

---

## 問題点と改善の余地

### 現在の問題
1. **重なりまくり**: これらの工夫があっても、Boidモデル自体が重なりを引き起こす可能性がある
2. **変な動き**: 多層的なシステムが複雑すぎて、予測困難な動きになる可能性がある
3. **計算コスト**: 多層的な衝突回避システムにより、計算コストが高い

### 改善の方向性
1. **Boidモデルの完全な排除**: 確率的発動（15%）でも重なりが発生する場合は、Boidモデルを完全に削除
2. **シンプルな衝突回避**: 予測回避や混雑度推定を簡略化し、基本的な反発力のみに
3. **個体差の強化**: より大きな個体差により、群としての協調性を弱める

---

## まとめ

`swarm_control_p5js`は、Boidモデルの問題を解決するために、以下の多層的な工夫を実装しています：

1. **Boidモデルの弱体化**: 15%の確率でしか発動しない
2. **多層的な衝突回避**: 緊急停止、予測回避、混雑度回避を重み付け合成
3. **個体差の導入**: 速度分布と個体パラメータの変動
4. **自律性の強化**: 相関ランダムウォーク、恒常性システム
5. **物理的制約の反映**: 2輪ロボットの差動駆動、向きベースの移動
6. **局所最小値からの脱出**: 輪郭検出、強制移動
7. **自然な動き**: 蛇行、動的パラメータ変更

しかし、これらの工夫があっても「重なりまくりで変な動きしかしない」問題が残っているため、**Boidモデルを完全に排除し、よりシンプルなアプローチに移行する必要がある**と考えられます。

