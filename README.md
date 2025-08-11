# ros1_msg_to_csv

ROSバッグファイルをCSV形式に変換するROS1パッケージです。ROSバッグファイルから各種メッセージを抽出し、データ解析や可視化に適したCSVファイルに変換します。

## 概要

`ros1_msg_to_csv`パッケージは、ROSバッグファイルに含まれる様々なメッセージタイプをCSV形式に変換するツールを提供します。センサーデータ、ジオメトリメッセージ、変換データなど、複数のメッセージタイプをサポートしており、ロボットシミュレーションデータや実験結果の後処理に特に有用です。

## 機能

- **複数メッセージタイプ対応**: 以下のROSメッセージタイプに対応
  - `sensor_msgs/JointState`
  - `geometry_msgs/WrenchStamped`
  - `tf2_msgs/TFMessage` (tfおよびtf_static)
  - `std_msgs`の標準メッセージ

- **バッチ処理**: 複数のバッグファイルを一度に処理可能
- **トピック選択**: 抽出するトピックを指定可能
- **変換データサポート**: TFデータの特別な処理とフレーム変換
- **自動出力整理**: 出力ファイルの整理されたディレクトリ構造を作成

## 使用方法

### 基本的な使用方法

パッケージにはlaunchファイルが用意されています：

```bash
roslaunch ros1_msg_to_csv bag_to_csv.launch
```

### 設定

`launch/bag_to_csv.launch`ファイルを編集して設定を行います：

#### 必須パラメータ

- **`bags`**: 処理するバッグファイルのパスのリスト
- **`topics`**: バッグファイルから抽出するトピックのリスト

#### オプションパラメータ

- **`tf`**: 変換設定
  - `parent_frame`: 変換データの親フレーム
  - `child_frames`: 抽出する子フレームのリスト
- **トピック固有設定**: 特定のトピックのフレーム変換設定

### 設定例

```xml
<rosparam>
  bags:
    - /workspace/vehicle_pose_fixed_position_1m_2025-08-10-05-36-08.bag
  topics:
    - /tf
    - /tf_static
    - /shi_follower_vehicle_1/cable_wrench_publisher/leaf_wrench
    - /shi_cable_feeder/joint_states
    - /shi_cable_follower1/joint_states
    - /shi_follower_vehicle_1/joint_states
  tf:
    parent_frame: world
    child_frames: [shi_leader_vehicle]
  /shi_follower_vehicle_1/cable_wrench_publisher/leaf_wrench:
    reference_frame: world
    source_frame: shi_leader_vehicle
</rosparam>
```

### コマンドライン使用

ノードを直接実行することも可能です：

```bash
rosrun ros1_msg_to_csv bag_to_csv _bags:="['/path/to/bagfile.bag']" _topics:="['/joint_states', '/tf']"
```

## 出力形式

### ファイル構成

各バッグファイルに対して以下の構造で出力ディレクトリが作成されます：
```
<バッグファイル名>/
├── <バッグファイル名>-<トピック1名>.csv
├── <バッグファイル名>-<トピック2名>.csv
└── ...
```

### CSV形式

各CSVファイルには以下が含まれます：
- **ヘッダー行**: メッセージフィールドを説明する列名
- **データ行**: タイムスタンプ付きのメッセージデータ

#### 例: JointState CSV
```csv
/header/frame_id,/header/stamp,/header/seq,/name[0],/position[0],/velocity[0],/effort[0],/name[1],/position[1],/velocity[1],/effort[1],...
base_link,1628123456789012345,1,joint1,0.5,0.1,10.2,joint2,-0.3,0.05,5.8,...
```

## サポートされているメッセージタイプ

### 現在サポート中

| メッセージタイプ | 説明 |
|------------------|------|
| `sensor_msgs/JointState` | ジョイントの位置、速度、力データ |
| `geometry_msgs/WrenchStamped` | 力とトルクの測定値 |
| `tf2_msgs/TFMessage` | 変換データ (tfおよびtf_static) |
| `std_msgs/*` | 標準メッセージタイプ |

### 新しいメッセージタイプの追加

新しいメッセージタイプのサポートを追加するには：

1. `include/ros1_msg_to_csv/msg/<パッケージ名>/`に新しいヘッダーファイルを作成
2. `BaseMsg`を継承するクラスを実装
3. `onInit()`と`save()`メソッドをオーバーライド
4. `Registrar`クラスを使用してメッセージタイプを登録
5. `include/ros1_msg_to_csv/msgs.hpp`に新しいヘッダーをインクルード

実装例：
```cpp
class NewMsg : public BaseMsg {
public:
  bool onInit(const std::string& filename) override {
    // CSVファイルの初期化とヘッダー書き込み
  }
  
  void save(rosbag::MessageInstance const m) override {
    // メッセージデータの抽出とCSVへの書き込み
  }
  
  static Registrar<NewMsg> registrar;
};

Registrar<NewMsg> NewMsg::registrar("package_name/MessageType");
```

## 使用例

### シミュレーションデータの処理

```bash
# ジョイント状態と力データを含むシミュレーションバッグファイルの処理
roslaunch ros1_msg_to_csv bag_to_csv.launch
```

設定例：
```xml
<rosparam>
  bags:
    - /workspace/simulation_data.bag
  topics:
    - /robot/joint_states
    - /force_sensor/wrench
    - /tf
</rosparam>
```

### 複数実験のバッチ処理

```xml
<rosparam>
  bags:
    - /data/experiment1.bag
    - /data/experiment2.bag
    - /data/experiment3.bag
  topics:
    - /joint_states
    - /imu/data
    - /tf
</rosparam>
```

## パフォーマンスに関する考慮事項

- 大きなバッグファイルは処理に時間がかかる場合があります
- 高頻度のトピックではCSVファイルが大きくなる可能性があります
- 必要なデータのみにトピックをフィルタリングすることを検討してください
- 複数の大きなバッグファイルを処理する際はディスク容量を監視してください

