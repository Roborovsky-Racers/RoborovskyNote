---
title: Roborovsky Note
---

# rosbag から自動走行モードの期間を抽出するスクリプト

## はじめに
本記事では、自動運転AIチャレンジ 2024 の実機走行で取得した rosbag から自動走行モードの期間を抽出するスクリプトを公開します。

## 自動走行モードの期間の抽出スクリプト

スクリプトは [Roborovsky-Racers/aic_tools リポジトリ](https://github.com/Roborovsky-Racers/aic_tools/blob/main/scripts/auto_mode_period_marker.py) に公開しています。
本スクリプトは `vehicle/status/control_mode` トピックを監視し、 値が `AUTONOMOUS`であるの期間の始点と終点を抽出して `auto_period.csv` に出力します。

auto_period.csv

|start|end|
|---|---|
|t_start_1|t_end_1|
|t_start_2|t_end_2|
|...|...|


### 実行手順
1. リポジトリ aic_tools をクローン＆ビルド
1. `ros2 run aic_tools auto_mode_period_marker.py`　を実行
1. 抽出対象の rosbag を play (`ros2 bag play BAG_FILE_NAME` など)
1. rosbag の再生が完了したら auto_mode_period_marker を Ctrl+c で終了
1. 最終的な出力：auto_period.csv が `ros2 run`の実行ディレクトリに保存される

### 抽出期間を用いて rosbag を分割する方法
TIER IV さんが公開している [ros2bag_extensions](https://github.com/tier4/ros2bag_extensions) が便利です。
[`ros2 bag slice`](https://github.com/tier4/ros2bag_extensions?tab=readme-ov-file#ros2-bag-slice) コマンドの start/end 時刻に対し、 auto_period.csv に出力された時刻を指定してください。

## さいごに
実機による走行で取得される rosbag は長時間になる場合も多いかと思います。
本スクリプトにより開発が効率化されたら幸いです。


---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
