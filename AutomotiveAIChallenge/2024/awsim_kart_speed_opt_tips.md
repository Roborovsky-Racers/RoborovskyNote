---
title: Roborovsky Note
---

# 予選大会におけるラップタイム短縮の工夫

## はじめに

本記事では、AIチャレンジ2024予選時にチーム Roborovsky が行っていた、ラップタイム短縮の工夫ポイントを紹介します。

## 工夫ポイント一覧

- Multi-Purpose-MPC による障害物のスムーズな回避
- ブースト加速のハンドリング
- 経路は曲率を最小化
- ブレーキ戦略
- 評価環境に合わせた根性のパラチュン

## 前提：予選のソフトウェア構成

チーム Roborovsky では車両制御にOSSである [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC) を改良して用いました。

ノードグラフの主要部は下図の通りです。 mpc_controller ノードが [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC) 改良版の ROS 2 ノードであり、障害物を回避しつつ目標経路に追従する走行コマンドを車両に publish します。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/preliminary_round_architecture/pre_round_architecture.png?raw=true" width="600"/>

詳細は別の記事 [予選大会におけるソフトウェア構成と30km/s速度制限を突破する方法](preliminary_round_architecture.md) をご覧ください。

## Multi-Purpose-MPC の活用

車両のコントローラのベースとして [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC) を用いたことにより障害物をスムーズに回避することができました。急ハンドルが抑制できるため高い走行速度を維持することができ、ラップタイムの短縮に繋がりました。

詳細は [こちらの記事](NotImplementedError.md) をご覧ください。


## ブースト加速のハンドリング

AIチャレンジ 2024 予選大会で上位に入賞するにはブースト加速の導入が必須でした。

> **ブースト加速 ※とは**
> シミュレータ (AWSIM) 本来の速度制限 30km/h を超える速度まで加速すること。極端に大きな加速度を高レートで車両に指令すると確率的に発生する。
> 経験的に、指令レートが高いほど発生確率が高まる。
> 詳細は [こちらの記事](preliminary_round_architecture.md) の「30km/h 速度制限の突破について」の節をご覧ください。
>
> ※ 「ブースト加速」は Roborovsky が命名した呼称であり、正式な呼称はありません。

ラップタイムを縮めるには **ブースト加速の発生確率を高めること** が重要です。
一方で、タイミングによっては **ブースト加速を無効化すること** も重要になります。急カーブ等でブースト加速してしまうと横転のリスクが高まるためです。

そこで、我々のチームでは下図のようなノード構成を取りました。
boost_commander は mpc_controller の車両コマンドを受け取り、そのまま車両に指令します。ただし ``boost_mode`` の ``true/false`` に応じて指令レートを 1500Hz/50Hz で切り替えます。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/awsim_kart_speed_opt_tips/boost_commander.png?raw=true" width="400px"/>

本来、 mpc_controller の制御レートはたかだか 40 Hz ですが、この方式によって車両のコマンドレートを飛躍的に高めることができ、任意のタイミングでブースト加速の発生確率を引き上げることができます。

## 経路は曲率を最小化

コースの走行経路を最適化するツールである [global_trajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)を[こちらの記事](global_trajectory_optimization.md)で紹介しました。
このツールには経路の最適化の方式として次の３つがあります。

* 走行時間の最小化
* 走行距離の最小化
* 曲率の最小化

開発を初めた当初は「走行時間の最小化」と「走行距離の最小化」で切り替えながらラップタイムの短縮を試行錯誤していました。しかしブースト加速を導入したところ、カーブでの横転が頻発するようになってしまいました。

この原因は、経路の最適化に使用する車両モデルのモデル化誤差にあると考えています。ブースト加速した場合の高い走行速度や、加速時に車体の前方が浮き上がるような挙動等は正しくモデル化されていません。そしてモデル化には大きな労力が伴います。

そこで精緻なモデル化は諦め、横転のリスクを下げるためにあえて「曲率の最小化」を選択しました。その結果、横転リスクが下がるとともに、高い走行速度を維持して走り切ることができるようになり、ラップタイムが大きく短縮されました。

## ブレーキ戦略

ブースト加速を多用しつつラップタイムを短縮するには

* いつブース加速を無効化するか
* いつブレーキをかけるか

が重要でした。常にブースト加速をしていてはすぐに横転してしまいますし、頻繁にブレーキをかけていては遅くなってしまいます。

そこで、下図のような戦略でブースト加速のON/OFFやブレーキタイミングを切り替えました。



## 最後に



---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
