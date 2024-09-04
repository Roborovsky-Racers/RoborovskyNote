---
title: Roborovsky Note
---

# 予選大会におけるラップタイム短縮の工夫

## はじめに

本記事では、AIチャレンジ2024予選時にチーム Roborovsky が行っていた、ラップタイム短縮の工夫ポイントを紹介します。

## 工夫ポイント一覧

- ブースト加速のハンドリング
- 経路は曲率最小経路を選択
- ブレーキ戦略
- 制御レートの向上

## 前提：予選のソフトウェア構成

チーム Roborovsky では車両制御にOSSである [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC) を改良して用いました。

ノードグラフの主要部は下図の通りです。 mpc_controller ノードが [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC) 改良版の ROS 2 ノードであり、障害物を回避しつつ目標経路に追従する走行コマンドを車両に publish します。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/preliminary_round_architecture/pre_round_architecture.png?raw=true" width="600"/>

詳細は別の記事 [予選大会におけるソフトウェア構成と30km/s速度制限を突破する方法](preliminary_round_architecture.md) をご覧ください。


## ブースト加速のハンドリング

AIチャレンジ 2024 予選大会で上位に入賞するにはブースト加速の導入が必須でした。

> **ブースト加速 ※とは**
> シミュレータ (AWSIM) 本来の速度制限 30km/h を超える速度まで加速すること。極端に大きな加速度を高レートで車両に指令すると確率的に発生する。
> 詳細は [こちらの記事](preliminary_round_architecture.md) の「30km/h 速度制限の突破について」の節をご覧ください。
>
> ※ 「ブースト加速」は Roborovsky がつけた呼称であり、正式な呼称はありません。

ラップタイムを縮めるには **ブースト加速の発生確率を高めること** が重要です。
一方で、タイミングによっては **ブースト加速を無効化すること** も重要になります。急カーブ等でブースト加速してしまうと横転のリスクが高まるためです。

そこで、我々のチームでは下図のようなノード構成を取りました。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/awsim_kart_speed_opt_tips/boost_commander.png?raw=true" width="400px"/>

mpc_controller は boost_commander に対し

- command (AckermannControlCommand)
- boost_mode (bool)

を一つのメッセージで publish します。boost_commander は boost_mode の true/false に応じ、 command の publish rate を切り替えます。mpc_controller の制御レートはたかだか 40 Hz ですが、この方式によって任意のタイミングで車両のコマンドレートを高めることができるようになります。



## 最後に



---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
