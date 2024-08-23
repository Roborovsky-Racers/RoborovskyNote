# 走行軌道の最適化ツールを自動運転AIチャレンジ2024コースに適用

## 本記事の要点

* 最適な走行軌道を生成するツール "[global_trajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)" を自動運転AIチャレンジ 2024 のコースに適用する方法を紹介する
* 適用したソースコードは [こちら](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/tree/master) に公開中

## はじめに

トラックを走行する上で最適な軌道を生成するツールとして [global_trajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) があります。



"最適" の評価指標として次の選択肢から選ぶことができます。
* 最短時間（mintime）
* 最短距離（shortest_path）
* 最小曲率（mincurv）

最短時間軌道の場合
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/global_trajectory_optimization/mintime.png?raw=true" width="400px"/>

最短距離軌道の場合
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/global_trajectory_optimization/shortest.png?raw=true" width="400px" />

本ツールの詳しい内容については [M.Eguchiさんによるこちらの記事](https://zenn.dev/tamago117/articles/b021d2fcb875cc) をご覧ください。私自身、本ツールを今回使用するにあたってこちらの記事に助けられました。ありがとうございます。

## 環境構築

[global_trajectory_optimization 本家](https://github.com/TUMFTM/global_racetrajectory_optimization) の最終更新はやや古く、 requirments.txt に従っても動作しない可能性があります。
そこで、適切なバージョンの Python パッケージをインストールするためのスクリプトを [こちら](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/blob/master/setup_env.bash) を用意しました。少なくとも自動運転AIチャレンジ 2024 の Docker 環境においては正しく機能することを確認しています。


環境構築手順


```bash
git clone git@github.com:Roborovsky-Racers/global_racetrajectory_optimization.git
cd global_racetrajectory_optimization
./setup_env.bash
```

