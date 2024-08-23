---
title: Roborovsky Note
---

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

なお、最適指標の比較に関するの詳しい内容については [M.Eguchiさんによるこちらの記事](https://zenn.dev/tamago117/articles/b021d2fcb875cc) が詳しいです。

## 環境構築

下記を実行すれば環境構築できます。少なくとも、自動運転AIチャレンジ 2024 の Docker 環境においては動作することを確認済みです。
```bash
git clone git@github.com:Roborovsky-Racers/global_racetrajectory_optimization.git
cd global_racetrajectory_optimization
./setup_env.bash
```

※補足
- [global_trajectory_optimization 本家](https://github.com/TUMFTM/global_racetrajectory_optimization) の最終更新はやや古く、 requirements.txt に従ってインストールしても動作しない可能性があります。
そこで、適切なバージョンを [requirements.txt](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/blob/master/requirements.txt) に反映させ、Python パッケージのインストールスクリプトを [こちら](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/blob/master/setup_env.bash) に用意しています。なお、 quadprog を requirements.txt に追加していないのは意図的です。


## 軌道生成の実行

下記を実行すれば軌道生成が開始されます。
```bash
cd /path/to/global_trajectory_optimization
python3 main_globaltraj.py
```

デフォルトでは最短時間（mintime）の軌道が生成されます。評価指標を変更したい場合は [こちら](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/blob/master/main_globaltraj.py#L60)を切り替えてください。


なお、自動運転AIチャレンジ 2024 のトラックの入力データは [こちら](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/blob/master/inputs/tracks/aic_2024.csv) に格納しています。こちらのデータを生成するにあたり [toki-1441さんのこちらの記事](https://qiita.com/toki-1441/items/615d781e3a20edf22cda) に大変助けられました。ありがとうございます。


最適化に使用される設定パラメータは[こちら](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/blob/master/params/racecar.ini) にあります。私達は左右の壁から少し離れた軌道を生成したかったため、車幅を実際より広めにするなどの変更が加えてあります。あくまで参考値としてご活用ください（[本家との差分はこちら](https://github.com/Roborovsky-Racers/global_racetrajectory_optimization/commit/6ca9c96f59bc73e3c38530e67c0212eaa6bdbd25)）。

※補足
- 本家のリポジトリをそのまま実行すると、 casadi のバージョンが原因（？）か `.../casadi/core/function_internal.cpp:1832: 'eval_sx' not defined for IpoptInterface.` というエラーが発生します。[こちらの Issue](https://github.com/TUMFTM/global_racetrajectory_optimization/issues/10) に従い、修正を加えてあります。


## 最後に
[global_trajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) を自動運転AIチャレンジ 2024 の環境で実行できるようにするまでにいくつかハードルがあったため記事にしてみました。参考になれば幸いです。


---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)