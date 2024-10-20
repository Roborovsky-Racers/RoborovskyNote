---
title: Roborovsky Note
---

# 事前練習日(2024/10/11)の rosbag を公開

## はじめに
自動運転AIチャレンジ 2024 では 2024/10/10-11 に事前練習として各チームの試走会が実施されました。
本記事ではチーム Roborovsky が事前練習にて取得した rosbag を公開します。

## rosbag を公開

Google Drive にて公開します。

- [事前練習日 10/11 に取得した rosbag](https://drive.google.com/drive/folders/1_eN6WKaxKKdhQa0Bp0BvelMzQcZJaFmC?usp=drive_link)

rosbag の説明
- 車両ID
  - A8, `ECU-RK-08`
- 総ファイル数
  - 11 ファイル
  - ※ インデックス 1~12 まであるが、インデックス 8 は無効なデータのため除外してある
  - `vehicle/status/control_mode` が `AUTONOMOUS` の期間のみを抽出し、計 11 のファイルに分割してある
- 各bagに含まれる走行の概要
  - 上記 Google Drive に `plot_png` というディレクトリがあり、そこに走行軌跡などをプロットした画像がある
  - 上記走行軌跡の画像や、その他bagデータのプロット方法などについては、[こちらの記事](plot_bag_data.md)も参照


## さいごに
実機による走行では様々な要因によって rosbag の取得が困難であったケースもあるかと思います。
本 rosbag が開発の助けになれば幸いです。


---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
