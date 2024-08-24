---
title: Roborovsky Note
---
# 予選大会におけるソフトウェア構成

## はじめに

本記事では、AIチャレンジ2024予選時におけるチーム Roborovsky のソフトウェア構成を紹介します。
また、本来の速度制限である 30km/h を突破する方法についても言及します。


## [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC)

Roborovsky の車両制御の中核は OSS の [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC) が担っています。
車両のモデル情報、コースの壁情報、目標経路、障害物情報を与えれば、モデル予測制御により目標並進速度、目標舵角を計算してくれます。
AIチャレンジ2024の競技にぴったりですね！

参考GIF（出典：[https://github.com/matssteinweg/Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC)）
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/preliminary_round_architecture/Multi-Purpose-MPC.gif?raw=true" width="400px" />
とはいえ、このままでは本競技に適用できません。Roborovsky ではここに機能追加・修正・ROS 2 ノード化などを施して使用しています。カスタム内容の詳細については、また別の機会に記事にします。

# 予選のソフトウェア構成

ノードグラフの主要な部分を抜粋して下図に示します。登場するのは mpc_controller ノードのみで、こちらが [Multi-Purpose-MPC](https://github.com/matssteinweg/Multi-Purpose-MPC) 改良版の ROS 2 ノードです。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/preliminary_round_architecture/pre_round_architecture.png?raw=true" width="600" />

mpc_controller の処理を要約すると次の通りです。
- 初期化時にコースの Occupancy Grid Map ※1 と目標経路 ※2 をファイルから読み込む
- topic から車両の現在状態と障害物位置を取得
- 障害物とコース壁を避けつつ目標経路に沿うような車両へのコマンドを生成（モデル予測制御）
- 車両へコマンドをpublish

>補足
> - ※1 Occupancy Grid Map の生成方法については別の記事 [lanelet2 を Occupancy Grid Map 形式に変換する](lanelet2_to_ogm.md) をご覧ください。
> - ※2 目標経路の生成方法については別の記事 [軌道最適化ツールを自動運転AIチャレンジ2024のコースに適用](global_trajectory_optimization.md) をご覧ください。



この構成であれば、私達の環境ではコースの完走（6周）タイムが 4分10秒 前後になることを確認しています。経路を工夫すればもう少しタイムを短縮できる気もします。


## 30km/h 速度制限の突破について

AIチャレンジ 2024 の予選シミュレーションでは、AWSIM側で速度制限 30km/h が設定されています。この制限速度はカートの仕様に対してやや控えめな印象で、急カーブ含め常に最大速度 30km/h を維持して完走することが可能です。したがって、最終的な上位争いのポイントはコース取りや障害物回避のスムーズさ、そして障害物配置ガチャの引きの強さ、あたりに収束するものと思われます。この場合、トップ争いのタイムは 04:00-04:10 のあたりと予想されます。
しかしながら、実際には 30km/h の速度制限を突破する方法が存在しました。

### 速度制限の突破はルール上よいのか？
予選においては許容されると認識しています。その根拠は、大会公式Slack において運営の方より、30km/h を超える速度を出す提出を許容する旨が書き込まれた点にあります。
おそらく、大会運営の皆様はもともと 30km/h 突破を想定されていなかったものと推察しますが、30km/h を超えるチームが複数出てきてしまった以上、提出履歴をチェック・修正してリセットすることは困難と判断されたものと思われます。

とはいえ 2024/08/24 現在、一部のチームのみが速度制限を突破する方法を把握している状態です。個人的には、ここに不公平感を感じています。そのため、大会運営の皆様の当初の意図から外れる内容で恐縮ではありますが、チーム Roborovsky が発見した速度制限の突破方法をここに記載しておきます。


### 速度制限の突破方法
私達が発見した速度制限の突破方法は簡単です。次の２つを満たせば、30km/h 制限速度を無視した大きな加速度が発生しやすいことが確認されています。（私達は勝手にこれをブースト加速とよんでいます）
- 指令加速度（AckermannControlCommand.longitudinal.acceleration）に大きな値を入れる
- 指令のレートを高くする

加速度の値は大きすぎると車両が宙に飛んでしまいます。適切な値を探してみてください。システムのアーキテクチャやコントローラによっても、加速度や指令レートの適切な値は変わると思われます。

なお、あくまでブースト加速が「発生しやすい」方法であり、発生タイミングをハンドリングする方法は見つけられていません。


### 速度制限の突破は乗りこなすのが難しい
速度制限が突破されたことにより、カーブ時に速度が大きすぎると横転してしまいます。加えてブースト加速のタイミングはハンドリングできないことも相まって、カートを「乗りこなす」難易度が上がった印象があります。この対処には工夫のしがいがあり、個人的には競技としての面白さが増しまたとポジティブに捉えています。
実機では非常に危険（そもそも実現不可能）と思いますが…。


# さいごに
本記事ではチーム Roborovsky のソフトウェア構成と、速度制限 30km/h を突破する方法をご紹介しました。参考になれば嬉しいです。

そして、大会運営の皆様には、大会企画当初のご想定から外れていると思われる内容を記載することをご容赦ください。あくまで公平性の向上を目的とした記事としてご理解いただけると幸いです。改めて、自動運転AIチャレンジの企画／運営に感謝申し上げます。




---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
