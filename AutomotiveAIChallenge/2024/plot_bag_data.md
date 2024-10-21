---
title: Roborovsky Note
---

# rosbag のデータをプロットするスクリプト

## はじめに
本記事では、自動運転AIチャレンジ 2024 の実機走行やAWSIMで取得した rosbag からデータを抽出しプロットするスクリプトを公開します。

## スクリプトの公開場所

スクリプトは [Roborovsky-Racers/aic_tools リポジトリ](https://github.com/Roborovsky-Racers/aic_tools/blob/main/scripts/analyze_bag.py) に公開しています。


### 実行手順
事前に、リポジトリ aic_tools をクローン＆ビルドしてください。

標準ではインストールされていない依存パッケージがいくつかあるため、[こちらの記事](install_additional_pkgs.md)と同様の方法でvenvを`colcon build`時に構築し、[こちらのbashスクリプト](https://github.com/Roborovsky-Racers/aic_tools/blob/main/scripts/analyze_bag.bash)経由で実行するようにしています。

実行時には引数を指定するようになっており、以下のコマンドでヘルプを表示することができます。
```bash
$ ros2 run aic_tools analyze_bag.bash -h
usage: analyze_bag.py [-h] [-r REFERENCE_PATH_CSV_PATH] [-m MAP_YAML_PATH] [-w] [-s] input_bag_directory

positional arguments:
  input_bag_directory   Directory where bag files (*.db3 and metadata.yaml) are stored

options:
  -h, --help            show this help message and exit
  -r REFERENCE_PATH_CSV_PATH, --reference_path_csv_path REFERENCE_PATH_CSV_PATH
                        Path to the reference path csv file
  -m MAP_YAML_PATH, --map_yaml_path MAP_YAML_PATH
                        Path to the occupancy grid map yaml file
  -w, --overwrite       Overwrite existing csv files
  -s, --save_plot       Save the plot as a png file
```

通常の用途では、bagファイル(`*.db3`と`metadata.yaml`)が存在するディレクトリのパスを第一引数で指定します。

実行例:
```bash
$ ros2 run aic_tools analyze_bag.bash /logs/20241011_training/rosbag_trim/rosbag2_2024_10_11-18_03_34_trim_1/ 
```

実行すると、以下の画像のように、走行時のekfによる自己位置推定結果(`/localization/kinematic_state`)とGNSSの取得データ(`/sensing/gnss/pose`)のプロットと専有格子地図が重ねて表示されます。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/plot_bag_data/plot_example.png?raw=true" width="600px" />
<!-- <img src=".images/plot_bag_data/plot_example.png?raw=true" width="600px" /> -->

### スクリプト処理の流れ

スクリプト処理の流れを以下に説明します。

1. 読み込むtopicの設定
2. bagファイルを読み込み、1で指定したtopicのデータをcsvに書き出す
3. 2で書き出したcsvファイルを読み込み pandas dataframe に格納
  ※手順2は少し重い処理となるため、同じbagファイルを指定した2回目以降の実行では、手順2をスキップすることで動作を高速化しています。
4. 各topicのデータのうち、最も長いtimestampを持つデータのtimestampを基準としてデータの補間を行うことで、全てのデータのtimestamp（とデータの長さ）を一致させる。
5. プロット処理: デフォルト動作では上記で説明した自己位置推定結果などがプロットされます。

### 別のデータをプロットしたい場合の手順
詳細はソースコードをご確認いただければと思いますが、概要だけ以下に示します。

#### 取り込みたいtopicの追加
[topic_handler.py](https://github.com/Roborovsky-Racers/aic_tools/blob/9b465278df674b3de2cfba9dbbe54ced9948550b/aic_tools/topic_handler.py) に対象のtopicの情報を持った以下のようなHandlerクラス定義を追加します。

例:
```python
class ImuDataHandler(TopicHandler):
    TOPIC_NAME = "/sensing/imu/imu_data"

    def extract_data(self, msg):
        return {"gyro_z": msg.angular_velocity.z}
```

`extract_data`関数の引数である`msg` にはtopicのメッセージ型でデータが渡されてくるようになっているので、各メッセージ定義を調べて必要なメンバ（全てのメンバを列挙する必要はありません）をdict型でreturnするようにしてください。
ここでdictのkeyとして設定した名称で、データがdataframeに登録されます。
なお、timestampは自動的に含まれるようになっているので、ここで明示的に記述する必要はありません。

#### 作成したHandlerクラスを`TARGET_HANDLERS`に追加する
[analyze_bag.py の `TARGET_HANDLER`定義](https://github.com/Roborovsky-Racers/aic_tools/blob/9b465278df674b3de2cfba9dbbe54ced9948550b/scripts/analyze_bag.py#L76-L82) に先程実装したHandlerクラスを追加します。この時、importの記述も必要となります。

```
from aic_tools.topic_handler import (
    ~~~
    ImuDataHandler,
    ~~~
)
    TARGET_HANDLERS = [
        ~~~
        ImuDataHandler,
        ~~~
    ]
```

これにより、dataframeに自動でデータが格納されるようになります。

#### データのプロット
まず、[既存のプロット処理](https://github.com/Roborovsky-Racers/aic_tools/blob/9b465278df674b3de2cfba9dbbe54ced9948550b/scripts/analyze_bag.py#L101-L126)を必要に応じてコメントアウトします。

その後、コメントアウトした前後どちらかに、例えば以下のように記述することでプロットを表示することができます。

```
    plt.plot(df.stamp, df.gyro_z)
    plt.show()
```

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/plot_bag_data/plot_example2.png?raw=true" width="300px" />
<!-- <img src=".images/plot_bag_data/plot_example2.png?raw=true" width="300px" /> -->


## さいごに
rosbagの解析は `ros2 bag play` を使うことも多いですが、実行に手間や時間がかかることが難点です。
plotjugglerでもrosbagデータのプロットが可能であり非常に有用ですが、ちょっとしたデータの加工をしたくなる場合もあり、その場合にはpythonスクリプトでデータを処理できると便利なことも多いかと思います。
本スクリプトを使用、あるいは改造や参考にしていただくことで、皆様のデータ解析や開発が効率化されましたら幸いです。


---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
