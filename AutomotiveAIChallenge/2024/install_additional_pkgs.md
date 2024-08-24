---
title: Roborovsky Note
---

# 予選オンライン評価環境に追加パッケージをインストールして使用する方法

## はじめに

AIチャレンジ2024予選では、各チームが作成したソースコード一式をオンライン環境にアップロードし、採点が行われるルールとなっています（詳細は[こちら](https://automotiveaichallenge.github.io/aichallenge-documentation-2024/preliminaries/submission.html)）。

オンライン環境では、大会運営側が用意したDockerfileを使用してDockerイメージがビルドされ、そのイメージから生成されたDockerコンテナ上で評価が実行されます。
これらの手順は自動化されており、かつ公式が管理するリポジトリの[Dockerfile](https://github.com/AutomotiveAIChallenge/aichallenge-2024/blob/main/Dockerfile)が使用されるようになっているため、参加者側がDockerイメージに手を加えることはできません。

本件について、大会公式Slackで弊チームから質問したところ、以下のような回答を得ました。

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/install_additional_library/slack_qa_pip_install.png?raw=true" width="600px" />

運営側に依頼することでDockerイメージにパッケージを追加していただくことも不可能ではないようですが、検討に時間がかかるようであり、また全ての参加者の要望を聞くことは難しいと思われるため、予選の短い期間内ではあまり現実的ではないと感じました。

そのため、Dockerfileに `apt-get install` や `pip install` などのコマンドを記述して、Dockerイメージのビルド時に追加のパッケージやライブラリをインストールすることは簡単にはできないため、独自開発するパッケージで追加の依存を持たせるためには少し工夫が必要となります。


## 追加パッケージをインストールする方法の例

Dockerfileに手を加えずとも追加パッケージやライブラリをインストールする方法がいくつかあるため、以下に紹介します。

### 1. package.xml に外部依存パッケージを記述する方法

ROSパッケージ開発を行う上で最もスタンダードな方法です。
各パッケージの `package.xml` に `<depend>` などで外部依存パッケージを記述しておくことで、 `rosdep install` コマンドの実行時に必要なパッケージの `apt-get install` が自動的に実行されます。

`rosdep` に関しては、 @strvさんの[こちらの記事](https://qiita.com/strv/items/ccd89c55957bd8dfada0)が詳しいです（記事中ではROS1が前提となっていますが、ROS 2でも同様に使用可能）。

オンライン環境のDockerfileでも `colcon build` 実行前に [rosdep install が実行される](https://github.com/AutomotiveAIChallenge/aichallenge-2024/blob/v2024.7.19.0/Dockerfile#L41)ため、有効な方法です。

しかしながら、この方法でインストールできるパッケージは、 [rosdistro](https://github.com/ros/rosdistro) に登録されている公式のパッケージに限定されてしまうため、登録されていないパッケージを使用したい場合、この方法では対応できません。

### 2. アップロードするソースコード一式に外部依存パッケージそのものを含んでおく方法

少し強引ですが、確実な方法です。

必要な外部依存パッケージをすべて開発環境のソースコードディレクトリ内に全て配置してビルド時などに適切に参照されるようにしておきます。

この方法であればオンライン環境でのDockerイメージビルド時に追加でインストール作業が必要ありません。

公式Slackで質問した際にも回答の一つでご提案いただきました（@Manotec_真野(代表)様 ありがとうございます！）

<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/AutomotiveAIChallenge/2024/.images/install_additional_library/slack_qa_pip_install2.png?raw=true" width="600px" />

ただ、外部依存パッケージが少数であったり構成がシンプルであればこの方法も十分実用的なのですが、外部依存がさらに別の依存を持っていたり規模が大きくなってくると、ビルド手順が非常に複雑になる場合も多く管理も面倒となってくるため、できれば避けたい方法です。

### 3. CMakeLists.txt に記述した execute_process を利用する方法

我々のチームが採用した方法です。

各パッケージの `CMakeLists.txt` に `execute_process` を記述しておくことで、ビルド時にある程度任意のコマンドを実行させることが可能です。

オンライン環境のDockerfileでは、[この部分](https://github.com/AutomotiveAIChallenge/aichallenge-2024/blob/v2024.7.19.0/Dockerfile#L42) の `colcon build` のタイミングで `execute_process` を実行させることができます。

我々はこの方法で `pip install` を実行させて追加の外部依存パッケージをインストールしています。

ただし、インストールパッケージによっては、標準環境にインストールされているパッケージと競合し、Autoware の実行環境を破壊してしまう場合もあるため、`virtualenv` を

作成した上で `pip install` を行う方法を取りました。

## 具体的なインストール方法の紹介

我々のチームが作成したROS 2パッケージで使用している `CMakeLists.txt` の具体的な記述例を以下に示します。

```cmake
cmake_minimum_required(VERSION 3.5.0)
project(my_ros2_package)

include(GNUInstallDirs)
find_package(ament_cmake_auto REQUIRED)

(中略)

# create python virtual environment
execute_process(
  COMMAND /usr/bin/python3 -m venv ${CMAKE_INSTALL_PREFIX}/.venv
  RESULT_VARIABLE venv_result
)
if(venv_result)
    message(FATAL_ERROR "Failed to create virtual environment")
endif()

# install python dependencies in virtual environment
execute_process(
    COMMAND ${CMAKE_INSTALL_PREFIX}/.venv/bin/pip install -r ./resources/requirements.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE install_result
)
if(install_result)
    message(FATAL_ERROR "Failed to install python dependencies in virtual environment")
endif()

ament_auto_package()
```

ここでは大きく分けて以下の2つの `execute_process` コマンドを記述しました。

### 1. `virtualenv` を構築

以下が対応部分です。
```cmake
# create python virtual environment
execute_process(
  COMMAND /usr/bin/python3 -m venv ${CMAKE_INSTALL_PREFIX}/.venv
  RESULT_VARIABLE venv_result
)
if(venv_result)
    message(FATAL_ERROR "Failed to create virtual environment")
endif()
```

venv の作成先として指定している `${CMAKE_INSTALL_PREFIX}/.venv` は、 `/path/to/colcon_ws/install/package_name/.venv` を指しています。
また、 `RESULT_VARIABLE` を用いて venv が正常に作成できたかどうかをチェックしています。

### 2. 構築した `virtualenv` に `pip install` で外部依存パッケージをインストール

以下が対応部分です。
```cmake
# install python dependencies in virtual environment
execute_process(
    COMMAND ${CMAKE_INSTALL_PREFIX}/.venv/bin/pip install -r ./resources/requirements.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE install_result
)
if(install_result)
    message(FATAL_ERROR "Failed to install python dependencies in virtual environment")
endif()
```

先程の手順で作成した venv に対して `pip install` を行っています。
ここでは、インストール対象パッケージとして `./resourece/requirements.txt` を指定していますが、もちろん直接パッケージ名を記述しても問題ありません。
なお、`./` はROS2 パッケージのルートディレクトリからの相対パスとなります。
また、先と同様に `RESULT_VARIABLE` で実行結果をチェックしています。

## venv を activate して実行する方法

上記では、venv に外部依存パッケージをインストールしましたが、 venv を利用して ROS2 node を実行するためには、
通常 `source .venv/bin/activate` を実行した上で `ros2 run` などを実行する必要があります。

ただし、オンライン評価環境においては、[run_evaluation.sh](https://github.com/AutomotiveAIChallenge/aichallenge-2024/blob/v2024.7.19.0/aichallenge/run_evaluation.bash) によって ROS2 node, launch が実行されるようになっており、 Dockerfile と同様に `run_evaluation.sh` を参加者側で直接変更することはできないため、venv をactivate した上で `ros2 launch` を実行させる、といったことはできません。

そこで、少し強引ですが、 launch から 直接 ROS2 node を実行するのではなく、 bash スクリプト経由で venv を activate させた状態で ROS2 node を実行させる方法を取りました。

具体的には、以下のような bash スクリプトを用意します (例: `my_ros2_package/scripts/run.bash`)。
```bash
#!/bin/bash
source $(ros2 pkg prefix my_ros2_package)/.venv/bin/activate
python3 $(ros2 pkg prefix my_ros2_package)/lib/my_ros2_package/my_ros2_node $@
```

そして、`CMakeLists.txt` に以下のように書きます。
```cmake
install(PROGRAMS
  scripts/run.bash
  DESTINATION lib/${PROJECT_NAME}
)
```

こうしておけば、 `ros2 run my_ros2_package run.bash` といった形で venv をactivate した状態で ROS 2 nodeを実行できるようになります。
後は、オンライン評価環境から実行される `aichallenge_submit_launch/launch/aichallenge_submit.launch.xml` や、 `reference.launch.xml` などの中で、以下のように上記 bash スクリプトを起動する記述をしておけばOKです。
```xml
<node pkg="my_ros2_package" exec="run.bash" name="my_ros2_node" output="screen" />
```

これらの方法が実際のオンライン評価環境でも問題なく動作することは確認済みです。

## 最後に

AIチャレンジ2024 予選において、追加の外部依存パッケージをインストールして利用する方法を紹介しました。

Python や pip, venv に重点を置いた内容となってしまいましたが、apt や cargo など他のパッケージマネージャを使用するといったことも可能かと思います。

ただし、今回紹介した execute_process や bashスクリプトを使用する方法は汎用性が高く、ある程度任意のコマンドが実行できてしまう面もあるため、くれぐれも悪用等はされないようご注意願えればと思います。


---
<img src="https://github.com/Roborovsky-Racers/RoborovskyNote/blob/main/.images/roborovsky_logo.png?raw=true" width="75" />

[Roborovsky Note トップページ](https://roborovsky-racers.github.io/RoborovskyNote/)
