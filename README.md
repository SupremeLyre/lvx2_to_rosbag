# lvx2_to_rosbag

## 概要
LivoxのLiDAR(Mid-360とか)のログ形式であるlvx2ファイルからROSbagを作成するためのスクリプト。

## 特徴
- for ROS1 noetic

## 動作環境
- OS: Ubuntu 20.04
- 言語: Python 3.7
- その他依存関係: ROS2 noetic

## インストール方法
1. 必要な依存関係をインストール  
   特になし

3. リポジトリをクローン
   ```sh
   git clone https://github.com/DHA-Tappuri/lvx2_to_rosbag.git
   cd lvx2_to_rosbag
   ```

## 使い方
- サンプルデータの変換
- サンプルデータをLivoxのサポートからダウンロードしてくる  
  https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid360/Indoor_sampledata.lvx2  
- サンプルデータをpythonスクリプト(lvx2_to_rosbag.py)と同じフォルダに入れる
- スクリプトを実行する
   ```sh
   python3 lvx2_to_rosbag.py --in_file ./Indoor_sampledata.lvx2 --out_file ./Indoor_sampledata.bag --pc2_topic livox_points --pc2_frame_id livox_frame
   ```

## パラメータ
- `in_file` 変換対象のlvx2ファイル
- `out_file` 変換先のbagファイル
- `pc2_topic` 点群ファイルのPointCloud2形式の出力トピック名(default : livox_points)
- `pc2_frame_id` 点群ファイルのフレームID(default : livox_frame)

## ライセンス
[MIT](LICENSE) など、プロジェクトのライセンスを記載します。

## 備考
- タイムスタンプは同一フレームデータ内のタイムスタンプの平均値を出しています．これで正しいのかは不明．
- 生成されるbagファイルはROS1(noetic)用です．多分ROS2でも動くけどROS2用に変更してくれる方歓迎です．  
  (lvx2ファイルから点群データを抜き取る部分と，それをrosbagへ書き込む部分が独立しているので多少は楽なはず)
- コードが汚いので気をつけて

## 問い合わせ
- dha.tappuri@gmail.co.jp

---

