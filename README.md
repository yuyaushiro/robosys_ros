robosys_ros
========================
robosys2016 2nd homework  

動作方法  
-------------------
cv_cameraとimage_viewの２つのパッケージを用意  
端末を開き以下を実行  
```  
$ roslaunch robosys_ros ball_detector.launch
```  
動画のデモ  

ball_detector_node
-------------------

知能ロボットコンテストなどで使用するカラーボールを検出し、ボールの座標（pixel）と色をパブリッシュする.  
検出したいボールの色(HSVそれぞれの値の範囲)を指定し,色を抽出.  
円形度の閾値を指定しボール（円形）以外の物体除去を行い,ボールのみを検出.  

カメラドライバノードよりカメラ映像をサブスクライブし,処理を行う.  
ボールのx座標,y座標,色（コード）の3つの情報を検出したボールの個数分,可変長配列として送信するためのメッセージファイルを作成し,それにより検出結果をパブリッシュしている.  
その他には,検出したボールを矩形で示した映像と,赤青黄それぞれの抽出したマスク画像をパブリッシュしている
コンフィグファイルを作成しdynamic_reconfigureパッケージを利用することで,検出したいボールの色と円形度閾値のGUIによる調整を行えるようにした.  

### Subscribe ###

* image_rect_color (sensor_msgs/Image)
* camera_info (sensor_msgs/CameraInfo)

### Publish ###

* ball_detections (robosys_ros/BallDetectionArray) 検出したボールの情報の配列
* ball_detections_image (sensor_msgs/Image) 検出したボールを矩形で示した映像
* COLOR_extract_image (sensor_msgs/Image) 抽出したマスク映像(COLOR = red, blue, yellow)

### Parameters ###

* ball_threshold/circularity (int: default 70) 円形度閾値[%]
* ball_threshold/label_area (int 400) ラベリング最小面積[pixel]
* COLOR_threshold/hue_lower (int) 色相下限
* COLOR_threshold/hue_upper (int) 色相上限
* COLOR_threshold/saturation_lower (int) 彩度下限
* COLOR_threshold/saturation_upper (int) 彩度上限
* COLOR_threshold/value_lower (int) 明度下限
* COLOR_threshold/value_upper (int) 明度上限  
(COLOR = red, blue, yellow)

Nodelet
-------------------

robosys_ros/BallDetectorNodelet
