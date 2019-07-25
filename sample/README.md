# BodyPublisherItemの使い方

- wrl(body)ファイルのあるロボットをROSから使えるようにする

## dae(urdf)ファイルを作る
- export-collada

## robot.cnoidファイルを作る
- ロボットの初期位置・関節角度列の指定
- BodyPublisherの controllerOptions: の記述 (使う関節及びPIDゲイン)

## robot.launchファイルを作る
- fullbody_controller: joints: にコントロールしたいjointを記述
- robotの名前をcnoidファイルと一致させる
- dae(urdf) ファイルの場所を設定する
