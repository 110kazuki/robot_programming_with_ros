# ros1_with_esp32

ros1とESP32(もしくはarduinoデバイス)を使って簡単にデータのやり取りを行ってみるサンプル

# 初期設定
1. ubuntu 18.04 LTSがインストールされたパソコン(仮想環境も可)を用意
1. ros melodicをインストール  
    > http://wiki.ros.org/melodic/Installation/Ubuntu  
1. catkin workspaceの作成  
    > http://wiki.ros.org/ja/catkin/Tutorials/create_a_workspace  
1. rosserialのインストール  
    ```  
    $ sudo apt-get install ros-melodic-rosserial
    $ sudo apt-get install ros-melodic-rosserial-arduino
    ```
1. arduino用のros1ライブラリ(roslib)をコンパイル
    ターミナルを二つ使用  
    ターミナル①  
    ```
    $ roscore
    ```
    ターミナル②  
    ```
    $ cd ~/ros_lib
    $ rosrun rosserial_arduino makelibraries.py .
    ```
1. コンパイルしたroslibをarduinoの開発を行うPCのarduino IDEのlibrariesディレクトリ内にコピー  
    windows  
    ```
    arduino IDEがインストールされたディレクトリ(Program files(x86など)/Arduino/libraries/~
    ```

    mac
    ```
    Applications/Arduino/Contents/Java/libraries/~
    ```

    ubuntu
    ```
    arduino_IDEを解凍したディレクトリ(arduino-1.8.19)/libraries/~
    ```

# パッケージのコンパイル　　
ROSで用いる特定の目的のためのノードなどを詰め合わせたもの  
パッケージのひな型はcatkinのコマンドを使って作成することができるが，プログラム内で使うライブラリにあわせてCMakeList.txt，package.xmlを編集する必要がある．

パッケージの構成  
&emsp;/package  
&emsp;&emsp;├ package.xml  
&emsp;&emsp;├ CMakeList.txt  
&emsp;&emsp;├ /src  
&emsp;&emsp;│&emsp;└ C++ nodes (.cpp)  
&emsp;&emsp;├ /scripts    
&emsp;&emsp;│&emsp;└ Python nodes (.py)  
&emsp;&emsp;├ /msg  
&emsp;&emsp;│&emsp;└ msg files (.msg)  
&emsp;&emsp;└ /launch  
&emsp;&emsp;　&emsp;└ launch files (.launch)

- パッケージの作成  
    ```
    $ catkin_create_pkg [package名 ] [依存パッケージ1] … [依存パッケージN]
    ```
        
# rosserialの使い方
rosserialは、シリアル通信経由でROSのメッセージをホストPCとArduinoデバイス間でやり取りするためのパッケージ．
USB接続でのシリアル通信だけでなく，Wi-Fiを利用したシリアル通信も行うことができる．
> http://wiki.ros.org/ja/rosserial

- ubuntuでarduinoをUSBシリアル経由で扱うための前準備  
ubuntuではarduinoデバイスをUSBシリアルで接続するたびに，許可属性(アクセス権限)を付与する必要がある. これを行わない限りスケッチの書き込みやシリアルポートを利用できない．
許可属性の設定を行うのは面倒なので，自動で許可属性が設定されるようにする． 
    
    1. arduinoデバイスをUSBケーブルでubuntuに接続している場合はケーブルを抜く

    1. カーネルが出力するdmesgコマンド履歴をクリア
        ```
        $ sudo dmesg -c
        ```
    1. arduinoをUSBでubuntuに接続  
        仮想環境を使用している場合はホストPCから仮想環境へ接続を移す

    1. dmesgコマンド履歴を確認する  
        ```
        $ sudo dmesg
        ```
        arduinoデバイスをubuntuに接続したときのデバイス名は主に"ttyUSB*"，もしくは"ttyACM*" (*は数字)となる．

        ![dmeshスクショ](img/dmesg_new_serial_device.png)

        この時はESP32を接続するとデバイス名が"ttyUSB0"となった．  
        念のため,ハードウェアデバイスが登録されているディレクトリ /dev/~　にも確認したデバイス名が存在するかチェック
        ```
        $ ls /dev/ttyUSB* #ttyACM*の場合は /dev/ttyACM*
        ```

        ![devスクショ](img/ls_dev_tty.png)

        /dev/ttyUSB0が存在していることが確認できる.
    
    1. 一時的な許可属性の付与  
        arduinoデバイスの接続を解除するまで有効な許可属性の付与は以下のコマンドで行うことができる．
        ```
        $ sudo chmod 666 /dev/ttyUSB* # *部分は確認したデバイス名と一致させる
        ```
    
    1. arduinoデバイスを特定する情報を調べる  
        特定のarduinoデバイスを接続したとき，許可属性を付与するデバイスを自動的に認識させるために，デバイス固有の情報(ベンダーやシリアル番号等)を得る必要がある．
        ```
        $ udevadm info -q property -n /dev/ttyUSB0 | grep -E "ID_SERIAL_SHORT|ID_VENDOR_ID=|ID_MODEL_ID="
        ``` 

        ![udevadmスクショ](img/udevadm.png)

        表示された"ID_MODEL_ID"，"ID_SERIAL_SHORT"，"ID_VENDOR_ID"の値をメモしておく

    1. arduinoデバイスごとの許可属性を自動付与するためのudev設定ファイル(.rulesファイル)の作成
        ```
        $ cd /etc/udev/rules.d/
        $ sudo touch 99-serial.rules
        $ sudo gedit 99-serial.rules #vimのテキストエディタで開いてもOK
        ```
        テキストエディタが開かれるので，以下の情報を記述する．  
        ATTRS{idVendor}=“xxxx”の部分は先に控えた"ID_VENDOR_ID"，ATTRS{idProduct}==“xxxx”は"ID_MODEL_ID"，ATTRS{serial}==“xxx~”は"ID_SERIAL_SHORT"の値に置き換える．
        また，"name~"の部分は"設定したいデバイス名(例えばESP32_NO1)に置き換える.
        ```
        SUBSYSTEM==“tty”, ATTRS{idVendor}==“xxxx”, 
        ATTRS{idProduct}==“xxxx”, ATTRS{serial}==“xxx~”, 
        SYMLINK+=“name~”, MODE=“0666”
        ```
        複数のarduinoデバイスを同時に登録することも可能である．その場合はあらかじめそれぞれのデバイスの固有情報を調べておき，同じスクリプト内に上記のコマンドを複数列挙する．
        例えば，
        ```
        #ESP32 NO.1
        SUBSYSTEM==“tty”, ATTRS{idVendor}=“xxxx”, 
        ATTRS{idProduct}==“xxxx”, ATTRS{serial}==“xxx~”, 
        SYMLINK+=“name~”, MODE=“0666”

        #ESP32 NO.2
        SUBSYSTEM==“tty”, ATTRS{idVendor}=“yyyy”, 
        ATTRS{idProduct}==“yyyy”, ATTRS{serial}==“yyy~”, 
        SYMLINK+=“name~”, MODE=“0666”
        ```

    1. udev設定ファイルを反映
        ```
        $ sudo /etc/init.d/udev reload
        ```
        念のため再起動をおこなう．
        ```
        $ sudo reboot
        ```
        以上でarduinoデバイスの許可属性が自動で設定されるようになる．


- rosserialを使ってみる  
    rosserialはrosのパッケージであるため，実行前にroscoreを別のターミナルで立ち上げておく必要がある．  
    
    ターミナル①
    ```
    $ roscore
    ```

    rosserialの実行
    ターミナル②
    デバイス名は先にudev設定ファイルで設定したもの，_baud:=xxxのxxx部分はarduinoのスケッチ内で設定されているシリアルのボーレートと一致させる．
    ```
    $rosrun rosserial_python serial_node.py _port:=/dev/デバイス名 _baud:=xxx
    ```

    以上でarduinoのスケッチをrosの一つのノードとして扱うことができるようになる．
    上手く接続されているかを確認するにはrqt_graphを使う
    ```
    $ qrt_graph
    ```


