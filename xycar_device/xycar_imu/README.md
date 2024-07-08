# xycar_imu

arduino 0.12.1 : https://github.com/arduino/arduino-cli/releases/tag/0.12.1



1. 다음 명령어를 입력하여 udev 설정

   $ sudo echo "ACTION==\"add\", ATTRS{idVendor}==\"1b4f\", ATTRS{idProduct}==\"9d0f\", MODE:=\"0666\", SYMLINK+=\"ttyIMU\"" > /etc/udev/rules.d/11-xycar-imu.rules



2. 아두이노 0.12.1 버전을 다운 받은 뒤 다음 명령을 차례로 실행

   $ arduino-cli config init --additional-urls https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json

   $ arduino-cli core update-index

   $ arduino-cli core install arduino:samd@1.8.1

   $ tar xvf xycar_imu/firmware/SparkFunMPU9250-DMP.tgz

   $ mkdir -p $HOME/Arduino/libraries

   $ mv xycar_imu/firmware/SparkFunMPU9250-DMP/ $HOME/Arduino/libraries

   $ arduino-cli lib install --zip-path {SparkFunMPU9250-DMP.zip 경로}



3. 그리고 ~/.arduino15/package_sparkfun_index.json 파일을 연 뒤 "version": "1.8.3" 을 검색
   키값이 "toolsDependencies" 인 리스트를 다음과 같이 수정

```
     {
       "packager": "arduino",
       "name": "arm-none-eabi-gcc",
       "version": "7-2017q4"
     },
     {
       "packager": "arduino",
       "name": "bossac",
       "version": "1.7.0-arduino3"
     },
     {
       "packager": "arduino",
       "name": "openocd",
       "version": "0.10.0-arduino7"
     },
     {
       "packager": "arduino",
       "name": "CMSIS",
       "version": "4.5.0"
     },
     {
       "packager": "arduino",
       "name": "CMSIS-Atmel",
       "version": "1.2.0"
     }
```
(버전만 수정하면 됩니다.)



4. 그 후 아래 명령어를 실행 

   $ arduino-cli core install SparkFun:samd@1.8.3

   $ sed -i 's/version=1.6.0/version=1.8.3/g' $HOME/.arduino15/packages/SparkFun/hardware/samd/1.8.3/platform.txt

   $ PORT=\`readlink /dev/ttyIMU`

   $ arduino-cli compile -p /dev/$PORT -u -b SparkFun:samd:samd21_9dof {IMU 소스 디렉토리}

   

5. 소스 없이 디렉토리만 가지고 빌드 하려면 다음 명령어를 차례로 실행

   $ PORT=\`readlink /dev/ttyIMU`

   $ arduino-cli upload -p /dev/$PORT -b SparkFun:samd:samd21_9dof {IMU 소스 디렉토리}