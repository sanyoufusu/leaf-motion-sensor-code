#include <Wire.h>
#include <BLEDevice.h>


#define RESTRICT_PITCH //宏定义将滚动限制在±90度
#define SDA 21
#define SCL 22

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
//static BLEUUID    charUUID_1("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID    charUUID_2("81668039-ce56-4c2d-938a-6638fa9bf37c");
//static BLEUUID    charUUID_3("67131826-f527-4442-9022-dff78d5af492");
//static BLEUUID    charUUID_4("8f9903f1-4a01-4819-aa94-03de75cb821b");


static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
   // pClient->setMTU(517); //set client to request maximum MTU from server (default is  23otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID_2);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID_2.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

/*------------------------------Kalman--------------------------------------------------*/ 
class Kalman {
  public:
      Kalman();
  
      
      float getAngle(float newAngle, float newRate, float dt);// 角度应以度为单位，速率应以度/秒为单位，增量时间应以秒为单位
  
      void setAngle(float angle); // 用于设置角度，将其设置为起始角度
      float getRate(); // 返回无偏转速率
      /* 这些用于调整卡尔曼滤波器 */
      void setQangle(float Q_angle);
      /**
       * setQbias(float Q_bias)
       * Default value (0.003f) is in Kalman.cpp. 
       * Raise this to follow input more closely,
       * lower this to smooth result of kalman filter.
       */
      void setQbias(float Q_bias);
      void setRmeasure(float R_measure);
  
      float getQangle();
      float getQbias();
      float getRmeasure();
  
  private:
      /* 卡尔曼滤波器的变量 */
      float Q_angle; // 处理加速器的噪声
      float Q_bias; // 处理陀螺仪偏差噪声
      float R_measure; // 测量噪声方差
  
      float angle; // 卡尔曼滤波器计算的角度——2x1状态向量的一部分
      float bias; // 由卡尔曼滤波器计算的陀螺偏差——2x1状态向量的一部分
      float rate; // 根据速率和计算出的偏差计算的无偏速率-必须调用getAngle来更新速率
  
      float P[2][2]; // 误差协方差矩阵-2x2矩阵
};
/*-------------------------------------------------------------------------------------------*/
Kalman::Kalman() {
    /* 设置卡尔曼参数变量，这些变量也可以自由调整 */
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; // 因为我们假设偏差为0，并且我们知道起始角度（使用设定角度），所以误差协方差矩阵设置如下
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// 角度应以度为单位，速率应以度/秒为单位，增量时间应以秒为单位
float Kalman::getAngle(float newAngle, float newRate, float dt) {
    // 离散卡尔曼滤波时间更新方程-时间更新（“预测”）
    // 更新xhat-提前预测状态
    /* 第一步 */
    rate = newRate - bias;
    angle += dt * rate;

    // 更新估计误差协方差-提前投影误差协方差
    /* 第二步 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // 离散卡尔曼滤波测量更新方程-测量更新（“正确”）
    // 计算卡尔曼增益-计算卡尔曼增益
    /* 第四步 */
    float S = P[0][0] + R_measure; // 估计误差
    /* 第五步 */
    float K[2]; // 估计误差卡尔曼增益-这是一个2x1矢量
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 计算角度和偏差-使用测量zk（新角度）更新估计值
    /* 第三步 */
    float y = newAngle - angle; // 角度差
    /* 第六步 */
    angle += K[0] * y;
    bias += K[1] * y;

    // 计算估计误差协方差-更新误差协方差
    /* 第七步 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // 用于设置角度，将其设置为起始角度
float Kalman::getRate() { return this->rate; }; // 返回无偏差速率

/* 调整卡尔曼滤波器 */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };



/* 实例化*/
Kalman kalmanX; // 创建Kalman实例
Kalman kalmanY;

/* 定义IMU数据 */
double accX, accY, accZ;//加速度
double gyroX, gyroY, gyroZ;//陀螺仪（角度）
//int16_t tempRaw;//温度

double gyroXangle, gyroYangle; // 仅使用陀螺仪计算角度
double compAngleX, compAngleY; // 使用互补滤波器计算角度
double kalAngleX, kalAngleY; // 使用卡尔曼滤波器计算角度

uint32_t timer;//定义计时器
uint8_t i2cData[14]; //定义I2C数据的缓冲区

//TODO:制定校准程序

void setup() {
  Serial.begin(115200);//设置串口波特率
  Wire.begin(SDA,SCL);//I2C引脚准备
#if ARDUINO >= 157
  Wire.setClock(400000UL); //将I2C频率设置为400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; //将I2C频率设置为400kHz
#endif

  i2cData[0] = 7; //将采样率设置为1000Hz-8kHz/（7+1）=1000Hz
  i2cData[1] = 0x00; //禁用FSYNC并设置260 Hz Acc滤波、256 Hz陀螺滤波、8 KHz采样
  i2cData[2] = 0x00; //将陀螺满标度范围设置为±250度/秒
  i2cData[3] = 0x00; //将加速计满刻度范围设置为±2g
  while (i2cWrite(0x19, i2cData, 4, false)); //同时写入所有四个寄存器
  while (i2cWrite(0x6B, 0x01, true)); //带X轴陀螺仪参考和禁用睡眠模式的PLL

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // 读取"WHO_AM_I"寄存器
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); //等待传感器稳定下来

  /* 设置卡尔曼和陀螺仪的起始角 */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  //atan2输出-π到π（弧度）的值 
  //然后将其从弧度转换为度
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); //设置起始角度
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop() {
  /* 更新所有值 */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  //tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; //计算时间增量
  timer = micros();

  
  //atan2输出-π到π（弧度）
  //然后将其从弧度转换为度
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // 转换成°/s
  double gyroYrate = gyroY / 131.0; // 转换成°/s

#ifdef RESTRICT_PITCH

  //解决加速度计角度在-180和180度之间跳跃时的过渡问题
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); //使用卡尔曼滤波器计算角度

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; //反转速率，因此符合加速度计读数的限制
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  //这解决了加速度计角度在-180和180度之间跳跃时的过渡问题
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); //用卡尔曼滤波器计算角度

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; //反转速率，因此符合加速度计读数的限制
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // 用卡尔曼滤波器计算角度
#endif

  gyroXangle += gyroXrate * dt; // 无需任何滤波器即可计算陀螺角度
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; //使用无偏速率计算陀螺角度
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // 使用互补过滤器计算角度
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  //当陀螺仪角度漂移过大时，重置陀螺仪角度
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* 串口打印数据 */
if (doConnect == true) {
    if (connectToServer()) {
      //Serial.println("We are now connected to the BLE Server.");
    } else {
      //Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String newValue =/*String("1321P:") + String(-kalAngleY)+String("R:") +*/String(kalAngleX)+String("\t");
    Serial.println("new value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1500); // Delay a second between loops.


}
