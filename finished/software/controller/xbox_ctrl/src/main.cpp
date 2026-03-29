#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Preferences.h>
// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址


int led1 = 4;
Preferences preferences;

void saveBluetoothAddress(const String& addr) {
    preferences.begin("xbox", false);
    preferences.putString("bt_addr", addr);
    preferences.end();
}

String loadBluetoothAddress() {
    preferences.begin("xbox", true);
    String addr = preferences.getString("bt_addr", "");
    preferences.end();
    return addr;
}
XboxSeriesXControllerESP32_asukiaaa::Core* xboxController = nullptr;
  
String xbox_string()
{
  String str = String(xboxController->xboxNotif.btnY) + "," +
               String(xboxController->xboxNotif.btnX) + "," +
               String(xboxController->xboxNotif.btnB) + "," +
               String(xboxController->xboxNotif.btnA) + "," +
               String(xboxController->xboxNotif.btnLB) + "," +
               String(xboxController->xboxNotif.btnRB) + "," +
               String(xboxController->xboxNotif.btnSelect) + "," +
               String(xboxController->xboxNotif.btnStart) + "," +
               String(xboxController->xboxNotif.btnXbox) + "," +
               String(xboxController->xboxNotif.btnShare) + "," +
               String(xboxController->xboxNotif.btnLS) + "," +
               String(xboxController->xboxNotif.btnRS) + "," +
               String(xboxController->xboxNotif.btnDirUp) + "," +
               String(xboxController->xboxNotif.btnDirRight) + "," +
               String(xboxController->xboxNotif.btnDirDown) + "," +
               String(xboxController->xboxNotif.btnDirLeft) + "," +
               String(xboxController->xboxNotif.joyLHori) + "," +
               String(xboxController->xboxNotif.joyLVert) + "," +
               String(xboxController->xboxNotif.joyRHori) + "," +
               String(xboxController->xboxNotif.joyRVert) + "," +
               String(xboxController->xboxNotif.trigLT) + "," +
               String(xboxController->xboxNotif.trigRT) + "\n";
  return str;
}

/*
支持四种振动模式
left：上左电机动
right：上右电机动
center：下左电机和下右电机一起动，频率高力量小
shake：下左电机和下右电机一起动，频率低力量大

测试结果：
四种模式都可以调振动力度
下左电机和下右电机是绑定的，只能一起动，但是提供了两种振动模式，个人猜测是两种模式的原理是给电机不同的电压
可以随意搭配使用，但center和shake一起用的话执行的应该是shake
*/

// 配置参考
// repo.v.select.center = 0;
// repo.v.select.left = 0;
// repo.v.select.right = 0;
// repo.v.select.shake = 0;
// repo.v.power.center = 0; // x% power
// repo.v.power.left = 0;
// repo.v.power.right = 30;
// repo.v.power.shake = 0;
// repo.v.timeActive = 0; // 振动 x/100 秒，最大2.56秒(uint8_t)
// repo.v.timeSilent = 0;   // 静止 x/100 秒
// repo.v.countRepeat = 0;  // 循环次数 x+1

// 官方例程
void demoVibration()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  Serial.println("full power for 1 sec");
  repo.setAllOff();
  xboxController->writeHIDReport(repo);
  delay(2000);
  repo.setFullPowerFor1Sec();
  repo.v.timeActive = 50; 
   xboxController->writeHIDReport(repo);
  delay(2000); 
  // repo.v.select.center = true;
  // repo.v.select.left = false;
  // repo.v.select.right = false;
  // repo.v.select.shake = false;
  // repo.v.power.center = 30; // 30% power
  // repo.v.timeActive = 50;   // 0.5 second
  // Serial.println("run center 30\% power in half second");
  // xboxController.writeHIDReport(repo);
  // delay(2000);

  // repo.v.select.center = false;
  // repo.v.select.left = true;
  // repo.v.power.left = 30;
  // Serial.println("run left 30\% power in half second");
  // xboxController.writeHIDReport(repo);
  // delay(2000);

  // repo.v.select.left = false;
  // repo.v.select.right = true;
  // repo.v.power.right = 30;
  // Serial.println("run right 30\% power in half second");
  // xboxController.writeHIDReport(repo);
  // delay(2000);

  // repo.v.select.right = false;
  // repo.v.select.shake = true;
  // repo.v.power.shake = 30;
  // Serial.println("run shake 30\% power in half second");
  // xboxController.writeHIDReport(repo);
  // delay(2000);

  // repo.v.select.shake = false;
  // repo.v.select.center = true;
  // repo.v.power.center = 50;
  // repo.v.timeActive = 20;
  // repo.v.timeSilent = 20;
  // repo.v.countRepeat = 2;
  // Serial.println("run center 50\% power in 0.2 sec 3 times");
  // xboxController.writeHIDReport(repo);
  // delay(2000);
}

// 振动反馈，根据扳机按压力度调整振动力度
void demoVibration_2()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  static uint16_t TrigMax = XboxControllerNotificationParser::maxTrig;
  String str_1;
  repo.setAllOff();
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.power.left = (uint8_t)((float)xboxController->xboxNotif.trigLT / (float)TrigMax * 100);
  repo.v.power.right = (uint8_t)((float)xboxController->xboxNotif.trigRT / (float)TrigMax * 100);
  repo.v.timeActive = 50;
  xboxController->writeHIDReport(repo);
  str_1 = String(repo.v.power.left) + "," + String(repo.v.power.right) + "\n";

  Serial.print(str_1);
  delay(50);
}
//盖世小鸡
void demoVibration_3()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  static uint16_t TrigMax = XboxControllerNotificationParser::maxTrig;
  // String str_1="aa";
  repo.setAllOff();
  repo.v.select.center = true;
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.select.shake = true;
  // repo.v.power.center = (uint8_t)((float)xboxController.xboxNotif.trigLT / (float)TrigMax * 100);
  // repo.v.power.shake = (uint8_t)((float)xboxController.xboxNotif.trigRT / (float)TrigMax * 100);
  repo.v.power.center = 60;
  repo.v.power.shake = 60;
  repo.v.timeActive = 10;
  xboxController->writeHIDReport(repo);
  // str_1 = String(repo.v.power.center) + "," + String(repo.v.power.shake) + "\n";
  delay(500);
  repo.v.power.center = 0;
  repo.v.power.shake = 0;
  xboxController->writeHIDReport(repo);
  // Serial.print(str_1);
  // delay(10);
}

int i=0,cnt=0;
int connnect_flag=0;

void resetBluetoothAddressAndRestart() {
    saveBluetoothAddress(""); // 保存空地址
    Serial.println("Bluetooth address cleared, restarting...");
    delay(500);
    ESP.restart(); // 重启后会用空地址，自动进入全扫描模式
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1, /*rx =*/1, /*Tx =*/0);
  // Serial.println("Starting NimBLE Client");
  pinMode(led1, OUTPUT);
  String btAddr = loadBluetoothAddress();
  xboxController = new XboxSeriesXControllerESP32_asukiaaa::Core(btAddr);
  xboxController->begin();
  digitalWrite(led1, LOW);
}

void loop()
{
  if (Serial.available()) {
    String addr = Serial.readStringUntil('\n');
    addr.trim();
    if (addr.equalsIgnoreCase("set")) {
        resetBluetoothAddressAndRestart();
    } else if (addr.equalsIgnoreCase("get")) {
        String curAddr = loadBluetoothAddress();
        if (curAddr.length() == 17) {
            Serial.println("Current Bluetooth address: " + curAddr);
        } else {
            Serial.println("No address saved (scan all devices).");
        }
    } else if (addr.length() == 17) {
        saveBluetoothAddress(addr);
        Serial.println("Address saved: " + addr);
        delay(500);
        ESP.restart();
    } else {
        Serial.println("Invalid address format!");
    }
  }

  if (xboxController) xboxController->onLoop();
  if (xboxController && xboxController->isConnected())
  {
    if (xboxController->isWaitingForFirstNotification())
    {
      Serial.println("waiting for first notification");
    }
    else
    {
      if(connnect_flag==0)
      {
        connnect_flag=1;
        demoVibration_3();
      }
      Serial.print(xbox_string());
      Serial1.write(xboxController->xboxNotif.rx_buf, xboxController->xboxNotif.expectedDataLen);
      if(cnt++>5)
      {
        cnt=0;
        i=~i;
        digitalWrite(led1, i);
      }
      delay(10);
    }
  }
  else
  {
    connnect_flag=0;
    memset(xboxController->xboxNotif.rx_buf, 0, sizeof(xboxController->xboxNotif.rx_buf));
    Serial1.write(xboxController->xboxNotif.rx_buf, xboxController->xboxNotif.expectedDataLen);
    delay(10);
    if(cnt++>100)
    {
      cnt=0;
      i=~i;
      digitalWrite(led1, i);
    }
    if (xboxController && xboxController->getCountFailedConnection() > 2)
    {
      ESP.restart();
    }
  }
}
