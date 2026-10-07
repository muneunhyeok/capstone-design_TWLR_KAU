#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // BNO085 I2C 초기화
  if (!bno08x.begin_I2C()) {
    while (1) { delay(10); }
  }
  // 게임 회전 벡터(Game Rotation Vector) 리포트 활성화
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
}

void loop() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      // 쿼터니언 데이터(i, j, k, real)를 CSV 형태로 시리얼 출력
      Serial.print(sensorValue.un.gameRotationVector.i, 4);
      Serial.print(",");
      Serial.print(sensorValue.un.gameRotationVector.j, 4);
      Serial.print(",");
      Serial.print(sensorValue.un.gameRotationVector.k, 4);
      Serial.print(",");
      Serial.println(sensorValue.un.gameRotationVector.real, 4);
    }
  }
  delay(10);
}
