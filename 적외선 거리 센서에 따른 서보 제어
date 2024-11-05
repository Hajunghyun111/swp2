#include <Servo.h>

#define PIN_TRIG 12           // 초음파 센서 트리거 핀
#define PIN_ECHO 13           // 초음파 센서 에코 핀
#define PIN_SERVO 10          // 서보 핀
#define PIN_LED 9             // LED 핀
#define PIN_IR A0             // IR 센서 핀

#define SND_VEL 346.0         // 소리 속도 (m/s)
#define INTERVAL 20           // 루프 주기 (20ms 이상)
#define PULSE_DURATION 10     // 초음파 펄스 지속 시간 (usec)
#define TIMEOUT 30000         // 초음파 타임아웃 (usec)
#define SCALE (0.001 * 0.5 * SND_VEL)  // 거리 변환 계수

#define DIST_MIN 100.0        // 최소 거리 10cm
#define DIST_MAX 250.0        // 최대 거리 25cm
#define EMA_ALPHA 0.5         // EMA 필터 알파 값

Servo myservo;
float dist_ema = DIST_MAX;
unsigned long last_sampling_time = 0;

void setup() {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_TRIG, LOW);

    myservo.attach(PIN_SERVO);
    myservo.write(0);  // 서보 초기 위치 설정

    Serial.begin(1000000); // 시리얼 통신 속도 최대로 설정
}

void loop() {
    if (millis() - last_sampling_time < INTERVAL) return;
    last_sampling_time = millis();

    // IR 센서에서 거리 측정
    int a_value = analogRead(PIN_IR);
    float dist_raw_ir = (6762.0 / (a_value - 9) - 4.0) * 10.0 - 60.0;

    // 초음파 센서에서 거리 측정
    float dist_raw_sonar = measureSonarDistance();

    // 범위 필터 적용 (10cm ~ 25cm 범위)
    if (dist_raw_ir < DIST_MIN || dist_raw_ir > DIST_MAX) {
        dist_raw_ir = dist_ema;
        digitalWrite(PIN_LED, LOW); // 범위 밖이면 LED OFF
    } else {
        dist_ema = EMA_ALPHA * dist_raw_ir + (1 - EMA_ALPHA) * dist_ema;
        digitalWrite(PIN_LED, HIGH); // 범위 안에 있으면 LED ON
    }

    // 서보 각도 계산 (map() 함수 사용하지 않음)
    int servo_angle = (dist_ema - DIST_MIN) * (180.0 / (DIST_MAX - DIST_MIN));
    servo_angle = constrain(servo_angle, 0, 180);
    myservo.write(servo_angle);

    // 시리얼 플로터를 위한 출력
    Serial.print("IR Distance:"); Serial.print(dist_raw_ir);
    Serial.print(", Sonar Distance:"); Serial.print(dist_raw_sonar);
    Serial.print(", EMA Distance:"); Serial.print(dist_ema);
    Serial.print(", Servo Angle:"); Serial.println(servo_angle);
}

// 초음파 거리 측정 함수
float measureSonarDistance() {
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(PULSE_DURATION);
    digitalWrite(PIN_TRIG, LOW);
    float duration = pulseIn(PIN_ECHO, HIGH, TIMEOUT);
    return duration * SCALE; // 단위: mm
}
