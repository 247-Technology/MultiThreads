#include <ros.h>
#include <std_msgs/Float32.h>
#include <Adafruit_NeoPixel.h>
#include <NewPing.h>
#include <std_msgs/UInt8.h>
#include <Thread.h>

ros::NodeHandle nh;

std_msgs::Float32 distance_msg;
ros::Publisher distance_pub("distance", &distance_msg);

#define TRIGGER_PIN_0 3
#define ECHO_PIN_0 4
#define TRIGGER_PIN_1 6
#define ECHO_PIN_1 5
#define TRIGGER_PIN_2 9
#define ECHO_PIN_2 10

#define NUM_LEDS 120

Adafruit_NeoPixel strip(NUM_LEDS, 13, NEO_GRB + NEO_KHZ800);

NewPing sonar[3] = {
  NewPing(TRIGGER_PIN_0, ECHO_PIN_0),
  NewPing(TRIGGER_PIN_1, ECHO_PIN_1),
  NewPing(TRIGGER_PIN_2, ECHO_PIN_2)
};

Thread sendDataThread = Thread();


volatile bool serialBusy = false;

void ledCallback(const std_msgs::UInt8& msg) {
  uint8_t result = msg.data;
  ColorPrint(result);
}

ros::Subscriber<std_msgs::UInt8> led_control_sub("led_control", &ledCallback);

void sendData() {
  while (true) {
    / if (!serialBusy) {
      serialBusy = true;
      float kk = Distance();
      distance_msg.data = kk;

      Serial.print("Distance: ");
      Serial.print(kk);
      Serial.println(" cm");
      distance_pub.publish(&distance_msg);
      serialBusy = false;
    }

    delay(1000);
  }
}

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(distance_pub);
  nh.subscribe(led_control_sub);
  strip.begin();
  sendDataThread.onRun(sendData);
  sendDataThread.setInterval(1000);
  sendDataThread.run();
}

void loop() {
  nh.spinOnce();
}

float Distance() {
  unsigned long duration;
  int distance247;
  digitalWrite(TRIGGER_PIN_0, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_0, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN_0, LOW);
  duration = pulseIn(ECHO_PIN_0, HIGH);
  distance247 = int(duration / 2 / 29.412);
  return distance247;
}

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
  delay(wait);
}

void ColorPrint(uint8_t color) {
  Serial.print("Received color: ");
  Serial.println(color);
  switch (color) {
    case 1:
      colorWipe(strip.Color(255, 0, 0), 1);
      Serial.print("Red ");
      break;
    case 2:
      colorWipe(strip.Color(0, 255, 0), 1);
      Serial.print("Green ");
      break;
    case 3:
      colorWipe(strip.Color(0, 0, 255), 1);
      Serial.print("Blue ");
      break;
    case 4:
      colorWipe(strip.Color(255, 0, 255), 1);
      Serial.print("Pink ");
      break;
    case 5:
      colorWipe(strip.Color(255, 255, 0), 1);
      Serial.print("Yellow ");
      break;
    case 6:
      colorWipe(strip.Color(255, 255, 255), 1);
      Serial.print("White ");
      break;
    case 7:
      rainbow1(1);
      Serial.print("Rainbow");
      break;
    case 8:
      rainbow2(1);
      Serial.print("Rainbow2");
      break;
    default:
      colorWipe(strip.Color(0, 0, 0), 1);
      Serial.print("TurnOff! ");
      break;
  }
}

void rainbow1(int wait) {
  int firstPixelHue = 0;
  for (int a = 0; a < 30; a++) {
    for (int b = 0; b < 3; b++) {
      strip.clear();
      for (int c = b; c < strip.numPixels(); c += 3) {
        int hue = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue));
        strip.setPixelColor(c, color);
      }
      strip.show();
      delay(wait);
      firstPixelHue += 65536 / 90;
    }
  }
}

void rainbow2(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 4096; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }
}
