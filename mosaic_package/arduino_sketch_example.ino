/*
 * Arduino Rotary Sensor Reader
 * This sketch reads a rotary encoder/potentiometer and sends angle data via serial.
 * Upload this to your Arduino to work with the ROS2 arduino_reader node.
 */

// Pin configuration - adjust based on your setup
const int ROTARY_PIN = A0;  // Analog pin for potentiometer/rotary sensor

// Calibration values - adjust based on your sensor
const float MIN_ANGLE = 0.0;    // Minimum angle in degrees
const float MAX_ANGLE = 360.0;  // Maximum angle in degrees

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);

  // Configure analog pin
  pinMode(ROTARY_PIN, INPUT);

  // Wait for serial connection
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println("Arduino Rotary Sensor Ready!");
}

void loop() {
  // Read analog value (0-1023 for 10-bit ADC)
  int rawValue = analogRead(ROTARY_PIN);

  // Convert to angle (map 0-1023 to MIN_ANGLE-MAX_ANGLE)
  float angle = map(rawValue, 0, 1023, MIN_ANGLE * 100, MAX_ANGLE * 100) / 100.0;

  // Send angle to serial port
  // Format: just the number, e.g., "45.5"
  Serial.println(angle);

  // Small delay for stability (50ms = 20Hz update rate)
  delay(50);
}
