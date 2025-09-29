long getDistanceCM() {
  long sum = 0;
  int count = 5;
  for (int i = 0; i < count; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long d = pulseIn(echoPin, HIGH, 30000);
    if (d == 0) continue; // Skip bad readings
    sum += d;
    delay(10); // Small delay between pings
  }
  if (sum == 0) return 0;
  return (sum / count) * 0.034 / 2;
}
void Ping() {
  if (direction == 'F') {
    distance = getDistanceCM();
    Serial.println(distance);

    if (distance != 0 && distance < 12) {
      Serial1.print("Crash! ");
      Serial1.println(distance);
      Serial.println("Too close! Stopping.");
      direction = 'S';
      setDirection(direction);
    }
  }
}
