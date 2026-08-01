/*
 * blink_toolchain_check
 * Local copy of the standard Arduino "Blink" example (01.Basics/Blink).
 * Used to prove laptop + cable + driver + COM port + compiler + Uno all work.
 * A blinking L LED proves the programming connection. It tests NO servo hardware.
 */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
