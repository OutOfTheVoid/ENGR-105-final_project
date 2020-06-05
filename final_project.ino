#include <LiquidCrystal.h>
#include "heart_rate_sensor.hpp"
#include "step_counter.hpp"

// what's a valid heart rate in terms of statistical variation?
// the closer to zero, the tighter the requirement.
// (this filters out irregular pulses)
#define HEART_RATE_VARIANCE_CUTOFF 0.03f

// decalre peripherals
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
HeartRateSensor hrs(A0);
StepCounter step;

// period of LCD update
#define LCD_UPDATE_MS 50
// time of last LCD update
int last_lcd_update;

void setup() {
  // setup serial plotting if used in sensor code
  //Serial.begin(112500);
  // setup peripherals
  lcd.begin(16, 2);
  hrs.reset();
  step.begin();
  // begin timing LCD updates
  last_lcd_update = millis();
  // set the heartbeat pin to OUTPUT
  pinMode(13, OUTPUT);
}

// remember heartbeat variance
float var = 0.0f;

void loop() {
  // update sensors
  hrs.update(Serial);
  step.update(Serial);
  // update heartbeat led, only on if variance is low enough
  digitalWrite(13, hrs.get_heartbeat_signal() & (var < HEART_RATE_VARIANCE_CUTOFF));
  
  // if we need to update the LCD
  if (millis() - last_lcd_update > LCD_UPDATE_MS) {
    last_lcd_update = millis();
    // get heart rate
    float hr;
    hrs.get_heartrate_and_variance(hr, var);
    // print it out
    lcd.setCursor(0, 0);
    lcd.print("HR: ");
    if (var < HEART_RATE_VARIANCE_CUTOFF) {
      lcd.print(hr);
    } else {
      lcd.print("...");
    }
    lcd.print("    ");
    
    // get step count
    int step_count = step.get_step_count();
    // print it out
    lcd.setCursor(0, 1);
    lcd.print("STEPS: ");
    lcd.print(step_count);
    lcd.print("    ");
  }
}
