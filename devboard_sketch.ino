#include <Arduino.h>
#include "AD5593R.h"
#include <Wire.h>
#include "MAX86916_eda.h"
#include <math.h>   // for sqrt, fabs

#define m1_na A0      // Module 1 Network Array
#define m1_vd A1      // Module 1 Voltage Divider
#define m2_diode A2   // Module 2 Diode
#define m2_nmos A3    // Module 2 NMOS
#define m3_pd A6      // Module 3 Photo Diode
#define module_selector A7  // Module selector

#define dac_diode 0         // DAC diode
#define dac_nmos_gate 1     // NMOS Gate
#define dac_nmos_drain 2    // NMOS Drain
#define dac_pd_pair1 3      // Pair 1 photo diode LED
#define dac_pd_pair2 4      // Pair 2 photo diode LED

#define dac_0v 0        // DAC value to set 0V
#define dac_1p8v 1150   // DAC value to set 1.8V
#define dac_3p3v 2000   // DAC value to sett 3.3V

#define adc_conversion 0.00322
#define dac_conversion 0.00165

#define PERIOD 3
#define THRESHOLD 200

#define ref_voltage 3.3

MAX86916_eda ppg;

// stores sample values
const int MAX_SAMPLES = 3000;
float stored_ac[MAX_SAMPLES];
unsigned long stored_time[MAX_SAMPLES];
int stored_sample_count = 0;

unsigned long stored_beats[200];
int stored_beat_count = 0;

float stored_mean_bpm = 0;

String stored_exercise_type = "";
bool sample_available = false;


enum Command {
  M1_NETWORK_ARRAY,
  M1_VOLTAGE_DIVIDER,
  M2_DIODE_MODULE,
  M2_DIODE_RAW,
  M2_TRAN_MODULE_VDS,
  M2_TRAN_MODULE_VGS,
  M2_TRAN_RAW,
  M3_LED_FLASH,
  M3_PPG,
  M3_COLLECT_CURRENT,
  M3_STEP_LED,
  LIFI_TX,
  LIFI_RX,
  EXERCISE_CLASSIFY,   // NEW: exercise type classification from PPG
  SAMPLE_DATA, // NEW: lets you see the sample of the collected data after
  EXIT_COMMAND
};

void dac_setup();
void reset_dac();
Command mapStringToCommand(String command);
bool exit_condition();
void m1_network_array();
void m1_voltagedivider();
void m2_diode_module();
void m2_diode_module_raw();
void m2_tran_module_vds();
void m2_tran_module_vgs();
void m2_tran_module_raw();
void m3_led_flash();
void m3_collect_current();
void m3_step_led_brightness();
void m3_ppg();
bool read_pd();
void lifi_tx();
void send_byte(char my_byte);
void lifi_rx();
char get_byte();
void print_byte(char my_byte);
void detect_exercise_type();   // NEW function prototype
void print_sample_data();   // NEW sample data

void setup() {
  Wire.begin();
  Serial.begin(115200);
  dac_setup();
  reset_dac();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read the incoming command

    Command cmd = mapStringToCommand(command);

    // Switch-case structure to handle different commands
    switch (cmd) {
      case M1_NETWORK_ARRAY:
        m1_network_array();
        break;
      case M1_VOLTAGE_DIVIDER:
        m1_voltagedivider();
        break;
      case M2_DIODE_MODULE:
        m2_diode_module();
        break;
      case M2_DIODE_RAW:
        m2_diode_module_raw();
        break;
      case M2_TRAN_MODULE_VDS:
        m2_tran_module_vds();
        break;
      case M2_TRAN_MODULE_VGS:
        m2_tran_module_vgs();
        break;
      case M2_TRAN_RAW:
        m2_tran_module_raw();
        break;
      case M3_LED_FLASH:
        m3_led_flash();
        break;
      case M3_PPG:
        m3_ppg();
        break;
      case M3_COLLECT_CURRENT:
        m3_collect_current();
        break;
      case M3_STEP_LED:
        m3_step_led_brightness();
        break;
      case LIFI_TX:
        lifi_tx();
        break;
      case LIFI_RX:
        lifi_rx();
        break;
      case EXERCISE_CLASSIFY:      // NEW
        detect_exercise_type();
        break;
      case SAMPLE_DATA:
        print_sample_data();
        break;
      case EXIT_COMMAND:
        break;
      default:
        Serial.println("Unknown command");
        break;
    }
  }
}

Command mapStringToCommand(String command) {
  if (command == "m1na") return M1_NETWORK_ARRAY;
  if (command == "m1vd") return M1_VOLTAGE_DIVIDER;
  if (command == "m2d") return M2_DIODE_MODULE;
  if (command == "m2draw") return M2_DIODE_RAW;
  if (command == "m2tran_vgs") return M2_TRAN_MODULE_VGS;
  if (command == "m2tran_vds") return M2_TRAN_MODULE_VDS;
  if (command == "m2tranraw") return M2_TRAN_RAW;
  if (command == "m3led") return M3_LED_FLASH;
  if (command == "m3ppg") return M3_PPG;
  if (command == "m3pdcurrent") return M3_COLLECT_CURRENT;
  if (command == "m3stepled") return M3_STEP_LED;
  if (command == "lifi_tx") return LIFI_TX;
  if (command == "lifi_rx") return LIFI_RX;
  if (command == "exercise") return EXERCISE_CLASSIFY;   // NEW command string
  if (command == "sample") return SAMPLE_DATA; // NEW returns the samples
  if (command == "exit" || command == "e") return EXIT_COMMAND;
}

bool exit_condition(){
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command == "exit" || command == "e") {
      reset_dac();
      Serial.println("Ready to enter next command");
      return true;
    }
  }
  return false;
}

void dac_setup(){
  initADAC(0x10, 1, 1);
  setDACGain(1);
  setDACpin(dac_diode);
  setDACpin(dac_nmos_gate);
  setDACpin(dac_nmos_drain);
  setDACpin(dac_pd_pair1);
  setDACpin(dac_pd_pair2);
}

void reset_dac(){
  setDACVal(dac_diode,0);
  setDACVal(dac_nmos_gate,0);
  setDACVal(dac_nmos_drain,0);
  setDACVal(dac_pd_pair1,0);
  setDACVal(dac_pd_pair2,0);
}

void m1_network_array(){
  Serial.println("Make sure that module 1 is active");
  while(1){
    int analog_value = analogRead(m1_na);
    if(analog_value > 1000 && analog_value < 1020) Serial.println("S1 (3.3V)");
    else if (analog_value > 590 && analog_value < 615)Serial.println("S2 (1.95V)");
    else if (analog_value > 360 && analog_value < 375)Serial.println("S3 (1.18V)");
    else if (analog_value > 755 && analog_value < 775)Serial.println("S4 (2.50V)");
    else if (analog_value > 500 && analog_value < 520)Serial.println("S5 (1.65V)");
    else if (analog_value > 240 && analog_value < 260)Serial.println("S6 (0.804V)");
    else if (analog_value > 635 && analog_value < 660)Serial.println("S7 (2.12V)");
    else if (analog_value > 400 && analog_value < 425)Serial.println("S8 (1.35V)");
    else if (analog_value > 0 && analog_value < 200)Serial.println("S9 (0V)");

    if(exit_condition()) break;

    delay(20);
  }
}

void m1_voltagedivider(){
  Serial.println("Make sure that module 1 is active");
  while(1){
    Serial.print((analogRead(m1_vd)/1023.0)*ref_voltage);
    Serial.println("V");
    if(exit_condition()) break;
  }
}

void m2_diode_module(){
  Serial.println("Make sure that module 2 is active");
  int inc_interval = 10;
  int inc_delay = 1;
  int inc = 0;
  unsigned long prev_inc_time = 0;

  Serial.println("Diode current in (mA), Voltage applied across diode in (V)");
  while(1){
    if((inc <= dac_3p3v) && (millis() - prev_inc_time > inc_delay)){
      prev_inc_time = millis();
      setDACVal(dac_diode, inc);
      float resistor_voltage = (analogRead(m2_diode)*adc_conversion);
      Serial.print(resistor_voltage);
      Serial.print(",");
      float diode_voltage = inc*0.00165;
      Serial.print(diode_voltage);
      Serial.print(",");
      Serial.println(diode_voltage-resistor_voltage);
      inc = inc + inc_interval;
    }
    if((inc >= dac_3p3v) && (millis() - prev_inc_time > inc_delay)){
       prev_inc_time = millis();
       setDACVal(dac_diode, 0);
       inc = 0;
       Serial.println("Ready to enter next command");
       break;
    }
  }
}

void m2_diode_module_raw(){
  Serial.println("Make sure that module 2 is active");
  int inc_interval = 10;
  int inc_delay = 1;
  int inc = 0;
  unsigned long prev_inc_time = 0;

  Serial.println("Current in mA");
  while(1){
    if((inc <= dac_3p3v) && (millis() - prev_inc_time > inc_delay)){
      prev_inc_time = millis();
      setDACVal(dac_diode, inc);
      Serial.println(analogRead(m2_diode)*adc_conversion);
      inc = inc + inc_interval;
    }
    if((inc >= dac_3p3v) && (millis() - prev_inc_time > inc_delay)){
       prev_inc_time = millis();
       setDACVal(dac_diode, 0);
       inc = 0;
       Serial.println("Ready to enter next command");
       break;
    }
  }
}

void m2_tran_module_vds(){
  Serial.println("Make sure that module 2 is active");
  Serial.println("Vgs | Vds | Ic");
  while(1){
    for(int k = 0; k <= 2000; k = k + 200){
      setDACVal(dac_nmos_gate, k);
      for(int i = 0; i <= 2000; i = i + 10){
        setDACVal(dac_nmos_drain, i);
        Serial.print(k*dac_conversion);
        Serial.print(", ");
        Serial.print(i*dac_conversion);
        Serial.print(", ");
        Serial.println(analogRead(m2_nmos)*adc_conversion);
        delay(5);
      }
    }
    Serial.println("Ready to enter next command");
    break;
  }
}

void m2_tran_module_vgs(){
  Serial.println("Make sure that module 2 is active");
  Serial.println("Vds | Vgs | Ic");
  while(1){
    for(int k = 0; k <= 2000; k = k + 200){
      setDACVal(dac_nmos_drain, k);
      for(int i = 0; i <= 2000; i = i + 10){
        setDACVal(dac_nmos_gate, i);
        Serial.print(k*dac_conversion);
        Serial.print(", ");
        Serial.print(i*dac_conversion);
        Serial.print(", ");
        Serial.println(analogRead(m2_nmos)*adc_conversion);
        delay(5);
      }
    }
    Serial.println("Ready to enter next command");
    break;
  }
}

void m2_tran_module_raw(){
  Serial.println("Make sure that module 2 is active");
  while(1){
    for(int k = 0; k < 2000; k = k + 200){
      setDACVal(dac_nmos_gate, k);
      Serial.print("Gate voltage Vgs = ");
      Serial.print(k*dac_conversion);
      Serial.println(" V");
      for(int i = 0; i < 2000; i = i + 10){
        setDACVal(dac_nmos_drain, i);
        Serial.println(analogRead(m2_nmos)*adc_conversion);
        delay(5);
      }
    }
    Serial.println("Ready to enter next command");
    break;
  }
}

void m3_led_flash(){
  Serial.println("Make sure that module 3 is active");
  int time_period = 100;
  int previous_state = 0;

  unsigned long previous_toggle_time = 0;

  while(1){
    Serial.println(analogRead(m3_pd));
    if(millis() - previous_toggle_time >= time_period/2){
      previous_toggle_time = millis();
      if(previous_state == 0){
        setDACVal(dac_pd_pair1, 2250);
        previous_state = 1;
      }
      else{
        setDACVal(dac_pd_pair1, 0);
        previous_state = 0;
      }
    }
    if(exit_condition()) break;
  }
}

void m3_collect_current(){
  int count = 0;
  int incrementer = 10;
  int analog_value = 0;
  setDACVal(dac_pd_pair1, 2250);
  while(count < 3000){
    analog_value = analogRead(m3_pd)*adc_conversion*1000;
    Serial.println(analog_value/3);
    count = count + incrementer;
    delay(incrementer);
  }
  setDACVal(dac_pd_pair1, 0);
  Serial.println("Ready to enter next command");
}

void m3_step_led_brightness(){
  int led_brightness = 1650;
  int max_led_brightness = 2000;
  int brightness_incrementer = 50;

  int increment_counter = 0;
  int pd_current = 0;
  while(1){
    Serial.print(led_brightness*dac_conversion);
    Serial.print(",");
    pd_current = analogRead(m3_pd)*adc_conversion*1000;
    Serial.println(pd_current/3);
    if(led_brightness <= max_led_brightness){
      increment_counter = increment_counter + 1;
      if(increment_counter == brightness_incrementer){
        setDACVal(dac_pd_pair1, led_brightness);
        led_brightness = led_brightness + brightness_incrementer;
        increment_counter = 0;
      }    
    }
    else break;
    if(exit_condition()) break;
    delay(10);
  }
  setDACVal(dac_pd_pair1, 0);
  Serial.println("Ready to enter next command");
}

void m3_ppg(){
  int16_t blue;
  int16_t green;
  int16_t IR;
  int16_t red;
  int16_t IR_blue;
  int16_t IR_red;
  int16_t IR_green;

  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0x7F);
  ppg.setPulseAmplitudeIR(0x7F);
  ppg.setPulseAmplitudeGreen(0x7F);
  ppg.setPulseAmplitudeBlue(0x7F);

  while(1){
    Serial.print(ppg.getIR());  // IR
    Serial.print(",");
    Serial.println(ppg.getRed()); // Red
    if(exit_condition()) break;
  }
}

bool read_pd(){
  int pd_value = analogRead(m3_pd);
  return pd_value > THRESHOLD ? true : false;
}

void lifi_tx(){
  char* string = (char*)"This is a test transmission! ";
  int string_length;
  string_length = strlen(string);
  while(1){
    for(int i = 0; i < string_length; i ++)
    {
      send_byte(string[i]);
    }
    delay(1000);
    if(exit_condition()) break;
  }
}

void send_byte(char my_byte){
  setDACVal(dac_pd_pair2, 0);
  delay(PERIOD);

  // transmission of bits
  for(int i = 0; i < 8; i++)
  {
    if((my_byte&(0x01 << i))!=0) setDACVal(dac_pd_pair2, 2250);
    else setDACVal(dac_pd_pair2, 0);
    delay(PERIOD);
  }

  setDACVal(dac_pd_pair2, 2250);
  delay(PERIOD);
}

void lifi_rx(){
  Serial.println("Make sure that module 3 is active");
  bool previous_state;
  bool current_state;
  while(1){
    current_state = read_pd();
    if(!current_state && previous_state)
    {
      print_byte(get_byte());
    }
    previous_state = current_state;
    if(exit_condition()) break;
  }
}

char get_byte(){
  char ret = 0;
  delay(PERIOD*1.5);
  for(int i = 0; i < 8; i++)
  {
    ret = ret | (read_pd() << i);
    delay(PERIOD);
  }
  return ret;
}

void print_byte(char my_byte) {
  // simplest version: just print the character
  Serial.print(my_byte);
}

void detect_exercise_type() {
  Serial.println("Make sure Module 3 is active and the PPG sensor is on your finger.");
  Serial.println("Try to keep your finger steady. After completing a set of your exercise (i.e., running, weightlifting), put your finger on the sensor for ~30 seconds.");
  Serial.println("Collecting PPG data for ~30 seconds...");

  // Initialize PPG like in m3_ppg()
  ppg.begin();
  ppg.setup();
  ppg.setPulseAmplitudeRed(0x7F);
  ppg.setPulseAmplitudeIR(0x7F);
  ppg.setPulseAmplitudeGreen(0x7F);
  ppg.setPulseAmplitudeBlue(0x7F);

  const unsigned long MEASUREMENT_TIME_MS   = 30000UL;  // ~30 seconds
  const unsigned long MIN_BEAT_INTERVAL_MS  = 250UL;    // ~240 BPM max guard
  const unsigned long SAMPLE_PERIOD_MS      = 12UL;     // ~80 Hz sampling
  const int           MAX_BEATS             = 200;      // generous capacity

  unsigned long beat_timestamps[MAX_BEATS];
  int beat_count = 0;

  // DC removal / high-pass estimate
  float dc_estimate = 0.0f;
  const float alpha = 0.95f;

  // For simple peak detection
  float ac_prev2 = 0.0f;
  float ac_prev1 = 0.0f;
  float ac_cur   = 0.0f;

  unsigned long last_beat_time   = 0;
  unsigned long start_time       = millis();
  unsigned long last_sample_time = millis();

  // Reset storage before new measures
  stored_sample_count = 0;
  stored_beat_count   = 0;
  sample_available    = false;

  // Prime with 3 samples
  for (int i = 0; i < 3; i++) {
    int32_t ir = ppg.getIR();
    dc_estimate = dc_estimate + alpha * (ir - dc_estimate);
    float ac = (float)ir - dc_estimate;

    if (i == 0)      ac_prev2 = ac;
    else if (i == 1) ac_prev1 = ac;
    else             ac_cur   = ac;

    delay(SAMPLE_PERIOD_MS);
  }

  // AC threshold: small value, but can be tuned
  const float AC_THRESHOLD = 10.0f;

  // main sample loop
  while (millis() - start_time < MEASUREMENT_TIME_MS) {
    if (exit_condition()) {
      Serial.println("Measurement aborted by user.");
      return;
    }

    // Wait until next sample time
    if (millis() - last_sample_time < SAMPLE_PERIOD_MS) {
      continue;
    }
    last_sample_time = millis();

    // shift window
    ac_prev2 = ac_prev1;
    ac_prev1 = ac_cur;

    // new sample
    int32_t ir = ppg.getIR();
    dc_estimate = dc_estimate + alpha * (ir - dc_estimate);
    ac_cur = (float)ir - dc_estimate;

    unsigned long now = millis();

    // store each sample in globals
    if (stored_sample_count < MAX_SAMPLES) {
      stored_ac[stored_sample_count]   = ac_cur;           // AC value
      stored_time[stored_sample_count] = now - start_time; // ms from start
      stored_sample_count++;
    }

    // detect local peak at ac_prev1
    bool is_peak = (ac_prev1 > ac_prev2) && (ac_prev1 >= ac_cur) && (ac_prev1 > AC_THRESHOLD);

    if (is_peak && (now - last_beat_time) > MIN_BEAT_INTERVAL_MS) {
      if (beat_count < MAX_BEATS) {
        beat_timestamps[beat_count] = now;
        beat_count++;
      }

      // record beat time into global array
      if (stored_beat_count < 200) {
        stored_beats[stored_beat_count] = now - start_time; // ms from start
        stored_beat_count++;
      }

      last_beat_time = now;
    }
  } // end sampling

  if (beat_count < 3) {
    Serial.print("Not enough beats detected. Beats counted: ");
    Serial.println(beat_count);
    Serial.println("Try keeping still, ensure good contact, or increase measurement time and run 'exercise' again.");
    return;
  }

  // Compute BPM from beat intervals
  float sum_bpm    = 0.0f;
  float sum_bpm_sq = 0.0f;
  int   interval_count = beat_count - 1;

  unsigned long earliest = beat_timestamps[0];
  unsigned long latest   = beat_timestamps[beat_count - 1];
  float measured_seconds = (latest - earliest) / 1000.0f; // seconds measured between first and last beat

  for (int i = 1; i < beat_count; i++) {
    unsigned long dt = beat_timestamps[i] - beat_timestamps[i - 1];  // ms
    if (dt == 0) continue;
    float bpm = 60000.0f / (float)dt;
    sum_bpm    += bpm;
    sum_bpm_sq += bpm * bpm;
  }

  float mean_bpm = sum_bpm / interval_count;
  float var_bpm  = (sum_bpm_sq / interval_count) - (mean_bpm * mean_bpm);
  if (var_bpm < 0.0f) var_bpm = 0.0f;
  float std_bpm  = sqrt(var_bpm);

  Serial.print("Average heart rate over ~");
  Serial.print(measured_seconds, 1);
  Serial.print(" s: ");
  Serial.print(mean_bpm, 1);
  Serial.println(" BPM");

  Serial.print("Approx heart rate variability (std dev of BPM): ");
  Serial.print(std_bpm, 1);
  Serial.println(" BPM");

  // Classification rules (simple heuristic)
  String exercise_type;
  if (mean_bpm >= 140.0f) {
    exercise_type = "Anaerobic (high-intensity, short bursts)";
  } else if (mean_bpm >= 90.0f) {
    exercise_type = "Aerobic (moderate-intensity, sustainable)";
  } else {
    exercise_type = "Rest / very light activity (no clear exercise)";
  }

  Serial.print("Estimated exercise type (based on heart rate intensity): ");
  Serial.println(exercise_type);

  // store stats in globals for the "sample" command
  stored_mean_bpm       = mean_bpm;

  stored_exercise_type  = exercise_type;
  sample_available      = true;

  Serial.println();
  Serial.println("Notes & guidance:");
  Serial.println("- This classification uses heart rate intensity from a short (~30s) PPG sample.");
  Serial.println("- Aerobic exercise typically is moderate intensity and sustained (>= ~30 min).");
  Serial.println("- Anaerobic exercise is high intensity and typically occurs in short bursts (< ~10 min).");
  Serial.println("- If your heart rate is high but you only performed a short burst (sprinting/weightlifting), it is likely anaerobic.");
  Serial.println("- If your heart rate is moderately elevated and you have been exercising for a longer duration (30+ minutes), it is likely aerobic.");

  Serial.println();
  Serial.println("Examples:");
  if (mean_bpm >= 140.0f) {
    Serial.println("Anaerobic examples: sprinting, HIIT, heavy weightlifting, jumping");
    Serial.println("If your activity lasted less than ~10 minutes at this intensity, it is almost certainly anaerobic.");
  } else if (mean_bpm >= 90.0f) {
    Serial.println("Aerobic examples: walking briskly, jogging, cycling, swimming, dancing");
    Serial.println("If this elevated rate is sustained for 30+ minutes, it is aerobic exercise.");
  } else {
    Serial.println("Rest/Light: walking slowly, sitting, relaxing. To exercise, increase intensity or duration.");
  }

  Serial.println();
  Serial.println("Ready to enter next command");
}
void print_sample_data() {
  if (!sample_available) {
    Serial.println("No PPG sample collected yet. Run 'exercise' first.");
    return;
  }

  Serial.println("---Stored PPG sample data---");

  // Print all stored samples
  for (int i = 0; i < stored_sample_count; i++) {
    Serial.print(stored_time[i]);
    Serial.print(",");
    Serial.println(stored_ac[i]);
  }
  Serial.println("Time(ms), AC_value");

  Serial.print("Total samples: ");
  Serial.println(stored_sample_count);

  Serial.print("Total detected beats: ");
  Serial.println(stored_beat_count);

  Serial.print("Mean BPM: ");
  Serial.println(stored_mean_bpm);  

  Serial.print("Exercise Type: ");
  Serial.println(stored_exercise_type);
  Serial.println();

}
