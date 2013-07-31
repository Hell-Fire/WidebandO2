#include <RunningAverage.h>
#include <PID_v1.h>




/*
 Some configurations
*/


#define BOARD_TEENSYPP20


const float targetLambda = 1.0; // Narrowband simulation, cross point in lambda
const float widebandTop = 2.0; // Where the +5v of the linear wideband output is

const int initialRampUpTill = 255 / 14.0 * 7.5; // Heater warm up stage, this sets the top (TODO: Measure input voltage, max output should be 7.5v)
const int initialRampUpStep = 255 / (14.0 / 0.5); // Heater warm up stage, this is how fast the voltage ramps up per second, spec sheet says .5v a second).

double heatTarget = 2.3 / 1300.0 * 200.0; // Target voltage, this is pulse voltage divided by total target resistance, times the target cell resistance
int heatSampleTime = 500; // How often the heater is sampled for PID, in milliseconds
double pumpTarget = 0.45; // Target Nernst cell voltage
int pumpSampleTime = 40; // How often the nernst cell is sampled for PID, in milliseconds (should be slower than the RC filter feeding the opamp)

int speedOfLoop = 4; // How many millis it takes for loop to run, on Teensy++ 2.0, it's 4 millis

//#define DEBUG       // DEBUG enables human readable logging and info messages
//#define NO_SERIAL   // NO_SERIAL disables the CSV output of the controllers controls, a nice way to view this log is with OpenLogViewer - http://olv.diyefi.org/



/*
  Pin Definitions
*/
#ifdef BOARD_TEENSYPP20
const int PIN_VGND = PIN_F0; // Analog In - Level of the virtual ground
const int PIN_VS = PIN_F1; // Analog In - Reads Nernst Cell voltage
const int PIN_IPA = PIN_F2; // Analog In - Measures pump current via differential amp
const int POUT_VS = PIN_B6; // Digital Out - Pulses Nernst Cell to calculate internal resistance
const int POUT_NARROW = PIN_F7; // Digital Out - Feeds voltage divider, 0-1v signal to ECU
const int POUT_IP = PIN_B4; // Analog Out - Pump cell current output, keeps Nernst cell in stoich range
const int POUT_HEATER = PIN_B5; // Analog Out - Heater control, keeps Nernst cell at right temperature
const int POUT_WIDE = PIN_D1; // Analog Out - Wideband linear output


const int PIN_RPM = -1;

#else
#error "No board defined"
#endif

/*
  Start of code
*/
#ifdef DEBUG
#define DP(...) Serial.print(__VA_ARGS__)
#define DPL(...) Serial.println(__VA_ARGS__)
#else
#define DP(...)
#define DPL(...)
#endif



double heatIn, heatOut;
PID heater(&heatIn, &heatOut, &heatTarget, 800, 100, 1, REVERSE);
double pumpIn, pumpOut;
PID pump(&pumpIn, &pumpOut, &pumpTarget, 800, 100, 0.001, DIRECT);

RunningAverage vgndavg(1);
RunningAverage ipaavg(pumpSampleTime/speedOfLoop*5);
RunningAverage nernstavg(pumpSampleTime/speedOfLoop);

float volts(int adc) {
  return adc / 1024.0 * 5.0;
}

float vGndVolts() {
  vgndavg.addValue(volts(analogRead(PIN_VGND)));
  return vgndavg.getAverage();
}

float nernstVolts() {
  nernstavg.addValue(volts(analogRead(PIN_VS)));
  return nernstavg.getAverage();
}

float pumpVolts() {
  ipaavg.addValue(volts(analogRead(PIN_IPA)));
  return ipaavg.getAverage();
}

float nernstResistanceVolts() {
    int before = analogRead(PIN_VS); // Read nernst before pulse
    digitalWrite(POUT_VS, 1); // Pulse it
    delayMicroseconds(30); // Wait for the pulse cap to reach full voltage
    int after = analogRead(PIN_VS); // Read during the pulse
    digitalWrite(POUT_VS, 0); // Restore it
    delayMicroseconds(200); // Give the nernst cell and cap some time to equalise back out after
    return volts(after - before);
}

#define LAMBDA_LOOKUP_SIZE 20
const float lambdaLookup[LAMBDA_LOOKUP_SIZE] = {
  -2.22, 0.65,
  -1.82, 0.70,
  -1.11, 0.80,
  -0.50, 0.90,
   0.00, 1.016,
   0.33, 1.18,
   0.67, 1.43,
   0.94, 1.70,
   1.38, 2.42,
   2.54, 4.99
};

float lambda(float amps) {
  if (amps < lambdaLookup[0]) return 0.0; // Too rich for sensor
  if (amps > lambdaLookup[LAMBDA_LOOKUP_SIZE-2]) return 9.99; // To lean to care
  for (int x = 0; x < LAMBDA_LOOKUP_SIZE-2; x = x + 2) {
    if (amps >= lambdaLookup[x] && amps <= lambdaLookup[x+2]) {
      // Interpolate the lookup
      return (amps - lambdaLookup[x]) / (lambdaLookup[x+2] - lambdaLookup[x]) * (lambdaLookup[x+3] - lambdaLookup[x+1]) + lambdaLookup[x+1];
    }
  }
}

void serialHeader() {
  #ifndef NO_SERIAL
  Serial.println("millis,lambda,ipa,vgnd,heatIn,heatOut,heatTarget,pumpIn,pumpOut,pumpTarget");
  #endif
}

void serialLog(float l, float ipa, float vgnd) {
  // lambda, ipa, vgnd, heatIn, heatOut, heatTarget, pumpIn, pumpOut, pumpTarget
  #ifndef DEBUG
  #ifndef NO_SERIAL
    Serial.print(millis());
    Serial.print(",");
    Serial.print(l);
    Serial.print(",");
    Serial.print(ipa);
    Serial.print(",");
    Serial.print(vgnd);
    Serial.print(",");
    Serial.print(heatIn);
    Serial.print(",");
    Serial.print(heatOut);
    Serial.print(",");
    Serial.print(heatTarget);
    Serial.print(",");
    Serial.print(pumpIn);
    Serial.print(",");
    Serial.print(pumpOut);
    Serial.print(",");
    Serial.print(pumpTarget);
    Serial.println("");
  #endif
  #endif
}

void setup() {
  // Configure pins
  pinMode(PIN_VS, INPUT);
  pinMode(PIN_IPA, INPUT);
  pinMode(PIN_VGND, INPUT);
  
  pinMode(POUT_NARROW, OUTPUT);
  digitalWrite(POUT_NARROW, 0);
  
  pinMode(POUT_WIDE, OUTPUT);
  analogWrite(POUT_WIDE, 0);
  
  pinMode(POUT_VS, OUTPUT);
  digitalWrite(POUT_VS, 0);
  
  pinMode(POUT_IP, OUTPUT);
  analogWrite(POUT_IP, 128);
  
  pinMode(POUT_HEATER, OUTPUT);
  analogWrite(POUT_HEATER, 0);
  
  // Configure PIDs
  heater.SetOutputLimits(0, initialRampUpTill);
  heater.SetSampleTime(heatSampleTime);
  heater.SetMode(AUTOMATIC);
  pump.SetOutputLimits(0, 250);
  pump.SetSampleTime(pumpSampleTime);
  pump.SetMode(AUTOMATIC);
  
  // Start running
  delay(1000);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  serialHeader();
  
  DPL("Hello Master!");
  
  //DPL("Waiting for engine crank");
  // TODO: Watch RPM input, we're looking for at least post crank speeds (200RPM+)
  
  //DPL("Performing delay prior to warmup");
  //delay(5000); // Wait for some warm gasses through the exhaust
  
  // Slowly ramp up the sensors heater
  DPL("Warming up!");
  heatOut = 0;
  while (heatOut < initialRampUpTill) {
    if ((heatIn = nernstResistanceVolts()) < heatTarget) break; // Temperature is at target already, break out early to let PID take over
    heatOut += initialRampUpStep;
    analogWrite(POUT_HEATER, heatOut);
    serialLog(0.0,0.0,0.0);
    delay(1000);
  }
  DPL("Warm up complete");
}

#ifdef DEBUG
elapsedMillis debug;
#endif
void loop() {

  heatIn = nernstResistanceVolts();
  heater.Compute();
  analogWrite(POUT_HEATER, heatOut);
  
  float vgnd = vGndVolts();
  
  pumpIn = nernstVolts() - vgnd;
  pump.Compute();
  analogWrite(POUT_IP, pumpOut);
  
  // We rely on the circuit to give us a good range of mA to V, for a 61.9 ohm sense resistor, you need a 1 to 14.5395 ratio on the opamp, 22k and 330k across it gives a good result
  float ipa =  pumpVolts() - vgnd;
  float l = lambda(ipa);
  if (l > targetLambda) {
    digitalWrite(POUT_NARROW, 0); // Lean
  } else {
    digitalWrite(POUT_NARROW, 1); // Rich
  }
  
  unsigned int widebandOut;
  if (l < 0.65) widebandOut = 0;
  else if (l > widebandTop) widebandOut = 255;
  else widebandOut = ((l / widebandTop) - (0.65 / widebandTop)) * 255;
  analogWrite(POUT_WIDE, widebandOut);
  
  
  serialLog(l, ipa, vgnd);
  #ifdef DEBUG
  if (debug >= 500) {
    DP("Heater: Volts: "); DP(heatIn); DP(" Target: "); DP(heatTarget); DP(" Out: "); DPL(heatOut);
    DP("Nernst: Volts: "); DP(pumpIn); DP(" Target: "); DP(pumpTarget); DP(" Out: "); DPL(pumpOut);
    DP("VGnd: "); DP(vgnd); DP(" IpA: "); DP(ipa); DP(" Lambda: "); DPL(lambda(ipa));
    debug = 0;
  }
  #endif
}
