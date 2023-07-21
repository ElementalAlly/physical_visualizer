#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();

const uint16_t samples = 512;
const int samplingFrequency = 44100;
const int clockFrequency = 16000000;
const int sampling_ticks = 362;

const double signalFrequency = 4000;
const uint8_t amplitude = 100;
const double cycles = (((samples-1) * signalFrequency) / samplingFrequency);

const int SCL_INDEX = 0x00;
const int SCL_TIME = 0x01;
const int SCL_FREQUENCY = 0x02;
const int SCL_PLOT = 0x03;

const int stepsPerRevolution = 2038;

const int tickTime = 5;

const uint16_t stepper_ticks = 34000;

double vReal[samples];
double vImag[samples];

int stepperStates[4][4] = {
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};

class stepper {
  public:
    int target;
    int currentTick;
    int currentState;
    int ports[4];
    int prevTime = 0;
    stepper(int port1, int port2, int port3, int port4) {
      target = 0;
      currentTick = 0;
      currentState = 0;
      ports[0] = port1;
      ports[1] = port2;
      ports[2] = port3;
      ports[3] = port4;
      for(int i=0; i<4; i++) {
        pinMode(ports[i], OUTPUT);
        digitalWrite(ports[i], LOW);
      }
    }
    void setTarget(int newTarget) {
      target = newTarget;
    }
    void tick(uint16_t time) {
      if(time - prevTime >= stepper_ticks) {
         if(currentTick < target) {
            currentState = (currentState + 1) % 4;
            for(int i=0; i<4; i++) {
               writeToPort(ports[i], stepperStates[currentState][i]);
            }
            currentTick += 1;
         }
         else if(currentTick > target) {
            currentState = (currentState + 3) % 4;
            for(int i=0; i<4; i++) {
               writeToPort(ports[i], stepperStates[currentState][i]);
            }
            currentTick -= 1;
         }
         prevTime = TCNT1;
      }
    }
    void writeToPort(int port, int state) {
      if(state == 0) {
        digitalWrite(port, LOW);
      } else if(state == 1) {
        digitalWrite(port, HIGH);
      }
    }
    bool atTarget() {
      return currentTick == target;
    }
    int printTick() {
      return currentTick;
    }
    int printState() {
      return currentState;
    }
};

class sampler {
   public:
      int prevTime = 0;
      int numSamples = 0;
      int pin;
      arduinoFFT fft;
      sampler(int in_pin) {
         pin = in_pin;
         pinMode(in_pin, INPUT);
      }
      int tick(uint16_t time) {
         if (time - prevTime >= sampling_ticks) {
            vReal[numSamples] = analogRead(pin) - 511;//int8_t((amplitude * (sin((numSamples * (twoPi * cycles)) / samples))) / 2.0);
            vImag[numSamples] = 0;
            numSamples += 1;
            prevTime = TCNT1;
            if (numSamples >= samples) {
              uint16_t startTime = TCNT1;
               fft = arduinoFFT(vReal, vImag, samples, samplingFrequency);
               fft.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
               fft.Compute(FFT_FORWARD);
               uint16_t endTime = TCNT1;
               uint16_t eta = endTime - startTime;
               startTime = TCNT1;
               fft.ComplexToMagnitude();
               numSamples = 0;
               uint16_t endTime2 = TCNT1;
               uint16_t eta2 = endTime2 - startTime;
               /*Serial.print("compute: ");
               Serial.print(eta);
               Serial.print(" magnitude: ");
               Serial.println(eta2); */
               return 1;
            }
         }
         return 0;
      }
};


void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
   for (uint16_t i = 0; i < bufferSize; i++)
   {
      double abscissa;
      /* Print abscissa value */
      switch (scaleType)
      {
         case SCL_INDEX:
         abscissa = (i * 1.0);
      break;
      case SCL_TIME:
         abscissa = ((i * 1.0) / samplingFrequency);
      break;
      case SCL_FREQUENCY:
         abscissa = ((i * 1.0 * samplingFrequency) / samples);
      break;
   }
   Serial.print(abscissa, 6);
   if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
      Serial.print(" ");
      Serial.println(vData[i], 4);
   }
   Serial.println();
}

stepper myStepper(7, 6, 5, 4);
sampler mySampler(A1);

void setup() {
   // put your setup code here, to run once:
   Serial.begin(115200);
   while(!Serial);
   Serial.println("ready");
   Serial.println("Setup");

   TCCR1A = 0;
   TCCR1B = 1;

   Serial.println();
   Serial.println(sampling_ticks);
}

int status = 0;

void loop() {
   myStepper.tick(TCNT1);
   uint16_t StartTime = TCNT1;
   status = mySampler.tick(TCNT1);
   if (status == 1) {
      int startIndex = 0;
      int endIndex = 4;
      double peaks[7];
      int maxIndex[7];

      for (int i = 0; i < 7; i++) {
         /*Serial.print("Start: ");
         Serial.print(startIndex);
         Serial.print(", End:");
         Serial.print(endIndex);
         Serial.println(); */
         double max = 0;
         for (int j = startIndex; j < endIndex; j++) {
            if(max < vReal[j]) {
               max = vReal[j];
               maxIndex[i] = j;
            }
         }
         peaks[i] = max;
         startIndex = endIndex;
         endIndex *= 2;
      }
      /*for (int i = 0; i < 7; i++) {
         Serial.print(peaks[i]);
         Serial.print("     ");
      }
      Serial.println(); */
      //Serial.println(2038 / 2 * max((peaks[4] - 1000) / 4000, 0));
      myStepper.setTarget(peaks[4]);//(int)(2038 / 2 * max((peaks[4] - 1000) / 4000, 0)));
      uint16_t EndTime = TCNT1;
      uint16_t eta = EndTime - StartTime;
      Serial.println(eta);
   }
   /* if (TCNT1 > 50000) {
      TCNT1 = 0;
   } */
}

/* void loop() {
   // put your main code here, to run repeatedly:
   for (uint16_t i = 0; i < samples; i++) {
      vReal[i] = analogRead(A1);
      vImag[i] = 0.0;
      delayMicroseconds(22);
   }

   //Serial.println("Data:");
   //PrintVector(vReal, samples, SCL_TIME);

   FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
   //Serial.println("Weighed data:");
   //PrintVector(vReal, samples, SCL_TIME);
   FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
   //Serial.println("Computing Real values:");
   //PrintVector(vReal, samples, SCL_INDEX);
   //Serial.println("Computed Imaginary values:");
   //PrintVector(vImag, samples, SCL_INDEX);
   FFT.ComplexToMagnitude(vReal, vImag, samples);
   //Serial.println("Computed magnitudes:");
   //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);

   int startIndex = 0;
   int endIndex = 4;
   int peaks[7];

   for (int i = 0; i < 7; i++) {
     int max = 0;
     for (int j = startIndex; j < endIndex; j++) {
       if(max < vReal[j]) {
         max = vReal[j];
       }
     }
     peaks[i] = max;

     startIndex = endIndex;
    endIndex *= 2;
   }

  for (int i = 0; i < 7; i++) {
    Serial.print(peaks[i]);
    Serial.print("     ");
  }
  Serial.println();

   //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
   //Serial.println(x, 6);


} */
