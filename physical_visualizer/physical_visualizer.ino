class highPriorityTicks { // parent class for time-sensitive ticks. Allows for a consistent framework for real-time systems.
  private:
    const static uint16_t tickAmount = 0;
    int prevTime = 0;
  public:
    virtual void tick() { // Represents one call through the system, checking if it should be called yet, then executing code to advance to the next state.
      uint16_t time = TCNT1;
      if (time - prevTime >= tickAmount) {
        prevTime = TCNT1;
      }
    }
};

// Tick system contains all of the steppers, so all steppers can be ticked in one function.

class tickSystem {
  private:
    static const int objectCap = 10;
    int numObjects = 0;
    int currentObject = 0;
    highPriorityTicks* objects[objectCap];
  public:
    tickSystem() {}

    void addObj(highPriorityTicks* tickObject) { // Adding an object to this system, to be ticked together.
      if (numObjects >= objectCap) {
        abort();
      }
      objects[numObjects] = tickObject;
      numObjects += 1;
    }

    void tickAll() { // Tick all objects in the system.
      for (int i = 0; i < numObjects; i++) {
        objects[i]->tick();
      }
    }
    
    void tickNext() {
      objects[currentObject]->tick();
      currentObject = (currentObject + 1) % numObjects;
    }
};

tickSystem systemTicks;

/* From the lookup data through the Fast RSS is not my code. I found this code through
this repo: https://github.com/Klafyvel/AVR-FFT, but the original code is from
this author: abhilash_patel, from this article: https://www.instructables.com/ApproxFFT-Fastest-FFT-Function-for-Arduino/

The only modification I made to this code is to attempt to tick my stepper motors throughout
the Fast Fourier Transform, to smoothly make my motors turn even through an FFT. */ 

/* ======== APPROXFFT CODE STARTS HERE || CREDIT TO abhilash_patel AND Klafyvel ======== */

//---------------------------------lookup data------------------------------------//
byte isin_data[128]=
{0,  1,   3,   4,   5,   6,   8,   9,   10,  11,  13,  14,  15,  17,  18,  19,  20,
22,  23,  24,  26,  27,  28,  29,  31,  32,  33,  35,  36,  37,  39,  40,  41,  42,
44,  45,  46,  48,  49,  50,  52,  53,  54,  56,  57,  59,  60,  61,  63,  64,  65,
67,  68,  70,  71,  72,  74,  75,  77,  78,  80,  81,  82,  84,  85,  87,  88,  90,
91,  93,  94,  96,  97,  99,  100, 102, 104, 105, 107, 108, 110, 112, 113, 115, 117,
118, 120, 122, 124, 125, 127, 129, 131, 133, 134, 136, 138, 140, 142, 144, 146, 148,
150, 152, 155, 157, 159, 161, 164, 166, 169, 171, 174, 176, 179, 182, 185, 188, 191,
195, 198, 202, 206, 210, 215, 221, 227, 236};
unsigned int Pow2[14]={1,2,4,8,16,32,64,128,256,512,1024,2048,4096};
byte RSSdata[20]={7,6,6,5,5,5,4,4,4,4,3,3,3,3,3,3,3,2,2,2};
//---------------------------------------------------------------------------------//

float Approx_FFT(int in[],int N,float Frequency) {
  uint16_t StartTime = TCNT1;
  int a,c1,f,o,x,data_max,data_min=0;
  long data_avg,data_mag,temp11;
  byte scale,check=0;

  data_max=0;
  data_avg=0;
  data_min=0;

  for(int i=0;i<12;i++) {                 //calculating the levels
    if(Pow2[i]<=N) {
      o=i;
    }
  }
  a=Pow2[o];
  int out_r[a];   //real part of transform
  int out_im[a];  //imaginory part of transform

  for(int i=0;i<a;i++) {                //getting min max and average for scalling
    out_r[i]=0; out_im[i]=0;
    data_avg=data_avg+in[i];
    if(in[i]>data_max){data_max=in[i];}
    if(in[i]<data_min){data_min=in[i];}
  }

  data_avg=data_avg>>o;
  scale=0;
  data_mag=data_max-data_min;
  temp11=data_mag;

  //scaling data  from +512 to -512

  if(data_mag>1024) {
    while(temp11>1024) {
      temp11=temp11>>1;
      scale=scale+1;
    }
  }

  if(data_mag<1024) {
    while(temp11<1024) {
      temp11=temp11<<1;
      scale=scale+1;
    }
  }


  if(data_mag>1024) {
    for(int i=0;i<a;i++) {
      in[i]=in[i]-data_avg;
      in[i]=in[i]>>scale;
    }
    scale=128-scale;
  }

  if(data_mag<1024) {
    scale=scale-1;
    for(int i=0;i<a;i++) {
      in[i]=in[i]-data_avg;
      in[i]=in[i]<<scale;
    }
    scale=128+scale;
  }

  for(int i=0;i<12;i++) {                 //calculating the levels
    if(Pow2[i]<=N) {
      o=i;
    }
  }

  systemTicks.tickAll();

  x=0;
  for(int b=0;b<o;b++) {                    // bit reversal order stored in im_out array
    c1=Pow2[b];
    f=Pow2[o]/(c1+c1);
    for(int j=0;j<c1;j++) {
      x=x+1;
      out_im[x]=out_im[j]+f;
      systemTicks.tickAll();
    }
  }

  for(int i=0;i<a;i++) {           // update input array as per bit reverse order
    out_r[i]=in[out_im[i]];
    out_im[i]=0;
    systemTicks.tickAll();
  }

  uint16_t EndTime = TCNT1;
  uint16_t eta = EndTime - StartTime;
  // Serial.print("FFT Init: ");
  // Serial.println(eta);

  int i10,i11,n1,tr,ti;
  float e;
  int c,s,temp4;
  for(int i=0;i<o;i++) {                                    //fft
    i10=Pow2[i];              // overall values of sine/cosine
    i11=Pow2[o]/Pow2[i+1];    // loop with similar sine cosine
    e=1024/Pow2[i+1];  //1024 is equivalent to 360 deg
    e=0-e;
    n1=0;
    systemTicks.tickAll();

    for(int j=0;j<i10;j++) {
      c=e*j;    //c is angle as where 1024 unit is 360 deg
      while(c<0) {
        c=c+1024;
      }
      while(c>1024) {
        c=c-1024;
      }

      n1=j;

      systemTicks.tickAll();

      for(int k=0;k<i11;k++) {
        temp4=i10+n1;
        if(c==0) {
          tr=out_r[temp4];
          ti=out_im[temp4];
        }
        else if(c==256) {
          tr= -out_im[temp4];
          ti=out_r[temp4];
        }
        else if(c==512) {
          tr=-out_r[temp4];
          ti=-out_im[temp4];
        }
        else if(c==768) {
          tr=out_im[temp4];
          ti=-out_r[temp4];
        }
        else if(c==1024) {
          tr=out_r[temp4];
          ti=out_im[temp4];
        }
        else {
          tr=fast_cosine(out_r[temp4],c)-fast_sine(out_im[temp4],c);            //the fast sine/cosine function gives direct (approx) output for A*sinx
          ti=fast_sine(out_r[temp4],c)+fast_cosine(out_im[temp4],c);
        }

        systemTicks.tickAll();

        out_r[n1+i10]=out_r[n1]-tr;
        out_r[n1]=out_r[n1]+tr;
        if(out_r[n1]>15000 || out_r[n1]<-15000) {
          check=1;
        }   //check for int size, it can handle only +31000 to -31000,

        out_im[n1+i10]=out_im[n1]-ti;
        out_im[n1]=out_im[n1]+ti;
        if(out_im[n1]>15000 || out_im[n1]<-15000) {
          check=1;
        }

        n1=n1+i10+i10;
      }
    }

    if(check==1){                                             // scalling the matrics if value higher than 15000 to prevent varible from overflowing
      for(int i=0;i<a;i++) {
        out_r[i]=out_r[i]>>1;
        out_im[i]=out_im[i]>>1;
        systemTicks.tickAll();
      }
      check=0;
      scale=scale-1;                 // tracking overall scalling of input data
    }
  }


  if(scale>128) {
    scale=scale-128;
    for(int i=0;i<a;i++) {
      out_r[i]=out_r[i]>>scale;
      out_im[i]=out_im[i]>>scale;
      systemTicks.tickAll();
    }
    scale=0;
  }                                                   // revers all scalling we done till here,
  else {
    scale=128-scale;
  }                             // in case of nnumber getting higher than 32000, we will represent in as multiple of 2^scale

  /*
  for(int i=0;i<a;i++)
  {
    Serial.print(out_r[i]);Serial.print("\t");                    // un comment to print RAW o/p
    Serial.print(out_im[i]);
    Serial.print("i");Serial.print("\t");
    Serial.print("*2^");Serial.println(scale);
  }
  */

  //---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
  int fout,fm,fstp;
  float fstep;
  fstep=Frequency/N;
  fstp=fstep;
  fout=0;fm=0;

  for(int i=1;i<Pow2[o-1];i++) {               // getting amplitude from compex number
    /* Second part modified by klafyvel to output the result in the
    input array.
    */
    //out_r[i]=fastRSS(out_r[i],out_im[i]);
    in[i]=fastRSS(out_r[i],out_im[i]);
    // Approx RSS function used to calculated magnitude quickly

    out_im[i]=out_im[i-1]+fstp;
    if (fout<in[i]){
      fm=i; fout=in[i];
    }
    /* End of the second part modified by klafyvel. */

    systemTicks.tickAll();

    // un comment to print Amplitudes (1st value (offset) is not printed)
    // Serial.println(out_r[i]);// Serial.print("\t");
    //Serial.print("*2^");Serial.println(scale);
  }


  float fa,fb,fc;
  fa=out_r[fm-1];
  fb=out_r[fm];
  fc=out_r[fm+1];
  fstep=(fa*(fm-1)+fb*fm+fc*(fm+1))/(fa+fb+fc);

  return(fstep*Frequency/N);
}

//---------------------------------fast sine/cosine---------------------------------------//

int fast_sine(int Amp, int th) {
  int temp3,m1,m2;
  byte temp1,temp2, test,quad,accuracy;
  accuracy=5;    // set it value from 1 to 7, where 7 being most accurate but slowest
               // accuracy value of 5 recommended for typical applicaiton
  while(th>1024) {
    th=th-1024;
  }   // here 1024 = 2*pi or 360 deg
  while(th<0) {
    th=th+1024;
  }
  quad=th>>8;

  if(quad==1) {
    th=512-th;
  }
  else if(quad==2) {
    th= th-512;
  }
  else if(quad==3) {
    th= 1024-th;
  }

  temp1= 0;
  temp2= 128;     //2 multiple
  m1=0;
  m2=Amp;

  temp3=(m1+m2)>>1;
  Amp=temp3;
  for(int i=0;i<accuracy;i++) {
    test=(temp1+temp2)>>1;
    temp3=temp3>>1;
    if(th>isin_data[test]) {
      temp1=test; Amp=Amp+temp3; m1=Amp;
    }
    else if(th<isin_data[test]) {
      temp2=test; Amp=Amp-temp3; m2=Amp;
    }
  }

  if(quad==2) {
    Amp= 0-Amp;
  }
  else if(quad==3) {
    Amp= 0-Amp;
  }
  return(Amp);
}

int fast_cosine(int Amp, int th) {
  th=256-th;  //cos th = sin (90-th) formula
  return(fast_sine(Amp,th));
}

//--------------------------------------------------------------------------------//


//--------------------------------Fast RSS----------------------------------------//
int fastRSS(int a, int b) {
  if(a==0 && b==0){return(0);}
  int min,max,temp1,temp2;
  byte clevel;
  if(a<0){a=-a;}
  if(b<0){b=-b;}
  clevel=0;
  if(a>b) {
    max=a;min=b;
  }
  else {
    max=b;min=a;
  }

  if(max>(min+min+min)) {
    return max;
  }
  else {
    temp1=min>>3; if(temp1==0){temp1=1;}
    temp2=min;
    while(temp2<max) {
      temp2=temp2+temp1;clevel=clevel+1;
    }
    temp2=RSSdata[clevel];temp1=temp1>>1;
    for(int i=0;i<temp2;i++) {
      max=max+temp1;
    }
    return(max);
  }
}
//--------------------------------------------------------------------------------//

/* ======== APPROXFFT CODE ENDS HERE || CREDIT TO abhilash_patel AND Klafyvel ======== */

// Clearing and setting bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

const int samples = 512;
const int samplingFrequency = 16000;

const int stepsPerRevolution = 2038;
const int maxAngleDegrees = 75;
const uint16_t maxTicks = (uint16_t)((double)stepsPerRevolution / 360 * maxAngleDegrees);

int vReal[samples];
uint16_t amplitudes[samples];

class stepper: public highPriorityTicks {
  private:
    const int stepperStates[4][4] = {
      {1, 1, 0, 0},
      {0, 1, 1, 0},
      {0, 0, 1, 1},
      {1, 0, 0, 1}
    };
    const static uint16_t tickAmount = 37000;
    const static int limitSwitchBounceTolerance = 10;
    int prevTime = 0;
    int target;
    int direction;
    int currentTick;
    int currentState;
    int ports[4];
    int limitSwitchPin;

    bool atTarget() { // Checks if the motor is at the target.
      return currentTick == target;
    }

  public:
    stepper(int port1, int port2, int port3, int port4, int limitPin) { // Initializes the motor. Pins are initialized for output, and low is written. The limit switch's pin is initialized to input. The stepper is then added to the tick system.
      target = 0;
      direction = 0;
      currentTick = 0;
      currentState = 0;
      ports[0] = port1;
      ports[1] = port2;
      ports[2] = port3;
      ports[3] = port4;
      limitSwitchPin = limitPin;
      for (int i=0; i<4; i++) {
        pinMode(ports[i], OUTPUT);
        digitalWrite(ports[i], LOW);
      }
      pinMode(limitSwitchPin, INPUT_PULLUP);
      systemTicks.addObj(this);
    }

    void setTarget(int newTarget) { // Sets the new target for the motor to go to. If the motor has started to move up and a higher peak is detected, the motor will move to the higher peak, and vice versa. Less extreme amplitudes are ignored.
      if (direction == 0) {
        target = newTarget;
        if (newTarget < currentTick) {
          direction = -1;
        }
        if (newTarget > currentTick) {
          direction = 1;
        }
      }
      if (direction == 1) {
        target = max(target, newTarget);
      }
      if (direction == -1) {
        target = min(target, newTarget);
      }
    }

    void reset() { // Resets the module's position to the bottom, just before the limit switch is pressed down.
      target = -stepsPerRevolution;
      int count = 0;
      while (count < limitSwitchBounceTolerance) {
        tick();
        if (digitalRead(limitSwitchPin) == LOW) {
          count++;
        }
        else {
          count = 0;
        }
      }
      Serial.println("Limit switch down");
      target = stepsPerRevolution;
      count = 0;
      while (count < limitSwitchBounceTolerance) {
        tick();
        if (digitalRead(limitSwitchPin) == HIGH) {
          count++;
        }
        else {
          count = 0;
        }
      }
      Serial.println("Limit switch up");
      currentTick = -50;
      setTarget(0);
    }

    void tick() override { // If enough time has passed, steps the motor once towards the target. If the target has been reached, direction is reset.
      uint16_t time = TCNT1;
      if (time - prevTime >= tickAmount) {
        if (digitalRead(limitSwitchPin) == LOW) {
          currentTick = -50;
        }
        if (currentTick < target) {
          currentState = (currentState + 1) % 4;
          for (int i=0; i<4; i++) {
            digitalWrite(ports[i], stepperStates[currentState][i]);
          }
          currentTick += 1;
        }
        else if (currentTick > target) {
          currentState = (currentState + 3) % 4;
          for (int i=0; i<4; i++) {
            digitalWrite(ports[i], stepperStates[currentState][i]);
          }
          currentTick -= 1;
        }
        else if (atTarget()) {
          direction = 0;
        }
        prevTime = TCNT1;
      }
    }
};

/* === Testing data, defined by the sum of the sin waves defined by these parameters === */

/* const double signal1Freq = 440;
const double signal1Cycles = (((samples-1) * signal1Freq) / samplingFrequency);
const double signal1Amp = 511;
const double signal2Freq = 1000;
const double signal2Cycles = (((samples-1) * signal2Freq) / samplingFrequency);
const double signal2Amp = 0;
const double signal3Freq = 2000;
const double signal3Cycles = (((samples-1) * signal3Freq) / samplingFrequency);
const double signal3Amp = 0;
const double twoPi = 6.283185307; */

/* === Testing data done === */

/* === These are sampler states. GET_SAMPLE means that sampler only gets a sample from the mic, where FFT means that an FFT has been run. === */

#define GET_SAMPLE 0
#define FFT 1

/* === Sampler state end === */

class sampler {
  private:
    const static uint16_t tickAmount = 600; // 726;
    int prevTime = 0;
    int numSamples = 0;
    int pin;
  public:
    sampler(int inPin) { // Initializes the sampler to read from the inPin.
      pin = inPin;
      pinMode(inPin, INPUT);
    }

    int tick() { // If enough time has passed, record the state of the pin. If enough samples have been collected, run the FFT and return the FFT state.
      uint16_t time = TCNT1;
      if (time - prevTime >= tickAmount) {
        // Serial.println(time - prevTime);
        vReal[numSamples] = analogRead(pin) - 512;
        // Debugging, using the sum of the waves defined by the frequencies above the sampler class
        // vReal[numSamples] = int16_t(signal1Amp * (sin(numSamples * (twoPi * signal1Cycles) / samples)) + signal2Amp * (sin(numSamples * (twoPi * signal2Cycles) / samples)) + signal3Amp * (sin(numSamples * (twoPi * signal3Cycles) / samples)));
        numSamples += 1;
        prevTime = TCNT1;
        if (numSamples >= samples) {
          uint16_t startTime = TCNT1;
          Approx_FFT(vReal, samples, 44100);
          uint16_t endTime = TCNT1;
          uint16_t eta = endTime - startTime;
          numSamples = 0;
          return FFT;
        }
      }
      return GET_SAMPLE;
    }
};

stepper stepper1(7, 6, 5, 4, 3);
stepper stepper2(22, 24, 26, 28, 30);
stepper stepper3(23, 25, 27, 29, 31);
stepper stepper4(32, 34, 36, 38, 40);
stepper stepper5(33, 35, 37, 39, 41);
stepper stepper6(42, 44, 46, 48, 50); 
stepper stepper7(43, 45, 47, 49, 51);
sampler mySampler(A1);

void setup() { // Initialize registers, and reset all the modules.
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Setup");
  delay(1000);
  Serial.println("---------");

  // setting up the cpu clock to be read for real-time systems
  TCCR1A = 0;
  TCCR1B = 1;

  // Setting the ADC clock circuit multiplier to 16, for faster Analog Reads
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);

  Serial.println();
  Serial.println("Reset started");
  Serial.println("===========");
  stepper1.reset();
  stepper2.reset();
  stepper3.reset();
  stepper4.reset();
  stepper5.reset();
  stepper6.reset();
  stepper7.reset();
  Serial.println("===========");
  Serial.println("Reset completed");
  delay(1000);
}

int status = 0;

void printList(int* list, int length) { // print all entries of a list of length "length".
  for (int i = 0; i < length; i++) {
    Serial.print(list[i]);
    Serial.print("     ");
  }
  Serial.println();
}

void loop() { // Tick all motors, and tick the sampler. If the FFT has been executed, find the peaks in various ranges that scale exponentially, then sets the motors targets to the adjusted peaks.
  systemTicks.tickNext();
  status = mySampler.tick();
  if (status == FFT) {

    int startIndex = 2;
    int endIndex = 4;
    uint16_t peaks[7];
    int targets[7];
    int maxIndex[7];
    double factors[7] = {(double)maxTicks / 3000.0, (double)maxTicks / 8000.0, (double)maxTicks / 8000.0, (double)maxTicks / 8000.0, (double)maxTicks / 8000.0, (double)maxTicks / 6000.0, (double)maxTicks / 4000.0};
    uint16_t floors[7] = {0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 7; i++) { // make 7-bucket function
      int max = 0;
      for (int j = startIndex; j < endIndex; j++) {
        if (max < vReal[j]) {
          max = vReal[j];
          maxIndex[i] = j;
        }
      }
      peaks[i] = max;
      if (peaks[i] > floors[i]) {
        targets[i] = (peaks[i] - floors[i]) * factors[i];
      }
      else {
        targets[i] = 0;
      }
      if (targets[i] > maxTicks) {
        targets[i] = maxTicks;
      }
      startIndex = endIndex;
      endIndex *= 2;
    }

    /* Serial.print("Raw: ");
    printList(peaks, 7); */
    /* Serial.print("Targets: ");
    printList(targets, 7); */
      
    stepper1.setTarget(targets[0]);
    stepper2.setTarget(targets[1]);
    stepper3.setTarget(targets[2]);
    stepper4.setTarget(targets[3]);
    stepper5.setTarget(targets[4]);
    stepper6.setTarget(targets[5]);
    stepper7.setTarget(targets[6]);
  }
}