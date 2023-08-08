#include "arduinoFFT.h"
#include "fix_fft.h"
#include "data.h"

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

arduinoFFT FFT = arduinoFFT();

const int samples = 512;
const int16_t sample_power = 9;
const int samplingFrequency = 44100;
const int clockFrequency = 16000000;

const double signalFrequency = 1000;
const uint8_t amplitude = 5;
double cycles = (((samples-1) * signalFrequency) / samplingFrequency);
const int sin_samples[4] = {0, 1, 0, -1};

float frequency = 44000; // Actually useless for our test.

const int stepsPerRevolution = 2038;

const int tickTime = 5;

int vReal[samples];
int16_t vImag[samples];
uint16_t amplitudes[samples];

int stepperStates[4][4] = {
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};

class high_priority_ticks {
   public:
      uint16_t tick_amount = 0;
      int prevTime = 0;
      virtual void tick(uint16_t time) {
         if(time - prevTime >= tick_amount) {
            prevTime = TCNT1;
         }
      }
};

class tick_system {
   public:
      high_priority_ticks* objects[10];
      int num_objects = 0;
      tick_system() {}
      void add_obj(high_priority_ticks* tick_object) {
         objects[num_objects] = tick_object;
         num_objects += 1;
      }
      void tick_all() {
         for (int i = 0; i < num_objects; i++) {
            objects[i]->tick(TCNT1);
         }
      }
};

tick_system system_ticks;

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

   system_ticks.tick_all();

   x=0;
   for(int b=0;b<o;b++) {                    // bit reversal order stored in im_out array
      c1=Pow2[b];
      f=Pow2[o]/(c1+c1);
      for(int j=0;j<c1;j++) {
         x=x+1;
         out_im[x]=out_im[j]+f;
         system_ticks.tick_all();
      }
   }

   for(int i=0;i<a;i++) {           // update input array as per bit reverse order
      out_r[i]=in[out_im[i]];
      out_im[i]=0;
      system_ticks.tick_all();
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
      system_ticks.tick_all();

      for(int j=0;j<i10;j++) {
         c=e*j;    //c is angle as where 1024 unit is 360 deg
         while(c<0) {
            c=c+1024;
         }
         while(c>1024) {
            c=c-1024;
         }

         n1=j;

         system_ticks.tick_all();

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

            system_ticks.tick_all();

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
            system_ticks.tick_all();
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
         system_ticks.tick_all();
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

      system_ticks.tick_all();

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

class stepper: public high_priority_ticks {
  public:
    int target;
    int currentTick;
    int currentState;
    int ports[4];
    int prevTime = 0;
    uint16_t stepper_ticks = 34000;
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
      system_ticks.add_obj(this);
    }
    void setTarget(int newTarget) {
      target = newTarget;
    }
    void tick(uint16_t time) override {
      if(time - prevTime >= stepper_ticks) {
         if(currentTick < target) {
            currentState = (currentState + 1) % 4;
            for(int i=0; i<4; i++) {
               writeToPin(ports[i], stepperStates[currentState][i]);
            }
            currentTick += 1;
         }
         else if(currentTick > target) {
            currentState = (currentState + 3) % 4;
            for(int i=0; i<4; i++) {
               writeToPin(ports[i], stepperStates[currentState][i]);
            }
            currentTick -= 1;
         }
         prevTime = TCNT1;
      }
    }
    void writeToPin(int port, int state) {
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

class sampler{
   public:
      int prevTime = 0;
      int numSamples = 0;
      int pin;
      int sampling_ticks = 362;
      sampler(int in_pin) {
         pin = in_pin;
         pinMode(in_pin, INPUT);
      }
      int tick(uint16_t time) {
         if (time - prevTime >= sampling_ticks) {
            // vReal[numSamples] = sin_samples[numSamples % 4];
            vReal[numSamples] = (analogRead(pin) - 512) / 8; // int16_t(amplitude * (sin(numSamples * (twoPi * cycles) / samples)));
            // Serial.println(vReal[numSamples]);
            //vImag[numSamples] = 0;
            numSamples += 1;
            prevTime = TCNT1;
            if (numSamples >= samples) {
              uint16_t startTime = TCNT1;
              Approx_FFT(vReal, samples, 44100);
              uint16_t endTime = TCNT1;
              uint16_t eta = endTime - startTime;
              numSamples = 0;
               /* fft = arduinoFFT(vReal, vImag, samples, samplingFrequency);
               fft.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
               fft.Compute(FFT_FORWARD);Instant noodles w/ extra veggie - 12:30
               uint16_t endTime = TCNT1;
               uint16_t eta = endTime - startTime;
               startTime = TCNT1;
               fft.ComplexToMagnitude();
               numSamples = 0;
               uint16_t endTime2 = TCNT1;
               uint16_t eta2 = endTime2 - startTime; */
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

stepper myStepper(7, 6, 5, 4);
stepper stepper2(14, 15, 16, 17);
sampler mySampler(A1);
int system_len = 1;

void setup() {
   // put your setup code here, to run once:
   Serial.begin(115200);
   while(!Serial);
   Serial.println("ready");
   Serial.println("Setup");

   TCCR1A = 0;
   TCCR1B = 1;

   Serial.println();
}

int status = 0;

void loop() {
   system_ticks.tick_all();
   //myStepper.tick(TCNT1);
   uint16_t StartTime = TCNT1;
   status = mySampler.tick(TCNT1);
   if (status == 1) {
      int startIndex = 2;
      int endIndex = 4;
      uint16_t peaks[7];
      int maxIndex[7];

      for (int i = 0; i < 7; i++) {
         /*Serial.print("Start: ");
         Serial.print(startIndex);
         Serial.print(", End:");
         Serial.print(endIndex);
         Serial.println(); */
         int max = 0;
         for (int j = startIndex; j < endIndex; j++) {
            if(max < vReal[j]) {
               max = vReal[j];
               maxIndex[i] = j;
            }
         }
         peaks[i] = max * 2;
         startIndex = endIndex;
         endIndex *= 2;
      }
      for (int i = 0; i < 7; i++) {
         Serial.print(peaks[i]);
         Serial.print("     ");
      }
      Serial.println();
      /* for (int i = 0; i < samples / 2; i++) {
        Serial.print(vReal[i]);
        Serial.print(" ");
      }
      while(1); */
      //Serial.println(2038 / 2 * max((peaks[4] - 1000) / 4000, 0));
      myStepper.setTarget(peaks[4]);//(int)(2038 / 2 * max((peaks[4] - 1000) / 4000, 0)));
      stepper2.setTarget(peaks[5]);
      /*uint16_t EndTime = TCNT1;
      uint16_t eta = EndTime - StartTime;
      Serial.println(eta); */
   }
}
