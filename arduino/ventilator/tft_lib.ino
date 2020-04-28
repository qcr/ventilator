// the regular Adafruit "TouchScreen.h" library only works on AVRs

// different mcufriend shields have Touchscreen on different pins
// and rotation.
// Run the TouchScreen_Calibr_native sketch for calibration of your shield
#ifdef ignore 
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;       // hard-wired for UNO shields anyway.
#include <TouchScreen.h>
#include <AccelStepper.h>




//Define Stepper Stuff
#define STEP 53
#define DIR 51

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR );
int top = 300;
int pos = 0;

bool start = false;

void setup(void)
{
    //Setup LCD screen for debugging
    tft.reset();
    ID = tft.readID();
    tft.begin(ID);
    Serial.begin(9600);
    tft.setRotation(Orientation);

    //Write basic start message to screen
    tft.fillScreen(WHITE);
    tft.setTextSize(2);
    tft.setTextColor(BLACK);
    tft.setCursor((tft.width() - 48) / 2, (tft.height() * 2) / 4);
    tft.print("START");


    //Setup Stepper
    stepper.setMaxSpeed(100);
    stepper.setAcceleration(3000);

    delay(1000);
}

void loop()
{
    uint16_t xpos, ypos;  //screen coordinates
    tp = ts.getPoint();   //tp.x, tp.y are ADC values

    // if sharing pins, you'll need to fix the directions of the touchscreen pins
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    // we have some minimum pressure we consider 'valid'
    // pressure of 0 means no pressing!

    if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE) {
        switch (Orientation) {
            case 0:
                xpos = map(tp.x, TS_LEFT, TS_RT, 0, tft.width());
                ypos = map(tp.y, TS_TOP, TS_BOT, 0, tft.height());
                break;
            case 1:
                xpos = map(tp.y, TS_TOP, TS_BOT, 0, tft.width());
                ypos = map(tp.x, TS_RT, TS_LEFT, 0, tft.height());
                break;
            case 2:
                xpos = map(tp.x, TS_RT, TS_LEFT, 0, tft.width());
                ypos = map(tp.y, TS_BOT, TS_TOP, 0, tft.height());
                break;
            case 3:
                xpos = map(tp.y, TS_BOT, TS_TOP, 0, tft.width());
                ypos = map(tp.y, TS_LEFT, TS_RT, 0, tft.height());
                break;
        }

        start = !start;

    }
    if (start){
        if (stepper.distanceToGo() == 0)
        {
        
        if (pos == 0){
            pos = top;
            delay(500);
        } else {
            pos = 0;
        }

        stepper.moveTo(pos);
        }
        stepper.run();
    }
}
#endif



