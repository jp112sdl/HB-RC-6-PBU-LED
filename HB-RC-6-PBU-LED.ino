//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2019-08-26 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=644p aes=no

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <SPI.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <Register.h>
#include <MultiChannelDevice.h>
#include <FastLED.h>
#include <Remote.h>
#include <Dimmer.h>

//Pin Definitionen (when using 644P(A): use Standard Pinout)

#ifdef __AVR_ATmega644P__
#define CONFIG_BUTTON_PIN 1       //PB1
#define WSLED_PIN         13      //PD5 GPIO Pin LED Anschluss
#define ONBOARD_LED_PIN1  18      //PC2
#define ONBOARD_LED_PIN2  19      //PC3
#define CC1101_CS         4       //PB4
#define CC1101_GDO0       10      //PD2
#define BTN1_PIN          24      //PA0
#define BTN2_PIN          25      //PA1
#define BTN3_PIN          26      //PA2
#define BTN4_PIN          27      //PA3
#define BTN5_PIN          28      //PA4
#define BTN6_PIN          29      //PA5
#endif


//Einstellungen für die RGB LEDs
#define WSNUM_LEDS    6
#define WSLED_TYPE    WS2812B    //LED Typ
#define WSCOLOR_ORDER GRB        //Farbreihenfolge
CRGB    leds[WSNUM_LEDS];


#define PEERS_PER_RGB_CHANNEL  1
#define PEERS_PER_RC_CHANNEL   1

using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
  {0xF3, 0x3E, 0x00},     // Device ID
  "JPRC6LED01",           // Device Serial
  {0xF3, 0x3E},           // Device Model
  0x10,                   // Firmware Version
  as::DeviceType::Dimmer, // Device Type
  {0x01, 0x00}            // Info Bytes
};

typedef AskSin<DualStatusLed<ONBOARD_LED_PIN1, ONBOARD_LED_PIN2>, NoBattery, Radio<LibSPI<CC1101_CS>, CC1101_GDO0>> Hal;
Hal hal;

typedef RemoteChannel<Hal, PEERS_PER_RC_CHANNEL, DimmerList0> RemoteChannelType;
typedef DimmerChannel<Hal, PEERS_PER_RGB_CHANNEL,DimmerList0> RGBLEDChannelType;
typedef DimmerAndRemoteDevice<Hal, RGBLEDChannelType, RemoteChannelType, 12, 1, 6, DimmerList0> RCLEDDevice;

// we need no PWM class
class DummyPWM {
public:
  void init(uint8_t __attribute__ ((unused)) pwm) {}
  void set(uint8_t __attribute__ ((unused)) pwm) {}
  void param(uint8_t __attribute__ ((unused)) speedMultiplier, uint8_t __attribute__ ((unused)) characteristic) {}
};

template<class HalType,class DimmerType,class PWM>
class RGBControl : public DimmerControl<HalType,DimmerType,PWM> {
public:
  typedef DimmerControl<HalType,DimmerType,PWM> BaseControl;
  RGBControl (DimmerType& dim) : BaseControl(dim) {
    for (int i = 0; i < WSNUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
    FastLED.addLeds<WSLED_TYPE, WSLED_PIN, WSCOLOR_ORDER>(leds, WSNUM_LEDS);
    FastLED.setBrightness(255);
  }

  virtual ~RGBControl () {}

  virtual void updatePhysical () {
    // first calculate all physical values of the dimmer channels
    BaseControl::updatePhysical();
    // set brightness and color to LEDs
    for( uint8_t i=0; i<this->physicalCount();  ) {
      uint8_t dimlevel = this->physical[i++];
      uint8_t collevel = this->physical[i++];
      leds[i/2] = CHSV((collevel * 1275L) / 1000, (collevel <  200) ? 255 : 0, dimlevel);
    }
    FastLED.show();
  }
};

RCLEDDevice sdev(devinfo, 0x20);
RGBControl<Hal,RCLEDDevice,DummyPWM> control(sdev);
ConfigButton<RCLEDDevice> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  uint8_t pins[4] = {0,0,0,0}; // all 4 PWM pins are 0
  control.init(hal,pins);
  remoteChannelISR(sdev.remoteChannel(1), BTN1_PIN);
  remoteChannelISR(sdev.remoteChannel(2), BTN2_PIN);
  remoteChannelISR(sdev.remoteChannel(3), BTN3_PIN);
  remoteChannelISR(sdev.remoteChannel(4), BTN4_PIN);
  remoteChannelISR(sdev.remoteChannel(5), BTN5_PIN);
  remoteChannelISR(sdev.remoteChannel(6), BTN6_PIN);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    hal.activity.savePower<Idle<true>>(hal);
  }
}
