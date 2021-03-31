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
#include <Switch.h>
#include <Remote.h>

//Pin Definitionen (when using 644P(A): use Standard Pinout)

#define CONFIG_BUTTON_PIN 1       //PB1
#define WSLED_PIN         13      //PD5
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


//Einstellungen für die RGB LEDs
#define WSNUM_LEDS    6
#define WSLED_TYPE    WS2812B    //LED Typ
#define WSCOLOR_ORDER GRB        //Farbreihenfolge
CRGB    leds[WSNUM_LEDS];


#define PEERS_PER_LED_CHANNEL  4
#define PEERS_PER_RC_CHANNEL   4

using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
  {0xF3, 0x3E, 0x00},     // Device ID
  "JPRC6LED01",           // Device Serial
  {0xF3, 0x3E},           // Device Model
  0x10,                   // Firmware Version
  as::DeviceType::Dimmer, // Device Type
  {0x01, 0x01}            // Info Bytes
};

typedef AskSin<DualStatusLed<ONBOARD_LED_PIN1, ONBOARD_LED_PIN2>, NoBattery, Radio<LibSPI<CC1101_CS>, CC1101_GDO0>> Hal;
Hal hal;

DEFREGISTER(OUReg0, MASTERID_REGS)
class OUList0 : public RegList0<OUReg0> {
  public:
    OUList0(uint16_t addr) : RegList0<OUReg0>(addr) {}
    void defaults () {
      clear();
    }
};

DEFREGISTER(LEDReg1, CREG_AES_ACTIVE)
class LEDList1 : public RegList1<LEDReg1> {
  public:
    LEDList1(uint16_t addr) : RegList1<LEDReg1>(addr) {}
    void defaults () {
      clear();
    }
};

DEFREGISTER(OUReg3, SWITCH_LIST3_STANDARD_REGISTER, PREG_ACTTYPE, PREG_ACTNUM, PREG_ACTINTENS);
typedef RegList3<OUReg3> SwPeerListEx;
class OUList3 : public SwitchList3Tmpl<SwPeerListEx> {
  public:
    OUList3(uint16_t addr) : SwitchList3Tmpl<SwPeerListEx>(addr) {}
    void defaults() {
      SwitchList3Tmpl<SwPeerListEx>::defaults();
    }
    void even () {
      SwitchList3Tmpl<SwPeerListEx>::even();
    }
    void odd () {
      SwitchList3Tmpl<SwPeerListEx>::odd();
    }
    void single () {
      SwitchList3Tmpl<SwPeerListEx>::single();
    }
};

class LEDChannel : public ActorChannel<Hal, LEDList1, OUList3, PEERS_PER_LED_CHANNEL, OUList0, SwitchStateMachine>  {
  public:
  class LEDOffDelayAlarm : public Alarm {
      LEDChannel& chan;
    public:
      LEDOffDelayAlarm (LEDChannel& c) : Alarm(0), chan(c) {}
      virtual ~LEDOffDelayAlarm () {}

      void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
        chan.ledOff(true);
      }
  };

  class LEDBlinkAlarm : public Alarm {
      LEDChannel& chan;
    private:
      uint8_t decis;
      bool even;
    public:
      LEDBlinkAlarm (LEDChannel& c) : Alarm(0), chan(c), decis(0), even(false) {}
      virtual ~LEDBlinkAlarm () {}

      void start(uint8_t d) {
        decis = d;
        even = false;
        sysclock.cancel(*this);
        this->set(decis2ticks(decis));
        sysclock.add(*this);
      }

      void trigger (__attribute__ ((unused)) AlarmClock& clock)  {
        this->set(decis2ticks(decis));
        leds[chan.number() -7 ] = CHSV(chan.Color(), chan.Color() < 200 ? 255 : 0, even ? chan.Brightness() : 0);
        even = !even;
        clock.add(*this);
      }
  };
  private:
    bool first;
    uint8_t color;
    uint8_t  brightness;
  protected:
    LEDOffDelayAlarm ledOffDelayAlarm;
    LEDBlinkAlarm    ledBlinkAlarm;
    typedef ActorChannel<Hal, LEDList1, OUList3, PEERS_PER_LED_CHANNEL, OUList0, SwitchStateMachine> BaseChannel;
  public:
    LEDChannel () : BaseChannel(), first(true), color(0), brightness(0), ledOffDelayAlarm(*this), ledBlinkAlarm(*this)  {}
    virtual ~LEDChannel() {}

    uint8_t Color() {
      return color;
    }
    uint8_t Brightness() {
      return brightness;
    }

    void updateLED(bool blackOut=false) {
      leds[number() -7 ] = CHSV(color, color < 200 ? 255 : 0, blackOut ? 0 : brightness);
    }

    void ledOff(bool setCh) {
      sysclock.cancel(ledOffDelayAlarm);
      sysclock.cancel(ledBlinkAlarm);
      updateLED(true);
      if (setCh) BaseChannel::set( 0x00, 0x00, 0xffff );
    }

    void setLedOffDelay(uint16_t dly) {
      DPRINT("set off delay ");DDECLN(dly);
      sysclock.cancel(ledOffDelayAlarm);
      ledOffDelayAlarm.set(dly);
      sysclock.add(ledOffDelayAlarm);
    }

    void ledOn(bool setCh) {
      sysclock.cancel(ledBlinkAlarm);
      updateLED();
      if (setCh) BaseChannel::set( 0xc8, 0x00, 0xffff );
    }

    void setLedColor(uint8_t val) {
      switch (val) {
        case 0:
          color = 0; // Black;
          brightness = 0;
          break;
        case 11:
          color = 0; // Red;
          break;
        case 21:
          color = 96; // Green;
          break;
        case 31:
          color = 64; // Yellow;
          break;
        case 41:
          color = 171; // Blue;
          break;
        case 51:
          color = 192; // Violet;
          break;
        case 61:
          color = 132; // Turquoise;
          break;
        case 71:
          color = 200; // White;
          break;
        case 81:
          color = 32;  // Orange;
          break;
      }
      updateLED();
    }

    void setLedBrightness(uint8_t val) {
      brightness = val;
      updateLED();
    }

    void setLedBlink(uint8_t decis) {
      if (decis > 0)
      ledBlinkAlarm.start(decis);
    }

   bool process (const ActionSetMsg& msg) {
      BaseChannel::set( msg.value(), msg.ramp(), msg.delay() );
      return true;
    }

    bool process (const ActionCommandMsg& msg) {
      static uint8_t lastmsgcnt = 0;
      if (msg.count() != lastmsgcnt) {
        lastmsgcnt = msg.count();
        setLedBrightness(msg.value(1));
        uint8_t color = msg.value(3);

        if (color == 0) {
          ledOff(true);
        } else {
          setLedColor(color);

          uint16_t t = ((msg.value(msg.len() - 2)) << 8) + (msg.value(msg.len() - 1));
          if (t > 0 && t != 0x83CA) {
            setLedOffDelay(AskSinBase::intTimeCvt(t));
          }
          ledOn(true);
          setLedBlink(msg.value(2));
        }
      }
      return true;
    }

    bool process (const RemoteEventMsg& msg) {
      bool lg = msg.isLong();
      Peer p(msg.peer());
      uint8_t cnt = msg.counter();
      OUList3 l3 = BaseChannel::getList3(p);
      if ( l3.valid() == true ) {
        typename OUList3::PeerList pl = lg ? l3.lg() : l3.sh();
        if ( lg == false || cnt != lastmsgcnt || pl.multiExec() == true ) {
          lastmsgcnt = cnt;
          DPRINT(F("ACT_TYPE   ")); DDECLN(pl.actType());  // Farbe
          DPRINT(F("ACT_NUM    ")); DDECLN(pl.actNum());   // BPM
          DPRINT(F("ACT_INTENS ")); DDECLN(pl.actIntens());// Helligkeit
          DPRINT(F("OFFDELAY   ")); DDECLN(pl.offDly());   // Ausschaltverzögerung
           if (pl.actType() == 0) {
            ledOff(true);
          } else {
            setLedColor(pl.actType());
            setLedBrightness(pl.actIntens());
            if (pl.offDly() > 0)
              setLedOffDelay(AskSinBase::byteTimeCvt(pl.offDly()));
            ledOn(true);
            setLedBlink(pl.actNum());
          }
        }
        return true;
      }
      return false;
    }

    void init() {
      ledOff(true);
      //this->changed(true);
      first = false;
    }

    uint8_t flags () const {
      return 0;
    }

    void configChanged() { }

    virtual void switchState(__attribute__((unused)) uint8_t oldstate, __attribute__((unused)) uint8_t newstate, __attribute__((unused)) uint32_t delay) {
      if ( newstate == AS_CM_JT_OFF ) {
        if (first == false ) {
          this->ledOff(false);
        }
      }
      if ( newstate == AS_CM_JT_ON ) {
        this->ledOn(false);
      }
      this->changed(true);
    }
};

typedef RemoteChannel<Hal, PEERS_PER_RC_CHANNEL, OUList0> RemChannel;

class OUDevice : public ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, 12, OUList0> {
  public:
    VirtChannel<Hal, RemChannel, OUList0>   rc[6];
    VirtChannel<Hal, LEDChannel, OUList0> ledc[6];
  public:
    typedef ChannelDevice<Hal, VirtBaseChannel<Hal, OUList0>, 12, OUList0> DeviceType;
    OUDevice (const DeviceInfo& info, uint16_t addr) : DeviceType(info, addr) {
      for (uint8_t i = 0; i < 6; i++) DeviceType::registerChannel(  rc[i], i+1);
      for (uint8_t i = 0; i < 6; i++) DeviceType::registerChannel(ledc[i], i+7);
    }
    virtual ~OUDevice () {}

    RemChannel& remoteChannel (uint8_t num)  {
      return   rc[num-1];
    }

    LEDChannel& ledChannel(uint8_t num)  {
      return ledc[num-7];
    }

    virtual void configChanged () {}
};

OUDevice sdev(devinfo, 0x20);
ConfigButton<OUDevice> cfgBtn(sdev);

void initFastLED() {
  FastLED.addLeds<WSLED_TYPE, WSLED_PIN, WSCOLOR_ORDER>(leds, WSNUM_LEDS);
  FastLED.setBrightness(255);

  uint32_t bootColors[3] = {CRGB::Red, CRGB::Green, CRGB::Blue };

  for (uint8_t i = 0; i < 3; i++) {
    fill_solid(leds, WSNUM_LEDS, bootColors[i]);
    FastLED.show();
    _delay_ms(400);
  }

  fill_solid(leds, WSNUM_LEDS, CRGB::Black);
  FastLED.show();
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  initFastLED();

  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  remoteChannelISR(sdev.remoteChannel(1), BTN1_PIN);
  remoteChannelISR(sdev.remoteChannel(2), BTN2_PIN);
  remoteChannelISR(sdev.remoteChannel(3), BTN3_PIN);
  remoteChannelISR(sdev.remoteChannel(4), BTN4_PIN);
  remoteChannelISR(sdev.remoteChannel(5), BTN5_PIN);
  remoteChannelISR(sdev.remoteChannel(6), BTN6_PIN);
  sdev.ledChannel(7).init();
  sdev.ledChannel(8).init();
  sdev.ledChannel(9).init();
  sdev.ledChannel(10).init();
  sdev.ledChannel(11).init();
  sdev.ledChannel(12).init();
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    //if ()
    //hal.activity.savePower<Idle<true>>(hal);
  }
  FastLED.show();
}
