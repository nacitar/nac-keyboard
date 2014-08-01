
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

#include <nx/core.h> // includes barebones type_traits impl for AVR

/// @brief teensy namespace
namespace teensy {

/// @brief Delay function ensuring an integral value known at compile time.
/// @details If you pass the function a variable instead of a number, then
/// the floating point math routines are going to get linked in to your program
/// making it over 2,000 bytes bigger. For some chips, that's more than how
/// much flash memory you have.
/// See: http://efundies.com/avr/avr_delay_using_c.htm
template <unsigned long milliseconds>
NX_FORCEINLINE void SleepMs() {
  _delay_ms(milliseconds);
}

/// @brief Processor frequencies supported by the Teensy.
/// @details See: https://www.pjrc.com/teensy/prescaler.html
enum class ProcessorFrequency {
  CPU_16MHz  = 0x00,
  CPU_8MHz   = 0x01,
  CPU_4MHz   = 0x02,
  CPU_2MHz   = 0x03,
  CPU_1MHz   = 0x04,
  CPU_500kHz = 0x05,
  CPU_250kHz = 0x06,
  CPU_125kHz = 0x07,
  CPU_62kHz  = 0x08
};

/// @brief Gets the processor frequency as specified by F_CPU.
NX_FORCEINLINE constexpr ProcessorFrequency GetProcessorFrequency() {
  return ProcessorFrequency::
      #if   F_CPU == 16000000
      CPU_16MHz
      #elif F_CPU ==  8000000
      CPU_8MHz
      #elif F_CPU ==  4000000
      CPU_4MHz
      #elif F_CPU ==  2000000
      CPU_2MHz
      #elif F_CPU ==  1000000
      CPU_1MHz
      #elif F_CPU ==   500000
      CPU_500kHz
      #elif F_CPU ==   250000
      CPU_250kHz
      #elif F_CPU ==   125000
      CPU_125kHz
      #elif F_CPU ==    62000
      CPU_62kHz
      #else
      #error "Invalid F_CPU value specified.  Check your makefile."
      CPU_2MHz // specified to prevent more errors than just the #error
      #endif
      ;
}

/// @brief Sets the processor frequency to the one specified in F_CPU.
/// @details Because nearly all examples and libraries are configured with a
/// F_CPU constant, dynamically changing the clock speed, while possible, is
/// likely a poor choice.  For this reason, it is not supported to specify a
/// frequency other than that of F_CPU.
/// See "Reduce Clock Speed" here: https://www.pjrc.com/teensy/low_power.html
/// See: https://www.pjrc.com/teensy/prescaler.html
NX_FORCEINLINE void SetProcessorFrequency() {
  CLKPR = 0x80;
  CLKPR = static_cast<uint8_t>(GetProcessorFrequency());
}

// TODO: remove
/// https://www.pjrc.com/teensy/pins.html
enum class Direction : bool {
  INPUT  = false,
  OUTPUT = true
};

/// @brief The type used by the AVR registers.
/// @details Using the type of an arbitrary register for this.
typedef std::remove_reference<decltype(DDRD)>::type*  RegisterPointer;

/// @brief Raw integral type of the AVR registers
/// @details Using the type of an arbitrary register for this.
typedef typename std::remove_volatile<
    typename std::remove_reference<decltype(DDRD)>::type>::type register_type;

template <register_type mask_, register_type value_>
using Value = nx::BitValue<register_type, mask_, value_>;
template <register_type mask_, register_type value_>
using Field = nx::BitField<register_type, mask_, value_>;
template <unsigned int... position_>
using Mask = nx::BitMask<register_type, position_...>;



class Port {
  RegisterPointer const directionRegister_;
  RegisterPointer const writeRegister_;
  RegisterPointer const readRegister_;
 public:

  NX_FORCEINLINE constexpr Port(
      RegisterPointer directionRegister,
      RegisterPointer writeRegister,
      RegisterPointer readRegister)
      : directionRegister_(directionRegister)
      , writeRegister_(writeRegister)
      , readRegister_(readRegister) {
  }

  // TODO: rework how we pass these in, perhaps more like set()
  template<class InputPins, class OutputPins>
  NX_FORCEINLINE void setDirection() {
    typedef nx::BitTransaction<
        Value<OutputPins::value, OutputPins::value>, // output
        Value<InputPins::value, 0u> // input
        > DirectionField;
    DirectionField::set(directionRegister_);
  }

  /// @brief Sets the bits specified.  For input pins, set bits mean to enable
  /// the internal pullup.
  template <class Value>
  NX_FORCEINLINE void set() {
    Value::set(writeRegister_);
  }
  // output is 1
  template <class Value>
  NX_FORCEINLINE void setDirection() {
    Value::set(directionRegister_);
  }

  /// @brief Gets the value of an input pin (TODO: does this work for output?)
  template <class Mask>
  NX_FORCEINLINE constexpr register_type get() const {
    return (*readRegister_) & Mask::value;
  }

  template <class Mask>
  NX_FORCEINLINE constexpr register_type getDirection() const {
    return (*directionRegister_) & Mask::value;
  }

};

template <unsigned int index_>
class Pin {
  Port* const port_;
 public:
  NX_FORCEINLINE constexpr Pin(Port* const port) : port_(port) {
  }

  template <bool value>
  NX_FORCEINLINE void set() {
    port_->set<Field<Mask<index_>::value,value>>();
  }
  template <Direction direction>
  NX_FORCEINLINE void setDirection() {
    port_->setDirection<Field<
        Mask<index_>::value,static_cast<register_type>(direction)>>();
  }

  NX_FORCEINLINE constexpr bool get() const {
    return static_cast<bool>(port_->get<Mask<index_>>());
  }
  NX_FORCEINLINE constexpr Direction getDirection() const {
    return static_cast<Direction>(static_cast<bool>(
        port_->getDirection<Mask<index_>>()));
  }
};


Port PortB(&DDRB,&PORTB,&PINB);
Port PortC(&DDRC,&PORTC,&PINC);
Port PortD(&DDRD,&PORTD,&PIND);
Port PortE(&DDRE,&PORTE,&PINE);
Port PortF(&DDRF,&PORTF,&PINF);

}  // namespace teensy

int main() {
  using namespace teensy;
  SetProcessorFrequency();

  // PB4 is tied to VCC in the ergodox pcb, supposedly for "hardware
  // convenience".
  // Supposedly, you can cut the track if you want to use PB4 for something,
  // but unless you do that, we should make this an input without pullup.
  // See: http://geekhack.org/index.php?topic=22780.2850
  // PB4 input
  PortB.setDirection<Value<Mask<4>::value,0>>();
  // PB4 no pullup
  PortB.set<Value<Mask<4>::value,0>>();

  // PD6 output
  PortD.setDirection<Value<
      Mask<6>::value, // pins to set
      Mask<6>::value // output pins
      >>();
  // PD6 pin
  Pin<6> teensyLED(&PortD);

  while (1) {
    teensyLED.set<true>();
    teensy::SleepMs<1000>();
    teensyLED.set<false>();
    teensy::SleepMs<1000>();
  }

  return 0;
}
