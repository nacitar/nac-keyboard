
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


typedef nx::Bits<register_type> Register;


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

  /// @brief Returns the direction register for the port.
  NX_FORCEINLINE constexpr RegisterPointer directionRegister() const {
    return directionRegister_;
  }
  /// @brief Returns the write register for the port.
  NX_FORCEINLINE constexpr RegisterPointer writeRegister() const {
    return writeRegister_;
  }
  /// @brief Returns the read register for the port.
  NX_FORCEINLINE constexpr RegisterPointer readRegister() const {
    return readRegister_;
  }

};

#if 0
template <unsigned int index_>
class Pin {
  Port* const port_;
 public:
  NX_FORCEINLINE constexpr Pin(Port* const port) : port_(port) {
  }
};
#endif


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
  Register::clear<Register::Mask<4>()>(PortB.directionRegister());

  // PB4 no pullup
  Register::clear<Register::Mask<4>()>(PortB.writeRegister());


  // PD6 output
  Register::set<Register::Mask<6>()>(PortD.directionRegister());

  // COM1A1=1
  // COM1A0=0
  // COM1B1=1
  // COM1B0=0
  // COM1C1=1
  // COM1C0=0
  // WGM11=0
  // WGM10=1
  //
  // CM means Compare Match
  // COM1A == 10 == Cleared CM when up, set on CM when down
  // COM1B == 10 == Cleared CM when up, set on CM when down
  // COM1C == 10 == Cleared CM when up, set on CM when down
  TCCR1A  = 0b10101001;  // set and configure fast PWM

  // ICNC1 = 0
  // ICES1 = 0
  // - = 0
  // WGM13 = 0
  // WGM12 = 1
  // CS12 = 0
  // CS11 = 0
  // CS10 = 1
  TCCR1B  = 0b00001001;  // set and configure fast PWM


  // WGM[3:0] = 0b0101 = 5

  while (1) {
    Register::set<Register::Mask<6>()>(PortD.writeRegister());
    teensy::SleepMs<1000>();
    Register::clear<Register::Mask<6>()>(PortD.writeRegister());
    teensy::SleepMs<1000>();
  }

  return 0;
}
