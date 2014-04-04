
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

#if defined(__GNUC__)
#define FORCE_INLINE inline __attribute__((always_inline))
#else
#define FORCE_INLINE inline
#warning "This platform doesn't have a proper FORCE_INLINE value defined."
#endif

template<class T> struct remove_pointer      {typedef T type;};
template<class T> struct remove_pointer<T*>  {typedef T type;};
template<class T> struct remove_reference      {typedef T type;};
template<class T> struct remove_reference<T&>  {typedef T type;};
template<class T> struct remove_reference<T&&> {typedef T type;};
template<class T> struct remove_volatile              {typedef T type;};
template<class T> struct remove_volatile<volatile T>  {typedef T type;};

/// @brief Accepts a pointer to data and a bit index, providing an interface to
/// set/get the value of the data bit in that position.
/// @todo Ensure only pointers are passed.
template <class PointerType>
class BitFlag {
  typedef typename remove_volatile<
      typename remove_pointer<PointerType>::type>::type FieldType;
  PointerType const data_;
  const FieldType mask_;
 public:
  FORCE_INLINE constexpr BitFlag(PointerType data,uint8_t index)
      : data_(data)
      , mask_(static_cast<FieldType>(static_cast<FieldType>(1) << index)) {
  }
  FORCE_INLINE void set(bool high) {
    if (high) {
      *data_ |= mask_;
    } else {
      *data_ &= ~mask_;
    }
  }
  FORCE_INLINE bool get() const {
    return static_cast<bool>(*data_ & mask_);
  }
};


/// @brief teensy namespace
namespace teensy {

/// @brief Delay function ensuring an integral value known at compile time.
/// @details If you pass the function a variable instead of a number, then
/// the floating point math routines are going to get linked in to your program
/// making it over 2,000 bytes bigger. For some chips, that's more than how
/// much flash memory you have.
/// See: http://efundies.com/avr/avr_delay_using_c.htm
template <unsigned long milliseconds>
FORCE_INLINE void SleepMs() {
  _delay_ms(milliseconds);
}

/// @brief The type used by the AVR registers.
/// @details Using the type of an arbitrary register for this.
typedef remove_reference<decltype(DDRD)>::type*  RegisterPointer;

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
FORCE_INLINE ProcessorFrequency GetProcessorFrequency() {
  constexpr ProcessorFrequency processorFrequency =
      ProcessorFrequency::
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
  return processorFrequency;
}

/// @brief Sets the processor frequency to the one specified in F_CPU.
/// @details Because nearly all examples and libraries are configured with a
/// F_CPU constant, dynamically changing the clock speed, while possible, is
/// likely a poor choice.  For this reason, it is not supported to specify a
/// frequency other than that of F_CPU.
/// See "Reduce Clock Speed" here: https://www.pjrc.com/teensy/low_power.html
/// See: https://www.pjrc.com/teensy/prescaler.html
FORCE_INLINE void SetProcessorFrequency() {
  CLKPR = 0x80;
  CLKPR = static_cast<uint8_t>(GetProcessorFrequency());
}

/// https://www.pjrc.com/teensy/pins.html
enum class PinDirection : bool {
  INPUT  = false,
  OUTPUT = true
};


/// @brief A type to represent a single bit in an AVR register
typedef BitFlag<RegisterPointer> RegisterBit;

class Port {
 public:
  FORCE_INLINE constexpr Port(
      RegisterPointer directionRegister,
      RegisterPointer writeRegister,
      RegisterPointer readRegister)
      : directionRegister_(directionRegister)
      , writeRegister_(writeRegister)
      , readRegister_(readRegister) {
  }
  FORCE_INLINE constexpr RegisterPointer directionRegister() const {
    return directionRegister_;
  }
  FORCE_INLINE constexpr RegisterPointer writeRegister() const {
    return writeRegister_;
  }
  FORCE_INLINE constexpr RegisterPointer readRegister() const {
    return readRegister_;
  }
 private:
  RegisterPointer const directionRegister_;
  RegisterPointer const writeRegister_;
  RegisterPointer const readRegister_;
};



class Pin {
  Port* const port_;
  const uint8_t index_;
 protected:
  RegisterBit directionBit_;
  RegisterBit writeBit_;
  RegisterBit readBit_;
 public:
  FORCE_INLINE constexpr Pin(Port* port,uint8_t index)
      : port_(port)
      , index_(index)
      , directionBit_(port->directionRegister(), index)
      , writeBit_(port->writeRegister(), index)
      , readBit_(port->readRegister(), index) {
  }
  FORCE_INLINE constexpr Port* port() const {
    return port_;
  }
  FORCE_INLINE constexpr uint8_t index() const {
    return index_;
  }
  FORCE_INLINE constexpr uint8_t bitFlag() const {
    return (1u << index_);
  }
  FORCE_INLINE PinDirection direction() const {
    return static_cast<PinDirection>(directionBit_.get());
  }
  FORCE_INLINE bool value() const { // TODO: does this work with output pins?
    return readBit_.get();
  }
};
class InputPin : public Pin {
 public:
  // using 'D' pins for type deduction
  FORCE_INLINE InputPin(Port* port, uint8_t index, bool pullup)
      : Pin(port,index) {
    directionBit_.set(static_cast<bool>(PinDirection::INPUT));
    setPullup(pullup);
  }
  FORCE_INLINE bool pullup() const { // TODO: does this work?
    return writeBit_.get();
  }
  FORCE_INLINE void setPullup(bool enabled) {
    return writeBit_.set(enabled);
  }
};
class OutputPin : public Pin {
 public:
  // using 'D' pins for type deduction
  FORCE_INLINE OutputPin(Port* port, uint8_t index, bool high)
      : Pin(port,index) {
    directionBit_.set(static_cast<bool>(PinDirection::OUTPUT));
    setValue(high);
  }
  FORCE_INLINE void setValue(bool high) {
    return writeBit_.set(high);
  }
};
teensy::Port PortB(&DDRB,&PORTB,&PINB);
teensy::Port PortC(&DDRC,&PORTC,&PINC);
teensy::Port PortD(&DDRD,&PORTD,&PIND);
teensy::Port PortE(&DDRE,&PORTE,&PINE);
teensy::Port PortF(&DDRF,&PORTF,&PINF);

}  // namespace teensy

int main() {
  teensy::SetProcessorFrequency();

  teensy::OutputPin teensyLED(&teensy::PortD, 6, false);

  // PB4 is tied to VCC in the ergodox pcb, supposedly for "hardware
  // convenience".
  // Supposedly, you can cut the track if you want to use PB4 for something,
  // but unless you do that, we should make this an input without pullup.
  // See: http://geekhack.org/index.php?topic=22780.2850
  teensy::InputPin pb4Vcc(&teensy::PortB, 4, false);

  while (1) {
    teensyLED.setValue(true);
    teensy::SleepMs<1000>();
    teensyLED.setValue(false);
    teensy::SleepMs<1000>();
  }

  return 0;
}
