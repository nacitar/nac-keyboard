
#include <stdint.h>
#include <avr/io.h>

#if defined(__GNUC__)
#define FORCE_INLINE inline __attribute__((always_inline))
#else
#define FORCE_INLINE inline
#warning "This platform doesn't have a proper FORCE_INLINE value defined."
#endif

namespace teensy {
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
    CPU_62kHz  = 0x08 };

  /// @brief The processor frequency as specified by F_CPU.
  constexpr ProcessorFrequency g_processorFrequency =
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

  /// @brief Sets the processor frequency to the one specified in F_CPU.
  /// @details Because nearly all examples and libraries are configured with a
  /// F_CPU constant, dynamically changing the clock speed, while possible, is
  /// likely a poor choice.  For this reason, it is not supported to specify a
  /// frequency other than that of F_CPU.
  /// See "Reduce Clock Speed" here: https://www.pjrc.com/teensy/low_power.html
  FORCE_INLINE void SetProcessorFrequency() {
    CLKPR = 0x80;
    CLKPR = (uint8_t)g_processorFrequency;
  }
}

int main() {
  teensy::SetProcessorFrequency();

  return 0;
}
