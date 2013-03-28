#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include "clksys_driver.h"

#include "Descriptors.h"

#include <LUFA/Version.h>
#include <LUFA/Drivers/USB/USB.h>

/** Endpoint address of the CDC device-to-host notification IN endpoint. */
#define CDC_NOTIFICATION_EPADDR        (ENDPOINT_DIR_IN  | 2)

/** Endpoint address of the CDC device-to-host data IN endpoint. */
#define CDC_TX_EPADDR                  (ENDPOINT_DIR_IN  | 3)

/** Endpoint address of the CDC host-to-device data OUT endpoint. */
#define CDC_RX_EPADDR                  (ENDPOINT_DIR_OUT | 4)


/** LUFA CDC Class driver interface configuration and state
 *  information. This structure is passed to all CDC Class driver
 *  functions, so that multiple instances of the same class within a
 *  device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
  {
    .Config =
    {
      .ControlInterfaceNumber         = 0,
      .DataINEndpoint                 =
      {
        .Address                = CDC_TX_EPADDR,
        .Size                   = CDC_TXRX_EPSIZE,
        .Banks                  = 1,
      },
      .DataOUTEndpoint                =
      {
        .Address                = CDC_RX_EPADDR,
        .Size                   = CDC_TXRX_EPSIZE,
        .Banks                  = 1,
      },
      .NotificationEndpoint           =
      {
        .Address                = CDC_NOTIFICATION_EPADDR,
        .Size                   = CDC_NOTIFICATION_EPSIZE,
        .Banks                  = 1,
      },
    },
  };

volatile uint8_t ok_to_send = 0;

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void) {
  // PORTE.OUT = 0xff;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void) {
  // PORTE.OUT = 0x00;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void) {
  bool ConfigSuccess = true;

  ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void) {
  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/* Callback for control line state changes. */
void
EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo) {
  bool dtr;

  dtr = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice
         & CDC_CONTROL_LINE_OUT_DTR);
  if (dtr) {
    PORTE.OUT = 0xff;
    ok_to_send = 1;
  } else {
    PORTE.OUT = 0x00;
    ok_to_send = 0;
  }
}

/*
 * Pins:
 *
 * Port E:
 *  3: LED output
 *
 */
void
setup_pins(void) {
  // Configure Port E as all outputs, initially at 0.
  PORTE.DIR = 0xff;
  PORTE.OUT = 0x00;
}

void
blink(int count) {
  int i;

  for (i = 0; i < count; i++) {
    PORTE.OUT = 0x00;
    _delay_ms(125);
    PORTE.OUT = 0xff;
    _delay_ms(250);
    PORTE.OUT = 0x00;
    _delay_ms(125);
  }

  _delay_ms(400);
}

void
setup_clocks(void) {
  // Enable the 32 MHz internal oscillator.
  CLKSYS_Enable(OSC_RC32MEN_bm);

  // Set the multiplication factor and clock reference for the PLL.
  // The USB clock runs at 48 MHz.
  // 32 MHz / 4 = 4 MHz, 48 MHz / 4 MHz = 6 (factor).
  OSC.PLLCTRL = (OSC_PLLSRC_RC32M_gc | (6 << OSC_PLLFAC_gp));

  // Wait for the 32 MHz clock to be ready.
  do {
  } while (CLKSYS_IsReady(OSC_RC32MRDY_bm) == 0);

  // Switch to the 32 MHz clock for the main system clock.
  // XXX Check the return value!
  CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_RC32M_gc);

  // Enable the PLL.
  CLKSYS_Enable(OSC_PLLEN_bm | OSC_RC32MEN_bm);

  // Turn off all the other clocks.
  CLKSYS_Disable(OSC_RC32KEN_bm | OSC_RC2MEN_bm | OSC_XOSCEN_bm);

  // Wait for the PLL to be ready.
  do {
  } while (CLKSYS_IsReady(OSC_PLLRDY_bm) == 0);
}

void
setup(void) {
  setup_clocks();
  setup_pins();
  // Disable the USART.
  USARTD0.CTRLB = 0x00;

  // Initialize the USB device.
  USB_Init();

  // Turn on all low level interrupts.
  PMIC.CTRL |= PMIC_LOLVLEN_bm;

  // Turn on interrupts.
  sei();
}


int
main(void) {
  uint8_t count = 0;

  setup();

  while(1) {
    if (count == 0) {
      // Toggle the Port E outputs.
      // XXX PORTE.OUTTGL = 0xff;
      if (USB_DeviceState == DEVICE_STATE_Configured && ok_to_send) {
        CDC_Device_SendString(&VirtualSerial_CDC_Interface, "Hello World!\r\n");
        CDC_Device_Flush(&VirtualSerial_CDC_Interface);
      }
    }
      
    //blink(2);
    //_delay_ms(2000);
    if (USB_DeviceState == DEVICE_STATE_Configured) {
      /* Must throw away unused bytes from the host, or it will lock up
         while waiting for the device */
      CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    }
    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();

    count++;
    _delay_ms(1);
  }
}
