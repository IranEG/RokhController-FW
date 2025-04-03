#include <stdio.h>
#include <stdint.h>
#include "tusb.h"
#include "time.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "bsp/board_api.h"
#include "usb_descriptors.h"
#include "pico/cyw43_arch.h"

#include "pico/bootrom.h"

#define SPI_PORT    spi1
#define SPI_SCK_PIN 10
#define SPI_RX_PIN  8
#define SPI_TX_PIN  11
#define SPI_CS_PIN  9

#define SPI_SPEED 1000000
#define MCP3204_CH_LX 0
#define MCP3204_CH_LY 1
#define MCP3204_CH_RX 2
#define MCP3204_CH_RY 3

struct AnalogStick {
  uint X;
    uint Y;
};

rokh_hid_gamepad_report_t report = {
  .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
  .hat = 0, .buttons = 0
};

enum {
    BLINK_NOT_MOUNTED   = 250,
    BLINK_MOUNTED       = 1000,
    BLINK_SUSPENDED     = 2500
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void print_i(char *msg)
{
    printf("[%llu]\t ", time_us_64());
    printf("%s", msg);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+

void led_blinking_task(void)
{
    static uint32_t start_ms = 0;
    static bool led_state = false;

    if (!blink_interval_ms)
      return;

    if (board_millis() - start_ms < blink_interval_ms) 
      return;
    
    start_ms += blink_interval_ms;

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

    led_state = 1 - led_state;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  print_i("Device Mounted\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
  print_i("Device Unmounted\n");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
  print_i("Device Suspended\n");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
  print_i("Device Resumed\n");
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  // skip if hid is not ready yet
  if ( !tud_hid_ready() )
    return;

  // use to avoid send multiple consecutive zero report for keyboard
  static bool has_gamepad_key = false;

  if ( btn ) {
    report.hat = GAMEPAD_HAT_UP;
    report.buttons = GAMEPAD_BUTTON_A;
    report.rx = 0xfff;
    report.ry= 0xfff;

    printf("Size of hid_gamepad_report_t %d\n", sizeof(rokh_hid_gamepad_report_t));
    printf("Size of report %d\n", sizeof(report));

    tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(rokh_hid_gamepad_report_t));
    has_gamepad_key = true;

  } else {
    report.hat = GAMEPAD_HAT_CENTERED;
    report.buttons = 0;
      
    if (has_gamepad_key)
      tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(rokh_hid_gamepad_report_t));
      
    has_gamepad_key = false;
  }
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) // not enough time
    return;
  
    start_ms += interval_ms;

  uint32_t btn = board_button_read();

  // Remote wakeup
  if (tud_suspended() && btn)
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  } else {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_GAMEPAD, btn);
  }
}

//--------------------------------------------------------------------+
// USB HID Things for later
//--------------------------------------------------------------------+

// Invoked when sent REPORT successfully to host, i.e. complete
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;

  print_i("Report sent\n");

  /*uint8_t next_report_id = report[0] + 1u;

  if (next_report_id < REPORT_ID_COUNT) {
    send_hid_report(next_report_id, board_button_read());
  }*/
}

// Invoked when REPORT fails to send to host, i.e. Not complete
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_failed_cb(uint8_t instance, hid_report_type_t report_type, uint8_t const* report, uint16_t xferred_bytes)
{
  print_i("Report failed: ");
  printf("Instance: %d, report_type: %d, report: ", instance, report_type);

  for (int i = 0; i < sizeof(rokh_hid_gamepad_report_t); i++) {
    printf("%x ", report[i]);
  }

  printf("xferred_bytes: %d\n", xferred_bytes);
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  /*if (report_type == HID_REPORT_TYPE_OUTPUT) {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD) {
      // bufsize should be (at least) 1
      if ( bufsize < 1 )
        return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK) {
        // Capslock On: disable blink, turn led on
        blink_interval_ms = 0;
        board_led_write(true);
      } else {
        // Caplocks Off: back to normal blink
        board_led_write(false);
        blink_interval_ms = BLINK_MOUNTED;
      }
    }
  }*/
}

//--------------------------------------------------------------------+
// Analog Stick SPI ADC
//--------------------------------------------------------------------+

// Initialize the SPI
void mcp3204_init()
{
  spi_init(SPI_PORT, SPI_SPEED);
  spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);

  gpio_init(SPI_CS_PIN);
  gpio_set_dir(SPI_CS_PIN, GPIO_OUT);
  gpio_put(SPI_CS_PIN, 1);
  
  return;
}

// Read Analog data from the ADC channel
int16_t mcp3204_read(int channel)
{
  uint8_t tx_buf[3];
  uint8_t rx_buf[3];

  tx_buf[0] = 0x06;
  tx_buf[1] = (channel & 0x07) << 6;
  tx_buf[2] = 0x00;

  gpio_put(SPI_CS_PIN, 0);

  spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 3);

  gpio_put(SPI_CS_PIN, 1);

  int16_t adc_value = ((rx_buf[1] & 0x0F) << 8) | rx_buf[2];
  int8_t adc_value_8 = (adc_value >> 4);

  return adc_value;
}

int main()
{ 
  board_init();
  printf("\e[1;1H\e[2J"); //Blank Serial Screen
  print_i("Board Initialized.\n");

  mcp3204_init();
  print_i("mcp3204 Initialized.\n");

  // Initialize the Wi-Fi chip
  if (cyw43_arch_init()) {
      print_i("Wi-Fi init failed\n");
      return -1;
  }
  print_i("CYW43 Initialized.\n");

  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
      board_init_after_tusb();
  }

  print_i("tud_init\n");
  
  while (true) {
    report.x = mcp3204_read(MCP3204_CH_LX);
    report.y = mcp3204_read(MCP3204_CH_LY);
    tud_task();
    led_blinking_task();
    hid_task();
  }
}
