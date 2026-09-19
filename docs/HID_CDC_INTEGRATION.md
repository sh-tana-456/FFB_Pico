# HID FFB + CDC 複合デバイス

## 関連ファイル

- `../main.c`: TinyUSB の初期化、CDC ログ、CDC 受信コールバック
- `../usb_descriptors.c`: HID と CDC ACM の USB 記述子
- `../hid/ffb_hid.c`: PID Force Feedback の TinyUSB HID コールバック
- `../hid/gamepad_hid.c`: ゲームパッド Input Report の送信

## `tusb_config.h` の要件

以下の HID/CDC 設定を維持します。

```c
#define CFG_TUD_HID 1
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_HID_EP_BUFSIZE 64
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_EP_BUFSIZE 64
```

## CMake

`tinyusb_device` と `tinyusb_board` をリンクします。MCP3204 を使うため `hardware_spi` も必要です。独自の複合デバイスとして CDC を提供するため、`pico_enable_stdio_usb(ffb 1)` は有効にしません。

```cmake
target_link_libraries(your_target
    pico_stdlib pico_multicore hardware_pwm hardware_adc hardware_uart
    hardware_spi tinyusb_device tinyusb_board)
```

## Linux での CDC ログ
```bash
ls -l /dev/ttyACM* /dev/serial/by-id/
cat /dev/ttyACM0
```

CDC ログの内容と周期は `main.c` の `cdc_logf()` と `CDC_LOG_INTERVAL_MS` で変更できます。MCP3204 は `motor/mcp3204.c` の SPI/DMA ドライバで取得し、`motor/motor_control.c` 側で電流値の検証・LPF・オフセット校正を行います。
