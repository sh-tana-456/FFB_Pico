# ゲームパッド入力の追加

通常ゲームパッドのHID Input Report（Report ID `0x01`）へ、軸やボタンを追加する方法をまとめます。FFB PIDレポートとは別のInput Reportですが、同じHIDインターフェースとINエンドポイントを共有します。

## 現在の入力

| HID入力 | Report内の型 | Pico側の入力 | 備考 |
|---|---|---|---|
| X | `int16_t` | UARTエンコーダー | ホイール位置、`-32767..32767` |
| Y | `uint16_t` | GPIO28 / ADC2 | アクセル、`0..32767` |
| Z | `uint16_t` | GPIO27 / ADC1 | ブレーキ、`0..32767` |
| Button 1 | bit 0 | GPIO2 | active-low |
| Button 2 | bit 1 | GPIO3 | active-low |
| Button 3 | bit 2 | GPIO5 | active-low |
| Button 4 | bit 3 | GPIO7 | active-low |

ボタンは内部プルアップを有効にしています。各スイッチは、GPIOとGNDの間に接続してください。未接続時は離した状態、GNDへ接続すると押下状態です。

## 関連ファイル

| ファイル | 役割 |
|---|---|
| `main.c` | ADC/GPIOの初期化、値の取得、`gamepad_hid_send()`呼び出し |
| `hid/gamepad_hid.h` | 送信関数の公開API |
| `hid/gamepad_hid.c` | C構造体へ値を詰め、Input Reportを送信 |
| `usb_descriptors.c` | ホストに見せるHID Report Descriptor |

## 軸を追加する場合

軸を1本追加する際は、次の4箇所を同時に変更します。

1. `usb_descriptors.c`のゲームパッドコレクションへUsageとビット幅を追加する。
2. `gamepad_report_t`へ同じ順序・同じ型のメンバーを追加する。
3. `gamepad_hid_send()`の宣言・定義へ引数を追加し、構造体へ代入する。
4. `main.c`で入力を初期化・取得し、送信関数へ渡す。

例えば16 bitのスロットル軸を追加する場合、Generic Desktop Usage Page (`0x05, 0x01`)で未使用のUsageを選びます。アクセルやブレーキと同様に`0..32767`を使うなら、Logical Minimum=`0`、Maximum=`32767`、Report Size=`16`にします。

軸値と記述子のLogical Minimum/Maximumは必ず一致させてください。符号なし軸を`int16_t`として送信すると、32767を超えた値が負数として解釈されます。

## ボタンを追加する場合

既存のボタンは1バイトの下位4 bitを使います。Button 5〜8は上位4 bitを使えるため、構造体サイズを変えずに追加できます。

1. `usb_descriptors.c`のButton Usage MaximumとReport Countを`0x04`から必要な数へ増やす。
2. PaddingのReport Sizeを、`8 - ボタン数`へ変更する。8ボタンにする場合はPaddingを削除する。
3. `main.c`へGPIO定義を追加し、`gamepad_buttons_init()`の配列へ追加する。
4. `gamepad_buttons_read()`が返すbitmaskの対応を確認する。

9個以上へ増やす場合は、ボタン用のバイトを増やします。記述子のReport Count、`gamepad_report_t`、送信関数のbitmask型を同時に`uint16_t`などへ変更してください。

## GPIOを選ぶときの注意

現在、モーター・エンコーダー・MCP3204・LEDが多くのGPIOを使用しています。新しい入力に割り当てる前に、`motor/encoder_uart.c`、`motor/mcp3204.c`、`motor/motor_control.c`、`main.c`のピン定義と重複しないことを確認してください。

ADCを追加する場合、RP2040のADC対応GPIOはGPIO26〜29です。現在はGPIO27とGPIO28をゲームパッド入力、GPIO26は未使用です。ADC入力はGPIOとして初期化せず、`adc_gpio_init()`と`adc_select_input()`を使用します。

## 検証

ビルド後に書き込み、USBを再列挙させます。

```bash
cmake --build build -j2
evtest /dev/input/eventX
```

`evtest`で追加した`ABS_*`または`BTN_*`が表示され、実際に操作したときに値や押下・解放イベントが変化することを確認します。HID記述子を変えた場合、OSが古い記述子を保持していることがあるため、一度USBを抜き差しするか、デバイスの再列挙を待ってください。

## FFBとの共存

ゲームパッドInputとPID Stateは同じHID INエンドポイントを共有します。`main.c`ではPID Stateを優先し、それ以外のタイミングで`gamepad_hid_send()`を呼びます。この制御を外すと、一方のレポートが失われたり、FFB対応ゲームでPID状態が更新されなかったりする可能性があります。
