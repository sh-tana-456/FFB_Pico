# FFB Pico ファームウェア

Raspberry Pi Pico を USB 複合デバイス（ゲームパッド HID、PID Force Feedback HID、CDC シリアル）として動作させ、ホイール位置に応じて三相モーターを駆動するファームウェアです。

## ドキュメント一覧

| 文書 | 内容 |
|---|---|
| [README.md](README.md) | ビルド、利用方法、ソース構成、開発時の変更箇所（この文書） |
| [DEVICE_SUPPORT.md](DEVICE_SUPPORT.md) | OS・テストツール・ゲームごとの動作確認状況と未検証項目 |
| [HID_CDC_INTEGRATION.md](HID_CDC_INTEGRATION.md) | TinyUSB 複合デバイスの構成と、HID/CDC を変更するときの注意 |
| [debug_log/README_MCP3204_FOC_debug.md](debug_log/README_MCP3204_FOC_debug.md) | MCP3204/FOC の過去の調査ログ |

## 構成

```text
main.c                         アプリケーションの入口、ペダル ADC、CDC ログ
usb_descriptors.c              USB 複合デバイスの記述子
hid/
  ffb_hid.c                    PID Force Feedback の受信・エフェクト合成
  gamepad_hid.c                通常ゲームパッド X/Y レポートの送信
motor/
  motor_control.c              FOC/PWM、電流処理、Core 1 のモーター制御
  encoder_uart.c               RS-485 UART エンコーダー、RX DMA、17 bit角度復号
  mcp3204.c                    MCP3204 の SPI/DMA サンプリング
```

処理の流れは次のとおりです。

```text
エンコーダー ──> encoder_uart ──> motor_control ──> main
                                                   │
PC PID FFB ──> ffb_hid ──────────────────────────> main ──> motor_control ──> PWM/モーター
ペダル ADC ──────────────────────────────────────> main ──> gamepad_hid ──> PC
CDC ログ ─────────────────────────────────────────> main ──> PC
```

## ビルド

Pico SDK を利用できる環境で、リポジトリ直下から実行します。

```bash
cmake -S . -B build
cmake --build build -j2
```

生成物は `build/ffb.uf2` と `build/ffb.elf` です。BOOTSEL を押しながら Pico を USB 接続して現れる RPI-RP2 ドライブへ `ffb.uf2` をコピーすると書き込めます。

デバッガ接続時は、使用している OpenOCD 設定に合わせて `build/ffb.elf` を GDB から書き込んでください。

## PC 側での利用

USB 接続後、ホストには次の機能が現れます。

- ゲームパッド HID: X 軸はホイール位置、Y 軸はペダル値です。
- PID Force Feedback HID: PC が送信したエフェクトを `ffb_hid` が合成し、モーター指令値に変換します。
- CDC ACM: FFB 状態、角度、電流制御状態の診断ログを出力します。

Linux の確認例です。デバイス番号は環境によって変わるため、最初に `ls -l /dev/input/event* /dev/ttyACM*` で確認してください。

```bash
fftest /dev/input/eventX
cat /dev/ttyACM0
```

`fftest` でエフェクトを開始すると CDC に `magnitude=`、`active=`、`effect=` などが出力されます。CDC は TinyUSB の複合デバイス機能であるため、`pico_enable_stdio_usb(ffb 1)` を有効にしないでください。

## Force Feedback の実装範囲

実装済みのエフェクトは Constant、Ramp、Square、Sine、Triangle、Sawtooth Up/Down、Spring、Damper、Inertia、Friction、Custom です。Envelope、Device Gain、開始・停止、エフェクトブロックの割り当ても処理します。実機で確認済みのホスト・ゲームの範囲は [DEVICE_SUPPORT.md](DEVICE_SUPPORT.md) を参照してください。

ホストソフトごとの PID レポート送信方法には差があるため、ゲームごとの互換性は実機確認が必要です。`ffb_hid_update()` に渡す X 軸の向きと、モーター配線によるトルクの向きが逆なら、`main.c` のホイール軸変換またはモーター側の符号を一方だけ反転してください。

## モジュールを変更するときの目安

- USB の Report ID や Usage、インターフェースを変える: `usb_descriptors.c`
- PID レポートの解釈や FFB の演算を変える: `hid/ffb_hid.c`
- 通常ゲームパッドの軸レポートを変える: `hid/gamepad_hid.c`
- エンコーダーの UART、RS-485、DMA、原点補正を変える: `motor/encoder_uart.c`
- MCP3204 の SPI ピン、速度、DMA 転送を変える: `motor/mcp3204.c`
- 電流 LPF、オフセット校正、FOC/PWM を変える: `motor/motor_control.c`
- ペダルの ADC 範囲、LED、CDC のログ内容を変える: `main.c`

## 注意

モーター、ゲートドライバ、電源を接続した状態での書き込みやデバッグは、意図しないトルクが発生する可能性があります。初回確認は車輪を浮かせ、非常停止手段と電流制限を用意した状態で行ってください。
