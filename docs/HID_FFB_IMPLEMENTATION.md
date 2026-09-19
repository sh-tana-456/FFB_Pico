# HID PID Force Feedback 実装

この文書は、PC が送信する USB HID PID (Physical Interface Device) レポートを、このファームウェアがどのように受信・保持・モーター指令へ変換するかを説明します。対象は `hid/ffb_hid.c`、`hid/gamepad_hid.c`、`usb_descriptors.c`、`main.c` です。

## 全体の流れ

```text
ゲーム / DirectInput / SDL
        │ HID Feature / Output Report
        v
TinyUSB: tud_hid_set_report_cb()
        │
        v
hid/ffb_hid.c: effect block の作成・各パラメータの保存
        │
        v
main.c: ffb_hid_update(wheel.x)
        │ int16_t: -10000..10000
        v
motor_control.c: 電流指令 + ソフトエンドストップ + FOC
```

PIDレポートは「トルクを直ちに送る」ものではありません。PC はまずエフェクトブロックを確保して各パラメータを設定し、その後に開始操作を送ります。`ffb_hid.c` は最大40個のブロックを保持し、現在有効な1ブロックを合成します。

## USB複合HIDの構成

1個のHIDインターフェースに、通常ゲームパッドとPID FFBのコレクションを含めています。記述子は `usb_descriptors.c` の `desc_gamepad` です。

| 用途 | Report type | Report ID | 送受信方向 | 実装 |
|---|---:|---:|---|---|
| ホイール X・アクセル Y・ブレーキ Z・Buttons 1–4 | Input | `0x01` | Pico → PC | `gamepad_hid_send()` |
| PID State | Input | `0x02` | Pico → PC | `ffb_hid_send_state()` |
| PID設定 | Output | 下表 | PC → Pico | `tud_hid_set_report_cb()` |
| ブロック作成・状態照会 | Feature | 下表 | 双方向 | `tud_hid_get/set_report_cb()` |

Input / Output / Feature は別種別なので、同じ数値のReport IDを持つことがあります。例えば`0x01`はゲームパッドInputとSet Effect Outputで共用されています。

HID INエンドポイントは1本です。`main.c`はPID Stateを10 msごとに優先送信し、その周期以外でゲームパッドInputを送信します。同じUSBフレームで両方を送らないためです。

### ゲームパッド入力のピン割当

Report ID `0x01`は7バイトです。Xは符号付き、Y/Zは`0..32767`です。ボタンはbit 0〜3がButtons 1〜4に対応します。

| HID入力 | Pico側の既定入力 | 備考 |
|---|---|---|
| X | UARTエンコーダーから換算したホイール位置 | `-32767..32767` |
| Y | GPIO28 / ADC2 | アクセル、ADC値を`0..32767`へ換算 |
| Z | GPIO27 / ADC1 | ブレーキ、ADC値を`0..32767`へ換算 |
| Button 1–4 | GPIO2 / GPIO3 / GPIO5 / GPIO7 | active-low、内部プルアップ有効 |

ボタンGPIOはスイッチの片側をGPIO、もう片側をGNDへ接続します。別の配線を使う場合は`main.c`先頭の`BUTTON_*_PIN`と`BRAKE_ADC_INPUT`を変更します。

## PID状態レポート

`ffb_hid_send_state()` はReport ID `0x02`を10 ms周期で送信します。

| バイト | ビット | 意味 |
|---:|---:|---|
| 0 | 0 | Device Paused |
| 0 | 1 | Actuators Enabled |
| 0 | 4 | Actuator Power |
| 1 | 0 | Effect Playing |
| 1 | 1–7 | 再生中の Effect Block Index |

この状態報告とBlock Load応答は、特にゲームやWine/DirectInputがエフェクトを有効なデバイスとして扱うために重要です。

## PCからのレポート処理

`tud_hid_set_report_cb()` はTinyUSBから呼ばれます。実装は、環境によって先頭にReport IDが含まれる場合と含まれない場合の双方を許容してから、`report_id`と`type`で分岐します。

### Feature Report

| ID | 名称 | 動作 |
|---:|---|---|
| `0x05` | Create New Effect | 次の空きブロックを割り当て、初期値を設定 |
| `0x06` | Block Load | 割当済みブロック、成功状態、空きメモリを返す |
| `0x07` | PID Pool | プールサイズ、最大同時エフェクト数、機能を返す |

Block Load StatusはPID Usageの列挙値なので、記述子ではArrayとして定義しています。Variableにすると一部のDirectInput実装が成功状態を解釈できません。

### Output Report

| ID | 名称 | 保存する主な値 |
|---:|---|---|
| `0x01` | Set Effect | type、duration、gain |
| `0x02` | Set Envelope | attack/fade level、attack/fade time |
| `0x03` | Set Condition | center、正負係数、飽和値、dead band |
| `0x04` | Set Periodic | magnitude、offset、phase、period |
| `0x05` | Set Constant Force | magnitude |
| `0x07` | Set Custom Force Data | Custom波形のサンプル列 |
| `0x09` | Set Ramp Force | ramp start/end |
| `0x0A` | Effect Operation | Start、Start Solo、Stop |
| `0x0C` | Device Control | actuator enable/disable、stop all、reset、pause/continue |
| `0x0D` | Device Gain | デバイス全体のgain |
| `0x0E` | Set Custom Force | Custom波形の個数・周期 |

有効ブロック番号は1〜40です。範囲外のブロック番号や短すぎるレポートは無視します。

## エフェクト合成

`ffb_hid_update(int16_t position)` は、HID X軸位置を受け取り、最終的に`-10000..10000`の力指令を返します。

| 種類 | 基本演算 |
|---|---|
| Constant | `magnitude`をそのまま出力 |
| Ramp | `ramp_start`から`ramp_end`へduration中に線形補間 |
| Square / Sine / Triangle / Saw | `period`と`phase`から波形を生成し、`magnitude + offset`を適用 |
| Spring | 中心位置との差に正負別係数を掛ける。dead bandと飽和値を適用 |
| Damper | 速度に正負別係数を掛ける |
| Inertia | 加速度に正負別係数を掛ける |
| Friction | 速度方向と逆向きになる係数を出力 |
| Custom | PCが送信した符号付き8 bitサンプルを周期再生 |

速度・加速度は位置差分から計算し、少なくとも1 ms間隔で更新します。個別エフェクトの`gain`とDevice Gain、Envelopeを最後に掛け、`-10000..10000`へ飽和させます。

`duration == 0`または`UINT16_MAX`は無期限として扱います。有限durationを超えたエフェクトは停止します。

## モーター制御との境界

`ffb_hid.c`はHID仕様の解釈だけを担当します。出力値は`main.c`から`motor_set_ffb_command()`へ渡され、`motor_control.c`が電流指令へ変換します。

モーター側では、ゲームのFFB仕様とは別にソフトエンドストップを適用します。これはConstant Forceなどが終端の外側へ回し続けないための安全機能です。PIDレポートやゲームパッドX軸のクランプだけではモーター出力を制限できません。

## CDC診断

`main.c`はCDCへ次のHID診断値を出力します。

- `magnitude`: `ffb_hid_update()`の最終出力
- `active`: エフェクト再生中か
- `effect` / `type` / `base` / `period`: 有効ブロックの情報
- `hid_rx`: 受信したHIDレポート総数
- `last`: 直近の`Report ID / report type / payload length`

エフェクトが動かない場合は、`active=1`、非ゼロの`magnitude`、増加する`hid_rx`を順に確認します。HIDは認識されていても、Block LoadやPID Stateの不整合でゲームがエフェクトを開始しないことがあります。

## 変更時の注意

- Report ID、Usage、ビット幅を変える場合は、`usb_descriptors.c`と`ffb_hid.c`の解析を必ず同時に更新します。
- PID Stateを送る頻度やサイズを変える場合は、ゲームパッドInputとのINエンドポイント共有を維持します。
- ギア追加などで回転方向が反転した場合は、`motor/motor_control.h`の`WHEEL_DIRECTION_REVERSED`を`1`にします。HID X軸とモーターへ渡すFFB `magnitude`を同時に反転するため、Spring/Damperなど位置依存エフェクトの整合性を保てます。片方だけを手動で反転しないでください。
- 新しいゲームを試す際は、CDCの`hid_rx`と`last`を記録すると、どのPIDレポートが不足しているか追跡できます。
