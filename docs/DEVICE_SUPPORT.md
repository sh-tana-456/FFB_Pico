
# 動作確認状況

この表は「実装されている機能」と「特定ホストで実機確認済みの組み合わせ」を分けて管理します。PID エフェクトの実装範囲は [README.md](README.md) を参照してください。

| 環境・ツール | 状態 | 確認内容 |
|---|---|---|
| Windows | 一部確認済み | HID と Constant Force の動作を確認済み。ゲームごとの全エフェクト互換性は未確認。 |
| Linux / `fftest` | 確認済み | FFB の有効化と CDC の `magnitude` 出力を確認済み。 |
| RaceRoom | 確認済み | FFB の有効化と CDC の `magnitude` 出力を確認済み。 |
| F1 2016 | 未確認 | FFB が有効にならない |

## 確認手順

Linux では入力デバイスと CDC ポートを確認してから、別の端末で次を実行します。

```bash
ls -l /dev/input/event* /dev/ttyACM*
fftest /dev/input/eventX
cat /dev/ttyACM0
```

`fftest` で力を発生させたとき、CDC に `active=1` と非ゼロの `magnitude=` が現れることを確認します。ホストにより HID 出力レポートの送信順序が異なるため、ゲームを追加で確認した場合は、OS・ゲーム名・使用したエフェクト・結果をこの表へ追記してください。

## 解決済みの記録

2026-09-05 の調査では、RS-485 の送受信切り替え中に PWM 割り込みが入るとエンコーダー受信フレームが壊れることがありました。現在の `motor/encoder_uart.c` は送信切り替え区間を割り込み保護し、DMA で受信する構成です。

## 継続課題

- PWM キャリア周波数を上げた場合の FOC 実行時間と安定性の確認
- MCP3204 の異常サンプルに対するハードウェア／EMI 対策
- 各ゲームでの PID Force Feedback 互換性の実機確認
