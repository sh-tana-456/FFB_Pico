
## **Supported Environment**

<table>
  <!-- <caption>HTMLの要素</caption> -->
  <thead>
    <tr>
      <th>OS</th> <th>動作</th> <th>Description</th>
    </tr>
  </thead>
  <tr>
    <td> Windows </td> <td>🔼</td> <td><code>Constant Force</code>は確認済み</td>
  </tr>
  <tr>
    <td> Linux </td> <td>❌️</td> <td><code>fftest</code>で<code>ffb_active</code>は確認済み。<code>magnitude</code>出力なし。</td>
  </tr>
</table>

****

## **Issue**

### On Linux

- *fftest*においてmagnitude取得できず
- RaceRoomでは`ffb_active`にもならない
- F1 2016でも同様に`ffb_active`にならずカクつく（おそらく返答失敗）

### On Windows

- Constant Force以外機能せず（idxをうまく返せていない）

****

## History

### update 2026/09/05

- CDCを有効化すると帯域が使われるのか、`magunitude`などの受信周期が明らかに長くなる（ガリガリする感じになる、たまに0点がモーター１回転分ずれる。）

> **解決**　-> RS485の送信受信のピン切り替え時にPWM割り込みが起こることで受信フレームが壊れていた模様。12byte受信しないと動かないようにしていたが、なぜか`Break Condition+8byte`ほどのデータを受信していた。（12byte以上と認識された?）
> -> ピン切り替え時の割り込みを禁止することで解決。

## ToDo

- エンコーダー読み取りをPWM割り込みでやる
- キャリアを8kHz以上にすると、浮動小数点の計算のせいか割り込み超過か何かで制御が破綻する
