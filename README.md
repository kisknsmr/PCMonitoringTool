# PCMonitoringTool（PC Performance Monitor）

CPU の温度・クロック・電圧や、メモリの使用状況をリアルタイムで表示し、記録する Windows 用アプリです。

**最新版: v0.9.0**（2026-10-12）

> 補足: 正式版の前の開発段階のため、版は 1 未満にしています。

## できること

- CPU の状態を表示します。
  - コアごとの使用率・クロック・温度・電圧
  - CPU 全体の消費電力・温度・平均使用率
  - 熱で性能が落ちているか（サーマルスロットリング）
- メモリとスワップ（メモリが足りないときに使うディスク領域）の使用量を表示します。
- CPU とメモリを多く使っているプロセス（アプリ）を表示します。
- 次の4種類の記録を、CSV ファイルに保存します。

| 画面の名前 | 内容 | 最初の間隔 |
|---|---|---|
| Fast System Log | システム全体の状態 | 1 秒 |
| Core Details Log | コアごとの詳しい状態 | 1 秒 |
| Active Detail Log | 動いているプロセスの詳しい状態 | 30 秒 |
| Periodic Snapshot | 定期的なまとめ | 600 秒 |

- コアの状態が変わったときだけ記録して、ファイルが大きくなるのを防げます。

## インストール

1. Windows に Python 3 を入れます。
2. 必要なライブラリを入れます。

```
pip install -r requirements.txt
```

> ⚠️ `DLLs` フォルダの3つのファイル（`LibreHardwareMonitorLib.dll`・`pcmonitor.dll`・`getcoreinfo.dll`）が必要です。フォルダごと `main.py` と同じ場所に置いてください。

## 使い方

1. `python main.py` で起動します。
2. 記録したい種類にチェックを入れ、間隔（秒）を選びます。
3. 「▶️ Start Logging」を押すと、記録が始まります。
4. 止めるときは「⏹️ Stop Logging」を押します。

- 「Log core details only on change」：コアの状態が変わったときだけ記録します。
- 「Start logging on launch」：起動と同時に記録を始めます。

> 補足: 温度などを読み取るため、管理者として実行する必要がある場合があります。

### 記録の保存場所

- `logs\起動した日時\` のフォルダに保存されます。
- ファイル名は `fast.csv`・`corelog.csv`・`detail.csv`・`snapshot.csv` です。

### 温度センサーの名前を調べる

1. `python corename.py` を実行します。
2. この PC の CPU 温度センサーの名前が一覧で出ます。

## 更新履歴

### v0.9.0（2026-10-12）
- 追加：CPU・メモリの状態を表示し、4種類の記録を保存できるようにしました
- 追加：変化したときだけ記録する機能を付けました
- 修正：記録ファイルを書き出すときのエラーを直しました

## フォルダ構成（開発者向け）

```
├─ main.py         アプリ本体
├─ corename.py     温度センサー名を調べる補助スクリプト
├─ config.json     記録の設定・画面の大きさ
├─ DLLs/           ハードウェア情報を読むための部品
├─ manifest.xml    Windows 用の実行設定
├─ app.ico         アイコン
├─ test/           部品の動作確認用スクリプト
└─ VERSION         現在の版
```

## 注意

- Windows 専用です。
- 設定は、終了するときに `config.json` に保存されます。
- `LibreHardwareMonitorLib.dll` は外部のライブラリです。配布するときは、そのライセンスに従ってください。
