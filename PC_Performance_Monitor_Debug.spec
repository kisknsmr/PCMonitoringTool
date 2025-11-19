# -*- mode: python ; coding: utf-8 -*-

import os
import glob
HERE = os.path.abspath(os.path.dirname(SPECPATH))

# --- Analysisブロック ---
# 依存関係を解析し、必要なファイルを集める
a = Analysis(
    ['main.py'],                 # メインのPythonスクリプト
    pathex=[HERE],               # スクリプトを探すパスにカレントディレクトリを追加
    binaries=[],                 # Python以外のバイナリ（通常は自動検出されるが、必要なら追加）
    datas=[                      # データファイル (DLLとICO)
        # 'DLLs' フォルダ内の *.dll を '.' (出力ディレクトリのルート) にコピー
        (f, '.') for f in glob.glob(os.path.join(HERE, 'DLLs', '*.dll'))
    ] + [
        # 'app.ico' を '.' (出力ディレクトリのルート) にコピー
        ('app.ico', '.')
    ],
    hiddenimports=[              # PyInstallerが自動検出できない可能性のあるライブラリ
        'pythonnet',
        'clr',
        'System' # .NET関連で必要になることがある
    ],
    hookspath=[],                # カスタムフックファイルのパス
    hooksconfig={},             # フックの設定
    runtime_hooks=[],            # 実行時に実行されるフック
    excludes=[],                 # 含めたくないモジュール
    noarchive=False,             # .pyzアーカイブを使用するか (Falseで通常使用)
    optimize=0                   # Pythonバイトコードの最適化レベル
)

# --- PYZブロック ---
# Pythonライブラリをまとめた .pyz アーカイブを作成
pyz = PYZ(a.pure)

# --- EXEブロック (デバッグ用に修正) ---
# 実行ファイル(.exe)を作成する設定
exe = EXE(
    pyz,                         # Pythonライブラリのアーカイブ
    a.scripts,                   # メインスクリプト ([main.py])
    [],                          # バイナリはCOLLECTで処理するので空リスト
    [],                          # データファイルはCOLLECTで処理するので空リスト
    [],                          # ZIPファイルはCOLLECTで処理するので空リスト
    exclude_binaries=True,       # EXE自体にバイナリを含めない (COLLECTで処理)
    name='PC_Performance_Monitor_Debug', # ★ デバッグ用として別名に ★
    debug='all',                 # ★ PyInstallerのデバッグ情報をすべて出力 ★
    bootloader_ignore_signals=False,
    strip=False,                 # シンボル情報を削除しない
    upx=False,                   # UPX圧縮を無効にする
    console=True,                # ★ コンソールウィンドウを表示する ★
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,            # アーキテクチャ (Noneで自動)
    codesign_identity=None,
    entitlements_file=None,
    icon='app.ico',              # EXEのアイコンファイル
    uac_admin=True               # 管理者権限を要求 (manifestは自動生成)
)

# --- COLLECTブロック ---
# EXEファイルと、それ以外の依存ファイル(バイナリ, データファイル)を
# 指定したフォルダ(name='PC_Performance_Monitor_Debug')にまとめる
coll = COLLECT(
    exe,                         # 上記で作成したEXEオブジェクト
    a.binaries,                  # Analysisで集めたバイナリファイル
    a.zipfiles,                  # Analysisで集めたZIPファイル
    a.datas,                     # Analysisで集めたデータファイル (DLL, ICOを含む)
    strip=False,                 # シンボル情報を削除しない
    upx=False,                   # UPX圧縮を無効にする
    upx_exclude=[],              # UPX圧縮から除外するファイル (upx=Falseなので影響なし)
    name='PC_Performance_Monitor_Debug' # ★ 出力フォルダ名もデバッグ用に変更 ★
)