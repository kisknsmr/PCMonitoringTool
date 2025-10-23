# -*- mode: python ; coding: utf-8 -*-

import os
import glob
HERE = os.path.abspath(os.path.dirname(SPECPATH))

a = Analysis(
    ['main.py'],
    pathex=[HERE],
    binaries=[],
    datas=[
        # ★ 'DLLs' フォルダの "中身 (*.dll)" を
        # ★ EXEと同じ場所 ('.') にコピーする
        (f, '.') for f in glob.glob(os.path.join(HERE, 'DLLs', '*.dll'))
    ] + [
        # ★ 'app.ico' もEXEと同じ場所 ('.') にコピー
        ('app.ico', '.')
    ],
    hiddenimports=[
        'pythonnet',
        'clr',
        'System'
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0
)

pyz = PYZ(a.pure)

# ★ COLLECTブロックを "削除"
# EXEブロックにすべてを渡す (レガシーフラット形式)
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    a.zipfiles,
    name='PC_Performance_Monitor',
    console=False,
    icon='app.ico',
    uac_admin=True
)