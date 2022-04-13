# -*- mode: python ; coding: utf-8 -*-

block_cipher = None


a = Analysis(['RobotDesignStudio.py'],
             pathex=['/home/tree/albero_xbot2_ws/src/modular/src/modular/web'],
             binaries=[],
             datas=[('../config_file.yaml', 'modular'), 
                    ('../robot_id.yaml', 'modular'), 
                    ('../esc_type.yaml', 'modular'), 
                    ('../repl.yaml', 'modular'), 
                    ('../modular_data/', 'modular/modular_data/'), 
                    ('../web/static/', 'modular/web/static/'), 
                    ('../web/templates/', 'modular/web/templates/')],
             hiddenimports=['email.mime.message', 'email.mime.image', 'email.mime.text', 'email.mime.multipart', 'email.mime.nonmultipart', 'email.mime.base', 'email.mime.audio'],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          [],
          exclude_binaries=True,
          name='RobotDesignStudio',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          console=True )
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=False,
               upx=True,
               upx_exclude=[],
               name='RobotDesignStudio')
