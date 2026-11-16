#!/usr/bin/env python3
"""Install a per-user Ubuntu application launcher (no root privileges)."""
from pathlib import Path

HERE=Path(__file__).resolve().parent


def main():
    target=Path.home()/'.local/share/applications/umi-slam.desktop'
    target.parent.mkdir(parents=True,exist_ok=True)
    app=str(HERE/'app.py').replace('\\','\\\\').replace('"','\\"').replace('`','\\`').replace('$','\\$')
    text=f'''[Desktop Entry]
Type=Application
Name=UMI 轨迹恢复
Comment=双目 IMU 离线轨迹后处理
Exec=python3 "{app}"
Icon=applications-science
Terminal=false
Categories=Science;Education;
'''
    if target.exists() and target.read_text()!=text:
        raise SystemExit(f'Existing launcher differs; inspect it before replacing: {target}')
    target.write_text(text); target.chmod(0o755)
    print(target)


if __name__=='__main__': main()
