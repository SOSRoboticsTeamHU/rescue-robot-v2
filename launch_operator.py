#!/usr/bin/env python3
"""
Cross-Platform Launcher for Rescue Robot Operator Interface
Creates desktop shortcuts and handles dependency checking on Windows/macOS/Linux
"""

import sys
import os
import platform
import subprocess
import shutil
from pathlib import Path


class CrossPlatformLauncher:
    """Cross-platform launcher for operator interface"""
    
    def __init__(self):
        self.project_root = Path(__file__).parent.absolute()
        self.platform = platform.system()
        self.is_windows = self.platform == "Windows"
        self.is_mac = self.platform == "Darwin"
        self.is_linux = self.platform == "Linux"
        
    def check_dependencies(self) -> bool:
        """Check if required dependencies are installed"""
        print("Checking dependencies...")
        
        required_packages = ["PyQt6", "zmq"]
        missing = []
        
        for package in required_packages:
            try:
                if package == "zmq":
                    __import__("zmq")
                elif package == "PyQt6":
                    __import__("PyQt6")
                print(f"  ✓ {package}")
            except ImportError:
                missing.append(package)
                print(f"  ✗ {package} (missing)")
        
        if missing:
            print(f"\nMissing dependencies: {', '.join(missing)}")
            response = input("Install missing dependencies? (y/n): ").strip().lower()
            if response == 'y':
                self.install_dependencies()
                return True
            else:
                print("Cannot proceed without dependencies.")
                return False
        
        return True
    
    def install_dependencies(self):
        """Install missing dependencies"""
        print("\nInstalling dependencies...")
        requirements_file = self.project_root / "requirements.txt"
        
        if not requirements_file.exists():
            print("ERROR: requirements.txt not found!")
            return False
        
        try:
            subprocess.check_call([
                sys.executable, "-m", "pip", "install", "-r", str(requirements_file)
            ])
            print("Dependencies installed successfully!")
            return True
        except subprocess.CalledProcessError as e:
            print(f"ERROR: Failed to install dependencies: {e}")
            return False
    
    def create_windows_shortcut(self):
        """Create Windows desktop shortcut and .bat file"""
        desktop = Path.home() / "Desktop"
        if not desktop.exists():
            # Try alternate location
            desktop = Path(os.environ.get("USERPROFILE", "")) / "Desktop"
        
        if not desktop.exists():
            print("WARNING: Desktop folder not found. Skipping shortcut creation.")
            return
        
        # Create .bat launcher file
        bat_file = self.project_root / "launch_operator.bat"
        python_exe = sys.executable.replace('\\', '\\\\')
        gui_script = (self.project_root / "mac" / "gui_master.py").as_posix().replace('\\', '\\\\')
        
        bat_content = f'''@echo off
cd /d "{self.project_root}"
"{python_exe}" "{gui_script}" %*
pause
'''
        
        with open(bat_file, 'w', encoding='utf-8') as f:
            f.write(bat_content)
        
        print(f"Created launcher: {bat_file}")
        
        # Create Windows shortcut using PowerShell (if possible)
        shortcut_path = desktop / "Rescue Robot Operator.lnk"
        
        # Try to create shortcut using PowerShell
        ps_script = f'''
$WshShell = New-Object -ComObject WScript.Shell
$Shortcut = $WshShell.CreateShortcut("{shortcut_path}")
$Shortcut.TargetPath = "{bat_file}"
$Shortcut.WorkingDirectory = "{self.project_root}"
$Shortcut.IconLocation = "{self.project_root}\\assets\\icon.ico,0"
$Shortcut.Description = "Rescue Robot Operator Interface"
$Shortcut.Save()
'''
        
        try:
            subprocess.run([
                "powershell", "-Command", ps_script
            ], check=True, capture_output=True)
            print(f"Created desktop shortcut: {shortcut_path}")
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("WARNING: Could not create desktop shortcut. Use launch_operator.bat instead.")
    
    def create_mac_launcher(self):
        """Create macOS .command launcher file"""
        command_file = self.project_root / "launch_operator.command"
        
        python_exe = sys.executable
        gui_script = self.project_root / "mac" / "gui_master.py"
        
        command_content = f'''#!/bin/bash
cd "{self.project_root}"
"{python_exe}" "{gui_script}" "$@"
'''
        
        with open(command_file, 'w') as f:
            f.write(command_content)
        
        # Make executable
        os.chmod(command_file, 0o755)
        print(f"Created launcher: {command_file}")
        
        # Note: macOS .app bundle creation would require additional tools
        print("To create a proper macOS .app bundle, consider using py2app or platypus")
    
    def create_linux_launcher(self):
        """Create Linux .sh launcher and .desktop file"""
        launcher_sh = self.project_root / "launch_operator.sh"
        
        python_exe = sys.executable
        gui_script = self.project_root / "mac" / "gui_master.py"
        
        sh_content = f'''#!/bin/bash
cd "{self.project_root}"
"{python_exe}" "{gui_script}" "$@"
'''
        
        with open(launcher_sh, 'w') as f:
            f.write(sh_content)
        
        os.chmod(launcher_sh, 0o755)
        print(f"Created launcher: {launcher_sh}")
        
        # Create .desktop file for Linux
        desktop_dir = Path.home() / ".local" / "share" / "applications"
        desktop_dir.mkdir(parents=True, exist_ok=True)
        
        desktop_file = desktop_dir / "rescue-robot-operator.desktop"
        icon_path = self.project_root / "assets" / "icon.png"
        
        desktop_content = f"""[Desktop Entry]
Version=1.0
Type=Application
Name=Rescue Robot Operator
Comment=Rescue Robot Operator Interface
Exec={launcher_sh} %F
Icon={icon_path if icon_path.exists() else ''}
Terminal=false
Categories=Utility;Science;
"""
        
        with open(desktop_file, 'w') as f:
            f.write(desktop_content)
        
        os.chmod(desktop_file, 0o644)
        print(f"Created desktop entry: {desktop_file}")
    
    def create_launcher_files(self):
        """Create platform-specific launcher files"""
        print("\nCreating launcher files...")
        
        if self.is_windows:
            self.create_windows_shortcut()
        elif self.is_mac:
            self.create_mac_launcher()
        elif self.is_linux:
            self.create_linux_launcher()
        else:
            print(f"WARNING: Unsupported platform: {self.platform}")
    
    def run_operator(self):
        """Run the operator interface"""
        gui_script = self.project_root / "mac" / "gui_master.py"
        
        if not gui_script.exists():
            print(f"ERROR: GUI script not found: {gui_script}")
            return False
        
        print("\nStarting operator interface...")
        try:
            subprocess.run([sys.executable, str(gui_script)] + sys.argv[1:])
            return True
        except KeyboardInterrupt:
            print("\nInterrupted by user")
            return False
        except Exception as e:
            print(f"ERROR: Failed to start operator interface: {e}")
            return False
    
    def setup(self):
        """Main setup routine"""
        print("=" * 60)
        print("Rescue Robot Operator - Cross-Platform Launcher")
        print("=" * 60)
        
        # Check dependencies
        if not self.check_dependencies():
            return False
        
        # Create launcher files
        self.create_launcher_files()
        
        return True


def main():
    """Main entry point"""
    launcher = CrossPlatformLauncher()
    
    # Check if just creating launcher files
    if len(sys.argv) > 1 and sys.argv[1] == "--setup-only":
        launcher.setup()
        return
    
    # Setup and run
    if launcher.setup():
        response = input("\nStart operator interface now? (y/n): ").strip().lower()
        if response == 'y':
            launcher.run_operator()
    else:
        print("Setup failed. Please fix errors and try again.")
        sys.exit(1)


if __name__ == "__main__":
    main()