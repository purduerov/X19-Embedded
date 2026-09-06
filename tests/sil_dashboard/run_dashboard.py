#!/usr/bin/env python3
"""
Launcher shortcut for X19 SIL Streamlit Testing Dashboard.
"""
import os
import sys
import subprocess

def main():
    dashboard_script = os.path.join(os.path.dirname(__file__), "dashboard_app.py")
    cmd = [sys.executable, "-m", "streamlit", "run", dashboard_script, "--server.port=8501", "--server.headless=false"]
    print(f"Starting X19 SIL Testing Dashboard on http://localhost:8501...")
    subprocess.run(cmd)

if __name__ == "__main__":
    main()
