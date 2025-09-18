# PID Motor Simulator

This repository contains a Python-based PID motor simulator with a PyQt6 GUI, designed to demonstrate PID control for a motor's RPM (Revolutions Per Minute). The simulator models a motor with a wheel, incorporates a Kalman filter to handle noisy RPM measurements, and allows for dynamic or static disturbances. The GUI displays a rotating wheel, an error plot (desired vs. actual RPM), an RPM plot, and provides controls to adjust PID gains, friction, noise, sensor frequency, and disturbance parameters. Users can also save/load configurations, pause/resume the simulation, reset it, and enable negative RPM for reverse rotation.

## Features
- **PID Controller**: Adjust Kp, Ki, Kd gains with high precision (up to 5 decimal places).
- **Kalman Filter**: Filters noisy RPM measurements from a simulated sensor.
- **GUI**: Built with PyQt6, showing a rotating wheel, error plot, RPM plot (desired, estimated, and true RPM), and actual RPM display.
- **Controls**: Adjust target RPM (0 to 5000, or -5000 to 5000 if negative RPM is enabled), friction, sensor noise, sensor frequency, and disturbance settings.
- **Disturbances**: Toggle static or dynamic (sinusoidal) disturbances with adjustable amplitude and frequency.
- **Configuration**: Save/load parameters to/from a config file (default: `config.cfg`).
- **Simulation Controls**: Start/stop, reset, and allow negative RPM via a checkbox.
- **Plots**: Real-time error and RPM plots using Matplotlib.

## Prerequisites
The simulator requires Python 3.8 or higher and the following packages:
- `pyqt6`
- `numpy`
- `scipy`
- `matplotlib`

## Installation

Below are instructions for installing dependencies and running the program on **Mac**, **Linux**, and **Windows** using different Python environment managers: **venv**, **Conda**, or **Anaconda**.

### Option 1: Using `venv` (Virtual Environment)
#### Mac/Linux
1. **Create and activate a virtual environment**:
   ```bash
   python3 -m venv pid_sim_env
   source pid_sim_env/bin/activate
   ```
2. **Install dependencies**:
   ```bash
   pip install pyqt6 numpy scipy matplotlib
   ```
3. **Run the program**:
   ```bash
   python main.py
   ```

#### Windows
1. **Create and activate a virtual environment**:
   ```cmd
   python -m venv pid_sim_env
   pid_sim_env\Scripts\activate
   ```
2. **Install dependencies**:
   ```cmd
   pip install pyqt6 numpy scipy matplotlib
   ```
3. **Run the program**:
   ```cmd
   python main.py
   ```

### Option 2: Using Conda
#### Mac/Linux/Windows
1. **Install Conda** (if not already installed):
   - Download and install Miniconda from [https://docs.conda.io/en/latest/miniconda.html](https://docs.conda.io/en/latest/miniconda.html).
   - Follow the installation instructions for your platform.
2. **Create and activate a Conda environment**:
   ```bash
   conda create -n pid_sim_env python=3.8
   conda activate pid_sim_env
   ```
3. **Install dependencies**:
   ```bash
   conda install numpy scipy matplotlib
   pip install pyqt6
   ```
   Note: `pyqt6` is installed via `pip` as it may not be available in Conda channels.
4. **Run the program**:
   ```bash
   python main.py
   ```

### Option 3: Using Anaconda
#### Mac/Linux/Windows
1. **Install Anaconda** (if not already installed):
   - Download and install Anaconda from [https://www.anaconda.com/products/distribution](https://www.anaconda.com/products/distribution).
   - Follow the installation instructions for your platform.
2. **Create and activate an Anaconda environment**:
   ```bash
   conda create -n pid_sim_env python=3.8
   conda activate pid_sim_env
   ```
3. **Install dependencies**:
   ```bash
   conda install numpy scipy matplotlib
   pip install pyqt6
   ```
4. **Run the program**:
   ```bash
   python main.py
   ```

## Running the Program
Clone or download this repository to your local machine:
```bash
git clone <repository-url>
cd <repository-directory>
```

### Running on Different Platforms
- **Mac/Linux**:
  - After setting up the environment (as above), run:
    ```bash
    python main.py
    ```
  - If using Conda/Anaconda, ensure the environment is activated (`conda activate pid_sim_env`).
- **Windows**:
  - Using **CMD**:
    - Activate the environment (e.g., `pid_sim_env\Scripts\activate` for `venv` or `conda activate pid_sim_env` for Conda/Anaconda).
    - Navigate to the repository directory:
      ```cmd
      cd path\to\repository
      ```
    - Run:
      ```cmd
      python main.py
      ```
  - Using **PowerShell**:
    - Activate the environment (e.g., `.\pid_sim_env\Scripts\Activate.ps1` for `venv` or `conda activate pid_sim_env`).
    - Run:
      ```powershell
      python main.py
      ```

### Running in IDEs
#### Visual Studio Code (VSCode) with Anaconda
1. Install VSCode: [https://code.visualstudio.com/](https://code.visualstudio.com/).
2. Install the Python extension in VSCode.
3. Set up the Conda environment (as above).
4. In VSCode:
   - Open the repository folder (`File > Open Folder`).
   - Select the Python interpreter:
     - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac) to open the Command Palette.
     - Type `Python: Select Interpreter` and choose the `pid_sim_env` environment (e.g., `~/miniconda3/envs/pid_sim_env/bin/python` or similar).
   - Open `main.py`, click the "Run" button (top-right triangle), or press `F5`.
   - Alternatively, open a terminal in VSCode (`Terminal > New Terminal`), ensure the environment is activated, and run:
     ```bash
     python main.py
     ```

#### Spyder with Anaconda
1. Install Anaconda (as above).
2. Install Spyder in the environment:
   ```bash
   conda activate pid_sim_env
   conda install spyder
   ```
3. Launch Spyder:
   ```bash
   spyder
   ```
4. In Spyder:
   - Open `main.py` from the repository directory.
   - Ensure the environment is set to `pid_sim_env` (check the Python interpreter in the status bar).
   - Click the green "Run" button or press `F5`.

## Notes for Windows Users
- Ensure you have a compatible Python version (3.8 or higher). Windows may require additional steps for GUI applications:
  - If PyQt6 fails to render properly, ensure your graphics drivers are updated.
  - If you encounter issues with `pip install pyqt6`, try:
    ```cmd
    pip install pyqt6 --user
    ```
- If Conda commands fail in CMD, ensure Anaconda is added to your PATH or use the Anaconda Prompt.

## Project Structure
- `main.py`: Entry point to run the simulator.
- `gui.py`: Contains the GUI implementation (PyQt6) and simulation logic.
- `pid.py`: PID controller implementation.
- `ekf.py`: Kalman filter implementation for noise filtering.
- `util.py`: Utility constants.
- `datanuff.py`: Placeholder for data buffering (not used in current implementation).

## Usage
1. Run `main.py` to launch the GUI.
2. Adjust parameters:
   - **Target RPM**: Set via slider or spin box (0 to 5000, or -5000 to 5000 if "Allow Negative RPM" is checked).
   - **PID Gains (Kp, Ki, Kd)**: Tune with 5-decimal precision.
   - **Friction (a)**: Motor friction coefficient.
   - **Noise Std**: Sensor noise standard deviation.
   - **Sensor Freq**: Measurement frequency (Hz).
   - **Disturbance**: Enable/disable, choose static/dynamic, set value/amplitude and frequency.
3. Control the simulation:
   - **Start/Stop**: Toggle simulation.
   - **Reset**: Reset to initial state (0 RPM).
   - **Save Config**: Save parameters to a `.cfg` file (default: `config.cfg`).
4. Observe the rotating wheel, error plot, RPM plot, and actual RPM display.

## Troubleshooting
- **ModuleNotFoundError**: Ensure all dependencies are installed in the active environment.
- **GUI not displaying**: Verify PyQt6 is installed correctly and your system supports Qt rendering.
- **Windows-specific issues**: Use Anaconda Prompt for Conda commands, or ensure `python` and `pip` point to the correct environment.