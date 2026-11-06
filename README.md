# Robô Móvel Autônomo - Plataforma BitDogLab (RP2040)

Este repositório contém os drivers, firmware, scripts de diagnóstico, medição de sensores e calibração desenvolvidos para a plataforma robótica educacional **BitDogLab (Raspberry Pi Pico / RP2040)** da **FEEC / UNICAMP**.

## 📁 Estrutura do Repositório

* **`drivers/`**: Módulos de controle do hardware (display OLED SH1107).
* **`diagnostics/`**: Testes da ponte H, motores DC, torque e rampa PWM.
* **`calibration/`**: Calibração de translação, giros de 90° em malha fechada e telemetria.
* **`sensors/`**: Log de cores RGB/Clear e detecção de linhas no piso (TCS34725).
* **`src/`**: Firmware principal de navegação (`main.py`).
