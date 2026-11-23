# main.py
# Inicia a calibracao automaticamente ao ligar o carrinho na bateria.
# Aguarda 1,5 s no boot para facilitar reconexao via USB/MicroPico depois.

import time
import sys
from machine import Pin
import calibrar_carrinho
from imu_calibracao import Motores

# Pisca o LED da Pico W para indicar que iniciou na bateria
try:
    led = Pin('LED', Pin.OUT)
    for _ in range(3):
        led.value(1)
        time.sleep_ms(150)
        led.value(0)
        time.sleep_ms(150)
except Exception:
    pass

time.sleep_ms(600)

try:
    calibrar_carrinho.main()
except KeyboardInterrupt:
    Motores().parar()
    print('Interrompido.')
except Exception as e:
    Motores().parar()
    print('ERRO no main.py:', repr(e))
    sys.print_exception(e)
finally:
    Motores().parar()
