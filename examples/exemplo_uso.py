# exemplo_uso.py
# Exemplo de como usar o carrinho depois de calibrar com calibrar_carrinho.py
# (usa os fatores salvos em calibracao_imu.json).

import time
from imu_calibracao import CarrinhoIMU

robo = CarrinhoIMU()
print('Calibracao carregada:', robo.cal)

# Anda 50 cm para frente (aperte B para parada de emergencia)
dist = robo.andar_cm(50)
print('Distancia medida: {:.1f} cm'.format(dist))
time.sleep_ms(500)

# Gira 90 graus no sentido horario (negativo = anti-horario)
ang = robo.girar_graus(90)
print('Angulo medido: {:.1f} deg'.format(ang))

