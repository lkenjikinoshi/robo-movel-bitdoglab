from machine import Pin, I2C
from machine import Pin, PWM, I2C, ADC, disable_irq, enable_irq
from time import sleep, sleep_ms, ticks_ms, ticks_diff
import math
import neopixel
from time import sleep_ms
from time import sleep_ms
from machine import Pin
from machine import PWM
import time

# Bloco de Setup
MPU6050_ADDR = 0x68
MPU6050_WHO_AM_I = 0x75
MPU6050_ID = 0x68
class MPU6050:
  def __init__(self, i2c, addr=MPU6050_ADDR, quiet=False):
    self.i2c = i2c
    self.addr = addr
    self.quiet = quiet
    self.offset_z = 0.0
    self.is_ready = False
    self.last_error = None
    self.identity = None
    self.initialize()
  def initialize(self):
    self.is_ready = False
    try:
      # Preserve the robot's original startup sequence: wake/configure first.
      # Some MPU6050 boards return an invalid WHO_AM_I value for a short time
      # after power-up (and compatible clones may expose another ID), so this
      # diagnostic must never prevent the gyro from becoming ready.
      self.i2c.writeto_mem(self.addr, 0x6B, b"\x00")
      self.i2c.writeto_mem(self.addr, 0x1B, b"\x00")
      self.i2c.writeto_mem(self.addr, 0x1C, b"\x18")
      try:
        self.identity = self.i2c.readfrom_mem(self.addr, MPU6050_WHO_AM_I, 1)[0]
      except Exception:
        self.identity = None
      self.last_error = None
      self.is_ready = True
    except Exception as exc:
      self.last_error = exc
      if not self.quiet:
        print("MPU6050: erro ao inicializar:", exc)
    return self.is_ready
  def set_i2c(self, i2c):
    self.i2c = i2c
  @staticmethod
  def _decode_i16(data, offset):
    value = (data[offset] << 8) | data[offset + 1]
    return value - 65536 if value > 32767 else value
  def _read_i16(self, reg):
    data = self.i2c.readfrom_mem(self.addr, reg, 2)
    return self._decode_i16(data, 0)
  def _gyro_dps(self, reg):
    return self._read_i16(reg) / 131.0
  def _accel_g(self, reg):
    return self._read_i16(reg) / 2048.0
  def gz(self):
    if not self.is_ready:
      return 0.0
    try:
      return self._gyro_dps(0x47) - self.offset_z
    except Exception as exc:
      self.last_error = exc
      self.is_ready = False
      return 0.0
  def acceleration(self):
    if not self.is_ready:
      return None
    try:
      data = self.i2c.readfrom_mem(self.addr, 0x3B, 6)
      return (
        self._decode_i16(data, 0) / 2048.0,
        self._decode_i16(data, 2) / 2048.0,
        self._decode_i16(data, 4) / 2048.0,
      )
    except Exception as exc:
      self.last_error = exc
      self.is_ready = False
      return None
  def ax(self):
    return self._accel_g(0x3B) if self.is_ready else 0.0
  def ay(self):
    return self._accel_g(0x3D) if self.is_ready else 0.0
  def az(self):
    return self._accel_g(0x3F) if self.is_ready else 0.0
  def calibrate(self, samples=300, delay=5):
    if not self.is_ready:
      return False
    try:
      total = 0.0
      for _ in range(samples):
        total += self._gyro_dps(0x47)
        sleep_ms(delay)
      self.offset_z = total / samples
      return True
    except Exception as exc:
      self.last_error = exc
      self.is_ready = False
      return False
INA226_ADDR = 0x40
CONFIG_REG = 0x00
SHUNT_VOLTAGE_REG = 0x01
BUS_VOLTAGE_REG = 0x02
class INA226:
  def __init__(self, i2c, addr=0x40, shunt_resistor=0.1):
    self.i2c = i2c
    self.addr = addr
    self.shunt_resistor = shunt_resistor
    self.is_ready = False
    try:
      self.configure()
      self.is_ready = True
    except Exception as exc:
      print("INA226: erro ao inicializar:", exc)
  def _write_register(self, reg, value):
    self.i2c.writeto_mem(self.addr, reg, bytearray([(value >> 8) & 0xFF, value & 0xFF]))
  def _read_register(self, reg):
    data = self.i2c.readfrom_mem(self.addr, reg, 2)
    return (data[0] << 8) | data[1]
  def _read_signed_register(self, reg):
    value = self._read_register(reg)
    return value - 65536 if value > 32767 else value
  def configure(self):
    self._write_register(CONFIG_REG, 0x4127)
  def voltage(self):
    if not self.is_ready: return 0.0
    return self._read_register(BUS_VOLTAGE_REG) * 1.25 / 1000
  def current(self):
    if not self.is_ready or self.shunt_resistor == 0: return 0.0
    shunt_voltage = self._read_signed_register(SHUNT_VOLTAGE_REG) * 2.5 / 1000000
    return -(shunt_voltage / self.shunt_resistor)
def _robo_ler_bateria():
  if _robo_ina226 is None or not _robo_ina226.is_ready:
    return None
  try:
    return _robo_ina226.voltage()
  except Exception as exc:
    print("INA226: falha na leitura:", exc)
    return None
def _robo_tensao_bateria():
  tensao = _robo_ler_bateria()
  return 0.0 if tensao is None else tensao
def _robo_corrente_robo():
  if _robo_ina226 is None or not _robo_ina226.is_ready:
    return 0.0
  try:
    return _robo_ina226.current()
  except Exception as exc:
    print("INA226: falha na leitura de corrente:", exc)
    return 0.0
np = neopixel.NeoPixel(Pin(7), 25)  # Matriz 5x5 do robo
led_vermelho = PWM(Pin(13), freq=1000)
led_verde = PWM(Pin(11), freq=1000)
led_azul = PWM(Pin(12), freq=1000)
class _RoboMissaoCancelada(Exception):
  pass
def _robo_cancelar_por_botao(_pin):
  global _robo_cancelado_setas
  _robo_cancelado_setas = True
  _robo_stby.value(0)
def _robo_verificar_parada():
  if _robo_controle_setas and (_robo_cancelado_setas or _robo_botao_b.value() == 0):
    _robo_parar()
    raise _RoboMissaoCancelada()
def _robo_habilitar_motores():
  estado_irq = disable_irq()
  try:
    _robo_verificar_parada()
    _robo_stby.value(1)
  finally:
    enable_irq(estado_irq)
def _robo_pausa_segura(ms):
  inicio = ticks_ms()
  while ticks_diff(ticks_ms(), inicio) < ms:
    _robo_verificar_parada()
    sleep_ms(min(10, max(1, ms - ticks_diff(ticks_ms(), inicio))))
  _robo_verificar_parada()
def _robo_aguardar_botao_a(valor):
  while True:
    while _robo_botao_a.value() != valor:
      _robo_pausa_segura(10)
    _robo_pausa_segura(30)
    if _robo_botao_a.value() == valor:
      return
def _robo_encerrar_setas():
  global _robo_controle_setas, _robo_diagnostico_ok
  _robo_parar()
  if _robo_botao_b is not None:
    _robo_botao_b.irq(handler=None)
  _robo_controle_setas = False
  _robo_diagnostico_ok = False
  _robo_led(0, 0, 0)
  _robo_mostrar_contagem(0)
def _robo_pwm(valor):
  return max(0, min(65535, int(valor)))
def _robo_led(r, g, b):
  led_vermelho.duty_u16(45000 if r else 0)
  led_verde.duty_u16(45000 if g else 0)
  led_azul.duty_u16(45000 if b else 0)
def _robo_alerta(cor, segunda_cor=None):
  _robo_parar()
  while True:
    _robo_led(*cor)
    _robo_pausa_segura(500)
    _robo_led(*(segunda_cor if segunda_cor is not None else (0, 0, 0)))
    _robo_pausa_segura(500)
def _robo_parar():
  _robo_stby.value(0)
  _robo_esq_frente.value(0)
  _robo_esq_tras.value(0)
  _robo_esq_pwm.duty_u16(0)
  _robo_dir_frente.value(0)
  _robo_dir_tras.value(0)
  _robo_dir_pwm.duty_u16(0)
def _robo_mover_reto(esq_frente, esq_tras, dir_frente, dir_tras, tempo):
  if not _robo_diagnostico_ok:
    _robo_parar()
    return
  t = max(0, float(tempo))
  _robo_parar()
  _robo_habilitar_motores()
  if t <= 0:
    return
  _robo_pausa_segura(50)
  _robo_esq_frente.value(esq_frente)
  _robo_esq_tras.value(esq_tras)
  _robo_dir_frente.value(dir_frente)
  _robo_dir_tras.value(dir_tras)
  _robo_esq_pwm.duty_u16(_robo_pwm(_robo_vel_movimento))
  _robo_dir_pwm.duty_u16(_robo_pwm(_robo_vel_movimento))
  _robo_esperar_movimento(t)
  _robo_parar()
def _robo_pivot_esq(velocidade):
  if not _robo_diagnostico_ok:
    _robo_parar()
    return
  d = _robo_pwm(velocidade)
  _robo_habilitar_motores()
  _robo_esq_frente.value(0)
  _robo_esq_tras.value(1)
  _robo_esq_pwm.duty_u16(d)
  _robo_dir_frente.value(1)
  _robo_dir_tras.value(0)
  _robo_dir_pwm.duty_u16(d)
def _robo_pivot_dir(velocidade):
  if not _robo_diagnostico_ok:
    _robo_parar()
    return
  d = _robo_pwm(velocidade)
  _robo_habilitar_motores()
  _robo_esq_frente.value(1)
  _robo_esq_tras.value(0)
  _robo_esq_pwm.duty_u16(d)
  _robo_dir_frente.value(0)
  _robo_dir_tras.value(1)
  _robo_dir_pwm.duty_u16(d)
def _robo_frente(tempo):
  _robo_mover_reto(0, 1, 0, 1, tempo)
def _robo_tras(tempo):
  if not _robo_diagnostico_ok:
    _robo_parar()
    return
  t = max(0, float(tempo))
  if t <= 0:
    _robo_parar()
    return
  _robo_habilitar_motores()
  _robo_esq_frente.value(1)
  _robo_esq_tras.value(0)
  _robo_esq_pwm.duty_u16(_robo_pwm(_robo_vel_movimento))
  _robo_dir_frente.value(1)
  _robo_dir_tras.value(0)
  _robo_dir_pwm.duty_u16(_robo_pwm(_robo_vel_movimento))
  _robo_esperar_movimento(t)
  _robo_parar()
def _robo_mostrar_contagem(numero):
  padrao = _robo_numeros_contagem.get(int(numero))
  for i in range(25):
    np[i] = (0, 0, 0)
  if padrao is not None:
    for y in range(5):
      for x in range(5):
        if padrao[y * 5 + x]:
          np[_robo_matriz_indices[y][x]] = _robo_cor_contagem
  np.write()
def _robo_aguardar_com_contagem(espera):
  restante = max(0.0, float(espera))
  while restante > 0:
    _robo_mostrar_contagem(min(5, int(math.ceil(restante))))
    intervalo = min(1.0, restante)
    _robo_pausa_segura(int(intervalo * 1000))
    restante -= intervalo
  _robo_mostrar_contagem(0)
def _robo_inicializar(espera=5):
  global _robo_pronto, _robo_diagnostico_ok, _robo_angulo, _robo_giro_tempo
  _robo_diagnostico_ok = False
  _robo_parar()
  _robo_led(0, 0, 0)
  if espera > 0:
    print("Preparando robo. Aguarde", espera, "s" if _robo_controle_setas else "s para iniciar")
    _robo_aguardar_com_contagem(espera)
  falha_mpu = _robo_mpu is None or not _robo_mpu.is_ready
  if falha_mpu:
    if _robo_mpu_sda_alt is None:
      print("MPU6050 nao encontrado. Verifique SDA=GP{} e SCL=GP{}.".format(_robo_mpu_sda, _robo_mpu_scl))
    else:
      print("MPU6050 nao encontrado. Verifique GP{}/GP{} ou GP{}/GP{}.".format(_robo_mpu_sda, _robo_mpu_scl, _robo_mpu_sda_alt, _robo_mpu_scl_alt))
    _robo_pronto = False
  else:
    print("MPU6050 encontrado em", _robo_mpu_bus)
    print("Calibrando giro. Nao mexa no robo.")
    _robo_pronto = _robo_mpu.calibrate()
    falha_mpu = not _robo_pronto
  falha_i2c = falha_mpu or not globals().get("_robo_display_ok", True)
  _robo_angulo = 0.0
  _robo_giro_tempo = ticks_ms()
  tensao = _robo_ler_bateria()
  print("Bateria:", "sem leitura" if tensao is None else "{:.2f} V".format(tensao))
  if tensao is None:
    print("Diagnostico: INA226 indisponivel; robo parado.")
    _robo_alerta((1, 1, 0))
  elif tensao < 3.6 and falha_i2c:
    print("Diagnostico: bateria baixa e falha I2C; robo parado.")
    _robo_alerta((1, 0, 0), (1, 1, 0))
  elif tensao < 3.6:
    print("Diagnostico: bateria abaixo de 3.6 V; robo parado.")
    _robo_alerta((1, 0, 0))
  elif falha_i2c:
    print("Diagnostico: MPU ou OLED indisponivel; robo parado.")
    _robo_alerta((1, 1, 0))
  _robo_led(0, 1, 0)
  print("Robo pronto! Aguarde o aviso para apertar A." if _robo_controle_setas else "Robo pronto! Iniciando em 2 s.")
  _robo_pausa_segura(2000)
  _robo_led(0, 0, 0)
  _robo_diagnostico_ok = True
def _robo_iniciar_setas(espera=5):
  global _robo_orientacao_setas, _robo_ultima_direcao_setas, _robo_falha_setas
  global _robo_botao_a, _robo_botao_b, _robo_controle_setas, _robo_cancelado_setas, _robo_giro_tempo
  _robo_parar()
  _robo_botao_a = Pin(5, Pin.IN, Pin.PULL_UP)
  _robo_botao_b = Pin(6, Pin.IN, Pin.PULL_UP)
  _robo_cancelado_setas = False
  _robo_controle_setas = True
  _robo_botao_b.irq(handler=_robo_cancelar_por_botao, trigger=Pin.IRQ_FALLING, hard=True)
  _robo_verificar_parada()
  _robo_orientacao_setas = 0
  _robo_ultima_direcao_setas = None
  _robo_falha_setas = False
  _robo_inicializar(espera)
  print("Aperte e solte A para iniciar. B cancela a missao.")
  _robo_led(0, 1, 0)
  _robo_aguardar_botao_a(1)
  _robo_aguardar_botao_a(0)
  _robo_aguardar_botao_a(1)
  _robo_led(0, 0, 0)
  _robo_giro_tempo = ticks_ms()
def _robo_finalizar_setas():
  global _robo_ultima_direcao_setas
  _robo_parar()
  _robo_ultima_direcao_setas = None
def _robo_pivot_setas(direcao, velocidade_esq, velocidade_dir):
  de = _robo_pwm(velocidade_esq)
  dd = _robo_pwm(velocidade_dir)
  _robo_habilitar_motores()
  if direcao == "L":
    _robo_esq_frente.value(0)
    _robo_esq_tras.value(1)
    _robo_dir_frente.value(1)
    _robo_dir_tras.value(0)
  else:
    _robo_esq_frente.value(1)
    _robo_esq_tras.value(0)
    _robo_dir_frente.value(0)
    _robo_dir_tras.value(1)
  _robo_esq_pwm.duty_u16(de)
  _robo_dir_pwm.duty_u16(dd)
def _robo_girar(graus, direcao="L", setas=False):
  global _robo_angulo, _robo_giro_tempo
  if not _robo_diagnostico_ok:
    _robo_parar()
    return False if setas else None
  if not _robo_pronto:
    _robo_inicializar(0)
  if not _robo_mpu.is_ready:
    _robo_parar()
    if setas:
      print("Giro por setas cancelado: MPU6050 indisponivel.")
    return False if setas else None
  alvo = abs(float(graus))
  if alvo <= 0:
    _robo_parar()
    return True if setas else None
  direcao = "L" if direcao == "L" else "R"
  if direcao == "L":
    pwm_esq = _robo_pwm_giro_avanco_setas
    pwm_dir = _robo_pwm_giro_re_setas
  else:
    pwm_esq = _robo_pwm_giro_re_setas
    pwm_dir = _robo_pwm_giro_avanco_setas
  acumulado = 0.0
  inicio = ticks_ms()
  t_ant = inicio
  limite_ms = max(_robo_timeout_min_ms, int(alvo * _robo_timeout_ms_por_grau))
  _robo_pausa_segura(10)
  while acumulado < alvo and ticks_diff(ticks_ms(), inicio) < limite_ms:
    _robo_verificar_parada()
    agora = ticks_ms()
    dt = min(ticks_diff(agora, t_ant) / 1000.0, 0.05)
    t_ant = agora
    gz = _robo_mpu.gz() * _robo_sinal_giro_z
    if setas and not _robo_mpu.is_ready:
      break
    if abs(gz) < _robo_zona_morta_giro:
      gz = 0.0
    delta = gz * dt if direcao == "L" else -gz * dt
    if delta > 0:
      acumulado += delta
    _robo_angulo += gz * dt
    if setas and acumulado >= alvo:
      break
    if setas:
      _robo_pivot_setas(direcao, pwm_esq, pwm_dir)
    elif direcao == "L":
      _robo_pivot_esq(_robo_vel_giro)
    else:
      _robo_pivot_dir(_robo_vel_giro)
    _robo_atualizar_display_instrumentos(False)
    _robo_pausa_segura(10)
  _robo_parar()
  _robo_giro_tempo = ticks_ms()
  if not setas:
    _robo_pausa_segura(200)
    print("Giro", "esquerda" if direcao == "L" else "direita", round(acumulado, 1), "graus")
    return
  sucesso = _robo_mpu.is_ready and acumulado >= alvo
  _robo_pausa_segura(_robo_pausa_apos_giro_setas_ms)
  if sucesso:
    print("Giro por setas", "esquerda" if direcao == "L" else "direita", round(acumulado, 1), "graus")
  elif not _robo_mpu.is_ready:
    print("Giro interrompido: falha de leitura do MPU6050. Avanco cancelado.")
  else:
    print("Giro interrompido: limite de", limite_ms, "ms atingido; angulo", round(acumulado, 1), "graus. Avanco cancelado.")
  return sucesso
def _robo_ir_para(direcao):
  _robo_verificar_parada()
  global _robo_orientacao_setas, _robo_ultima_direcao_setas, _robo_falha_setas
  if not _robo_diagnostico_ok:
    _robo_parar()
    return False
  if _robo_falha_setas:
    _robo_parar()
    return False
  direcao = int(direcao) % 4
  if _robo_ultima_direcao_setas == direcao:
    _robo_pausa_segura(_robo_pausa_setas_repetidas_ms)
  giro = direcao - _robo_orientacao_setas
  if giro < 0:
    giro += 4
  giro_ok = True
  if giro != 0:
    _robo_parar()
    _robo_pausa_segura(_robo_pausa_antes_giro_setas_ms)
  if giro == 1:
    giro_ok = _robo_girar(90, "R", True)
  elif giro == 2:
    giro_ok = _robo_girar(180, "R", True)
  elif giro == 3:
    giro_ok = _robo_girar(90, "L", True)
  if not giro_ok:
    _robo_falha_setas = True
    _robo_parar()
    return False
  _robo_frente(_robo_tempo_bloco_setas)
  _robo_orientacao_setas = direcao
  _robo_ultima_direcao_setas = direcao
  return True
def _robo_giro():
  global _robo_angulo, _robo_giro_tempo
  if not _robo_pronto or not _robo_mpu.is_ready:
    return _robo_angulo
  agora = ticks_ms()
  dt = min(ticks_diff(agora, _robo_giro_tempo) / 1000.0, 0.05)
  _robo_giro_tempo = agora
  gz = _robo_mpu.gz() * _robo_sinal_giro_z
  if abs(gz) < _robo_zona_morta_giro:
    gz = 0.0
  _robo_angulo += gz * dt
  return _robo_angulo
def _robo_aceleracao_x():
  if not _robo_pronto or not _robo_mpu.is_ready:
    return 0.0
  return _robo_mpu.ax() * 9.80665
def _robo_aceleracao_y():
  if not _robo_pronto or not _robo_mpu.is_ready:
    return 0.0
  return _robo_mpu.ay() * 9.80665
def _robo_aceleracao_z():
  if not _robo_pronto or not _robo_mpu.is_ready:
    return 0.0
  return _robo_mpu.az() * 9.80665
def _robo_esperar_movimento(tempo):
  duracao_ms = int(max(0, float(tempo)) * 1000)
  inicio = ticks_ms()
  while ticks_diff(ticks_ms(), inicio) < duracao_ms:
    _robo_verificar_parada()
    _robo_atualizar_display_instrumentos(True)
    _robo_pausa_segura(40)
def _robo_atualizar_display_instrumentos(atualizar_giro=True):
  global _robo_display_giro_ultimo_ms
  if not _robo_display_giro_ativo and not _robo_display_acel_x_ativo and not _robo_display_acel_y_ativo and not _robo_display_acel_z_ativo and not _robo_display_transferidor_ativo and not _robo_display_tensao_bateria_ativo and not _robo_display_corrente_robo_ativo:
    return
  agora = ticks_ms()
  if ticks_diff(agora, _robo_display_giro_ultimo_ms) < 200:
    return
  _robo_display_giro_ultimo_ms = agora
  try:
    if _robo_display_transferidor_ativo:
      valor_giro = _robo_giro() if atualizar_giro else _robo_angulo
      _robo_desenhar_transferidor_360(valor_giro)
      return
    if _robo_display_giro_ativo:
      valor_giro = _robo_giro() if atualizar_giro else _robo_angulo
      _robo_escrever_valor_display(valor_giro, _robo_display_giro_linha_y, _robo_display_giro_alinhamento, "")
    if _robo_display_acel_x_ativo:
      _robo_escrever_valor_display(_robo_aceleracao_x(), _robo_display_acel_x_linha_y, _robo_display_acel_x_alinhamento, "m/s2", 1)
    if _robo_display_acel_y_ativo:
      _robo_escrever_valor_display(_robo_aceleracao_y(), _robo_display_acel_y_linha_y, _robo_display_acel_y_alinhamento, "m/s2", 1)
    if _robo_display_acel_z_ativo:
      _robo_escrever_valor_display(_robo_aceleracao_z(), _robo_display_acel_z_linha_y, _robo_display_acel_z_alinhamento, "m/s2", 1)
    if _robo_display_tensao_bateria_ativo:
      _robo_escrever_valor_display(_robo_tensao_bateria(), _robo_display_tensao_bateria_linha_y, _robo_display_tensao_bateria_alinhamento, " V", 2)
    if _robo_display_corrente_robo_ativo:
      _robo_escrever_valor_display(_robo_corrente_robo(), _robo_display_corrente_robo_linha_y, _robo_display_corrente_robo_alinhamento, " A", 2)
    oled.show()
  except Exception:
    pass
def _robo_escrever_valor_display(valor, linha_y, alinhamento, sufixo="", casas=None):
  if casas is None:
    texto = str(round(valor, 4)) + sufixo
  else:
    texto = str(round(valor, casas)) + sufixo
  if alinhamento == "LEFT":
    x = 3
    x_clear = 3
  elif alinhamento == "RIGHT":
    x = max(64, 128 - len(texto) * 8)
    x_clear = 64
  else:
    x = max(3, (128 - len(texto) * 8) // 2)
    x_clear = max(3, x - 32)
  oled.fill_rect(x_clear, linha_y, 128 - x_clear, 8, 0)
  oled.text(texto, x, linha_y, 1)
def _robo_desenhar_transferidor_360(valor_giro):
  angulo = valor_giro % 360
  texto = "{:.1f} graus".format(angulo)
  cx = _display_width // 2
  cy = min(_display_height - 26, max(34, _display_height // 2 + 8))
  raio = min(26, max(18, min(_display_width, _display_height) // 3))
  oled.fill(0)
  oled.text("Giro 360", max(0, (_display_width - 64) // 2), 0, 1)
  oled.text(texto, max(0, (_display_width - len(texto) * 8) // 2), 10, 1)
  for a in range(0, 360, 10):
    rad = math.radians(a - 90)
    x = int(cx + math.cos(rad) * raio)
    y = int(cy + math.sin(rad) * raio)
    oled.pixel(x, y, 1)
  for a in (0, 90, 180, 270):
    rad = math.radians(a - 90)
    x1 = int(cx + math.cos(rad) * (raio - 4))
    y1 = int(cy + math.sin(rad) * (raio - 4))
    x2 = int(cx + math.cos(rad) * (raio + 2))
    y2 = int(cy + math.sin(rad) * (raio + 2))
    oled.line(x1, y1, x2, y2, 1)
  rad = math.radians(angulo - 90)
  px = int(cx + math.cos(rad) * (raio - 5))
  py = int(cy + math.sin(rad) * (raio - 5))
  oled.line(cx, cy, px, py, 1)
  oled.pixel(cx, cy, 1)
  oled.pixel(cx + 1, cy, 1)
  oled.pixel(cx - 1, cy, 1)
  oled.pixel(cx, cy + 1, 1)
  oled.pixel(cx, cy - 1, 1)
  oled.show()
def _feedback_check():
  if "_robo_verificar_parada" in globals():
    _robo_verificar_parada()
def _feedback_wait(ms):
  if "_robo_pausa_segura" in globals():
    _robo_pausa_segura(ms)
  else:
    sleep_ms(ms)
def _feedback_led_off():
  led_vermelho.duty_u16(0)
  led_verde.duty_u16(0)
  led_azul.duty_u16(0)
def _feedback_led(colour):
  _feedback_check()
  led_vermelho.duty_u16(int(colour[0] * 257 * 30 / 100))
  led_verde.duty_u16(int(colour[1] * 257 * 30 / 100))
  led_azul.duty_u16(int(colour[2] * 257 * 30 / 100))
LED_MATRIX = [[24,23,22,21,20],[15,16,17,18,19],[14,13,12,11,10],[5,6,7,8,9],[4,3,2,1,0]]
def _feedback_matrix_clear():
  for i in range(25):
    np[i] = (0, 0, 0)
  np.write()
def _feedback_matrix(pattern, colour, brightness, name):
  global _matriz_status, _matriz_desenho, _matriz_cor, _matriz_brilho, _matriz_leds_count
  _feedback_check()
  for i in range(25):
    np[i] = (0, 0, 0)
  rgb = tuple(int(value * brightness * 0.7 / 100) for value in colour)
  for y in range(5):
    for x in range(5):
      if pattern[y * 5 + x]:
        np[LED_MATRIX[y][x]] = rgb
  np.write()
  _matriz_status = "ON"
  _matriz_desenho = name
  _matriz_cor = colour
  _matriz_brilho = brightness
  _matriz_leds_count = sum(1 for i in range(25) if np[i] != (0, 0, 0))
buzzer = PWM(Pin(21))
def _feedback_sound(notes):
  try:
    for frequency, duration in notes:
      _feedback_check()
      buzzer.duty_u16(0)
      buzzer.freq(frequency)
      buzzer.duty_u16(13762)
      _feedback_wait(duration)
  finally:
    buzzer.duty_u16(0)
def _feedback_blink(colour):
  try:
    for _ in range(3):
      _feedback_led(colour)
      _feedback_wait(200)
      _feedback_led_off()
      _feedback_wait(200)
  finally:
    _feedback_led_off()
def _feedback_heart(colour):
  try:
    for _ in range(2):
      for brightness in [10,20,30,20,10]:
        _feedback_matrix([0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,0,1,0,0], colour, brightness, "Coracao")
        _feedback_wait(90)
    _feedback_matrix([0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,0,1,0,0], colour, 25, "Coracao")
  except:
    _feedback_matrix_clear()
    raise
_robo_ina226 = None
try:
  _robo_power_i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
  _robo_ina226 = INA226(_robo_power_i2c, addr=64, shunt_resistor=0.1)
except Exception as exc:
  print("INA226: falha no barramento:", exc)
_robo_stby = Pin(20, Pin.OUT, value=0)
_robo_esq_frente = Pin(4, Pin.OUT)
_robo_esq_tras = Pin(9, Pin.OUT)
_robo_esq_pwm = PWM(Pin(8))
_robo_esq_pwm.duty_u16(0)
_robo_esq_pwm.freq(1000)
_robo_dir_frente = Pin(18, Pin.OUT)
_robo_dir_tras = Pin(19, Pin.OUT)
_robo_dir_pwm = PWM(Pin(16))
_robo_dir_pwm.duty_u16(0)
_robo_dir_pwm.freq(1000)
_robo_controle_setas = False
_robo_cancelado_setas = False
_robo_botao_a = None
_robo_botao_b = None
_robo_mpu = None
_robo_mpu_bus = None
try:
  _robo_i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
  _robo_mpu = MPU6050(_robo_i2c)
  _robo_mpu_bus = "I2C0 GP0/GP1"
except Exception as exc:
  print("MPU6050: falha no barramento:", exc)
if _robo_mpu is None or not _robo_mpu.is_ready:
  try:
    _robo_i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
    _robo_mpu = MPU6050(_robo_i2c)
    _robo_mpu_bus = "I2C1 GP2/GP3"
  except Exception as exc:
    print("MPU6050: falha no barramento alternativo:", exc)
_robo_vel_movimento = 35000
_robo_vel_giro = 35000
_robo_sinal_giro_z = -1
_robo_zona_morta_giro = 0.8
_robo_timeout_min_ms = 1500
_robo_timeout_ms_por_grau = 120
_robo_tempo_bloco_setas = 0.8
_robo_pausa_setas_repetidas_ms = 300
_robo_pausa_antes_giro_setas_ms = 200
_robo_pausa_apos_giro_setas_ms = 250
# Compensacao do recuo no pivo: potencia por sentido de movimento da lateral.
# Polaridade do comando frente: (0, 1) avanca; (1, 0) recua.
_robo_pwm_giro_avanco_setas = 40000
_robo_pwm_giro_re_setas = 36000
_robo_matriz_indices = [[24,23,22,21,20],[15,16,17,18,19],[14,13,12,11,10],[5,6,7,8,9],[4,3,2,1,0]]
_robo_cor_contagem = (0, 54, 54)
_robo_numeros_contagem = {
  1: [0,0,1,0,0, 0,1,1,0,0, 0,0,1,0,0, 0,0,1,0,0, 0,1,1,1,0],
  2: [1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,1],
  3: [1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1],
  4: [1,0,0,0,1, 1,0,0,0,1, 1,1,1,1,1, 0,0,0,0,1, 0,0,0,0,1],
  5: [1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,1, 0,0,0,0,1, 1,1,1,1,1]
}
_robo_orientacao_setas = 0
_robo_ultima_direcao_setas = None
_robo_falha_setas = False
_robo_mpu_sda = 0
_robo_mpu_scl = 1
_robo_mpu_sda_alt = 2
_robo_mpu_scl_alt = 3
_robo_pronto = False
_robo_diagnostico_ok = False
_robo_angulo = 0.0
_robo_giro_tempo = ticks_ms()
try:
  _robo_display_giro_ativo
except NameError:
  _robo_display_giro_ativo = False
  _robo_display_giro_linha_y = 8
  _robo_display_giro_alinhamento = "CENTER"
  _robo_display_giro_ultimo_ms = 0
try:
  _robo_display_acel_x_ativo
except NameError:
  _robo_display_acel_x_ativo = False
  _robo_display_acel_x_linha_y = 18
  _robo_display_acel_x_alinhamento = "CENTER"
try:
  _robo_display_acel_y_ativo
except NameError:
  _robo_display_acel_y_ativo = False
  _robo_display_acel_y_linha_y = 28
  _robo_display_acel_y_alinhamento = "CENTER"
try:
  _robo_display_acel_z_ativo
except NameError:
  _robo_display_acel_z_ativo = False
  _robo_display_acel_z_linha_y = 38
  _robo_display_acel_z_alinhamento = "CENTER"
try:
  _robo_display_transferidor_ativo
except NameError:
  _robo_display_transferidor_ativo = False
try:
  _robo_display_tensao_bateria_ativo
except NameError:
  _robo_display_tensao_bateria_ativo = False
  _robo_display_tensao_bateria_linha_y = 48
  _robo_display_tensao_bateria_alinhamento = "CENTER"
try:
  _robo_display_corrente_robo_ativo
except NameError:
  _robo_display_corrente_robo_ativo = False
  _robo_display_corrente_robo_linha_y = 48
  _robo_display_corrente_robo_alinhamento = "CENTER"
try:
  _robo_iniciar_setas(5)

  # Inicializar LEDs (desligar todos)
  led_vermelho.duty_u16(0)
  led_verde.duty_u16(0)
  led_azul.duty_u16(0)

  _feedback_led((59, 130, 246))
  _feedback_wait(1000)
  _feedback_matrix([0,1,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0], (250, 204, 21), 25, "Feliz")
  _feedback_wait(1000)
  _feedback_sound([[1500,100]])
  _feedback_wait(900)
  _robo_ir_para(0)
  _robo_ir_para(0)
  _feedback_matrix([0,1,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0], (59, 130, 246), 25, "Feliz")
  _feedback_wait(1000)
  _feedback_sound([[1500,100]])
  _feedback_wait(900)
  _robo_ir_para(0)
  _robo_ir_para(0)
  _feedback_matrix([0,1,0,1,0,0,1,0,1,0,0,0,0,0,0,0,1,1,1,0,1,0,0,0,1], (239, 68, 68), 25, "Triste")
  _feedback_wait(1000)
  _feedback_blink((250, 204, 21))
  _feedback_sound([[1500,100]])
  _feedback_wait(900)
  _robo_ir_para(3)
  _robo_ir_para(3)
  _robo_ir_para(0)
  _feedback_heart((239, 68, 68))
  _feedback_sound([[1500,100]])
  _feedback_wait(900)
  _robo_ir_para(2)
  _feedback_led((250, 204, 21))
  _feedback_wait(1000)
  _robo_ir_para(1)
  _feedback_led((34, 197, 94))
  _feedback_wait(1000)
  _feedback_matrix([0,1,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0], (34, 197, 94), 25, "Feliz")
  _feedback_wait(1000)
  _feedback_sound([[392,100],[440,100],[494,100],[523,100]])
  _feedback_wait(600)
  _robo_finalizar_setas()
except _RoboMissaoCancelada:
  print("Missao cancelada pelo botao B. Execute novamente para tentar outro percurso.")
finally:
  _robo_encerrar_setas()
