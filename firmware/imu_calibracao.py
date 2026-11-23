# imu_calibracao.py
# Biblioteca de controle e calibracao do carrinho (BitDogLab v7 + MPU6050 + TCS34725)
#
# Hardware:
#   Motores: LE_F=GP4, LE_R=GP9, PWM_E=GP8 | LD_F=GP18, LD_R=GP19, PWM_D=GP16 | STBY=GP20
#   Botoes:  A=GP5, B=GP6
#   IMU MPU6050: I2C0 SDA=GP0, SCL=GP1 (0x68 ou 0x69)
#   Cor TCS34725: I2C1 SDA=GP2, SCL=GP3 (0x29)
#
# Uso em outros scripts depois de rodar calibrar_carrinho.py:
#   from imu_calibracao import CarrinhoIMU
#   robo = CarrinhoIMU()
#   robo.andar_cm(50)     # anda 50 cm para frente (negativo = re)
#   robo.girar_graus(90)  # gira 90 graus horario (negativo = anti-horario)

from machine import Pin, PWM, I2C
import time
import struct
import json

# ---------------- PINAGEM ----------------
I2C_IMU_ID = 0
IMU_SDA_PIN = 0
IMU_SCL_PIN = 1
IMU_FREQ = 400000

I2C_COR_ID = 1
COR_SDA_PIN = 2
COR_SCL_PIN = 3
COR_FREQ = 100000
TCS_ADDR = 0x29

BTN_A_PIN = 5
BTN_B_PIN = 6
BTN_C_PIN = 10

LE_F_PIN = 4
LE_R_PIN = 9
PWM_E_PIN = 8
LD_F_PIN = 18
LD_R_PIN = 19
PWM_D_PIN = 16
STBY_PIN = 20

PWM_FREQ = 1000
PWM_RETO_PADRAO = 30000
PWM_GIRO_PADRAO = 36000
PWM_GIRO_RAMPA_INI = 20000   # PWM inicial da rampa suave de arranque do giro
RAMPA_GIRO_MS = 150          # duracao da rampa de arranque do giro (ms)
FREIO_MS = 120               # tempo de freio ativo (curto-circuito do motor) ao parar
PRE_STBY_MS = 50

# Configuracao do giroscopio e correcao de reta
GYRO_EIXO_PADRAO = 2         # 2 = eixo Z (vertical)
GYRO_SINAL_Z = 1             # 1 = MPU6050 chip p/ cima (+gz = esquerda); -1 se invertido
KP_RETO_PADRAO = 220.0       # ganho proporcional da correcao de reta (PWM por grau)
MAX_CORR_RETO = 3500         # limite maximo de correcao dinamica no PWM
DEADBAND_ANG_DEG = 0.5       # zona morta (graus) para evitar zigue-zague nos motores

ARQUIVO_CALIBRACAO = 'calibracao_imu.json'

# Constantes de conversao MPU6050 (+/-2g e +/-250 deg/s)
G_CM_S2 = 980.665
ACCEL_ESCALA_CM_S2 = G_CM_S2 / 16384.0
GYRO_ESCALA_DEG_S = 1.0 / 131.0

# Registradores TCS34725
_TCS_CMD = 0x80
_TCS_AUTO = 0x20
_TCS_ENABLE = 0x00
_TCS_ATIME = 0x01
_TCS_CONTROL = 0x0F
_TCS_ID = 0x12
_TCS_CDATAL = 0x14
_TCS_ATIME_VAL = 0xFE  # ~4.8 ms integracao (C_MAX = 2048)
_TCS_C_MAX = 2048
_TCS_GANHOS = ((3, 60), (2, 16), (1, 4), (0, 1))


class MPU6050:
    """Driver leve em MicroPython puro para o MPU6050 no I2C0."""

    def __init__(self, i2c, addr=None):
        self.i2c = i2c
        if addr is None:
            achados = i2c.scan()
            if 0x68 in achados:
                addr = 0x68
            elif 0x69 in achados:
                addr = 0x69
            else:
                raise OSError('MPU6050 nao encontrado no I2C0 (GP0/GP1). Encontrados: {}'.format(
                    [hex(x) for x in achados]))
        self.addr = addr
        self.buf = bytearray(14)

        # Acorda o sensor e configura filtro passa-baixa (DLPF ~44 Hz)
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')  # PWR_MGMT_1: acordar
        time.sleep_ms(50)
        self.i2c.writeto_mem(self.addr, 0x19, b'\x04')  # SMPLRT_DIV: 200 Hz
        self.i2c.writeto_mem(self.addr, 0x1A, b'\x03')  # CONFIG: DLPF 44Hz/42Hz
        self.i2c.writeto_mem(self.addr, 0x1B, b'\x00')  # GYRO_CONFIG: +/-250 deg/s
        self.i2c.writeto_mem(self.addr, 0x1C, b'\x00')  # ACCEL_CONFIG: +/-2g
        time.sleep_ms(20)

        self.who_am_i = self.i2c.readfrom_mem(self.addr, 0x75, 1)[0]

    def ler_bruto(self):
        """Retorna (ax, ay, az, gx, gy, gz) em contagens brutas (int16)."""
        self.i2c.readfrom_mem_into(self.addr, 0x3B, self.buf)
        ax, ay, az, _temp, gx, gy, gz = struct.unpack('>hhhhhhh', self.buf)
        return ax, ay, az, gx, gy, gz

    def ler_unidades(self):
        """Retorna (ax, ay, az) em cm/s^2 e (gx, gy, gz) em graus/s."""
        ax, ay, az, gx, gy, gz = self.ler_bruto()
        return (
            ax * ACCEL_ESCALA_CM_S2,
            ay * ACCEL_ESCALA_CM_S2,
            az * ACCEL_ESCALA_CM_S2,
            gx * GYRO_ESCALA_DEG_S,
            gy * GYRO_ESCALA_DEG_S,
            gz * GYRO_ESCALA_DEG_S,
        )

    def medir_bias(self, amostras=200, intervalo_ms=4):
        """Mede o offset (bias + gravidade) com o robo COMPLETAMENTE PARADO."""
        sax = say = saz = sgx = sgy = sgz = 0.0
        for _ in range(amostras):
            ax, ay, az, gx, gy, gz = self.ler_unidades()
            sax += ax
            say += ay
            saz += az
            sgx += gx
            sgy += gy
            sgz += gz
            time.sleep_ms(intervalo_ms)
        n = float(amostras)
        return (sax / n, say / n, saz / n), (sgx / n, sgy / n, sgz / n)


class TCS34725:
    """Driver leve para o sensor de cor TCS34725 no I2C1."""

    def __init__(self, i2c, addr=TCS_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.buf = bytearray(8)
        self.ident = self.i2c.readfrom_mem(self.addr, _TCS_CMD | _TCS_ID, 1)[0]
        self._escrever(_TCS_ATIME, _TCS_ATIME_VAL)
        self._escrever(_TCS_ENABLE, 0x01)
        time.sleep_ms(5)
        self._escrever(_TCS_ENABLE, 0x03)

    def _escrever(self, reg, val):
        self.i2c.writeto_mem(self.addr, _TCS_CMD | reg, bytes([val]))

    def ganho(self, cod):
        self._escrever(_TCS_CONTROL, cod)
        time.sleep_ms(30)

    def ler_clear(self):
        self.i2c.readfrom_mem_into(self.addr, _TCS_CMD | _TCS_AUTO | _TCS_CDATAL, self.buf)
        return self.buf[0] | (self.buf[1] << 8)

    def ler_rgbc(self):
        self.i2c.readfrom_mem_into(self.addr, _TCS_CMD | _TCS_AUTO | _TCS_CDATAL, self.buf)
        b = self.buf
        return (
            b[2] | (b[3] << 8),
            b[4] | (b[5] << 8),
            b[6] | (b[7] << 8),
            b[0] | (b[1] << 8),
        )

    def medir_clear_medio(self, ms=250):
        soma = 0
        n = 0
        fim = time.ticks_add(time.ticks_ms(), ms)
        while time.ticks_diff(fim, time.ticks_ms()) > 0:
            soma += self.ler_clear()
            n += 1
            time.sleep_ms(6)
        return soma / max(1, n)

    def escolher_ganho(self):
        ult = 0.0
        for cod, mult in _TCS_GANHOS:
            self.ganho(cod)
            ult = self.medir_clear_medio(180)
            if ult <= 0.60 * _TCS_C_MAX:
                return cod, mult, ult
        return 0, 1, ult


class Motores:
    """Controle dos motores com a mesma pinagem do teste_tcs34725_v2.py."""

    def __init__(self):
        self.le_f = Pin(LE_F_PIN, Pin.OUT, value=0)
        self.le_r = Pin(LE_R_PIN, Pin.OUT, value=0)
        self.ld_f = Pin(LD_F_PIN, Pin.OUT, value=0)
        self.ld_r = Pin(LD_R_PIN, Pin.OUT, value=0)
        self.stby = Pin(STBY_PIN, Pin.OUT, value=0)
        self.pwm_e = PWM(Pin(PWM_E_PIN))
        self.pwm_d = PWM(Pin(PWM_D_PIN))
        self.pwm_e.freq(PWM_FREQ)
        self.pwm_d.freq(PWM_FREQ)
        self._roda_pivo = None   # 'E' ou 'D': roda que anda durante o pivo
        self.parar()

    def parar(self):
        """Desliga os motores (rodas soltas, param por inercia)."""
        self._roda_pivo = None
        self.pwm_e.duty_u16(0)
        self.pwm_d.duty_u16(0)
        self.stby.value(0)
        self.le_f.value(0)
        self.le_r.value(0)
        self.ld_f.value(0)
        self.ld_r.value(0)

    def frear(self, ms=FREIO_MS):
        """Freio ativo: as duas entradas de cada ponte em nivel alto (curto no motor),
        durante `ms`, depois desliga. Faz todos os carrinhos pararem no mesmo ponto."""
        self._roda_pivo = None
        self.le_f.value(1)
        self.le_r.value(1)
        self.ld_f.value(1)
        self.ld_r.value(1)
        self.pwm_e.duty_u16(65535)
        self.pwm_d.duty_u16(65535)
        self.stby.value(1)
        if ms > 0:
            time.sleep_ms(ms)
        self.parar()

    def pivo_esquerda(self, pwm=PWM_GIRO_PADRAO):
        """Gira para a esquerda sobre a roda ESQUERDA travada (freio ativo);
        so a roda direita anda para frente."""
        self.parar()
        self.le_f.value(1)       # esquerda: freio (IN1 = IN2 = 1)
        self.le_r.value(1)
        self.ld_f.value(0)       # direita: frente
        self.ld_r.value(1)
        self.stby.value(1)
        self.pwm_e.duty_u16(65535)
        time.sleep_ms(PRE_STBY_MS)
        self._roda_pivo = 'D'
        self.definir_pwm_pivo(pwm)

    def pivo_direita(self, pwm=PWM_GIRO_PADRAO):
        """Gira para a direita sobre a roda DIREITA travada (freio ativo);
        so a roda esquerda anda para frente."""
        self.parar()
        self.ld_f.value(1)       # direita: freio (IN1 = IN2 = 1)
        self.ld_r.value(1)
        self.le_f.value(0)       # esquerda: frente
        self.le_r.value(1)
        self.stby.value(1)
        self.pwm_d.duty_u16(65535)
        time.sleep_ms(PRE_STBY_MS)
        self._roda_pivo = 'E'
        self.definir_pwm_pivo(pwm)

    def definir_pwm_pivo(self, pwm):
        """Ajusta o PWM so da roda que esta andando no pivo (a outra continua freada)."""
        pwm = max(0, min(65535, int(pwm)))
        if self._roda_pivo == 'D':
            self.pwm_d.duty_u16(pwm)
        elif self._roda_pivo == 'E':
            self.pwm_e.duty_u16(pwm)

    def definir_pwm(self, pwm_e, pwm_d):
        """Ajusta os PWMs individualmente (util para correcao de rumo com giroscopio)."""
        pwm_e = max(0, min(65535, int(pwm_e)))
        pwm_d = max(0, min(65535, int(pwm_d)))
        self.pwm_e.duty_u16(pwm_e)
        self.pwm_d.duty_u16(pwm_d)

    def frente(self, pwm=PWM_RETO_PADRAO):
        self.parar()
        self.le_f.value(0)
        self.le_r.value(1)
        self.ld_f.value(0)
        self.ld_r.value(1)
        self.stby.value(1)
        time.sleep_ms(PRE_STBY_MS)
        self.definir_pwm(pwm, pwm)

    def re(self, pwm=PWM_RETO_PADRAO):
        self.parar()
        self.le_f.value(1)
        self.le_r.value(0)
        self.ld_f.value(1)
        self.ld_r.value(0)
        self.stby.value(1)
        time.sleep_ms(PRE_STBY_MS)
        self.pwm_e.duty_u16(pwm)
        self.pwm_d.duty_u16(pwm)

    def girar_horario(self, pwm=PWM_GIRO_PADRAO):
        """Gira no proprio eixo para a direita (horario)."""
        self.parar()
        self.le_f.value(0)
        self.le_r.value(1)
        self.ld_f.value(1)
        self.ld_r.value(0)
        self.stby.value(1)
        time.sleep_ms(PRE_STBY_MS)
        self.pwm_e.duty_u16(pwm)
        self.pwm_d.duty_u16(pwm)

    def girar_antihorario(self, pwm=PWM_GIRO_PADRAO):
        """Gira no proprio eixo para a esquerda (anti-horario)."""
        self.parar()
        self.le_f.value(1)
        self.le_r.value(0)
        self.ld_f.value(0)
        self.ld_r.value(1)
        self.stby.value(1)
        time.sleep_ms(PRE_STBY_MS)
        self.pwm_e.duty_u16(pwm)
        self.pwm_d.duty_u16(pwm)


def rampa_pwm(pwm_alvo, decorrido_us, pwm_ini=PWM_GIRO_RAMPA_INI, duracao_ms=RAMPA_GIRO_MS):
    """Interpola suavemente o PWM de `pwm_ini` ate `pwm_alvo` em `duracao_ms`."""
    if duracao_ms <= 0 or pwm_alvo <= pwm_ini:
        return pwm_alvo
    dur_us = duracao_ms * 1000
    if decorrido_us >= dur_us:
        return pwm_alvo
    return pwm_ini + int((pwm_alvo - pwm_ini) * decorrido_us // dur_us)


def carregar_calibracao(caminho=ARQUIVO_CALIBRACAO):
    try:
        with open(caminho, 'r') as f:
            return json.load(f)
    except OSError:
        return {}


def salvar_calibracao(dados, caminho=ARQUIVO_CALIBRACAO):
    atual = carregar_calibracao(caminho)
    atual.update(dados)
    with open(caminho, 'w') as f:
        json.dump(atual, f)
    return atual


class CarrinhoIMU:
    """Interface pronta para usar o carrinho calibrado em outros scripts."""

    def __init__(self, caminho_cal=ARQUIVO_CALIBRACAO):
        self.caminho_cal = caminho_cal
        self.btn_b = Pin(BTN_B_PIN, Pin.IN, Pin.PULL_UP)
        self.motores = Motores()
        self.i2c_imu = I2C(I2C_IMU_ID, sda=Pin(IMU_SDA_PIN), scl=Pin(IMU_SCL_PIN), freq=IMU_FREQ)
        self.imu = MPU6050(self.i2c_imu)
        self.cal = carregar_calibracao(caminho_cal)

    def recarregar_calibracao(self):
        self.cal = carregar_calibracao(self.caminho_cal)
        return self.cal

    def _params_gyro(self):
        """Retorna (eixo, sinal, fator) do giroscopio (calibrado em 360 deg ou cru)."""
        eixo = int(self.cal.get('gyro_eixo', GYRO_EIXO_PADRAO))
        sinal = float(self.cal.get('gyro_sinal', GYRO_SINAL_Z))
        fator = float(self.cal.get('gyro_fator_deg', 1.0))
        return eixo, sinal, fator

    def andar_cm(self, dist_cm, pwm=None, timeout_ms=15000, usar_gyro_reto=True, bias_gz=None):
        """Anda `dist_cm` para frente usando a velocidade calibrada (vel_media_cm_s)
        e o giroscopio (calibrado em 360 deg ou cru) para manter o carrinho reto.
        Botao B interrompe imediatamente por seguranca.
        """
        if 'vel_media_cm_s' not in self.cal:
            raise RuntimeError('Velocidade nao calibrada! Rode calibrar_carrinho.py (50 cm) primeiro.')

        alvo = abs(float(dist_cm))
        if alvo <= 0.0:
            return 0.0

        vel = float(self.cal['vel_media_cm_s'])
        if vel <= 0.0:
            raise RuntimeError('vel_media_cm_s invalida na calibracao.')

        if pwm is None:
            pwm = int(self.cal.get('pwm_cal_reto', PWM_RETO_PADRAO))

        tempo_alvo_us = int((alvo / vel) * 1000000.0)
        limite_us = min(tempo_alvo_us, timeout_ms * 1000)

        g_eixo, g_sinal, g_fator = self._params_gyro()
        trim_pwm = int(self.cal.get('trim_reto_pwm', 0))
        if usar_gyro_reto and bias_gz is None:
            self.motores.parar()
            time.sleep_ms(80)
            _, bias_g = self.imu.medir_bias(amostras=100, intervalo_ms=3)
            bias_gz = bias_g[g_eixo]

        ang = 0.0
        g_ant = 0.0

        self.motores.frente(pwm)
        if usar_gyro_reto and trim_pwm != 0:
            self.motores.definir_pwm(pwm + trim_pwm, pwm - trim_pwm)
        t0 = time.ticks_us()
        t_ant = t0
        try:
            while True:
                if self.btn_b.value() == 0:
                    break
                t_now = time.ticks_us()
                decorrido_us = time.ticks_diff(t_now, t0)
                if decorrido_us >= limite_us:
                    break

                if usar_gyro_reto:
                    dt = time.ticks_diff(t_now, t_ant) / 1000000.0
                    t_ant = t_now
                    if dt > 0.0:
                        try:
                            leitura = self.imu.ler_unidades()
                            # Positivo = esquerda (anti-horario)
                            g_cur = (leitura[3 + g_eixo] - bias_gz) * g_sinal * g_fator
                            ang += 0.5 * (g_ant + g_cur) * dt
                            g_ant = g_cur
                            if abs(ang) < DEADBAND_ANG_DEG:
                                corr = 0
                            else:
                                # ang > 0 (desviou p/ esquerda) -> aumenta esq, reduz dir
                                corr = int(KP_RETO_PADRAO * ang)
                                if corr > MAX_CORR_RETO:
                                    corr = MAX_CORR_RETO
                                elif corr < -MAX_CORR_RETO:
                                    corr = -MAX_CORR_RETO
                            total_corr = trim_pwm + corr
                            self.motores.definir_pwm(pwm + total_corr, pwm - total_corr)
                        except OSError:
                            pass

                time.sleep_ms(4)
        finally:
            self.motores.frear()

        dt_final_s = time.ticks_diff(time.ticks_us(), t0) / 1000000.0
        return vel * dt_final_s

    def girar_graus(self, angulo_deg, pwm=PWM_GIRO_PADRAO, timeout_ms=8000, bias_gz=None):
        """Gira `angulo_deg` em PIVO (sobre a roda do lado da curva, travada por freio ativo),
        usando o giroscopio calibrado em 360 deg (ou cru).
        angulo_deg > 0: horario (direita) | angulo_deg < 0: anti-horario (esquerda).
        Botao B interrompe imediatamente por seguranca.
        """
        alvo = abs(float(angulo_deg))
        if alvo <= 0.0:
            return 0.0

        horario = angulo_deg >= 0
        g_eixo, _g_sinal, g_fator = self._params_gyro()

        if bias_gz is None:
            self.motores.parar()
            time.sleep_ms(100)
            _, bias_g = self.imu.medir_bias(amostras=120, intervalo_ms=3)
            bias_gz = bias_g[g_eixo]

        ang = 0.0
        g_ant = 0.0
        pwm_ini = min(PWM_GIRO_RAMPA_INI, pwm)

        if horario:
            self.motores.pivo_direita(pwm_ini)
        else:
            self.motores.pivo_esquerda(pwm_ini)

        t0 = time.ticks_us()
        t_ant = t0
        rampa_concluida = (pwm_ini >= pwm)
        try:
            while True:
                if self.btn_b.value() == 0:
                    break
                t_now = time.ticks_us()
                decorrido_us = time.ticks_diff(t_now, t0)
                if decorrido_us >= timeout_ms * 1000:
                    break

                if not rampa_concluida:
                    p = rampa_pwm(pwm, decorrido_us)
                    self.motores.definir_pwm_pivo(p)
                    if p >= pwm:
                        rampa_concluida = True

                dt = time.ticks_diff(t_now, t_ant) / 1000000.0
                if dt <= 0.0:
                    continue
                t_ant = t_now

                try:
                    leitura = self.imu.ler_unidades()
                except OSError:
                    time.sleep_ms(2)
                    continue
                g_atual = (leitura[3 + g_eixo] - bias_gz) * g_fator
                ang += 0.5 * (g_ant + g_atual) * dt
                g_ant = g_atual

                if abs(ang) >= alvo:
                    break

                time.sleep_ms(4)
        finally:
            self.motores.frear()

        return abs(ang) if horario else -abs(ang)

