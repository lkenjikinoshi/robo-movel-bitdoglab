# BitDogLab - Calibracao V3: inclui percurso de movimentos do main.py
# Requer sh1107_bitdoglab.py salvo na Raspberry Pi Pico.
# Nunca movimentar sem supervisao e pista desobstruida.
from machine import Pin, PWM, I2C, ADC
import machine
import time
import math
import gc
import sys
from sh1107_bitdoglab import SH1107_I2C

# -------------------- CONFIGURACAO --------------------
ROBO_IDS = (1, 2, 3, 4, 5, 6)
POSICOES_MPU = ('central', 'traseira')
PERCURSOS = ('SO FRENTE', 'PERCURSO 3Q', 'MOVIMENTOS MAIN')
QUADRADOS = (1, 2, 3, 4)
REPETICOES = (5, 10, 15, 20)
SUPERFICIE = 'tapete'           # fixa, sem menu
BATERIA_ID = 'B01'             # edite caso troque de bateria

TEMPO_QUAD_MS = 800           # referencia a ser testada; NAO e distancia medida
PWM_RETO = 35000
PWM_GIRO = 33000
PWM_GIRO_FINAL = 24500      # desacelera proximidade de 90 graus
PWM_FREQ = 1000
PRE_STBY_MS = 50
FREQ_I2C = 400000           # 100000 se barramento apresentar erros
GYRO_SIGN = 1               # gyro positivo = esquerda na montagem validada
TURN_TARGET_DEG = 90.0
TURN_SLOW_DEG = 18.0
TURN_TIMEOUT_MS = 6500      # limite de seguranca; pode exigir ajuste apos medir os giros
TURN_SEM_MOV_MS = 1400      # diagnostico se o giro nao comecou
TURN_MOV_MIN_DEG = 4.0     # minimo esperado apos TURN_SEM_MOV_MS
IMU_INTERVAL_MS = 10
INA_INTERVAL_MS = 60
TRACE_INTERVAL_MS = 60
OLED_INTERVAL_MS = 300
COAST_MS = 300
CONTAGEM_MS = 3000
BIAS_N = 140
BIAS_INTERVAL_MS = 3
BIAS_STD_MAX = 0.8
AJUSTE_PULSO_MAX_MS = 450
AJUSTE_TOTAL_MAX_MS = 2500
JOY_MORTA = 6000
SHUNT_OHM = 0.1               # CONFERIR resistor do INA226 real
SHOW_OLED_MOVING = True     # mostra tensao, corrente e angulo durante os movimentos
# A tela e os sensores continuam atualizados entre trechos e nos menus.
MAX_TRACE = 500
SALVAR_TRACE = True

CSV_RESUMO = 'calib_v3.csv'
CSV_TRACE = 'traco_v3.csv'
CSV_BOOT = 'boot_v3.csv'
CSV_SESSAO = 'resumo_v3.csv'

# Configuracao sem OLED
PADRAO = {'robo': 1, 'pos': 'central', 'percurso': 'SO FRENTE',
          'quad': 1, 'reps': 5}

# -------------------- HARDWARE --------------------
BTN_A = Pin(5, Pin.IN, Pin.PULL_UP)
BTN_B = Pin(6, Pin.IN, Pin.PULL_UP)
JOY_Y = ADC(Pin(26))
LE_F = Pin(4, Pin.OUT, value=0)
LE_R = Pin(9, Pin.OUT, value=0)
LD_F = Pin(18, Pin.OUT, value=0)
LD_R = Pin(19, Pin.OUT, value=0)
STBY = Pin(20, Pin.OUT, value=0)
PWM_E = PWM(Pin(8))
PWM_D = PWM(Pin(16))
PWM_E.freq(PWM_FREQ)
PWM_D.freq(PWM_FREQ)
PWM_E.duty_u16(0)
PWM_D.duty_u16(0)


def parar():
    PWM_E.duty_u16(0)
    PWM_D.duty_u16(0)
    STBY.value(0)
    LE_F.value(0); LE_R.value(0)
    LD_F.value(0); LD_R.value(0)


def configurar_motores(tipo):
    # tipo: FRENTE, TRAS, ESQUERDA, DIREITA (giro no proprio eixo)
    parar()
    if tipo == 'FRENTE':
        le, ld = (0, 1), (0, 1)
    elif tipo == 'TRAS':
        le, ld = (1, 0), (1, 0)
    elif tipo == 'ESQUERDA':
        le, ld = (0, 1), (1, 0)
    elif tipo == 'DIREITA':
        le, ld = (1, 0), (0, 1)
    else:
        raise ValueError('Direcao desconhecida')
    LE_F.value(le[0]); LE_R.value(le[1])
    LD_F.value(ld[0]); LD_R.value(ld[1])
    STBY.value(1)
    time.sleep_ms(PRE_STBY_MS)


def pwm_acionar(valor):
    valor = min(65535, max(0, int(valor)))
    PWM_E.duty_u16(valor)
    PWM_D.duty_u16(valor)


class Tela:
    """Driver existente com atualizacao diferencial de paginas."""
    def __init__(self, bus, addr):
        self.dev = SH1107_I2C(128, 128, bus, addr=addr, rotate_180=False)
        self.ant = bytearray(self.dev.buffer)
        self.linhas = None
        self.telemetria = None

    def enviar(self):
        buf = self.dev.buffer
        for pagina in range(16):
            i = pagina * 128
            j = i + 128
            trecho = buf[i:j]
            if trecho == self.ant[i:j]:
                continue
            self.dev.write_cmd(0xB0 | pagina)
            self.dev.write_cmd(0x00)
            self.dev.write_cmd(0x10)
            self.dev.write_data(trecho)
            self.ant[i:j] = trecho

    def texto(self, a='', b='', c='', d=''):
        linhas = tuple(str(x)[:16] for x in (a, b, c, d))
        if linhas == self.linhas:
            return
        self.dev.fill_rect(0, 0, 128, 92, 0)
        for y, linha in zip((8, 29, 50, 71), linhas):
            self.dev.text(linha, 0, y, 1)
        self.enviar()
        self.linhas = linhas

    def medidas(self, v=None, i=None, theta=None):
        s1 = 'V:{} I:{}A'.format('--' if v is None else '{:.2f}'.format(v),
                               '--' if i is None else '{:.2f}'.format(i))
        s2 = 'Ang:{}g'.format('--' if theta is None else '{:+.1f}'.format(theta))
        if (s1, s2) == self.telemetria:
            return
        self.dev.fill_rect(0, 94, 128, 34, 0)
        self.dev.text(s1[:16], 0, 96, 1)
        self.dev.text(s2[:16], 0, 113, 1)
        self.enviar()
        self.telemetria = s1, s2


class MPU6050:
    def __init__(self, bus, endereco):
        self.bus = bus
        self.endereco = endereco
        self.bias = 0.0
        self.bus.writeto_mem(endereco, 0x6B, b'\x00')
        self.bus.writeto_mem(endereco, 0x1B, b'\x00')  # +-250 dps
        time.sleep_ms(80)

    def bruto(self):
        d = self.bus.readfrom_mem(self.endereco, 0x47, 2)
        v = (d[0] << 8) | d[1]
        if v & 0x8000:
            v -= 65536
        return v / 131.0

    def gz(self):
        return (self.bruto() - self.bias) * GYRO_SIGN

    def calibrar(self):
        soma, quadrado = 0.0, 0.0
        for _ in range(BIAS_N):
            x = self.bruto()
            soma += x
            quadrado += x * x
            time.sleep_ms(BIAS_INTERVAL_MS)
        media = soma / BIAS_N
        desvio = math.sqrt(max(0, quadrado / BIAS_N - media * media))
        if desvio <= BIAS_STD_MAX:
            self.bias = media
        return media, desvio


class INA226:
    def __init__(self, bus, addr=0x40):
        self.bus, self.addr = bus, addr
        self.bus.writeto_mem(addr, 0, b'\x41\x27')

    def ler(self):
        d = self.bus.readfrom_mem(self.addr, 2, 2)
        s = self.bus.readfrom_mem(self.addr, 1, 2)
        volts = ((d[0] << 8) | d[1]) * 0.00125
        v = (s[0] << 8) | s[1]
        if v & 0x8000:
            v -= 65536
        corrente = abs(v * 0.0000025 / SHUNT_OHM)
        return volts, corrente, v in (-32768, 32767)


oled = None
mpu = None
ina = None


class Painel:
    def __init__(self):
        self.theta = 0.0
        self.gz_ant = None
        self.us_ant = None
        self.ultimo_gyro = 0
        self.ultimo_ina = 0
        self.ultimo_oled = 0
        self.v = None
        self.i = None

    def zerar(self):
        self.theta = 0.0
        self.gz_ant = None
        self.us_ant = None

    def atualizar(self, tele=None, forcar=False):
        agora = time.ticks_ms()
        if tele is None:
            if mpu and (forcar or time.ticks_diff(agora, self.ultimo_gyro) >= 40):
                self.ultimo_gyro = agora
                try:
                    gz = mpu.gz()
                    us = time.ticks_us()
                    if self.us_ant is not None and self.gz_ant is not None:
                        dt = time.ticks_diff(us, self.us_ant)
                        if 0 < dt < 200000:
                            self.theta += (gz + self.gz_ant) * dt / 2000000.0
                    self.gz_ant, self.us_ant = gz, us
                except OSError:
                    self.gz_ant, self.us_ant = None, None
            if ina and (forcar or time.ticks_diff(agora, self.ultimo_ina) >= 250):
                self.ultimo_ina = agora
                try:
                    self.v, self.i, _ = ina.ler()
                except OSError:
                    self.v, self.i = None, None
        if oled and (forcar or time.ticks_diff(agora, self.ultimo_oled) >= OLED_INTERVAL_MS):
            self.ultimo_oled = agora
            try:
                if tele is not None:
                    oled.medidas(tele.v_agora, tele.i_agora, tele.theta)
                else:
                    oled.medidas(self.v, self.i, self.theta)
            except OSError as e:
                print('OLED falhou:', repr(e))

    def finalizar(self, tele):
        self.theta = tele.theta
        self.gz_ant = None
        self.us_ant = None
        self.ultimo_ina = time.ticks_ms() - 1000
        self.atualizar(forcar=True)

painel = Painel()
_ultima_msg = None


def tela(a='', b='', c='', d='', atualizar=True):
    global _ultima_msg
    t = tuple(str(x) for x in (a, b, c, d))
    if t != _ultima_msg:
        print('TELA:', ' | '.join(t))
        _ultima_msg = t
    if oled:
        try:
            oled.texto(*t)
        except OSError as e:
            print('Falha OLED:', repr(e))
    if atualizar:
        painel.atualizar(forcar=True)


def soltar(botao):
    while botao.value() == 0:
        painel.atualizar()
        time.sleep_ms(12)
    time.sleep_ms(18)


def esperar_a():
    soltar(BTN_A)
    while BTN_A.value():
        painel.atualizar()
        time.sleep_ms(12)
    soltar(BTN_A)


def escolher(titulo, opcoes):
    pos = 0
    soltar(BTN_A); soltar(BTN_B)
    tela(titulo, 'A: muda', str(opcoes[pos]), 'B: confirma')
    while True:
        if BTN_B.value() == 0:
            soltar(BTN_B)
            return opcoes[pos]
        if BTN_A.value() == 0:
            soltar(BTN_A)
            pos = (pos + 1) % len(opcoes)
            tela(titulo, 'A: muda', str(opcoes[pos]), 'B: confirma')
        painel.atualizar()
        time.sleep_ms(12)


def contar():
    inicio = time.ticks_ms()
    ultimo = None
    while True:
        falta = CONTAGEM_MS - time.ticks_diff(time.ticks_ms(), inicio)
        if falta <= 0:
            return True
        if BTN_B.value() == 0:
            soltar(BTN_B)
            return False
        segundos = (falta + 999) // 1000
        if segundos != ultimo:
            ultimo = segundos
            tela('PARTIDA EM {}s'.format(segundos), 'Nao toque', 'B: cancela', '')
        painel.atualizar()
        time.sleep_ms(12)


# -------------------- MEDICAO --------------------
class Telemetria:
    """IMU em tempo real, com angulo GLOBAL preservado por todo percurso.
    Os angulos locais dos segmentos sao diferencas em relacao ao inicio de
    cada segmento. Nao integrar atraves de falhas I2C ou intervalos longos.
    """
    def __init__(self):
        self.theta = 0.0
        self.gz_ant = None
        self.us_ant = None
        self.theta_ref = 0.0
        self.n_gyro = 0
        self.erros_gyro = 0
        self.erros_ina = 0
        self.erros_consec = 0
        self.dt_max_us = 0
        self.ultimo_ina = time.ticks_ms()
        self.ultimo_trace = time.ticks_ms()
        self.t0 = time.ticks_ms()
        self.trace = []
        self.trace_perdido = 0
        self.v_agora = None
        self.i_agora = None
        self.v_soma = 0.0
        self.i_soma = 0.0
        self.n_ina = 0
        self.v_min = None
        self.i_max = None
        self.sat = 0
        self.status = 'ok'

    def reiniciar_integracao(self):
        # Intervalos ociosos (menus/posicionamento) nao entram na integral.
        # Mantemos theta global; somente reiniciamos o par de amostras.
        self.gz_ant = None
        self.us_ant = None

    def imu(self):
        try:
            gz = mpu.gz()
            t = time.ticks_us()
        except OSError:
            self.erros_gyro += 1
            self.erros_consec += 1
            self.us_ant = None
            self.gz_ant = None
            if self.erros_consec >= 3:
                self.status = 'erro_mpu'
            return None
        self.erros_consec = 0
        if self.us_ant is not None:
            dt = time.ticks_diff(t, self.us_ant)
            if dt > self.dt_max_us:
                self.dt_max_us = dt
            if 0 < dt < 120000:
                self.theta += (gz + self.gz_ant) * dt / 2000000.0
            elif dt >= 120000:
                self.status = 'lacuna_gyro'
        self.us_ant, self.gz_ant = t, gz
        self.n_gyro += 1
        return gz

    def ler_ina(self, em_movimento):
        if ina is None or not em_movimento:
            return
        agora = time.ticks_ms()
        if time.ticks_diff(agora, self.ultimo_ina) < INA_INTERVAL_MS:
            return
        self.ultimo_ina = agora
        try:
            v, i, sat = ina.ler()
            self.v_agora, self.i_agora = v, i
            self.v_soma += v
            self.i_soma += i
            self.n_ina += 1
            self.v_min = v if self.v_min is None else min(v, self.v_min)
            self.i_max = i if self.i_max is None else max(i, self.i_max)
            if sat:
                self.sat += 1
        except OSError:
            self.erros_ina += 1

    def log(self, fase, forcar=False):
        if not SALVAR_TRACE:
            return
        agora = time.ticks_ms()
        if not forcar and time.ticks_diff(agora, self.ultimo_trace) < TRACE_INTERVAL_MS:
            return
        if len(self.trace) < MAX_TRACE:
            self.trace.append((time.ticks_diff(agora, self.t0), fase, self.theta,
                               self.theta - self.theta_ref, self.gz_ant,
                               self.v_agora, self.i_agora))
        else:
            self.trace_perdido += 1
        self.ultimo_trace = agora

    def ciclo(self, fase, motor=True):
        self.imu()
        self.ler_ina(motor)
        self.log(fase)
        if SHOW_OLED_MOVING:
            painel.atualizar(tele=self)

    def repouso(self, duracao_ms, fase):
        inicio = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), inicio) < duracao_ms:
            self.ciclo(fase, False)
            if self.status != 'ok':
                break
            time.sleep_ms(IMU_INTERVAL_MS)
        self.log(fase, True)

    def dados(self):
        return {
            'v_media': self.v_soma / self.n_ina if self.n_ina else None,
            'i_media': self.i_soma / self.n_ina if self.n_ina else None,
            'v_min': self.v_min, 'i_max': self.i_max,
            'n_ina': self.n_ina, 'n_gyro': self.n_gyro,
            'erros_gyro': self.erros_gyro, 'erros_ina': self.erros_ina,
            'dt_max_ms': self.dt_max_us / 1000.0,
            'sat': self.sat, 'trace_perdido': self.trace_perdido,
        }


def giro_90(direcao, tele, fase):
    """Gira aproximadamente 90 graus com realimentacao do MPU6050.

    CORRECAO V6.1: calcula o progresso pela magnitude da rotacao local,
    sem assumir que o sinal de gz corresponde a uma direcao fisica.
    O sinal medido continua preservado em tele.theta e nos arquivos CSV.

    IMPORTANTE: conferir com as rodas suspensas que o comando DIREITA
    realmente gira a DIREITA; a magnitude nao detecta motores invertidos.
    Retorna (angulo_com_inercia, tempo_motor_ms, status).
    """
    configurar_motores(direcao)
    tele.reiniciar_integracao()
    tele.imu()
    origem = tele.theta
    tele.theta_ref = origem
    inicio_ms = time.ticks_ms()
    inicio_us = time.ticks_us()
    fim_us = inicio_us
    status = 'ok'
    pico_progresso = 0.0
    try:
        pwm_acionar(PWM_GIRO)
        while True:
            decorrido = time.ticks_diff(time.ticks_ms(), inicio_ms)
            if BTN_B.value() == 0:
                status = 'cancelado_B'
                break
            if decorrido >= TURN_TIMEOUT_MS:
                status = 'timeout_giro'
                break

            # Priorizar IMU; nenhuma transferencia de framebuffer durante o giro.
            tele.ciclo(fase, True)
            if tele.status != 'ok':
                status = tele.status
                break

            deslocamento = tele.theta - origem  # angulo assinado
            progresso = abs(deslocamento)      # independente da montagem do MPU
            if progresso > pico_progresso:
                pico_progresso = progresso

            if progresso >= TURN_TARGET_DEG:
                break

            # Caso o motor nao esteja girando, interrompe de maneira segura.
            if decorrido >= TURN_SEM_MOV_MS and pico_progresso < TURN_MOV_MIN_DEG:
                status = 'giro_sem_movimento'
                break

            if progresso >= TURN_TARGET_DEG - TURN_SLOW_DEG:
                pwm_acionar(PWM_GIRO_FINAL)

            time.sleep_ms(IMU_INTERVAL_MS)
    finally:
        parar()
        fim_us = time.ticks_us()

    tempo_ms = time.ticks_diff(fim_us, inicio_us) / 1000.0
    # Ler a inercia mesmo se o giro foi abortado, para registrar o ocorrido.
    tele.imu()
    tele.repouso(COAST_MS, fase + '_inercia')
    angulo_local = tele.theta - origem
    tele.log(fase + '_fim', True)
    if status == 'ok' and tele.status != 'ok':
        status = tele.status
    print('GIRO', direcao, 'angulo={:.1f}g'.format(angulo_local),
          'motor={:.0f}ms'.format(tempo_ms), 'status='+status)
    return angulo_local, tempo_ms, status


def avancar_quadrados(n, tele, fase):
    """Avanca n quadrados temporizados; devolve tempos e erro angular LOCAL.
    So o giro global theta continua de um segmento para outro.
    """
    tele.reiniciar_integracao()
    tele.imu()
    origem = tele.theta
    tele.theta_ref = origem
    configurar_motores('FRENTE')
    inicio_ms = time.ticks_ms()
    inicio_us = time.ticks_us()
    duracao = TEMPO_QUAD_MS * n
    max_abs_local = 0.0
    status = 'ok'
    try:
        pwm_acionar(PWM_RETO)
        while True:
            passou = time.ticks_diff(time.ticks_ms(), inicio_ms)
            if BTN_B.value() == 0:
                status = 'cancelado_B'
                break
            if passou >= duracao:
                break
            tele.ciclo(fase, True)
            if tele.status != 'ok':
                status = tele.status
                break
            max_abs_local = max(max_abs_local, abs(tele.theta - origem))
            restante = duracao - passou
            time.sleep_ms(min(IMU_INTERVAL_MS, max(1, restante)))
    finally:
        parar()
        fim_us = time.ticks_us()
    tempo = time.ticks_diff(fim_us, inicio_us) / 1000.0
    tele.imu()
    ang_desligar = tele.theta - origem
    max_abs_local = max(max_abs_local, abs(ang_desligar))
    tele.log(fase + '_parada', True)
    tele.repouso(COAST_MS, fase + '_inercia')
    ang_final = tele.theta - origem
    max_abs_local = max(max_abs_local, abs(ang_final))
    tele.log(fase + '_fim', True)
    if status == 'ok':
        status = tele.status
    return {'tempo_ms': tempo, 'desvio_deg': ang_final,
            'desvio_max_deg': max_abs_local,
            'residual_deg': ang_final - ang_desligar}, status


def ajuste_joystick(tele):
    """Apenas no FINAL do percurso, na direcao em que o robo terminou.
    Para cima: frente; para baixo: re. B confirma, A descarta.
    Os pulsos NAO equivalem a um unico deslocamento continuo.
    """
    soma_frente = 0
    soma_re = 0
    pulsos = 0
    tela('Ajuste pos final', 'Joy cima/baixo', 'B: confirma', 'A: descarta')
    while True:
        if BTN_A.value() == 0:
            soltar(BTN_A)
            return soma_frente, soma_re, pulsos, False, 'descartado_A'
        if BTN_B.value() == 0:
            soltar(BTN_B)
            return soma_frente, soma_re, pulsos, True, tele.status
        jy = 32768 - JOY_Y.read_u16()
        if abs(jy) <= JOY_MORTA:
            painel.atualizar(tele=tele)
            time.sleep_ms(12)
            continue
        if soma_frente + soma_re >= AJUSTE_TOTAL_MAX_MS:
            tela('Limite ajuste', 'B: confirma', 'A: descarta')
            time.sleep_ms(20)
            continue
        sentido_f = jy > 0
        configurar_motores('FRENTE' if sentido_f else 'TRAS')
        tele.reiniciar_integracao()  # nao somar espera pelo joystick
        tele.imu()
        inicio_ms = time.ticks_ms()
        try:
            pwm_acionar(PWM_RETO)
            while True:
                movimento = 32768 - JOY_Y.read_u16()
                if (sentido_f and movimento <= JOY_MORTA) or (not sentido_f and movimento >= -JOY_MORTA):
                    break
                if BTN_A.value() == 0 or BTN_B.value() == 0:
                    break
                passou = time.ticks_diff(time.ticks_ms(), inicio_ms)
                if passou >= AJUSTE_PULSO_MAX_MS or soma_frente + soma_re + passou >= AJUSTE_TOTAL_MAX_MS:
                    break
                tele.ciclo('ajuste_frente' if sentido_f else 'ajuste_re', True)
                if tele.status != 'ok':
                    break
                time.sleep_ms(IMU_INTERVAL_MS)
        finally:
            parar()
        dt = time.ticks_diff(time.ticks_ms(), inicio_ms)
        if sentido_f:
            soma_frente += dt
        else:
            soma_re += dt
        pulsos += 1
        tele.imu()
        tele.repouso(80, 'ajuste_parado')
        if tele.status != 'ok':
            return soma_frente, soma_re, pulsos, False, tele.status
        tela('Ajuste total ms', 'F:{} R:{}'.format(soma_frente, soma_re),
             'B: confirma', 'A: descarta')
        while abs(32768 - JOY_Y.read_u16()) > JOY_MORTA:
            if BTN_A.value() == 0 or BTN_B.value() == 0:
                break
            painel.atualizar(tele=tele)
            time.sleep_ms(12)


# -------------------- REGISTROS CSV --------------------
CSV_COL = (
    'id','evento','robo','bateria','superficie','mpu_pos','percurso','quadrados',
    'rep','pwm_reto','tempo_ref_ms','tempo_reto_total_ms','seg_por_quad_reto',
    't_reto1_ms','t_reto2_ms','t_reto3_ms',
    'desvio_reto1_deg','desvio_reto2_deg','desvio_reto3_deg',
    'desvio_max1_deg','desvio_max2_deg','desvio_max3_deg',
    'giro_dir_deg','giro_esq_deg','t_giro_dir_ms','t_giro_esq_ms',
    'erro_angular_global_antes_ajuste_deg','erro_angular_global_final_deg',
    'ajuste_frente_ms','ajuste_re_ms','ajuste_pulsos',
    'v_media_mov','i_media_mov','v_min_amostrada','i_max_amostrada',
    'n_ina','n_gyro','erros_mpu','erros_ina','dt_max_ms','bias_dps',
    'bias_std_dps','status'
)
CSV_TRACE_COL = ('id','robo','percurso','rep','t_ms','fase',
                 'ang_global_deg','ang_local_deg','gz_dps','v','i')
CSV_SESSAO_COL = (
    'id_sessao','robo','bateria','superficie','posicao_mpu','percurso',
    'quadrados','reps_previstas','reps_validas',
    'tensao_media_v','corrente_media_a','erro_global_medio_deg',
    'erro_global_dp_deg','erro_global_abs_medio_deg','erro_global_max_abs_deg',
    'tempo_reto_medio_s_quad','ajuste_frente_medio_ms','ajuste_re_medio_ms',
    'giro_dir_medio_deg','giro_esq_medio_deg',
    'desvio_reto1_medio_deg','desvio_reto2_medio_deg','desvio_reto3_medio_deg'
)

def formatar(v):
    if v is None:
        return ''
    if isinstance(v, float):
        return '{:.5f}'.format(v)
    return str(v)


def salvar_csv(nome, cab, registro):
    novo = False
    try:
        with open(nome, 'r'):
            pass
    except OSError:
        novo = True
    with open(nome, 'a') as f:
        if novo:
            f.write(','.join(cab) + '\n')
        f.write(','.join(formatar(v) for v in registro) + '\n')
        f.flush()


def proximo_id():
    maior = 0
    try:
        with open(CSV_RESUMO, 'r') as f:
            for linha in f:
                primeiro = linha.split(',', 1)[0]
                if primeiro.isdigit():
                    maior = max(maior, int(primeiro))
    except OSError:
        pass
    return maior + 1


def registrar_boot():
    try:
        motivo = machine.reset_cause()
    except Exception:
        motivo = -1
    salvar_csv(CSV_BOOT, ('reset_cause','nota'),
               (motivo,'nao_identifica_causa_sozinho'))


def linha_csv(id_, cfg, rep, evento, d=None, status=''):
    x = d or {}
    nomes = ('tempo_reto_total','s_quad','t_reto1','t_reto2','t_reto3',
             'desvio_reto1','desvio_reto2','desvio_reto3',
             'max1','max2','max3','giro_dir','giro_esq','t_giro_dir','t_giro_esq',
             'erro_global_antes','erro_global_final','ajuste_f','ajuste_r','pulsos',
             'v_media','i_media','v_min','i_max','n_ina','n_gyro',
             'erros_gyro','erros_ina','dt_max_ms','bias','bias_std')
    return (id_, evento, cfg['robo'], BATERIA_ID, SUPERFICIE, cfg['pos'],
            cfg['percurso'], cfg['quad'], rep, PWM_RETO,
            TEMPO_QUAD_MS * cfg['quad']) + tuple(x.get(nome) for nome in nomes) + (status,)


def salvar_trace(id_, cfg, rep, samples):
    if not SALVAR_TRACE or not samples:
        return
    novo = False
    try:
        with open(CSV_TRACE, 'r'):
            pass
    except OSError:
        novo = True
    with open(CSV_TRACE, 'a') as f:
        if novo:
            f.write(','.join(CSV_TRACE_COL) + '\n')
        for t, fase, ag, ar, gz, v, i in samples:
            f.write(','.join(formatar(x) for x in
                             (id_, cfg['robo'], cfg['percurso'], rep,
                              t, fase, ag, ar, gz, v, i)) + '\n')
        f.flush()


def media(vals):
    return sum(vals) / len(vals) if vals else None


def dp_amostral(vals):
    if len(vals) < 2:
        return None
    m = media(vals)
    return math.sqrt(sum((v-m)*(v-m) for v in vals) / (len(vals)-1))


def fnum(x, modelo):
    return '--' if x is None else modelo.format(x)


class Estatisticas:
    def __init__(self):
        self.n = 0
        self.n_ina = 0
        self.v_soma = 0.0
        self.i_soma = 0.0
        self.angulos = []
        self.tempos = []
        self.ajustes_f = []
        self.ajustes_r = []
        self.giros_dir = []
        self.giros_esq = []
        self.retos = ([], [], [])

    def adicionar(self, tele, d):
        self.n += 1
        self.n_ina += tele.n_ina
        self.v_soma += tele.v_soma
        self.i_soma += tele.i_soma
        self.angulos.append(d['erro_global_antes'])
        self.tempos.append(d['s_quad'])
        self.ajustes_f.append(d['ajuste_f'])
        self.ajustes_r.append(d['ajuste_r'])
        if d.get('giro_dir') is not None:
            self.giros_dir.append(d['giro_dir'])
        if d.get('giro_esq') is not None:
            self.giros_esq.append(d['giro_esq'])
        for idx, key in enumerate(('desvio_reto1', 'desvio_reto2', 'desvio_reto3')):
            if d.get(key) is not None:
                self.retos[idx].append(d[key])

    def vmedia(self):
        return self.v_soma / self.n_ina if self.n_ina else None

    def imedia(self):
        return self.i_soma / self.n_ina if self.n_ina else None


def salvar_resumo(id_sessao, cfg, e):
    valores = (id_sessao, cfg['robo'], BATERIA_ID, SUPERFICIE, cfg['pos'],
               cfg['percurso'], cfg['quad'], cfg['reps'], e.n,
               e.vmedia(), e.imedia(), media(e.angulos), dp_amostral(e.angulos),
               media([abs(a) for a in e.angulos]),
               max([abs(a) for a in e.angulos]) if e.angulos else None,
               media(e.tempos), media(e.ajustes_f), media(e.ajustes_r),
               media(e.giros_dir), media(e.giros_esq),
               media(e.retos[0]), media(e.retos[1]), media(e.retos[2]))
    salvar_csv(CSV_SESSAO, CSV_SESSAO_COL, valores)


def mostrar_resumo(e, cfg):
    pagina = 0
    soltar(BTN_A)
    soltar(BTN_B)
    while True:
        if pagina == 0:
            linhas = ('RESULTADOS {}/{}'.format(e.n, cfg['reps']),
                      'Vmed {}V'.format(fnum(e.vmedia(), '{:.3f}')),
                      'Imed {}A'.format(fnum(e.imedia(), '{:.3f}')),
                      'Ang {}g'.format(fnum(media(e.angulos), '{:+.2f}')))
        elif pagina == 1:
            arr = [abs(x) for x in e.angulos]
            linhas = ('ERRO GLOBAL',
                      'DP {}g'.format(fnum(dp_amostral(e.angulos), '{:.2f}')),
                      'AbsMed {}g'.format(fnum(media(arr), '{:.2f}')),
                      'MaxAbs {}g'.format(fnum(max(arr) if arr else None, '{:.2f}')))
        elif pagina == 2:
            linhas = ('TEMPO E AJUSTE',
                      'Reto {}s/q'.format(fnum(media(e.tempos), '{:.3f}')),
                      'Joy+ {}ms'.format(fnum(media(e.ajustes_f), '{:.0f}')),
                      'Joy- {}ms'.format(fnum(media(e.ajustes_r), '{:.0f}')))
        else:
            linhas = ('PERCURSO 3Q',
                      'D {}g'.format(fnum(media(e.giros_dir), '{:+.1f}')),
                      'E {}g'.format(fnum(media(e.giros_esq), '{:+.1f}')),
                      'A: pg B: nova')
        tela(*linhas, atualizar=False)
        print('RESUMO:', linhas)
        while True:
            if BTN_A.value() == 0:
                soltar(BTN_A)
                pagina = (pagina + 1) % (4 if cfg['percurso'] == 'PERCURSO 3Q' else 3)
                break
            if BTN_B.value() == 0:
                soltar(BTN_B)
                return
            painel.atualizar()
            time.sleep_ms(15)



def percurso_movimentos_main(tele):
    """Reproduz a sequencia de direcoes chamada no main.py.
    Cada comando avanca um bloco (0.8 s); giros sao feitos pelo MPU6050.
    Direcoes: 0=cima, 1=direita, 2=baixo, 3=esquerda.
    """
    comandos = (0, 0, 0, 0, 3, 3, 0, 0, 2, 1)
    orientacao = 0
    tempos = []
    status = 'ok'

    for indice, direcao in enumerate(comandos, 1):
        # No main.py, comandos consecutivos na mesma direcao recebem
        # uma pausa de 300 ms antes do proximo bloco.
        if indice > 1 and direcao == comandos[indice - 2]:
            tele.repouso(300, 'main_pausa_repetida')

        diferenca = (direcao - orientacao) % 4

        if diferenca:
            if diferenca == 2:
                giros = ('DIREITA', 'DIREITA')
            elif diferenca == 1:
                giros = ('DIREITA',)
            else:
                giros = ('ESQUERDA',)

            for numero_giro, sentido in enumerate(giros, 1):
                tela('MAIN giro {}/{}'.format(indice, len(comandos)),
                     'Direita 90g' if sentido == 'DIREITA' else 'Esquerda 90g',
                     'B: cancela', atualizar=False)
                _, _, status = giro_90(sentido, tele, 'main_giro{}'.format(indice))
                if status != 'ok':
                    return tempos, status
            orientacao = direcao

        tela('MAIN bloco {}/10'.format(indice), 'Frente 1 bloco',
             'B: cancela', atualizar=False)
        info, status = avancar_quadrados(1, tele, 'main_reto{}'.format(indice))
        tempos.append(info)
        if status != 'ok':
            return tempos, status

    return tempos, status


def ensaio(id_, cfg, rep, resumo):
    parar()
    tipo = cfg['percurso']
    quad = cfg['quad']
    tela('Ensaio {}/{}'.format(rep, cfg['reps']),
         ('3Q com giros' if tipo == 'PERCURSO 3Q' else ('Movimentos main (10 blocos)' if tipo == 'MOVIMENTOS MAIN' else '{} quad frente'.format(quad))),
         'Posicione robo', 'A: iniciar')
    esperar_a()
    time.sleep_ms(160)
    tela('Calibrando MPU', 'Nao mexa')
    try:
        bias, sd = mpu.calibrar()
    except OSError as exc:
        tela('Falha MPU', str(exc)[:16])
        return False
    if sd > BIAS_STD_MAX:
        tela('Gyro instavel', 'std {:.2f}dps'.format(sd))
        return False
    painel.zerar()
    painel.atualizar(forcar=True)
    if not contar():
        tela('Cancelado', 'Sem movimento')
        return False
    d = {'bias':bias, 'bias_std':sd}
    salvar_csv(CSV_RESUMO, CSV_COL,
               linha_csv(id_, cfg, rep, 'INICIO', d, 'executando'))
    gc.collect()
    tele = Telemetria()
    tempos_reto = []
    f = r = pulsos = 0
    status = 'ok'
    try:
        if tipo == 'MOVIMENTOS MAIN':
            # Sequencia equivalente aos comandos _robo_ir_para() do main.py.
            infos, status = percurso_movimentos_main(tele)
            tempos_reto = [x['tempo_ms'] for x in infos]
            for idx, info in enumerate(infos[:3], 1):
                d['t_reto{}'.format(idx)] = info['tempo_ms']
                d['desvio_reto{}'.format(idx)] = info['desvio_deg']
                d['max{}'.format(idx)] = info['desvio_max_deg']
        else:
            # Percursos originais da calibracao v2.
            for segmento in range(1, (4 if tipo == 'PERCURSO 3Q' else 2)):
                if tipo == 'PERCURSO 3Q' and segmento == 2:
                    tela('GIRO 1/2', 'Direita 90g', 'B: cancela', atualizar=False)
                    ang, ms, status = giro_90('DIREITA', tele, 'giro_dir')
                    d['giro_dir'], d['t_giro_dir'] = ang, ms
                    if status != 'ok':
                        print('Falha giro direito:', status, 'angulo:', ang)
                        break
                if tipo == 'PERCURSO 3Q' and segmento == 3:
                    tela('GIRO 2/2', 'Esquerda 90g', 'B: cancela', atualizar=False)
                    ang, ms, status = giro_90('ESQUERDA', tele, 'giro_esq')
                    d['giro_esq'], d['t_giro_esq'] = ang, ms
                    if status != 'ok':
                        print('Falha giro esquerdo:', status, 'angulo:', ang)
                        break
                n = 1 if tipo == 'PERCURSO 3Q' else quad
                tela('TRECHO {}/{}'.format(segmento, 3 if tipo == 'PERCURSO 3Q' else 1),
                     '{} quadrado(s)'.format(n), 'B: cancela', atualizar=False)
                info, status = avancar_quadrados(n, tele, 'reto{}'.format(segmento))
                tempos_reto.append(info['tempo_ms'])
                d['t_reto{}'.format(segmento)] = info['tempo_ms']
                d['desvio_reto{}'.format(segmento)] = info['desvio_deg']
                d['max{}'.format(segmento)] = info['desvio_max_deg']
                if status != 'ok':
                    break
        if status == 'ok':
            d['erro_global_antes'] = tele.theta
            painel.finalizar(tele)
            f, r, pulsos, valido, status = ajuste_joystick(tele)
            d['erro_global_final'] = tele.theta
        else:
            d['erro_global_antes'] = tele.theta
            d['erro_global_final'] = tele.theta
    finally:
        parar()
    d.update(tele.dados())
    d['tempo_reto_total'] = sum(tempos_reto) if tempos_reto else None
    d['s_quad'] = d['tempo_reto_total'] / ((10 if tipo == 'MOVIMENTOS MAIN' else (3 if tipo == 'PERCURSO 3Q' else quad)) * 1000.0) if len(tempos_reto) == (10 if tipo == 'MOVIMENTOS MAIN' else (3 if tipo == 'PERCURSO 3Q' else 1)) else None
    d['ajuste_f'], d['ajuste_r'], d['pulsos'] = f, r, pulsos
    painel.finalizar(tele)
    try:
        salvar_trace(id_, cfg, rep, tele.trace)
        salvar_csv(CSV_RESUMO, CSV_COL, linha_csv(id_, cfg, rep, 'FIM', d, status))
    except OSError as exc:
        print('Erro CSV:', repr(exc))
        tela('Erro ao gravar', 'Veja Thonny')
        return False
    if status == 'ok':
        resumo.adicionar(tele, d)
    tela('Ensaio {}'.format(rep), status,
         'Ang {}g'.format(fnum(d.get('erro_global_antes'), '{:+.2f}')),
         'Reto {}s/q'.format(fnum(d.get('s_quad'), '{:.3f}')))
    print('ENSAIO', id_, 'status', status,
          'giro dir', d.get('giro_dir'), 'giro esq', d.get('giro_esq'),
          'erro final', d.get('erro_global_antes'))
    time.sleep_ms(350)
    return status == 'ok'



def decidir_apos_falha():
    """A repete a MESMA repeticao, B pula para a proxima.

    O motor ja foi desligado e o resultado gravado como falha no CSV.
    """
    parar()
    tela('TESTE INTERROMPIDO', 'A: repetir', 'B: proximo', 'Ver status serial')
    print('Falha: A = repetir a repeticao; B = pular.')
    # Evita interpretar o pressionamento anterior como nova escolha.
    soltar(BTN_A)
    soltar(BTN_B)
    while True:
        if BTN_A.value() == 0:
            soltar(BTN_A)
            return True
        if BTN_B.value() == 0:
            soltar(BTN_B)
            return False
        painel.atualizar()
        time.sleep_ms(15)


def configurar_sessao():
    if oled is None:
        print('Sem OLED: usando PADRAO:', PADRAO)
        return dict(PADRAO)
    cfg = {'robo': escolher('Numero robo', ROBO_IDS),
           'pos': escolher('Posicao MPU', POSICOES_MPU),
           'percurso': escolher('Percurso', PERCURSOS)}
    if cfg['percurso'] == 'SO FRENTE':
        cfg['quad'] = escolher('Quadrados', QUADRADOS)
    elif cfg['percurso'] == 'PERCURSO 3Q':
        cfg['quad'] = 3
        tela('Percurso fixo', '3 quadrados', 'F D90 F E90 F')
        time.sleep_ms(350)
    else:
        cfg['quad'] = 10
        tela('Percurso MAIN', '10 blocos', 'Rota do main.py')
        time.sleep_ms(500)
    cfg['reps'] = escolher('Repeticoes', REPETICOES)
    return cfg


def main():
    global oled, mpu, ina
    parar()
    registrar_boot()
    b0 = I2C(0, sda=Pin(0), scl=Pin(1), freq=FREQ_I2C)
    b1 = I2C(1, sda=Pin(2), scl=Pin(3), freq=FREQ_I2C)
    s0, s1 = b0.scan(), b1.scan()
    print('I2C0:', [hex(x) for x in s0], 'I2C1:', [hex(x) for x in s1])
    end_oled = 0x3C if 0x3C in s1 else (0x3D if 0x3D in s1 else None)
    if end_oled is not None:
        try:
            oled = Tela(b1, end_oled)
            print('OLED SH1107 pronto')
        except Exception as e:
            print('OLED falhou', repr(e))
            oled = None
    end_mpu = 0x68 if 0x68 in s0 else (0x69 if 0x69 in s0 else None)
    if end_mpu is None:
        tela('MPU ausente', 'Nao mover motores')
        return
    mpu = MPU6050(b0, end_mpu)
    if 0x40 in s1:
        try:
            ina = INA226(b1)
        except OSError as e:
            print('INA indisponivel', repr(e))
    tela('Calibrando gyro', 'Nao mover robo')
    try:
        mpu.calibrar()
    except OSError as e:
        print('Bias inicial falhou', repr(e))
    painel.zerar()
    painel.atualizar(forcar=True)
    prox_id = proximo_id()
    while True:
        cfg = configurar_sessao()
        print('SESSAO', cfg)
        resumo = Estatisticas()
        inicio_sessao = prox_id
        for rep in range(1, cfg['reps'] + 1):
            while True:
                sucesso = False
                try:
                    sucesso = ensaio(prox_id, cfg, rep, resumo)
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    parar()
                    print('Erro ensaio', repr(e))
                    sys.print_exception(e)
                # Cada tentativa usa ID proprio para preservar os CSVs.
                prox_id += 1
                if sucesso or oled is None:
                    break
                if not decidir_apos_falha():
                    break
        parar()
        print('RESUMO N', resumo.n, 'V', resumo.vmedia(), 'I', resumo.imedia(),
              'angulo medio', media(resumo.angulos), 'DP', dp_amostral(resumo.angulos))
        try:
            salvar_resumo(inicio_sessao, cfg, resumo)
        except OSError as e:
            print('Erro resumo', repr(e))
        if oled:
            mostrar_resumo(resumo, cfg)
        else:
            return


try:
    main()
except KeyboardInterrupt:
    print('Interrompido pelo usuario')
except Exception as e:
    print('ERRO FATAL', repr(e))
    sys.print_exception(e)
finally:
    parar()


