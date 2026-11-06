from machine import Pin, PWM, I2C, ADC
from time import sleep, sleep_ms, ticks_ms, ticks_diff, ticks_add
import framebuf
import ujson

# ============================================================
# CONFIGURACAO DO EXPERIMENTO
# ============================================================
QUADRADOS_OPCOES = [1, 2, 3, 4]
TOTAL_REPETICOES_OPCOES = [5, 10, 15, 20]  # NOVO: amostragem maior, escolhida no boot

ESTIMATIVA_TEMPO_POR_QUAD_MS = 763

# NOVO: intervalo de amostragem de tensao/corrente durante o movimento (ms)
AMOSTRAGEM_INTERVALO_MS = 50

# Mestre kenji fazendo kenjices

MPU_I2C = 0
MPU_SDA = 0
MPU_SCL = 1

OLED_I2C = 1
OLED_SDA = 2
OLED_SCL = 3

INA_I2C = 1
INA_SDA = 2
INA_SCL = 3

BTN_A = Pin(5, Pin.IN, Pin.PULL_UP)
BTN_B = Pin(6, Pin.IN, Pin.PULL_UP)
JOY_Y = ADC(Pin(26))
JOY_CENTRO = 32768
JOY_ZONA_MORTA = 5000

LEFT_FWD = Pin(4, Pin.OUT)
LEFT_BWD = Pin(9, Pin.OUT)
LEFT_PWM = PWM(Pin(8))

RIGHT_FWD = Pin(18, Pin.OUT)
RIGHT_BWD = Pin(19, Pin.OUT)
RIGHT_PWM = PWM(Pin(16))

STBY = Pin(20, Pin.OUT)

PWM_FREQ = 1000
SPEED_MOVE = 35000

CORRENTE_MAX_A = 1.5
ARQUIVO_CALIBRACAO = "calibracao.json"

# ============================================================
# OLED SSD1306
# ============================================================
SET_CONTRAST = 0x81
SET_ENTIRE_ON = 0xA4
SET_NORM_INV = 0xA6
SET_DISP = 0xAE
SET_MEM_ADDR = 0x20
SET_COL_ADDR = 0x21
SET_PAGE_ADDR = 0x22
SET_DISP_START_LINE = 0x40
SET_SEG_REMAP = 0xA0
SET_MUX_RATIO = 0xA8
SET_COM_OUT_DIR = 0xC0
SET_DISP_OFFSET = 0xD3
SET_COM_PIN_CFG = 0xDA
SET_DISP_CLK_DIV = 0xD5
SET_PRECHARGE = 0xD9
SET_VCOM_DESEL = 0xDB
SET_CHARGE_PUMP = 0x8D

class SSD1306:
    def __init__(self, width, height, external_vcc=False):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = height // 8
        self.poweron()
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00,
            SET_MEM_ADDR, 0x00,
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08,
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL, 0x30,
            SET_CONTRAST, 0xFF,
            SET_ENTIRE_ON,
            SET_NORM_INV,
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01
        ):
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def fill(self, col):
        self.framebuf.fill(col)

    def pixel(self, x, y, col):
        self.framebuf.pixel(x, y, col)

    def text(self, string, x, y, col=1):
        self.framebuf.text(string, x, y, col)

class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.buffer = bytearray((height // 8) * width + 1)
        self.buffer[0] = 0x40
        self.framebuf = framebuf.FrameBuffer1(memoryview(self.buffer)[1:], width, height)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_framebuf(self):
        self.i2c.writeto(self.addr, self.buffer)

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

    def poweron(self):
        pass

    def msg(self, l0="", l1="", l2="", l3=""):
        self.fill(0)
        self.text(str(l0)[:16], 0, 0)
        self.text(str(l1)[:16], 0, 16)
        self.text(str(l2)[:16], 0, 32)
        self.text(str(l3)[:16], 0, 48)
        self.show()

# ============================================================
# MPU6050
# ============================================================
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.offset_z = 0.0
        self.is_ready = False
        try:
            self.i2c.writeto_mem(addr, 0x6B, b"\x00")
            self.i2c.writeto_mem(addr, 0x1B, b"\x00")
            self.is_ready = True
        except Exception as exc:
            print("MPU6050: erro ao inicializar:", exc)

    def _read_i16(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        value = (data[0] << 8) | data[1]
        return value - 65536 if value > 32767 else value

    def _gz_raw(self):
        if not self.is_ready:
            return 0.0
        return self._read_i16(0x47) / 131.0

    def gz(self):
        return self._gz_raw() - self.offset_z

    def calibrate(self, samples=300):
        if not self.is_ready:
            return False
        total = 0.0
        for _ in range(samples):
            total += self._gz_raw()
            sleep_ms(5)
        self.offset_z = total / samples
        return True

# ============================================================
# INA226
# ============================================================
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
        if not self.is_ready:
            return 0.0
        return self._read_register(BUS_VOLTAGE_REG) * 1.25 / 1000

    def current(self):
        if not self.is_ready or self.shunt_resistor == 0:
            return 0.0
        shunt_voltage = self._read_signed_register(SHUNT_VOLTAGE_REG) * 2.5 / 1000000
        return -(shunt_voltage / self.shunt_resistor)

# ============================================================
# CONTROLES
# ============================================================
def wait_btn_a_press():
    while BTN_A.value() == 1:
        sleep_ms(30)
    sleep_ms(80)
    while BTN_A.value() == 0:
        sleep_ms(30)

def wait_btn_b_release():
    while BTN_B.value() == 0:
        sleep_ms(30)

def choose_index(title, options):
    index = 0
    while True:
        oled.msg(title, "A muda opcao", str(options[index]), "B p/ confirmar")
        while BTN_A.value() == 1 and BTN_B.value() == 1:
            sleep_ms(20)
        if BTN_B.value() == 0:
            while BTN_B.value() == 0:
                sleep_ms(20)
            return index
        if BTN_A.value() == 0:
            index = (index + 1) % len(options)
            while BTN_A.value() == 0:
                sleep_ms(20)
            sleep_ms(120)

# ============================================================
# MOTORES
# ============================================================
def setup_motors():
    LEFT_PWM.freq(PWM_FREQ)
    RIGHT_PWM.freq(PWM_FREQ)
    stop()

def pwm(value):
    return max(0, min(65535, int(value)))

def stop():
    LEFT_FWD.value(0)
    LEFT_BWD.value(0)
    LEFT_PWM.duty_u16(0)
    RIGHT_FWD.value(0)
    RIGHT_BWD.value(0)
    RIGHT_PWM.duty_u16(0)

def forward(duty=SPEED_MOVE):
    STBY.value(1)
    LEFT_FWD.value(0)
    LEFT_BWD.value(1)
    RIGHT_FWD.value(0)
    RIGHT_BWD.value(1)
    LEFT_PWM.duty_u16(pwm(duty))
    RIGHT_PWM.duty_u16(pwm(duty))

def backward(duty=SPEED_MOVE):
    STBY.value(1)
    LEFT_FWD.value(1)
    LEFT_BWD.value(0)
    RIGHT_FWD.value(1)
    RIGHT_BWD.value(0)
    LEFT_PWM.duty_u16(pwm(duty))
    RIGHT_PWM.duty_u16(pwm(duty))

def read_battery():
    try:
        return ina.voltage()
    except Exception:
        return 0.0

def read_current():
    try:
        return abs(ina.current())
    except Exception:
        return 0.0

def halt_message(l0, l1="", l2="", l3=""):
    stop()
    while True:
        oled.msg(l0, l1, l2, l3)
        sleep(1)

# ============================================================
# UMA REPETICAO
# ============================================================
def run_repetition(target):
    """
    NOVO: alem de mover o robo, agora amostra tensao (INA226) e corrente
    continuamente durante o movimento (a cada AMOSTRAGEM_INTERVALO_MS).
    Isso da, por repeticao:
      - v_min / current_max reais (antes ficavam travados no valor inicial)
      - tensao_media: media da tensao durante o deslocamento, para cruzar
        com seg_por_quad numa regressao linear tensao x velocidade.
    """
    status = "ok"

    tempo_base_ms = target * ESTIMATIVA_TEMPO_POR_QUAD_MS
    oled.fill(0)
    oled.text("Andando...", 10, 20, 1)
    oled.show()

    v_initial = read_battery()
    v_min = v_initial
    current_initial = read_current()
    current_max = current_initial
    soma_tensao = v_initial
    n_amostras = 1

    def amostrar():
        nonlocal v_min, current_max, soma_tensao, n_amostras
        v = read_battery()
        i = read_current()
        soma_tensao += v
        n_amostras += 1
        if v < v_min:
            v_min = v
        if i > current_max:
            current_max = i

    forward(SPEED_MOVE)
    t_fim = ticks_add(ticks_ms(), tempo_base_ms)
    while ticks_diff(t_fim, ticks_ms()) > 0:
        sleep_ms(AMOSTRAGEM_INTERVALO_MS)
        amostrar()
    stop()

    tempo_ajuste_ms = 0

    while True:
        oled.fill(0)
        oled.text("Ajuste o robo", 10, 10, 1)
        oled.text("com o Joystick", 10, 20, 1)
        oled.text("B p/ confirmar", 10, 45, 1)
        oled.show()

        jy = JOY_Y.read_u16()
        dy = JOY_CENTRO - jy

        if dy > JOY_ZONA_MORTA:
            t_inicio = ticks_ms()
            forward(SPEED_MOVE)
            while (JOY_CENTRO - JOY_Y.read_u16()) > JOY_ZONA_MORTA:
                sleep_ms(AMOSTRAGEM_INTERVALO_MS)
                amostrar()
            stop()
            tempo_ajuste_ms += ticks_diff(ticks_ms(), t_inicio)

        elif dy < -JOY_ZONA_MORTA:
            t_inicio = ticks_ms()
            backward(SPEED_MOVE)
            while (JOY_CENTRO - JOY_Y.read_u16()) < -JOY_ZONA_MORTA:
                sleep_ms(AMOSTRAGEM_INTERVALO_MS)
                amostrar()
            stop()
            tempo_ajuste_ms -= ticks_diff(ticks_ms(), t_inicio)

        if BTN_B.value() == 0:
            while BTN_B.value() == 0:
                sleep_ms(20)
            break

        sleep_ms(20)

    elapsed_ms = tempo_base_ms + tempo_ajuste_ms
    tensao_media = soma_tensao / n_amostras
    # OBS: mpu.gz() so da a velocidade angular no instante final, nao um
    # desvio acumulado. Se quiser desvio angular de verdade (integral de gz
    # ao longo do trajeto), me avisa que eu adiciono isso tambem.
    desvio = mpu.gz() if mpu.is_ready else 0.0
    seg_por_quad = (elapsed_ms / 1000.0) / target

    return (elapsed_ms, seg_por_quad, v_initial, v_min, tensao_media,
            current_initial, current_max, desvio, status)

def next_experiment_id(filename):
    last_id = 0
    try:
        with open(filename, "r") as existing:
            first = True
            for line in existing:
                if first:
                    first = False
                    continue
                parts = line.strip().split(",")
                if parts and parts[0].isdigit():
                    value = int(parts[0])
                    if value > last_id:
                        last_id = value
    except OSError:
        pass
    return "{:02d}".format(last_id + 1)

# ============================================================
# MAIN
# ============================================================
i2c_mpu = I2C(MPU_I2C, sda=Pin(MPU_SDA), scl=Pin(MPU_SCL), freq=400000)
i2c_oled = I2C(OLED_I2C, sda=Pin(OLED_SDA), scl=Pin(OLED_SCL), freq=400000)
i2c_ina = I2C(INA_I2C, sda=Pin(INA_SDA), scl=Pin(INA_SCL), freq=400000)

oled_scan = i2c_oled.scan()
oled_addr = 0x3C if 0x3C in oled_scan else (0x3D if 0x3D in oled_scan else 0x3C)
oled = SSD1306_I2C(128, 64, i2c_oled, addr=oled_addr)
mpu = MPU6050(i2c_mpu)
ina = INA226(i2c_ina)
setup_motors()

if not ina.is_ready:
    try:
        scan = i2c_ina.scan()
    except Exception:
        scan = []
    halt_message("ERRO INA226", "I2C1 GP2/GP3", "scan: {}".format(scan), "resetar")

if not mpu.is_ready:
    print("Aviso: MPU6050 nao encontrado - desvio angular nao sera registrado.")

quad_index = choose_index("Quadrados alvo", QUADRADOS_OPCOES)
target = QUADRADOS_OPCOES[quad_index]

# NOVO: escolhe quantas repeticoes rodar (amostragem maior p/ regressao)
rep_index = choose_index("Repeticoes", TOTAL_REPETICOES_OPCOES)
TOTAL_REPETICOES = TOTAL_REPETICOES_OPCOES[rep_index]

filename = "quad_{}.csv".format(target)
experiment_id = next_experiment_id(filename)
oled.msg("ID automatico", "Experimento", experiment_id, "aguarde")
sleep(1.5)

if mpu.is_ready:
    oled.msg("Calibrando MPU", "Nao mexa robo", "", "")
    mpu.calibrate()

new_file = False
try:
    with open(filename, "r"):
        pass
except OSError:
    new_file = True

csv = open(filename, "a")
if new_file:
    # NOVO: coluna tensao_media, usada na regressao tensao x velocidade
    csv.write(
        "id,repeticao,quadrados_alvo,tempo_ms,segundos_por_quadrado,"
        "bateria_inicial,bateria_minima,tensao_media,corrente_inicial,corrente_maxima,"
        "desvio_angular,status\n"
    )
csv.flush()

resultados_validos = []       # seg_por_quad de cada rep valida
tensoes_medias_validas = []   # tensao_media de cada rep valida (mesmo indice)

for repetition in range(1, TOTAL_REPETICOES + 1):
    oled.msg("Rep {}/{}".format(repetition, TOTAL_REPETICOES),
             "{} quadrados".format(target),
             "A inicia",
             "B marca fim")
    wait_btn_a_press()

    (elapsed_ms, seg_por_quad, v_initial, v_min, tensao_media,
     current_initial, current_max, desvio, status) = run_repetition(target)

    csv.write("{},{},{},{},{},{:.2f},{:.2f},{:.2f},{:.3f},{:.3f},{:.2f},{}\n".format(
        experiment_id, repetition, target, elapsed_ms,
        "{:.3f}".format(seg_por_quad) if seg_por_quad is not None else "",
        v_initial, v_min, tensao_media, current_initial, current_max, desvio, status
    ))
    csv.flush()

    if status == "ok":
        resultados_validos.append(seg_por_quad)
        tensoes_medias_validas.append(tensao_media)
        oled.msg("Rep {}/{} OK".format(repetition, TOTAL_REPETICOES),
                  "{:.3f} s/quad".format(seg_por_quad),
                  "V med {:.2f}".format(tensao_media),
                  "Imax {:.2f}A".format(current_max))
    else:
        oled.msg("Rep {}/{} FALHOU".format(repetition, TOTAL_REPETICOES),
                  status,
                  "Imax {:.2f}A".format(current_max),
                  "")
    sleep(1.5)

csv.close()
stop()

if resultados_validos:
    media = sum(resultados_validos) / len(resultados_validos)
    v_referencia = sum(tensoes_medias_validas) / len(tensoes_medias_validas)
    try:
        with open(ARQUIVO_CALIBRACAO, "w") as f:
            # NOVO: pares (tensao_media, seg_por_quad) prontos para regressao linear
            ujson.dump({
                "segundos_por_quadrado": media,
                "v_referencia": v_referencia,
                "amostras_seg_por_quad": resultados_validos,
                "amostras_tensao_media": tensoes_medias_validas,
            }, f)
    except OSError as e:
        print("Erro ao salvar calibracao:", e)

    while True:
        oled.msg("CONCLUIDO",
                  "{:.3f} s/quad".format(media),
                  "{:.2f}V ref".format(v_referencia),
                  filename[:16])
        sleep(1)
else:
    while True:
        oled.msg("SEM DADOS VALIDOS", "todas falharam", "veja o CSV:", filename[:16])
        sleep(1)
