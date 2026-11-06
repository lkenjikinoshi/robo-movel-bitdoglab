# BitDogLab + TCS34725 -- log de cores no tapete + tempo por quadrado
#
# O robo anda para frente e o sensor grava TODAS as leituras (R, G, B, Clear)
# na RAM, a ~200 amostras/s. Quando o robo para, o codigo:
#   1) mostra as passagens pelas linhas escuras (queda do Clear) e o tempo
#      entre linhas consecutivas  ==> tempo de 1 quadrado;
#   2) resume as cores que o sensor viu;
#   3) salva TRES arquivos CSV (nada e sobrescrito, so acrescentado/novo):
#        cores_NNN.csv       -> todas as amostras da corrida NNN
#        resumo_corridas.csv -> 1 linha por corrida (acumula todas)
#        linhas_tapete.csv   -> linhas escuras detectadas + intervalos
#   4) mostra no OLED 3 telas: resumo, medias R/G/B/C e arquivos salvos.
#
# Durante o movimento o OLED mostra R, G, B, Clear atuais, tempo decorrido
# e quantidade de amostras (atualizacao lenta, para nao tirar amostras).
#
# Hardware (igual ao teste_robo_calibracao_v2.py):
#   motores: LE_F=4 LE_R=9 PWM_E=8 | LD_F=18 LD_R=19 PWM_D=16 | STBY=20
#   botoes: A=GP5 (confirma) B=GP6 (cancela/para)
#   I2C1: SDA=GP2 SCL=GP3 -> TCS34725 (0x29), INA226 opcional (0x40)
#
# SEGURANCA: teste primeiro com o robo suspenso; depois pista desobstruida.
# O robo para sozinho apos 3,6 s ou ao apertar B. Leituras sao salvas em CSV.

from machine import Pin, PWM, I2C
import time
import sys
import array
import gc
import os

# Driver OLED SH1107 usado na BitDogLab
try:
    from sh1107_bitdoglab import SH1107_I2C
except ImportError:
    SH1107_I2C = None

# ---------------- CONFIGURACAO ----------------
ROBO_ID = 1
BATERIA_ID = 'B01'            # edite se trocar de bateria

I2C_ID = 1
SDA_PIN = 2
SCL_PIN = 3
I2C_FREQ = 100000             # barramento compartilhado OLED/TCS/INA
TCS_ADDR = 0x29
INA_ADDR = 0x40

# Integracao CURTA: a linha e fina e passa rapido pelo sensor.
# tempo de integracao = 2.4 ms * (256 - ATIME)
#   0xFF = 2.4 ms | 0xFE = 4.8 ms | 0xFD = 7.2 ms | 0xF6 = 24 ms
# contagem maxima do canal = 1024 * (256 - ATIME)
ATIME = 0xFE
GANHO_AUTO = True             # escolhe o maior ganho que nao satura no claro
GANHO_FIXO = 2                # usado se GANHO_AUTO=False: 0=1x 1=4x 2=16x 3=60x
PERIODO_US = 5000             # periodo de amostragem (~ tempo de integracao)

PWM_FREQ = 1000
PWM_RETO = 35000              # igual ao codigo de calibracao
PRE_STBY_MS = 50
DURACAO_MAX_MS = 3600        # passo 3: anda por ate 3,6 s
PARAR_APOS_LINHAS = None      # ex.: 4 para parar apos 4 linhas; None = so tempo/B
MAX_AMOSTRAS = 7400           # suficiente para 36 s a cada 5 ms

# Deteccao de linha (histerese sobre o canal Clear):
#   limiar = base + fracao * (C_claro - base)
#   base = C medido sobre a linha (passo 2) ou 0 se o passo 2 for pulado
ENTRA = 0.35                  # Clear abaixo disso => entrou na linha
SAI = 0.50                    # Clear acima disso  => saiu da linha
MIN_INTERVALO_LINHAS_MS = 250  # ignora "linhas" mais proximas que isso
CONTAGEM_MS = 3000

IMPRIMIR_A_CADA = 0           # 0 evita despejar milhares de linhas no Thonny; CSV continua completo
CSV_LINHAS = 'linhas_tapete.csv'
CSV_RESUMO = 'resumo_corridas.csv'
OLED_ADDRS = (0x3C, 0x3D)
OLED_ATUALIZACAO_MS = 1000      # atualiza durante o movimento sem sobrecarregar o I2C
OLED_LINHAS = 6                 # 128x128 px, fonte 8x8: 6 linhas de 16 caracteres
TELA_RESUMO_MS = 6000           # tempo das telas 1 e 2 (A avanca antes)
# False: tabelas longas (linhas, cores) vao so ao Thonny; o OLED mostra as 3
# telas finais. True: espelha tudo no OLED (muito lento: 1,1 s por pagina).
ESPELHAR_DETALHES_NO_OLED = False

# ---------------- TCS34725 ----------------
REG_ENABLE = 0x00
REG_ATIME = 0x01
REG_CONTROL = 0x0F
REG_ID = 0x12
REG_CDATAL = 0x14
CMD = 0x80
AUTO = 0x20
GANHOS = ((3, 60), (2, 16), (1, 4), (0, 1))

INT_US = 2400 * (256 - ATIME)
C_MAX = min(65535, 1024 * (256 - ATIME))

# ---------------- HARDWARE ----------------
BTN_A = Pin(5, Pin.IN, Pin.PULL_UP)
BTN_B = Pin(6, Pin.IN, Pin.PULL_UP)

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


# OLED ativo globalmente: mensagens enviadas por log() tambem vao ao display.
OLED_ATIVO = None


def log(*args):
    """Imprime no Thonny e percorre a mesma mensagem no OLED, em paginas."""
    global OLED_ATIVO
    msg = ' '.join(str(x) for x in args)
    print(msg)  # console USB/Thonny
    if OLED_ATIVO is None:
        return

    linhas = []
    for original in msg.split('\n'):
        texto = original.strip()
        if not texto:
            continue
        while len(texto) > 16:
            corte = texto.rfind(' ', 0, 17)
            if corte <= 0:
                corte = 16
            linhas.append(texto[:corte])
            texto = texto[corte:].strip()
        linhas.append(texto)

    if not linhas:
        return
    for inicio in range(0, len(linhas), OLED_LINHAS):
        try:
            OLED_ATIVO.texto(linhas[inicio:inicio + OLED_LINHAS])
            time.sleep_ms(1100)
        except Exception:
            break


def logd(*args):
    """Mensagem de detalhe (tabelas longas): console; OLED so se configurado."""
    if ESPELHAR_DETALHES_NO_OLED:
        log(*args)
    else:
        print(' '.join(str(x) for x in args))


def parar():
    PWM_E.duty_u16(0)
    PWM_D.duty_u16(0)
    STBY.value(0)
    LE_F.value(0)
    LE_R.value(0)
    LD_F.value(0)
    LD_R.value(0)


def preparar_frente():
    parar()
    LE_F.value(0)
    LE_R.value(1)
    LD_F.value(0)
    LD_R.value(1)
    STBY.value(1)
    time.sleep_ms(PRE_STBY_MS)


def pwm_ligar():
    PWM_E.duty_u16(PWM_RETO)
    PWM_D.duty_u16(PWM_RETO)


def soltar(botao):
    while botao.value() == 0:
        time.sleep_ms(10)
    time.sleep_ms(25)


def aguardar(msg, oled=None, linhas_oled=None):
    """A -> True, B -> False. Mostra a instrucao no OLED e no Thonny."""
    log(msg)
    if linhas_oled is not None:
        tela(oled, linhas_oled)
    soltar(BTN_A)
    soltar(BTN_B)
    while True:
        if BTN_A.value() == 0:
            soltar(BTN_A)
            return True
        if BTN_B.value() == 0:
            soltar(BTN_B)
            return False
        time.sleep_ms(10)


def pausa_tela(ms=None):
    """Mantem a tela atual. ms=None espera so o botao A; senao, ms ou A."""
    soltar(BTN_A)
    fim = None if ms is None else time.ticks_add(time.ticks_ms(), ms)
    while fim is None or time.ticks_diff(fim, time.ticks_ms()) > 0:
        if BTN_A.value() == 0:
            soltar(BTN_A)
            return
        time.sleep_ms(10)


def contagem(oled=None):
    inicio = time.ticks_ms()
    ultimo = None
    while True:
        falta = CONTAGEM_MS - time.ticks_diff(time.ticks_ms(), inicio)
        if falta <= 0:
            return True
        if BTN_B.value() == 0:
            soltar(BTN_B)
            tela(oled, ['PARTIDA CANCELADA', 'Botao B pressionado'])
            return False
        s = (falta + 999) // 1000
        if s != ultimo:
            ultimo = s
            log('Partida em {} s (B cancela)'.format(s))
            tela(oled, ['PREPARANDO ROBO', 'Partida em:', '{} segundos'.format(s), 'B: cancelar'])
        time.sleep_ms(10)


class TelaOLED:
    """Interface SH1107 identica a usada no codigo de calibracao do robo.
    Ate 6 linhas de 16 caracteres (128x128 px, fonte 8x8)."""
    Y = (2, 23, 44, 65, 86, 107)

    def __init__(self, bus, addr):
        self.dev = SH1107_I2C(128, 128, bus, addr=addr, rotate_180=False)
        self.ant = bytearray(self.dev.buffer)
        self.linhas = None

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

    def texto(self, linhas):
        linhas = tuple(str(x)[:16] for x in linhas[:OLED_LINHAS])
        if linhas == self.linhas:
            return
        self.dev.fill_rect(0, 0, 128, 128, 0)
        for y, linha in zip(self.Y, linhas):
            self.dev.text(linha, 0, y, 1)
        self.enviar()
        self.linhas = linhas


def iniciar_oled(bus, achados):
    """Inicializa o SH1107 no endereco encontrado no barramento."""
    if SH1107_I2C is None:
        log('ERRO: arquivo sh1107_bitdoglab.py nao foi encontrado.')
        return None
    addr = 0x3C if 0x3C in achados else (0x3D if 0x3D in achados else None)
    if addr is None:
        log('ERRO: OLED nao detectado em 0x3C/0x3D.')
        return None
    try:
        disp = TelaOLED(bus, addr)
        disp.texto(['TESTE TCS34725', 'OLED funcionando', 'Inicializando...', 'Aguarde'])
        log('OLED SH1107 pronto em', hex(addr))
        return disp
    except Exception as e:
        log('ERRO ao inicializar OLED:', repr(e))
        return None


def tela(oled, linhas):
    if oled is None:
        return
    try:
        oled.texto(linhas)
    except Exception as e:
        print('Falha ao atualizar OLED:', repr(e))


class TCS34725:
    def __init__(self, bus, addr=0x29):
        self.bus = bus
        self.addr = addr
        self.buf = bytearray(8)
        self.ident = self.reg(REG_ID)
        self.escrever(REG_ATIME, ATIME)
        self.escrever(REG_ENABLE, 0x01)      # power on
        time.sleep_ms(5)
        self.escrever(REG_ENABLE, 0x03)      # power on + ADC

    def reg(self, r):
        return self.bus.readfrom_mem(self.addr, CMD | r, 1)[0]

    def escrever(self, r, v):
        self.bus.writeto_mem(self.addr, CMD | r, bytes([v]))

    def definir_ganho(self, cod):
        self.escrever(REG_CONTROL, cod)
        time.sleep_ms(3 * INT_US // 1000 + 5)   # espera ciclos novos

    def ler_buf(self):
        # sem alocacao: buf = C(2) R(2) G(2) B(2)
        self.bus.readfrom_mem_into(self.addr, CMD | AUTO | REG_CDATAL, self.buf)

    def rgbc(self):
        self.ler_buf()
        b = self.buf
        return (b[2] | (b[3] << 8), b[4] | (b[5] << 8),
                b[6] | (b[7] << 8), b[0] | (b[1] << 8))


def ler_vbat(bus):
    try:
        d = bus.readfrom_mem(INA_ADDR, 0x02, 2)
        return ((d[0] << 8) | d[1]) * 0.00125
    except OSError:
        return None


# ---------------- DETECTOR DE LINHAS ----------------
class DetectorLinha:
    """Histerese sobre o Clear. Evento = centro do trecho escuro (media entre
    a 1a e a ultima amostra abaixo do limiar), melhor que usar so a borda."""

    def __init__(self, thr_in, thr_out, min_int_us, periodo_us):
        self.thr_in = thr_in
        self.thr_out = thr_out
        self.min_int = min_int_us
        self.periodo = periodo_us
        self.escuro = False
        self.t_in = 0
        self.t_ult = 0
        self.c_min = 0
        self.ult_evento = None
        self.eventos = []          # (t_centro_us, largura_us, c_min)

    def passo(self, t, c):
        """Retorna True quando um evento de linha acabou de ser fechado."""
        if not self.escuro:
            if c < self.thr_in:
                self.escuro = True
                self.t_in = t
                self.t_ult = t
                self.c_min = c
            return False
        if c > self.thr_out:
            return self.fechar()
        self.t_ult = t
        if c < self.c_min:
            self.c_min = c
        return False

    def fechar(self):
        self.escuro = False
        centro = (self.t_in + self.t_ult) // 2
        largura = self.t_ult - self.t_in + self.periodo
        if self.ult_evento is not None and centro - self.ult_evento < self.min_int:
            return False
        self.ult_evento = centro
        self.eventos.append((centro, largura, self.c_min))
        return True


# ---------------- CALIBRACAO DE NIVEIS ----------------
def medir(sensor, ms=300):
    n = 0
    sr = sg = sb = sc = 0
    fim = time.ticks_add(time.ticks_ms(), ms)
    while time.ticks_diff(fim, time.ticks_ms()) > 0:
        r, g, b, c = sensor.rgbc()
        sr += r
        sg += g
        sb += b
        sc += c
        n += 1
        time.sleep_ms(6)
    return sr / n, sg / n, sb / n, sc / n


def escolher_ganho(sensor):
    """Maior ganho cujo Clear no CLARO fica abaixo de 60% do fundo de escala."""
    ult = None
    for cod, mult in GANHOS:
        sensor.definir_ganho(cod)
        ult = medir(sensor, 200)
        if ult[3] <= 0.6 * C_MAX:
            return cod, mult, ult
    return 0, 1, ult


# ---------------- COLETA ----------------
def coletar(sensor, bus, ina_ok, thr_in, thr_out, oled=None):
    n_max = min(MAX_AMOSTRAS, DURACAO_MAX_MS * 1000 // PERIODO_US + 20)
    ta = array.array('I', bytes(4 * n_max))
    ra = array.array('H', bytes(2 * n_max))
    ga = array.array('H', bytes(2 * n_max))
    ba = array.array('H', bytes(2 * n_max))
    ca = array.array('H', bytes(2 * n_max))
    ea = array.array('B', bytes(n_max))       # 1 = dentro de linha escura
    det = DetectorLinha(thr_in, thr_out,
                        MIN_INTERVALO_LINHAS_MS * 1000, PERIODO_US)
    dur_us = DURACAO_MAX_MS * 1000
    dur_s = DURACAO_MAX_MS // 1000
    buf = sensor.buf
    i = 0
    erros = 0
    erros_seq = 0
    atraso_max = 0
    v_min = None
    motivo = 'tempo_max'

    gc.collect()
    gc.disable()
    preparar_frente()
    t0 = time.ticks_us()          # mesma referencia do codigo de calibracao
    pwm_ligar()
    prox = t0
    ult_ina = t0
    t_parada = 0
    prox_oled = t0
    tela(oled, ['ROBO ANDANDO', 'Tempo: 0.0/{}s'.format(dur_s),
                'R:0 G:0', 'B:0 C:0', 'Amostras: 0', 'B: parar'])
    try:
        while True:
            agora = time.ticks_us()
            atraso = time.ticks_diff(agora, prox)
            if atraso < 0:
                continue
            if atraso > atraso_max:
                atraso_max = atraso
            if atraso > PERIODO_US:
                prox = agora              # perdeu amostras: nao fazer rajada
            prox = time.ticks_add(prox, PERIODO_US)
            t = time.ticks_diff(agora, t0)

            if BTN_B.value() == 0:
                motivo = 'botao_B'
                break
            if t >= dur_us:
                motivo = 'tempo_max'
                break
            if i >= n_max:
                motivo = 'buffer_cheio'
                break
            try:
                sensor.ler_buf()
            except OSError:
                erros += 1
                erros_seq += 1
                if erros_seq >= 5:
                    motivo = 'erro_i2c'
                    break
                continue
            erros_seq = 0

            c = buf[0] | (buf[1] << 8)
            ta[i] = t
            ca[i] = c
            ra[i] = buf[2] | (buf[3] << 8)
            ga[i] = buf[4] | (buf[5] << 8)
            ba[i] = buf[6] | (buf[7] << 8)
            fechou = det.passo(t, c)
            ea[i] = 1 if det.escuro else 0
            i += 1

            # Atualizacao lenta do OLED para reduzir interferencia na amostragem I2C.
            if time.ticks_diff(agora, prox_oled) >= OLED_ATUALIZACAO_MS * 1000:
                prox_oled = agora
                tela(oled, ['ROBO ANDANDO',
                            'Tempo:{:.1f}/{}s'.format(t / 1000000.0, dur_s),
                            'R:{} G:{}'.format(ra[i - 1], ga[i - 1]),
                            'B:{} C:{}'.format(ba[i - 1], ca[i - 1]),
                            'Amostras:{}'.format(i),
                            'Linhas:{} B:para'.format(len(det.eventos))])

            if (PARAR_APOS_LINHAS is not None and fechou
                    and len(det.eventos) >= PARAR_APOS_LINHAS):
                motivo = 'n_linhas'
                break

            if ina_ok and time.ticks_diff(agora, ult_ina) >= 100000:
                ult_ina = agora
                v = ler_vbat(bus)
                if v is not None and (v_min is None or v < v_min):
                    v_min = v
    finally:
        parar()
        t_parada = time.ticks_diff(time.ticks_us(), t0)
        gc.enable()
        tela(oled, ['ROBO PARADO', 'Amostras: {}'.format(i),
                    'Calculando...', 'Aguarde'])
    if det.escuro:
        det.fechar()
    return {'n': i, 't': ta, 'r': ra, 'g': ga, 'b': ba, 'c': ca, 'e': ea,
            'det': det, 'motivo': motivo, 'erros': erros,
            'atraso_max_us': atraso_max, 'v_min': v_min, 't_parada': t_parada}


# ---------------- ANALISE ----------------
def media(v):
    return sum(v) / len(v) if v else None


def dp_amostral(v):
    if len(v) < 2:
        return None
    m = media(v)
    return (sum((x - m) * (x - m) for x in v) / (len(v) - 1)) ** 0.5


def fnum(x, modelo):
    return '--' if x is None else modelo.format(x)


def fcsv(x, modelo):
    """Como fnum, mas campo vazio no CSV quando nao ha valor."""
    return '' if x is None else modelo.format(x)


def classificar(r, g, b, c, dentro, ref):
    """Rotulo indicativo (balanco de branco pelo claro medido no passo 1)."""
    if dentro:
        return 'LINHA'
    rb = r / ref['r']
    gb = g / ref['g']
    bb = b / ref['b']
    tot = rb + gb + bb
    if tot <= 0:
        return 'SEM_SINAL'
    pr, pg, pb = rb / tot, gb / tot, bb / tot
    frac = c / ref['c']
    if max(pr, pg, pb) - min(pr, pg, pb) < 0.10:
        if frac >= 0.75:
            return 'BRANCO'
        if frac >= 0.40:
            return 'CINZA'
        return 'ESCURO'
    if pb >= pr and pb >= pg:
        rot = 'AZUL'
    elif pr >= pg:
        rot = 'VERMELHO'
    else:
        rot = 'VERDE'
    return rot + ('_ESC' if frac < 0.40 else '')


def existe(nome):
    try:
        os.stat(nome)
        return True
    except OSError:
        return False


def maior_id(nome):
    maior = 0
    try:
        with open(nome, 'r') as f:
            for linha in f:
                p = linha.split(',', 1)[0]
                if p.isdigit():
                    maior = max(maior, int(p))
    except OSError:
        pass
    return maior


def nome_cores(id_):
    return 'cores_{:03d}.csv'.format(id_)


def proximo_id():
    """Maior id ja usado nos CSV de resumo, mesmo se um deles foi apagado,
    e nunca reaproveita um cores_NNN.csv que ja exista."""
    id_ = max(maior_id(CSV_LINHAS), maior_id(CSV_RESUMO)) + 1
    while existe(nome_cores(id_)):
        id_ += 1
    return id_


def anexar(nome, cab, linhas):
    """Acrescenta linhas ao CSV (cria com cabecalho se for novo).
    Nunca sobrescreve registros anteriores."""
    novo = not existe(nome)
    with open(nome, 'a') as f:
        if novo:
            f.write(cab + '\n')
        for s in linhas:
            f.write(s + '\n')
        f.flush()


def mostrar_linhas(det, c_claro):
    ev = det.eventos
    logd('\n=== LINHAS ESCURAS DETECTADAS ===')
    if not ev:
        logd('Nenhuma linha detectada.')
        logd('Veja o Clear no CSV e ajuste ENTRA/SAI, ou meca a linha no passo 2.')
        return []
    logd('linha | t_centro_ms | largura_ms | C_min (%claro) | intervalo_ms')
    ints = []
    prev = None
    for k, (centro, larg, cmin) in enumerate(ev, 1):
        itv = None if prev is None else (centro - prev) / 1000.0
        if itv is not None:
            ints.append(itv)
        logd('{:5d} | {:11.1f} | {:10.1f} | {:5d} ({:3.0f}%) | {}'.format(
            k, centro / 1000.0, larg / 1000.0, int(cmin),
            100.0 * cmin / c_claro, fnum(itv, '{:.1f}')))
        prev = centro
    if ints:
        logd('\n--- TEMPO POR QUADRADO (linha -> linha seguinte) ---')
        logd('n={} | media={:.1f} ms ({:.3f} s) | dp={} ms | min={:.1f} | max={:.1f}'.format(
            len(ints), media(ints), media(ints) / 1000.0,
            fnum(dp_amostral(ints), '{:.1f}'), min(ints), max(ints)))
        if len(ints) >= 2:
            logd('media dos intervalos 2..n = {:.1f} ms'.format(media(ints[1:])))
        logd('Obs.: se o robo partiu SOBRE a linha 1, o 1o intervalo inclui a')
        logd('aceleracao (e o que acontece num comando de 1 quadrado).')
    else:
        logd('\nApenas 1 linha detectada: nao ha intervalo para calcular.')
    return ints


def salvar_cores(nome, id_, run, ref, rotulos, t_mov_ms):
    """cores_NNN.csv: 1 linha por amostra. Retorna True se gravou tudo."""
    n = run['n']
    ta, ra, ga, ba, ca, ea = run['t'], run['r'], run['g'], run['b'], run['c'], run['e']
    cab = ('corrida,tempo_mov_ms,amostras_total,'
           't_ms,R,G,B,C,R8,G8,B8,classe,em_linha')
    pref = '{},{:.1f},{},'.format(id_, t_mov_ms, n)   # constante em todas as linhas
    if IMPRIMIR_A_CADA:
        logd('\n--- amostras (1 de cada {}) ---'.format(IMPRIMIR_A_CADA))
        logd(cab)
    try:
        with open(nome, 'w') as f:      # arquivo novo: id unico, nunca reaproveitado
            f.write(cab + '\n')
            for i in range(n):
                r, g, b = ra[i], ga[i], ba[i]
                s = '{:.3f},{},{},{},{},{},{},{},{},{}'.format(
                    ta[i] / 1000.0, r, g, b, ca[i],
                    min(255, int(255 * r / ref['r'])),
                    min(255, int(255 * g / ref['g'])),
                    min(255, int(255 * b / ref['b'])),
                    rotulos[i], ea[i])
                f.write(pref + s + '\n')
                if IMPRIMIR_A_CADA and i % IMPRIMIR_A_CADA == 0:
                    print(s)
            f.flush()
        logd('Amostras salvas em', nome)
        return True
    except OSError as e:
        print('Erro ao gravar amostras:', repr(e))
        return False


def salvar_linhas(id_, run, cfg, ref):
    """linhas_tapete.csv (formato original, acumulativo)."""
    det = run['det']
    n = run['n']
    cab_l = ('id,robo,bateria,v_repouso,v_min,motivo,linha,t_centro_ms,'
             'largura_ms,c_min,intervalo_ms,thr_in,thr_out,c_claro,c_linha,'
             'ganho,atime,periodo_us,pwm,duracao_ms,amostras')
    comum = (id_, ROBO_ID, BATERIA_ID, fnum(cfg['v_rep'], '{:.3f}'),
             fnum(run['v_min'], '{:.3f}'), run['motivo'])
    fim = (int(cfg['thr_in']), int(cfg['thr_out']), int(ref['c']),
           '' if cfg['c_linha'] is None else int(cfg['c_linha']),
           cfg['ganho'], ATIME, PERIODO_US, PWM_RETO,
           int(run['t_parada'] / 1000), n)
    linhas = []
    if not det.eventos:
        linhas.append(','.join(str(x) for x in comum + (0, '', '', '', '') + fim))
    prev = None
    for k, (centro, larg, cmin) in enumerate(det.eventos, 1):
        itv = '' if prev is None else '{:.1f}'.format((centro - prev) / 1000.0)
        linhas.append(','.join(str(x) for x in comum + (
            k, '{:.1f}'.format(centro / 1000.0),
            '{:.1f}'.format(larg / 1000.0), int(cmin), itv) + fim))
        prev = centro
    try:
        anexar(CSV_LINHAS, cab_l, linhas)
        logd('Linhas salvas em', CSV_LINHAS)
        return True
    except OSError as e:
        print('Erro ao gravar linhas:', repr(e))
        return False


def salvar_resumo(id_, run, cfg, t_mov_ms, medias, arquivo):
    """resumo_corridas.csv: 1 linha por corrida, acumulativo."""
    cab = ('id,robo,bateria,motivo,tempo_mov_ms,amostras,'
           'media_R,media_G,media_B,media_C,v_repouso,v_min,'
           'linhas_detectadas,arquivo_cores')
    s = ','.join((str(id_), str(ROBO_ID), BATERIA_ID, run['motivo'],
                  '{:.1f}'.format(t_mov_ms), str(run['n']),
                  fcsv(medias[0], '{:.2f}'), fcsv(medias[1], '{:.2f}'),
                  fcsv(medias[2], '{:.2f}'), fcsv(medias[3], '{:.2f}'),
                  fcsv(cfg['v_rep'], '{:.3f}'), fcsv(run['v_min'], '{:.3f}'),
                  str(len(run['det'].eventos)), arquivo))
    try:
        anexar(CSV_RESUMO, cab, (s,))
        logd('Resumo salvo em', CSV_RESUMO)
        return True
    except OSError as e:
        print('Erro ao gravar resumo:', repr(e))
        return False


def processar(id_, run, cfg, oled=None):
    ref = cfg['ref']
    n = run['n']
    ta, ra, ga, ba, ca, ea = run['t'], run['r'], run['g'], run['b'], run['c'], run['e']
    det = run['det']
    logd('\n=== CORRIDA {} ==='.format(id_))
    t_fim = ta[n - 1] / 1000.0 if n else 0
    # Tempo total de movimento: motores ligados ate o comando de parar.
    t_mov_ms = run['t_parada'] / 1000.0
    logd('motivo da parada: {} | amostras: {} | tempo de movimento: {:.0f} ms | '
         'ultima amostra: {:.0f} ms'.format(run['motivo'], n, t_mov_ms, t_fim))
    if n > 1:
        logd('taxa efetiva: {:.0f} Hz | maior atraso do laco: {:.2f} ms | erros I2C: {}'.format(
            (n - 1) * 1e6 / (ta[n - 1] - ta[0]),
            run['atraso_max_us'] / 1000.0, run['erros']))
    logd('bateria: repouso={}V | minima sob carga={}V'.format(
        fnum(cfg['v_rep'], '{:.2f}'), fnum(run['v_min'], '{:.2f}')))

    # Uma passada: saturacao, somas (medias), rotulos e resumo de cores.
    # As medias saem das MESMAS amostras que vao para o CSV.
    sat = 0
    sr = sg = sb = sc = 0
    rotulos = []
    resumo = {}
    for i in range(n):
        c = ca[i]
        if c >= C_MAX - 1:
            sat += 1
        sr += ra[i]
        sg += ga[i]
        sb += ba[i]
        sc += c
        rot = classificar(ra[i], ga[i], ba[i], c, ea[i], ref)
        rotulos.append(rot)
        d = resumo.get(rot)
        if d is None:
            resumo[rot] = [1, c, c]
        else:
            d[0] += 1
            if c < d[1]:
                d[1] = c
            if c > d[2]:
                d[2] = c
    if n:
        medias = (sr / n, sg / n, sb / n, sc / n)
    else:
        medias = (None, None, None, None)
    logd('amostras saturadas (Clear no fundo de escala): {}{}'.format(
        sat, '  <-- reduza o ganho/integracao' if sat else ''))
    logd('medias: R={} G={} B={} C={}'.format(
        fnum(medias[0], '{:.1f}'), fnum(medias[1], '{:.1f}'),
        fnum(medias[2], '{:.1f}'), fnum(medias[3], '{:.1f}')))

    ints = mostrar_linhas(det, ref['c'])

    logd('\n=== CORES VISTAS (rotulos indicativos) ===')
    logd('classe        | amostras | %tempo | Clear min..max')
    for rot in sorted(resumo, key=lambda k: -resumo[k][0]):
        d = resumo[rot]
        logd('{:13s} | {:8d} | {:5.1f}% | {}..{}'.format(
            rot, d[0], 100.0 * d[0] / n, d[1], d[2]))

    # ---- salvamento: cada arquivo e independente; erro em um nao impede os outros
    tela(oled, ['ROBO PARADO', 'Salvando CSV...', 'Amostras: {}'.format(n),
                'Nao desligue'])
    nome = nome_cores(id_)
    ok_cores = salvar_cores(nome, id_, run, ref, rotulos, t_mov_ms)
    rotulos = None
    gc.collect()
    ok_resumo = salvar_resumo(id_, run, cfg, t_mov_ms, medias,
                              nome if ok_cores else nome + ' (FALHOU)')
    ok_linhas = salvar_linhas(id_, run, cfg, ref)

    # ---- telas finais (mesmos numeros gravados nos CSV)
    tela(oled, ['RESUMO CORR {:03d}'.format(id_), 'Tempo movimento:',
                '{:.2f} s'.format(t_mov_ms / 1000.0), 'Amostras:', str(n),
                'Linhas: {}'.format(len(det.eventos))])
    print('TELA 1: tempo {:.2f} s | amostras {}'.format(t_mov_ms / 1000.0, n))
    pausa_tela(TELA_RESUMO_MS)

    tela(oled, ['MEDIAS LEITURAS', 'R: {}'.format(fnum(medias[0], '{:.1f}')),
                'G: {}'.format(fnum(medias[1], '{:.1f}')),
                'B: {}'.format(fnum(medias[2], '{:.1f}')),
                'C: {}'.format(fnum(medias[3], '{:.1f}')),
                'n={}'.format(n)])
    print('TELA 2: medias R/G/B/C', medias)
    pausa_tela(TELA_RESUMO_MS)

    mk = lambda ok: 'OK' if ok else 'XX'
    tela(oled, ['SALVO COM ERROS' if not (ok_cores and ok_resumo and ok_linhas)
                else 'ARQUIVOS SALVOS',
                '{} {}'.format(mk(ok_cores), nome),
                '{} resumo_'.format(mk(ok_resumo)), '   corridas.csv',
                '{} linhas_tapete'.format(mk(ok_linhas)), 'A: continuar'])
    print('TELA 3: cores={} resumo={} linhas={}'.format(ok_cores, ok_resumo, ok_linhas))
    pausa_tela(None)
    return ints


def main():
    parar()
    log('\n=== TCS34725: LOG DE CORES + TEMPO POR QUADRADO ===')
    log('I2C{} SDA=GP{} SCL=GP{} | integracao {:.1f} ms | periodo {:.1f} ms'.format(
        I2C_ID, SDA_PIN, SCL_PIN, INT_US / 1000.0, PERIODO_US / 1000.0))
    bus = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
    achados = bus.scan()
    log('Dispositivos I2C{}:'.format(I2C_ID), [hex(x) for x in achados])

    # Inicializa o OLED ANTES de verificar o TCS. Assim, erros do sensor
    # tambem aparecem no display, em vez de encerrar silenciosamente.
    oled = iniciar_oled(bus, achados)
    global OLED_ATIVO
    OLED_ATIVO = oled
    if TCS_ADDR not in achados:
        log('ERRO: TCS34725 nao encontrado em 0x29. Confira VCC/GND/SDA/SCL.')
        tela(oled, ['ERRO TCS34725', 'Nao detectado', 'Verifique SDA GP2', 'SCL GP3 / VCC'])
        return
    ina_ok = INA_ADDR in achados
    tela(oled, ['TCS DETECTADO', 'Inicializando', 'sensor de cor...', 'Aguarde'])
    log('TCS34725 encontrado em 0x29. Inicializando...')
    try:
        sensor = TCS34725(bus, TCS_ADDR)
    except Exception as e:
        log('Falha ao inicializar TCS34725:', repr(e))
        tela(oled, ['ERRO AO INICIAR', 'TCS34725', 'Confira conexoes', 'e alimentacao'])
        raise
    log('ID do sensor:', hex(sensor.ident), '(esperado 0x44 ou 0x4D)')
    tela(oled, ['TCS34725 OK', 'ID: {}'.format(hex(sensor.ident)), 'Sensor pronto', 'Aguarde PASSO 1'])
    time.sleep_ms(900)

    # PASSO 1: claro (+ ganho)
    if not aguardar('\nPASSO 1/3: coloque o sensor sobre uma area CLARA (branco do tapete), '
                    'na altura de uso. A = medir | B = cancelar'):
        log('Cancelado.')
        return
    if GANHO_AUTO:
        cod, mult, (r, g, b, c) = escolher_ganho(sensor)
    else:
        sensor.definir_ganho(GANHO_FIXO)
        cod = GANHO_FIXO
        mult = dict(GANHOS)[cod]
        r, g, b, c = medir(sensor, 300)
    ref = {'r': max(r, 1), 'g': max(g, 1), 'b': max(b, 1), 'c': max(c, 1)}
    log('Ganho {}x | claro: R={:.0f} G={:.0f} B={:.0f} C={:.0f} (fundo de escala {})'.format(
        mult, r, g, b, c, C_MAX))
    tela(oled, ['CLARO MEDIDO', 'R:{} G:{}'.format(int(r), int(g)),
                'B:{} C:{}'.format(int(b), int(c)), 'Preparando passo 2'])
    if c < 0.03 * C_MAX:
        log('AVISO: sinal muito fraco. Aproxime o sensor ou confira o LED do modulo.')
    if c > 0.9 * C_MAX:
        log('AVISO: perto da saturacao. Reduza ATIME ou use GANHO_FIXO menor.')

    # PASSO 2: linha (opcional)
    base = 0.0
    c_linha = None
    if aguardar('\nPASSO 2/3: coloque o sensor sobre a LINHA escura. '
                'A = medir | B = pular (usa so a fracao do claro)'):
        rl, gl, bl, cl = medir(sensor, 300)
        log('linha: R={:.0f} G={:.0f} B={:.0f} C={:.0f} ({:.0f}% do claro)'.format(
            rl, gl, bl, cl, 100.0 * cl / ref['c']))
        if cl >= 0.7 * ref['c']:
            log('AVISO: contraste baixo; o sensor estava mesmo sobre a linha?'
                ' Medida ignorada.')
        else:
            c_linha = cl
            base = cl
    thr_in = base + ENTRA * (ref['c'] - base)
    thr_out = base + SAI * (ref['c'] - base)
    log('Limiares de Clear: entra na linha < {:.0f} | sai da linha > {:.0f}'.format(
        thr_in, thr_out))
    tela(oled, ['CALIBRACAO OK', 'Limiar: {}'.format(int(thr_in)),
                'Pronto para andar', 'A: iniciar no passo 3'])
    cfg = {'ref': ref, 'c_linha': c_linha, 'thr_in': thr_in,
           'thr_out': thr_out, 'ganho': mult, 'v_rep': None}

    id_ = proximo_id()
    while True:
        if not aguardar('\nPASSO 3/3: posicione o robo no INICIO (sensor ~1 cm antes da '
                        '1a linha, ou sobre ela), pista livre. '
                        'A = iniciar (36 s max.) | B = encerrar'):
            break
        if not contagem(oled):
            log('Cancelado.')
            continue
        cfg['v_rep'] = ler_vbat(bus) if ina_ok else None
        print('ANDANDO por ate 36 s... (B para parar)')
        run = coletar(sensor, bus, ina_ok, thr_in, thr_out, oled)
        print('ROBO PARADO.')
        try:
            processar(id_, run, cfg, oled)
        except Exception as e:
            # nunca perder o controle do robo por erro na analise/gravacao
            parar()
            print('Erro ao processar corrida:', repr(e))
            sys.print_exception(e)
            tela(oled, ['ERRO AO SALVAR', repr(e)[:16], 'Veja o Thonny'])
        gc.collect()
        id_ = proximo_id()
    log('Encerrado.')
    tela(oled, ['TESTE ENCERRADO', 'CSV gravados', 'Abra arquivos', 'pelo Thonny'])


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        parar()
        log('Interrompido pelo usuario.')
    except Exception as e:
        parar()
        log('ERRO:', repr(e))
        sys.print_exception(e)
    finally:
        parar()
