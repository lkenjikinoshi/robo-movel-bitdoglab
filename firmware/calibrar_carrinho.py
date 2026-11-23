# calibrar_carrinho.py
# Calibracao interativa do acelerometro (50 cm) e giroscopio (90 graus)
# usando o sensor de cor TCS34725 para detectar os pontos escuros no fundo claro.
#
# Salva os parametros em calibracao_imu.json para uso com imu_calibracao.CarrinhoIMU.
#
# Controles (BitDogLab v7):
#   Botao A (GP5) = confirmar / Calibrar 50 cm
#   Botao B (GP6) = cancelar / parada de emergencia / Calibrar 90 graus
#   A + B juntos  = sair do menu

from machine import Pin, I2C
import time
import gc
import sys

from imu_calibracao import (
    I2C_IMU_ID, IMU_SDA_PIN, IMU_SCL_PIN, IMU_FREQ,
    I2C_COR_ID, COR_SDA_PIN, COR_SCL_PIN, COR_FREQ, TCS_ADDR,
    BTN_A_PIN, BTN_B_PIN, BTN_C_PIN,
    PWM_RETO_PADRAO, PWM_GIRO_PADRAO, PWM_GIRO_RAMPA_INI,
    GYRO_EIXO_PADRAO, GYRO_SINAL_Z, KP_RETO_PADRAO, MAX_CORR_RETO, DEADBAND_ANG_DEG,
    ARQUIVO_CALIBRACAO,
    MPU6050, TCS34725, Motores,
    rampa_pwm, salvar_calibracao, carregar_calibracao,
)

try:
    from sh1107_bitdoglab import SH1107_I2C
except ImportError:
    SH1107_I2C = None

# ---------------- CONFIGURACAO ----------------
DISTANCIA_REF_CM = 50.0      # distancia real entre os 2 pontos (cm)
ANGULO_CAL_DEG = 360.0       # volta completa (360 deg) para calibrar os 90 deg
PAUSA_ETAPA_MS = 250         # pausa entre etapas do percurso do botao C
GIRO_AVANCO_CM = 12.0        # deslocamento para frente durante o pivo de 90 deg
GIRO_LATERAL_CM = 10.0       # deslocamento para o lado (novo sentido) durante o pivo de 90 deg
COMP_POS_CURVA_CM = 0.0      # compensacao extra apos cada curva (se precisar ajuste fino)

# Percurso do botao C: ('R', dist_cm) = reto | ('E', ang_deg) = girar esquerda
PERCURSO_BOTAO_C = (
    ('R', 40.0),
    ('E', 90.0),
    ('R', 40.0),
    ('E', 90.0),
    ('R', 20.0),
)

PWM_CAL_RETO = PWM_RETO_PADRAO
PWM_CAL_GIRO = PWM_GIRO_PADRAO

CONTAGEM_MS = 3000           # contagem regressiva antes de mover
TIMEOUT_RETO_MS = 10000      # seguranca: para sozinho apos 10 s
TIMEOUT_GIRO_MS = 8000       # seguranca para giro de 90 deg
TIMEOUT_GIRO_360_MS = 16000  # seguranca para volta completa de 360 deg
MIN_ENTRE_PONTOS_MS = 180    # tempo minimo no claro entre Ponto 1 e Ponto 2

# ---------------- HARDWARE AUXILIAR ----------------
BTN_A = Pin(BTN_A_PIN, Pin.IN, Pin.PULL_UP)
BTN_B = Pin(BTN_B_PIN, Pin.IN, Pin.PULL_UP)
BTN_C = Pin(BTN_C_PIN, Pin.IN, Pin.PULL_UP)

oled = None
_ultima_tela = None
NOMES_EIXOS = ('X', 'Y', 'Z')


class TelaOLED:
    Y = (2, 23, 44, 65, 86, 107)

    def __init__(self, bus, addr):
        self.dev = SH1107_I2C(128, 128, bus, addr=addr, rotate_180=False)
        self.ant_linhas = None

    def mostrar(self, linhas):
        linhas = tuple(str(x)[:16] for x in linhas[:6])
        if linhas == self.ant_linhas:
            return
        self.dev.fill_rect(0, 0, 128, 128, 0)
        for y, s in zip(self.Y, linhas):
            self.dev.text(s, 0, y, 1)
        self.dev.show()
        self.ant_linhas = linhas


def tela(*linhas):
    """Exibe no console e no OLED (nunca chamar com o robo em movimento)."""
    global _ultima_tela
    msg = tuple(str(x)[:16] for x in linhas[:6])
    if msg != _ultima_tela:
        print('>>', ' | '.join(msg))
        _ultima_tela = msg
    if oled:
        try:
            oled.mostrar(msg)
        except Exception:
            pass


def soltar(btn):
    while btn.value() == 0:
        time.sleep_ms(10)
    time.sleep_ms(25)


def esperar_a():
    """A -> True, B -> False."""
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


def ler_menu():
    """'A', 'B', 'C' ou 'S' (A+B juntos)."""
    soltar(BTN_A)
    soltar(BTN_B)
    soltar(BTN_C)
    while True:
        a0 = BTN_A.value() == 0
        b0 = BTN_B.value() == 0
        c0 = BTN_C.value() == 0
        if c0 and not a0 and not b0:
            soltar(BTN_C)
            return 'C'
        if a0 or b0:
            time.sleep_ms(150)
            a = a0 or BTN_A.value() == 0
            b = b0 or BTN_B.value() == 0
            if a and b:
                soltar(BTN_A)
                soltar(BTN_B)
                return 'S'
            if a:
                soltar(BTN_A)
                return 'A'
            soltar(BTN_B)
            return 'B'
        time.sleep_ms(10)


def contagem():
    inicio = time.ticks_ms()
    ultimo = -1
    while True:
        falta = CONTAGEM_MS - time.ticks_diff(time.ticks_ms(), inicio)
        if falta <= 0:
            return True
        if BTN_B.value() == 0:
            soltar(BTN_B)
            return False
        s = (falta + 999) // 1000
        if s != ultimo:
            ultimo = s
            tela('PARTIDA EM {}s'.format(s), 'Nao toque robo', 'B: cancelar')
        time.sleep_ms(10)


# ---------------- PASSO 0: LIMIARES DE COR ----------------
def calibrar_cor(tcs):
    tela('PASSO 0a: FUNDO', 'Sensor no FUNDO', 'CLARO na altura', 'de uso.', 'A: medir', 'B: sair')
    if not esperar_a():
        return None
    cod, mult, c_fundo = tcs.escolher_ganho()
    print('Ganho TCS: {}x | Clear fundo = {:.1f}'.format(mult, c_fundo))

    tela('PASSO 0b: PONTO', 'Sensor sobre o', 'PONTO ESCURO', '(mesma altura).', 'A: medir', 'B: sair')
    if not esperar_a():
        return None
    c_ponto = tcs.medir_clear_medio(250)
    print('Clear ponto escuro = {:.1f}'.format(c_ponto))

    if c_ponto >= c_fundo * 0.85:
        print('AVISO: contraste baixo entre fundo ({:.0f}) e ponto ({:.0f})!'.format(c_fundo, c_ponto))
        tela('CONTRASTE BAIXO', 'Fundo {:.0f}'.format(c_fundo), 'Ponto {:.0f}'.format(c_ponto), 'A: continuar', 'B: sair')
        if not esperar_a():
            return None

    # Histerese: entra no ponto abaixo de 50% do caminho, sai acima de 65%
    limiar_entrar = c_ponto + 0.45 * (c_fundo - c_ponto)
    limiar_sair = c_ponto + 0.65 * (c_fundo - c_ponto)
    print('Limiares Clear -> entrar: {:.1f} | sair: {:.1f}'.format(limiar_entrar, limiar_sair))
    return {
        'tcs_ganho_cod': cod,
        'tcs_ganho_mult': mult,
        'c_fundo': round(c_fundo, 1),
        'c_ponto': round(c_ponto, 1),
        'limiar_entrar': round(limiar_entrar, 1),
        'limiar_sair': round(limiar_sair, 1),
    }


# ---------------- [A] CALIBRACAO DE VELOCIDADE / DISTANCIA (50 CM) ----------------
def calibrar_distancia_50cm(imu, tcs, motores, cor_cfg):
    # Ordem obrigatoria: 1o calibrar o giroscopio (360->90), 2o calibrar a velocidade (50 cm)
    cal_atual = carregar_calibracao()
    if 'gyro_fator_deg' not in cal_atual or 'gyro_eixo' not in cal_atual or 'gyro_sinal' not in cal_atual:
        print('Ordem de calibracao: faca primeiro B (360->90) antes de A (50 cm)!')
        tela('ORDEM CALIBR.', '1o: Giro 360 (B)', '2o: 50 cm    (A)', 'Calibre B antes!', 'A: menu')
        esperar_a()
        return

    tela('CALIBRAR 50 CM', 'Sensor 2-3 cm', 'ANTES do Ponto 1', 'alinhado ao P2.', 'A: iniciar', 'B: voltar')
    if not esperar_a():
        return
    if not contagem():
        return

    lim_in = cor_cfg['limiar_entrar']
    lim_out = cor_cfg['limiar_sair']

    # Usa o giroscopio JA CALIBRADO para manter reto e medir o trim natural dos motores
    g_eixo = int(cal_atual['gyro_eixo'])
    g_sinal = float(cal_atual['gyro_sinal'])
    g_fator = float(cal_atual['gyro_fator_deg'])

    tela('MEDINDO BIAS...', 'Robo PARADO!')
    _, bias_g = imu.medir_bias(amostras=150, intervalo_ms=3)
    bias_gz = bias_g[g_eixo]

    estado = 0  # 0=aguardando P1, 1=dentro de P1, 2=entre P1 e P2, 3=P2 achado
    t1 = 0
    t2 = 0
    t_saiu1 = 0
    erros_i2c = 0
    erros_seq = 0
    ang = 0.0
    g_ant = 0.0
    soma_corr = 0
    n_corr = 0
    motivo = 'timeout'

    tela('ANDANDO...', 'Buscando P1->P2', 'B: parar')
    gc.collect()

    motores.frente(PWM_CAL_RETO)
    t0 = time.ticks_us()
    t_ant = t0
    try:
        while True:
            if BTN_B.value() == 0:
                motivo = 'botao_B'
                break

            t_now = time.ticks_us()
            if time.ticks_diff(t_now, t0) >= TIMEOUT_RETO_MS * 1000:
                motivo = 'timeout'
                break

            dt = time.ticks_diff(t_now, t_ant) / 1000000.0
            t_ant = t_now

            # 1) Correcao de rumo com giroscopio calibrado (mesma lei de controle do percurso)
            if dt > 0.0:
                try:
                    leitura = imu.ler_unidades()
                    g_cur = (leitura[3 + g_eixo] - bias_gz) * g_sinal * g_fator
                    ang += 0.5 * (g_ant + g_cur) * dt
                    g_ant = g_cur
                    if abs(ang) < DEADBAND_ANG_DEG:
                        corr = 0
                    else:
                        corr = int(KP_RETO_PADRAO * ang)
                        if corr > MAX_CORR_RETO:
                            corr = MAX_CORR_RETO
                        elif corr < -MAX_CORR_RETO:
                            corr = -MAX_CORR_RETO
                    motores.definir_pwm(PWM_CAL_RETO + corr, PWM_CAL_RETO - corr)
                    # Acumula a correcao media entre P1 e P2 para aprender o trim dos motores
                    if estado in (1, 2):
                        soma_corr += corr
                        n_corr += 1
                except OSError:
                    pass

            # 2) Leitura do sensor de cor para detectar Ponto 1 e Ponto 2
            try:
                c = tcs.ler_clear()
                erros_seq = 0
            except OSError:
                erros_i2c += 1
                erros_seq += 1
                if erros_seq >= 10:
                    motivo = 'erro_I2C'
                    break
                time.sleep_ms(2)
                continue

            if estado == 0:
                if c < lim_in:
                    t1 = t_now
                    estado = 1
            elif estado == 1:
                if c > lim_out:
                    t_saiu1 = t_now
                    estado = 2
            elif estado == 2:
                if time.ticks_diff(t_now, t_saiu1) >= MIN_ENTRE_PONTOS_MS * 1000:
                    if c < lim_in:
                        t2 = t_now
                        estado = 3
                        motivo = 'ok'
                        break

            time.sleep_ms(3)
    finally:
        motores.frear()

    if estado != 3 or t2 <= t1:
        print('Calibracao 50 cm nao concluida (motivo={}, estado={}, erros_i2c={}).'.format(
            motivo, estado, erros_i2c))
        tela('NAO CONCLUIDO', 'Motivo: ' + motivo, 'Estado: {}'.format(estado),
             'I2C err: {}'.format(erros_i2c), 'A: menu')
        esperar_a()
        return

    dt_s = time.ticks_diff(t2, t1) / 1000000.0
    if dt_s < 0.1:
        print('ERRO: tempo entre pontos muito curto ({:.3f} s).'.format(dt_s))
        tela('TEMPO CURTO', '{:.3f} s'.format(dt_s), 'Repita o teste', 'A: menu')
        esperar_a()
        return

    vel_media = DISTANCIA_REF_CM / dt_s
    trim_pwm = int(round(soma_corr / max(1, n_corr))) if n_corr > 0 else 0

    dados = {
        'distancia_ref_cm': DISTANCIA_REF_CM,
        'tempo_50cm_s': round(dt_s, 4),
        'vel_media_cm_s': round(vel_media, 3),
        'pwm_cal_reto': PWM_CAL_RETO,
        'trim_reto_pwm': trim_pwm,
    }
    salvar_calibracao(dados)

    print('\n=== CALIBRACAO 50 CM SALVA ===')
    print('Tempo P1->P2: {:.3f} s | Velocidade: {:.2f} cm/s | Trim PWM: {} (erros I2C: {})'.format(
        dt_s, vel_media, trim_pwm, erros_i2c))
    print('Salvo em:', ARQUIVO_CALIBRACAO)

    tela('50 CM CALIBRADO!',
         'Vel: {:.1f} cm/s'.format(vel_media),
         'Tempo: {:.2f} s'.format(dt_s),
         'Trim PWM: {}'.format(trim_pwm),
         'A: menu')
    esperar_a()


# ---------------- [B] CALIBRACAO DE GIRO (360 GRAUS -> 90 GRAUS) ----------------
def calibrar_giro_360deg(imu, tcs, motores, cor_cfg):
    tela('CALIBRAR 360->90', 'Sensor antes de', '1 marca escura.', 'Gira 360 ESQ no', 'eixo. A: ir', 'B: voltar')
    if not esperar_a():
        return
    if not contagem():
        return

    tela('MEDINDO BIAS...', 'Robo PARADO!')
    _, bias_g = imu.medir_bias(amostras=250, intervalo_ms=4)
    print('Bias gyro (deg/s): X={:.3f} Y={:.3f} Z={:.3f}'.format(*bias_g))

    lim_in = cor_cfg['limiar_entrar']
    lim_out = cor_cfg['limiar_sair']

    # Integra os 3 eixos do giroscopio para detectar automaticamente o eixo vertical (yaw)
    ang = [0.0, 0.0, 0.0]
    g_ant = [0.0, 0.0, 0.0]

    estado = 0  # 0=aguardando 1a passagem, 1=dentro da marca, 2=dando a volta (360), 3=2a passagem
    a1 = None
    a2 = None
    t1 = 0
    t2 = 0
    t_saiu1 = 0
    motivo = 'timeout'

    tela('GIRANDO 360 ESQ', 'Marca -> volta', '-> mesma marca', 'B: parar')
    gc.collect()

    # Gira em PIVO para a esquerda (roda esquerda freada), igual as curvas do percurso
    pwm_ini = min(PWM_GIRO_RAMPA_INI, PWM_CAL_GIRO)
    motores.pivo_esquerda(pwm_ini)
    t0 = time.ticks_us()
    t_ant = t0
    rampa_concluida = (pwm_ini >= PWM_CAL_GIRO)
    try:
        while True:
            if BTN_B.value() == 0:
                motivo = 'botao_B'
                break

            t_now = time.ticks_us()
            decorrido_us = time.ticks_diff(t_now, t0)
            if decorrido_us >= TIMEOUT_GIRO_360_MS * 1000:
                motivo = 'timeout'
                break

            if not rampa_concluida:
                p = rampa_pwm(PWM_CAL_GIRO, decorrido_us)
                motores.definir_pwm_pivo(p)
                if p >= PWM_CAL_GIRO:
                    rampa_concluida = True

            dt = time.ticks_diff(t_now, t_ant) / 1000000.0
            if dt <= 0.0:
                continue
            t_ant = t_now

            # 1) Leitura e integracao do giroscopio
            try:
                _, _, _, gx, gy, gz = imu.ler_unidades()
                g_cur = (gx - bias_g[0], gy - bias_g[1], gz - bias_g[2])
                for k in range(3):
                    ang[k] += 0.5 * (g_ant[k] + g_cur[k]) * dt
                    g_ant[k] = g_cur[k]
            except OSError:
                pass

            # 2) Leitura do sensor de cor para detectar 1a e 2a passagem na mesma marca (360 deg)
            try:
                c = tcs.ler_clear()
            except OSError:
                time.sleep_ms(2)
                continue

            if estado == 0:
                if c < lim_in:
                    a1 = (ang[0], ang[1], ang[2])
                    t1 = t_now
                    estado = 1
            elif estado == 1:
                if c > lim_out:
                    t_saiu1 = t_now
                    estado = 2
            elif estado == 2:
                # So aceita a 2a passagem apos >400 ms e pelo menos 180 deg brutos girados
                giro_parcial = max(abs(ang[k] - a1[k]) for k in range(3))
                if time.ticks_diff(t_now, t_saiu1) >= 400000 and giro_parcial >= 180.0:
                    if c < lim_in:
                        a2 = (ang[0], ang[1], ang[2])
                        t2 = t_now
                        estado = 3
                        motivo = 'ok'
                        break

            time.sleep_ms(3)
    finally:
        motores.frear()

    if estado != 3 or a1 is None or a2 is None:
        print('Calibracao 360 deg nao concluida (motivo={}, estado={}).'.format(motivo, estado))
        tela('NAO CONCLUIDO', 'Motivo: ' + motivo, 'Estado: {}'.format(estado), 'A: menu')
        esperar_a()
        return

    da = [a2[k] - a1[k] for k in range(3)]
    eixo = max(range(3), key=lambda k: abs(da[k]))
    ang_bruto_360 = da[eixo]
    dt_s = time.ticks_diff(t2, t1) / 1000000.0

    if abs(ang_bruto_360) < 100.0:
        print('ERRO: angulo bruto muito pequeno para 360 deg ({:.2f} deg).'.format(ang_bruto_360))
        tela('SINAL MUITO BAIXO', 'da={:.1f} deg'.format(ang_bruto_360), 'Verifique MPU', 'A: menu')
        esperar_a()
        return

    # Como girou para a ESQUERDA (anti-horario), sinal garante + = esquerda
    sinal = 1 if ang_bruto_360 >= 0 else -1
    fator_deg = ANGULO_CAL_DEG / abs(ang_bruto_360)
    ang_bruto_90 = abs(ang_bruto_360) / 4.0

    dados = {
        'angulo_ref_deg': ANGULO_CAL_DEG,
        'gyro_eixo': eixo,
        'gyro_eixo_nome': NOMES_EIXOS[eixo],
        'gyro_sinal': sinal,
        'gyro_da_bruto_360': round(abs(ang_bruto_360), 3),
        'gyro_da_bruto_90': round(ang_bruto_90, 3),
        'gyro_fator_deg': round(fator_deg, 6),
        'tempo_360deg_s': round(dt_s, 3),
        'pwm_cal_giro': PWM_CAL_GIRO,
    }
    salvar_calibracao(dados)

    print('\n=== CALIBRACAO 360 -> 90 GRAUS SALVA ===')
    print('Eixo de giro detectado: {}{} (+ = esquerda)'.format('+' if sinal > 0 else '-', NOMES_EIXOS[eixo]))
    print('Angulo bruto 360 (X,Y,Z): ({:.2f}, {:.2f}, {:.2f}) deg'.format(*da))
    print('90 graus equivalem a {:.2f} deg brutos | Fator: {:.5f}'.format(ang_bruto_90, fator_deg))
    print('Tempo 360 deg: {:.2f} s | Salvo em: {}'.format(dt_s, ARQUIVO_CALIBRACAO))

    tela('360->90 CALIBR!',
         'Eixo: {}{}'.format('+' if sinal > 0 else '-', NOMES_EIXOS[eixo]),
         '360: {:.1f} deg'.format(abs(ang_bruto_360)),
         '90:  {:.1f} deg'.format(ang_bruto_90),
         'Fator: {:.3f}'.format(fator_deg),
         'A: menu')
    esperar_a()


# ---------------- [C] PERCURSO: 40cm -> 90E -> 43cm -> 90E -> 23cm ----------------
def _andar_reto_com_gyro(imu, motores, dist_cm, vel, pwm, bias_gz, g_eixo, g_sinal, g_fator, trim_pwm=0):
    """Anda `dist_cm` reto usando o trim dos motores + correcao suave pelo giroscopio."""
    tempo_alvo_us = int((dist_cm / vel) * 1000000.0)
    ang = 0.0
    g_ant = 0.0
    motivo = 'ok'

    motores.frente(pwm)
    if trim_pwm != 0:
        motores.definir_pwm(pwm + trim_pwm, pwm - trim_pwm)
    t0 = time.ticks_us()
    t_ant = t0
    try:
        while True:
            if BTN_B.value() == 0:
                motivo = 'botao_B'
                break
            t_now = time.ticks_us()
            if time.ticks_diff(t_now, t0) >= tempo_alvo_us:
                break

            dt = time.ticks_diff(t_now, t_ant) / 1000000.0
            t_ant = t_now
            if dt > 0.0:
                try:
                    leitura = imu.ler_unidades()
                    g_cur = (leitura[3 + g_eixo] - bias_gz) * g_sinal * g_fator
                    ang += 0.5 * (g_ant + g_cur) * dt
                    g_ant = g_cur
                    if abs(ang) < DEADBAND_ANG_DEG:
                        corr = 0
                    else:
                        # ang > 0 (desvio p/ esquerda) -> aumenta esq, reduz dir
                        corr = int(KP_RETO_PADRAO * ang)
                        if corr > MAX_CORR_RETO:
                            corr = MAX_CORR_RETO
                        elif corr < -MAX_CORR_RETO:
                            corr = -MAX_CORR_RETO
                    total_corr = trim_pwm + corr
                    motores.definir_pwm(pwm + total_corr, pwm - total_corr)
                except OSError:
                    pass

            time.sleep_ms(3)
    finally:
        motores.frear()

    dt_s = time.ticks_diff(time.ticks_us(), t0) / 1000000.0
    print('  Reto {:.1f} cm -> motivo={}, tempo={:.2f}s, ang_final={:.1f}deg'.format(
        dist_cm, motivo, dt_s, ang))
    return motivo


def _girar_esquerda(imu, motores, angulo_deg, pwm, bias_gz, g_eixo, g_fator):
    """Gira `angulo_deg` para a esquerda em PIVO: roda esquerda travada (freio ativo),
    so a direita anda. Usa giroscopio calibrado e rampa suave de arranque."""
    ang = 0.0
    g_ant = 0.0
    motivo = 'ok'
    pwm_ini = min(PWM_GIRO_RAMPA_INI, pwm)

    motores.pivo_esquerda(pwm_ini)
    t0 = time.ticks_us()
    t_ant = t0
    rampa_concluida = (pwm_ini >= pwm)
    try:
        while True:
            if BTN_B.value() == 0:
                motivo = 'botao_B'
                break
            t_now = time.ticks_us()
            decorrido_us = time.ticks_diff(t_now, t0)
            if decorrido_us >= TIMEOUT_GIRO_MS * 1000:
                motivo = 'timeout'
                break

            if not rampa_concluida:
                p = rampa_pwm(pwm, decorrido_us)
                motores.definir_pwm_pivo(p)
                if p >= pwm:
                    rampa_concluida = True

            dt = time.ticks_diff(t_now, t_ant) / 1000000.0
            t_ant = t_now
            if dt <= 0.0:
                continue

            try:
                leitura = imu.ler_unidades()
            except OSError:
                time.sleep_ms(2)
                continue

            g_cur = (leitura[3 + g_eixo] - bias_gz) * g_fator
            ang += 0.5 * (g_ant + g_cur) * dt
            g_ant = g_cur

            if abs(ang) >= angulo_deg:
                break

            time.sleep_ms(3)
    finally:
        motores.frear()

    dt_s = time.ticks_diff(time.ticks_us(), t0) / 1000000.0
    print('  Giro {:.1f} deg ESQ -> motivo={}, tempo={:.2f}s, medido={:.1f}deg'.format(
        angulo_deg, motivo, dt_s, abs(ang)))
    return motivo


def executar_percurso_c(imu, motores):
    cal = carregar_calibracao()
    if 'gyro_fator_deg' not in cal:
        print('Giroscopio ainda nao calibrado!')
        tela('NAO CALIBRADO', '1o: Giro 360 (B)', '2o: 50 cm    (A)', 'Calibre B antes!', 'A: menu')
        esperar_a()
        return
    if 'vel_media_cm_s' not in cal:
        print('Velocidade ainda nao calibrada!')
        tela('NAO CALIBRADO', 'Faca a calibr.', 'de 50 cm (A)', 'antes do C!', 'A: menu')
        esperar_a()
        return

    if not contagem():
        return

    vel = float(cal['vel_media_cm_s'])
    pwm_reto = int(cal.get('pwm_cal_reto', PWM_CAL_RETO))
    trim_pwm = int(cal.get('trim_reto_pwm', 0))
    g_eixo = int(cal['gyro_eixo'])
    g_sinal = float(cal['gyro_sinal'])
    g_fator = float(cal['gyro_fator_deg'])

    tela('MEDINDO BIAS...', 'Robo PARADO!')
    _, bias_g = imu.medir_bias(amostras=200, intervalo_ms=4)
    bias_gz = bias_g[g_eixo]
    print('\n=== PERCURSO BOTAO C (vel={:.2f} cm/s, pwm={}, trim={}, gyro_fator={:.4f}) ==='.format(
        vel, pwm_reto, trim_pwm, g_fator))

    tela('PERCURSO C', '40->90E->43->', '90E->23 cm', 'B: parar')
    gc.collect()

    motivo = 'ok'
    etapa_atual = 0
    total_etapas = len(PERCURSO_BOTAO_C)
    for idx, (tipo, valor) in enumerate(PERCURSO_BOTAO_C, 1):
        etapa_atual = idx
        if tipo == 'R':
            tem_curva_antes = (idx > 1 and PERCURSO_BOTAO_C[idx - 2][0] == 'E')
            tem_curva_depois = (idx < total_etapas and PERCURSO_BOTAO_C[idx][0] == 'E')
            dist_etapa = valor
            if tem_curva_antes:
                dist_etapa -= GIRO_LATERAL_CM
                dist_etapa += COMP_POS_CURVA_CM
            if tem_curva_depois:
                dist_etapa -= GIRO_AVANCO_CM
            dist_etapa = max(0.0, dist_etapa)
            motivo = _andar_reto_com_gyro(
                imu, motores, dist_etapa, vel, pwm_reto, bias_gz, g_eixo, g_sinal, g_fator, trim_pwm)
        else:
            motivo = _girar_esquerda(
                imu, motores, valor, PWM_CAL_GIRO, bias_gz, g_eixo, g_fator)
        if motivo != 'ok':
            break
        # Pausa curta entre etapas para estabilizar inercia
        t_pausa = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t_pausa) < PAUSA_ETAPA_MS:
            if BTN_B.value() == 0:
                motivo = 'botao_B'
                break
            time.sleep_ms(10)
        if motivo != 'ok':
            break

    motores.parar()
    print('Percurso finalizado: motivo={} (etapa {}/{})'.format(
        motivo, etapa_atual, len(PERCURSO_BOTAO_C)))
    tela('PERCURSO ' + ('OK!' if motivo == 'ok' else 'PARADO'),
         'Motivo: ' + motivo[:8],
         'Etapa: {}/{}'.format(etapa_atual, len(PERCURSO_BOTAO_C)),
         'A: menu')
    esperar_a()


# ---------------- MAIN ----------------
def main():
    global oled
    motores = Motores()
    motores.parar()

    # Inicializa I2C1 (TCS34725 + OLED opcional)
    i2c_cor = I2C(I2C_COR_ID, sda=Pin(COR_SDA_PIN), scl=Pin(COR_SCL_PIN), freq=COR_FREQ)
    achados_cor = i2c_cor.scan()
    if SH1107_I2C and (0x3C in achados_cor or 0x3D in achados_cor):
        try:
            oled = TelaOLED(i2c_cor, 0x3C if 0x3C in achados_cor else 0x3D)
        except Exception as e:
            print('OLED indisponivel:', repr(e))
    print('I2C1 (GP2/GP3):', [hex(x) for x in achados_cor])
    if TCS_ADDR not in achados_cor:
        tela('ERRO TCS34725', 'Nao achado I2C1', 'Confira GP2/GP3')
        return
    tcs = TCS34725(i2c_cor, TCS_ADDR)
    print('TCS34725 OK (ID={})'.format(hex(tcs.ident)))

    # Inicializa I2C0 (MPU6050)
    i2c_imu = I2C(I2C_IMU_ID, sda=Pin(IMU_SDA_PIN), scl=Pin(IMU_SCL_PIN), freq=IMU_FREQ)
    print('I2C0 (GP0/GP1):', [hex(x) for x in i2c_imu.scan()])
    try:
        imu = MPU6050(i2c_imu)
    except OSError as e:
        print(e)
        tela('ERRO MPU6050', 'Nao achado I2C0', 'Confira GP0/GP1')
        return
    print('MPU6050 OK (addr={}, WHO_AM_I={})'.format(hex(imu.addr), hex(imu.who_am_i)))

    # Se ja existe calibracao de cor salva, reaproveita e vai direto ao menu
    cal_salva = carregar_calibracao()
    if 'limiar_entrar' in cal_salva and 'limiar_sair' in cal_salva:
        cor_cfg = {
            'tcs_ganho_cod': cal_salva.get('tcs_ganho_cod', 2),
            'tcs_ganho_mult': cal_salva.get('tcs_ganho_mult', 16),
            'c_fundo': cal_salva.get('c_fundo', 0),
            'c_ponto': cal_salva.get('c_ponto', 0),
            'limiar_entrar': cal_salva['limiar_entrar'],
            'limiar_sair': cal_salva['limiar_sair'],
        }
        tcs.ganho(cor_cfg['tcs_ganho_cod'])
        print('Calibracao de cor carregada de {}: lim_in={}, lim_out={}'.format(
            ARQUIVO_CALIBRACAO, cor_cfg['limiar_entrar'], cor_cfg['limiar_sair']))
    else:
        cor_cfg = calibrar_cor(tcs)
        if cor_cfg is None:
            tela('CANCELADO')
            return
        salvar_calibracao(cor_cfg)

    # Menu principal (ordem: 1o B giro 360->90, 2o A velocidade 50cm, 3o C percurso)
    while True:
        cal = carregar_calibracao()
        st_g = 'OK' if 'gyro_fator_deg' in cal else '--'
        st_a = 'OK' if 'vel_media_cm_s' in cal else '--'
        tela('CARRINHO IMU',
             '1.B: 360->90[{}]'.format(st_g),
             '2.A: 50cm   [{}]'.format(st_a),
             '3.C: Percurso U',
             'A+B: sair')
        op = ler_menu()
        if op == 'S':
            break
        try:
            if op == 'A':
                calibrar_distancia_50cm(imu, tcs, motores, cor_cfg)
            elif op == 'B':
                calibrar_giro_360deg(imu, tcs, motores, cor_cfg)
            elif op == 'C':
                executar_percurso_c(imu, motores)
        except Exception as e:
            motores.parar()
            print('Erro:', repr(e))
            sys.print_exception(e)
            tela('ERRO', repr(e)[:16])
            time.sleep_ms(1500)
        gc.collect()

    motores.parar()
    print('\nCalibracao final em {}:'.format(ARQUIVO_CALIBRACAO))
    print(carregar_calibracao())
    tela('FINALIZADO', 'Dados salvos em', ARQUIVO_CALIBRACAO)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        Motores().parar()
        print('Interrompido pelo usuario.')
    finally:
        Motores().parar()

