# BitDogLab (RP2040) - Script de Diagnóstico Corrigido (V3)
# Projeto IC - FEEC / UNICAMP
#
# CORREÇÕES DESTA V3:
# 1. Inversão dos pinos corrigida: 
#    - LADO ESQUERDO: GP18 (Fwd), GP19 (Rev), GP16 (PWM)
#    - LADO DIREITO:  GP4 (Fwd), GP9 (Rev), GP8 (PWM)
# 2. Diagnóstico de Torque e Carga Individual por Roda/Motor.
# 3. Gravação dos dados dos ensaios em arquivos .csv na memória Flash.

from machine import Pin, PWM, I2C
import time
import os

# ----------------- CONFIGURAÇÕES DE HARDWARE E ARQUIVOS -----------------
PWM_FREQ = 1000
PRE_STBY_MS = 50
PWM_PADRAO = 35000  # Duty cycle padrão (0-65535)

CSV_RESUMO = "diag_motores_resumo.csv"
CSV_RAMPA = "diag_motores_rampa.csv"

# Pinos dos Motores (Mapeamento Corrigido)
# Lado ESQUERDO (Anteriormente invertido como GP4/GP9/GP8)
LE_F = Pin(18, Pin.OUT, value=0)
LE_R = Pin(19, Pin.OUT, value=0)
PWM_E = PWM(Pin(16))

# Lado DIREITO (Anteriormente invertido como GP18/GP19/GP16)
LD_F = Pin(4, Pin.OUT, value=0)
LD_R = Pin(9, Pin.OUT, value=0)
PWM_D = PWM(Pin(8))

# Habilitação da Ponte H (Standby)
STBY = Pin(20, Pin.OUT, value=0)

PWM_E.freq(PWM_FREQ)
PWM_D.freq(PWM_FREQ)
PWM_E.duty_u16(0)
PWM_D.duty_u16(0)

# Botões
BTN_A = Pin(5, Pin.IN, Pin.PULL_UP)
BTN_B = Pin(6, Pin.IN, Pin.PULL_UP)

# I2C para OLED e INA226
i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)

# Display OLED SH1107
oled = None
try:
    from sh1107_bitdoglab import SH1107_I2C
    scan = i2c.scan()
    addr = 0x3C if 0x3C in scan else (0x3D if 0x3D in scan else None)
    if addr:
        oled = SH1107_I2C(128, 128, i2c, addr=addr)
except Exception as e:
    print("OLED não inicializado:", e)

# Sensor INA226
ina_addr = 0x40 if 0x40 in i2c.scan() else None

def ler_bateria_corrente():
    if not ina_addr:
        return None, None
    try:
        d_v = i2c.readfrom_mem(ina_addr, 0x02, 2)
        d_i = i2c.readfrom_mem(ina_addr, 0x01, 2)
        v = ((d_v[0] << 8) | d_v[1]) * 0.00125
        raw_i = (d_i[0] << 8) | d_i[1]
        if raw_i & 0x8000:
            raw_i -= 65536
        i_amp = abs(raw_i * 0.0000025 / 0.1)
        return v, i_amp
    except Exception:
        return None, None

def existe_arquivo(nome):
    try:
        os.stat(nome)
        return True
    except OSError:
        return False

def proximo_id(nome_csv):
    maior = 0
    if not existe_arquivo(nome_csv):
        return 1
    try:
        with open(nome_csv, "r") as f:
            for linha in f:
                partes = linha.split(",", 1)
                if partes[0].isdigit():
                    maior = max(maior, int(partes[0]))
    except Exception:
        pass
    return maior + 1

def salvar_csv_resumo(nome_teste, pwm_e, pwm_d, sent_e, sent_d, duracao_ms, v_min, i_max, status):
    cabecalho = "id,nome_teste,pwm_esq,pwm_dir,sentido_esq,sentido_dir,duracao_ms,v_min_v,i_max_a,status\n"
    novo = not existe_arquivo(CSV_RESUMO)
    id_teste = proximo_id(CSV_RESUMO)
    
    v_str = f"{v_min:.3f}" if (v_min is not None and v_min != 99.0) else ""
    i_str = f"{i_max:.3f}" if (i_max is not None and i_max != 0.0) else ""
    
    linha = f"{id_teste},{nome_teste},{pwm_e},{pwm_d},{sent_e},{sent_d},{duracao_ms},{v_str},{i_str},{status}\n"
    
    try:
        with open(CSV_RESUMO, "a") as f:
            if novo:
                f.write(cabecalho)
            f.write(linha)
            f.flush()
    except Exception as e:
        print("Erro CSV:", e)

def salvar_csv_rampa(id_sessao, pwm_nivel, v_min, i_max, status):
    cabecalho = "id_sessao,pwm_nivel,v_min_v,i_max_a,status\n"
    novo = not existe_arquivo(CSV_RAMPA)
    
    v_str = f"{v_min:.3f}" if (v_min is not None and v_min != 99.0) else ""
    i_str = f"{i_max:.3f}" if (i_max is not None and i_max != 0.0) else ""
    
    linha = f"{id_sessao},{pwm_nivel},{v_str},{i_str},{status}\n"
    
    try:
        with open(CSV_RAMPA, "a") as f:
            if novo:
                f.write(cabecalho)
            f.write(linha)
            f.flush()
    except Exception as e:
        print("Erro CSV:", e)

def exibir_texto(l1="", l2="", l3="", l4="", l5=""):
    print(f"[{l1}] {l2} | {l3} | {l4} | {l5}")
    if oled:
        try:
            oled.fill(0)
            oled.text(str(l1)[:16], 0, 8, 1)
            oled.text(str(l2)[:16], 0, 28, 1)
            oled.text(str(l3)[:16], 0, 48, 1)
            oled.text(str(l4)[:16], 0, 68, 1)
            oled.text(str(l5)[:16], 0, 88, 1)
            oled.show()
        except Exception as err:
            print("Erro OLED:", err)

def parar():
    PWM_E.duty_u16(0)
    PWM_D.duty_u16(0)
    STBY.value(0)
    LE_F.value(0); LE_R.value(0)
    LD_F.value(0); LD_R.value(0)

def soltar_botoes():
    while BTN_A.value() == 0 or BTN_B.value() == 0:
        time.sleep_ms(20)
    time.sleep_ms(30)

def contagem_regressiva(segundos=3):
    for i in range(segundos, 0, -1):
        if BTN_B.value() == 0:
            soltar_botoes()
            return False
        exibir_texto("ATENCAO!", f"Iniciando em {i}s", "Rodas livres?", "B: Cancelar")
        time.sleep_ms(1000)
    return True

def acionar_motores(pwm_esq, pwm_dir, sentido_esq="F", sentido_dir="F"):
    parar()
    LE_F.value(1 if sentido_esq == "F" else 0)
    LE_R.value(1 if sentido_esq == "R" else 0)
    LD_F.value(1 if sentido_dir == "F" else 0)
    LD_R.value(1 if sentido_dir == "R" else 0)
    STBY.value(1)
    time.sleep_ms(PRE_STBY_MS)
    PWM_E.duty_u16(min(65535, max(0, pwm_esq)))
    PWM_D.duty_u16(min(65535, max(0, pwm_dir)))

def rodar_teste_temporizado(nome_teste, pwm_e, pwm_d, duracao_ms=3000, sent_e="F", sent_d="F"):
    if not contagem_regressiva(3):
        exibir_texto("TESTE CANCELADO", "A: Menu principal")
        time.sleep_ms(1500)
        return

    exibir_texto("EM EXECUCAO...", nome_teste, f"PWM E:{pwm_e} D:{pwm_d}", "B: PARAR EMERG")
    acionar_motores(pwm_e, pwm_d, sent_e, sent_d)
    
    inicio = time.ticks_ms()
    v_min, i_max = 99.0, 0.0
    status = "concluido"
    
    while time.ticks_diff(time.ticks_ms(), inicio) < duracao_ms:
        if BTN_B.value() == 0:
            parar()
            soltar_botoes()
            status = "cancelado_emergencia"
            salvar_csv_resumo(nome_teste, pwm_e, pwm_d, sent_e, sent_d, duracao_ms, v_min, i_max, status)
            exibir_texto("PARADA EMERGENCIA!", "Salvo no CSV!", "A: Continuar")
            time.sleep_ms(2000)
            return
        
        v, i_amp = ler_bateria_corrente()
        if v:
            v_min = min(v_min, v)
            i_max = max(i_max, i_amp)
        time.sleep_ms(50)

    parar()
    salvar_csv_resumo(nome_teste, pwm_e, pwm_d, sent_e, sent_d, duracao_ms, v_min, i_max, status)
    
    info_pwr = f"Vmin:{v_min:.2f}V Imax:{i_max:.2f}A" if v_min != 99.0 else "INA226 N/A"
    exibir_texto("TESTE CONCLUIDO", nome_teste, info_pwr, "Salvo no CSV!", "A: Proximo")
    soltar_botoes()
    while BTN_A.value() == 1:
        time.sleep_ms(30)
    soltar_botoes()

def rodar_teste_torque_manual(lado_nome, pwm_e, pwm_d):
    """Teste de torque com PWM baixo e duração estendida para travar rodas manualmente e testar força"""
    if not contagem_regressiva(3):
        return
    
    exibir_texto("TESTE DE TORQUE", lado_nome, "Segure a roda", "com os dedos!", "B: PARAR")
    acionar_motores(pwm_e, pwm_d, "F", "F")
    
    inicio = time.ticks_ms()
    v_min, i_max = 99.0, 0.0
    
    while time.ticks_diff(time.ticks_ms(), inicio) < 5000:
        if BTN_B.value() == 0:
            break
        v, i_amp = ler_bateria_corrente()
        if v:
            v_min = min(v_min, v)
            i_max = max(i_max, i_amp)
        time.sleep_ms(50)
        
    parar()
    salvar_csv_resumo(f"TORQUE_{lado_nome}", pwm_e, pwm_d, "F", "F", 5000, v_min, i_max, "concluido")
    exibir_texto("TORQUE TESTADO", lado_nome, f"Imax stall: {i_max:.2f}A", "Salvo no CSV", "A: Continuar")
    soltar_botoes()
    while BTN_A.value() == 1:
        time.sleep_ms(30)
    soltar_botoes()

def rodar_varredura_rampa():
    if not contagem_regressiva(3):
        return

    exibir_texto("RAMPA PWM", "Descobrindo ponto", "de partida minima", "B: Interromper")
    steps = [15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000]
    id_sessao = proximo_id(CSV_RAMPA)
    
    for p in steps:
        exibir_texto("RAMPA PWM", f"Nivel PWM: {p}", "Observe as rodas", "B: Interromper")
        acionar_motores(p, p, "F", "F")
        t_start = time.ticks_ms()
        v_min, i_max = 99.0, 0.0
        interrompido = False
        
        while time.ticks_diff(time.ticks_ms(), t_start) < 1000:
            if BTN_B.value() == 0:
                parar()
                soltar_botoes()
                interrompido = True
                salvar_csv_rampa(id_sessao, p, v_min, i_max, "interrompido")
                break
            v, i_amp = ler_bateria_corrente()
            if v:
                v_min = min(v_min, v)
                i_max = max(i_max, i_amp)
            time.sleep_ms(50)
            
        if interrompido:
            exibir_texto("RAMPA INTERROMPIDA", "Salvo no CSV", "A: Voltar menu")
            time.sleep_ms(1500)
            return
            
        salvar_csv_rampa(id_sessao, p, v_min, i_max, "ok")

    parar()
    exibir_texto("RAMPA FINALIZADA", "Resultados em CSV!", f"Rampa ID: {id_sessao}", "A: Voltar menu")
    soltar_botoes()
    while BTN_A.value() == 1:
        time.sleep_ms(30)
    soltar_botoes()

# ----------------- MENU PRINCIPAL DE TESTES -----------------
TESTES = [
    ("1. Apenas ESQUERDO", lambda: rodar_teste_temporizado("SO ESQUERDO", PWM_PADRAO, 0)),
    ("2. Apenas DIREITO", lambda: rodar_teste_temporizado("SO DIREITO", 0, PWM_PADRAO)),
    ("3. AMBOS p/ FRENTE", lambda: rodar_teste_temporizado("AMBOS FRENTE", PWM_PADRAO, PWM_PADRAO)),
    ("4. Torque ESQUERDA", lambda: rodar_teste_torque_manual("ESQUERDA", 25000, 0)),
    ("5. Torque DIREITA", lambda: rodar_teste_torque_manual("DIREITA", 0, 25000)),
    ("6. Rampa PWM Partida", rodar_varredura_rampa),
    ("7. AMBOS p/ TRAS", lambda: rodar_teste_temporizado("AMBOS TRAS", PWM_PADRAO, PWM_PADRAO, sent_e="R", sent_d="R")),
    ("8. Giro Eixo ESQ", lambda: rodar_teste_temporizado("GIRO PIVO ESQ", PWM_PADRAO, PWM_PADRAO, sent_e="R", sent_d="F")),
    ("9. Giro Eixo DIR", lambda: rodar_teste_temporizado("GIRO PIVO DIR", PWM_PADRAO, PWM_PADRAO, sent_e="F", sent_d="R")),
]

def main():
    parar()
    pos = 0
    soltar_botoes()
    print("--- DIAGNOSTICO DE MOTORES E CHASSI BITDOGLAB V3 (PINOS CORRIGIDOS) ---")
    
    while True:
        nome_t, func_t = TESTES[pos]
        v_bat, _ = ler_bateria_corrente()
        str_bat = f"Bat: {v_bat:.2f}V" if v_bat else "BitDogLab V7"
        
        exibir_texto("DIAG. V3 CORRIGIDO", str_bat, f">{nome_t}", "A: Troca Teste", "B: EXECUTA")
        
        while True:
            if BTN_A.value() == 0:
                soltar_botoes()
                pos = (pos + 1) % len(TESTES)
                break
            if BTN_B.value() == 0:
                soltar_botoes()
                func_t()
                break
            time.sleep_ms(20)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        parar()
        print("Finalizado pelo usuario.")
    except Exception as e:
        parar()
        print("Erro no teste:", e)
