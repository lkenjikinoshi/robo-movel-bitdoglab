# 🤖 Robô Móvel Autônomo — BitDogLab (RP2040)

Repositório dedicado ao desenvolvimento de uma plataforma robótica móvel educacional baseada na **BitDogLab** e no microcontrolador **Raspberry Pi Pico (RP2040)**.

O projeto reúne firmware em MicroPython, drivers de hardware, ferramentas de diagnóstico e rotinas de calibração para investigar e aprimorar o comportamento de robôs móveis utilizados em atividades educacionais na **FEEC / UNICAMP**.

O desenvolvimento envolve experimentação prática com motores DC, sensores inerciais e de cor, além de métodos de medição e controle aplicados à navegação de robôs.

## 🎯 Objetivos

- Desenvolver e organizar o firmware de controle de robôs móveis educacionais.
- Investigar diferenças de comportamento entre unidades aparentemente semelhantes.
- Realizar testes de motores, ponte H e resposta a diferentes níveis de PWM.
- Implementar procedimentos de calibração de deslocamentos e giros.
- Avaliar o uso de sensores para melhorar a repetibilidade dos movimentos.
- Criar ferramentas de diagnóstico e registro de dados que facilitem a análise experimental.

## 🛠️ Plataforma e Tecnologias

| Componente | Descrição |
|---|---|
| Microcontrolador | Raspberry Pi Pico — RP2040 |
| Plataforma | BitDogLab |
| Linguagem | MicroPython |
| Motores | Motores DC com acionamento por ponte H |
| Sensores inerciais | MPU6050 — acelerômetro e giroscópio |
| Sensor de cor | TCS34725 — leitura RGB e Clear |
| Monitoramento elétrico | INA226 — tensão e corrente |
| Interface | Display OLED SH1107 |
| Ambiente de desenvolvimento | Thonny / MicroPython |

## 📁 Estrutura do Repositório

```text
robo-movel-bitdoglab/
│
├── calibration/   # Rotinas de calibração e experimentos
├── diagnostics/   # Testes de motores, ponte H e PWM
├── drivers/       # Drivers e módulos de controle de hardware
├── examples/      # Exemplos de utilização e experimentação
├── firmware/      # Arquivos e recursos do firmware
├── sensors/       # Testes, aquisição e registro de dados dos sensores
├── src/           # Código-fonte da aplicação e navegação
│
├── README.md      # Documentação do projeto
└── .gitignore
```

### Principais módulos

- **`calibration/`** — Experimentos de calibração de deslocamentos, giros e parâmetros de movimento.
- **`diagnostics/`** — Ferramentas para investigar o acionamento dos motores, a ponte H, o torque e a resposta a diferentes valores de PWM.
- **`drivers/`** — Módulos reutilizáveis para comunicação e controle dos periféricos da BitDogLab.
- **`sensors/`** — Scripts de aquisição, análise e registro de dados dos sensores, incluindo o TCS34725.
- **`src/`** — Código relacionado à lógica principal do robô e à execução de movimentos.
- **`firmware/`** — Arquivos auxiliares e recursos utilizados na execução do sistema embarcado.
- **`examples/`** — Programas de referência para testes e experimentação.

## 🔬 Desenvolvimento e Experimentação

Uma das principais frentes do projeto é a investigação da **repetibilidade dos movimentos dos robôs móveis**.

Durante os experimentos, são avaliados fatores como:

- Variações de velocidade entre motores;
- Assimetria mecânica entre chassis;
- Influência dos parâmetros de PWM;
- Erros acumulados durante deslocamentos e giros;
- Comportamento dos sensores durante a navegação;
- Diferenças de desempenho entre unidades do mesmo modelo.

Os scripts de diagnóstico e calibração permitem registrar dados experimentais e comparar o comportamento dos robôs, auxiliando na identificação das causas de desvios e na elaboração de estratégias de controle.

## 🚀 Executando o Projeto

### Requisitos

- Uma placa BitDogLab equipada com Raspberry Pi Pico / RP2040;
- Firmware MicroPython compatível;
- Computador com Thonny ou outro ambiente de desenvolvimento MicroPython;
- Robô montado com seus motores e sensores conectados.

### Procedimento básico

1. Configure a placa BitDogLab com o firmware MicroPython.
2. Conecte a placa ao computador via USB.
3. Abra os arquivos do repositório no ambiente de desenvolvimento.
4. Transfira para a placa os módulos e scripts necessários.
5. Execute inicialmente os programas de diagnóstico e calibração.
6. Após verificar o funcionamento dos componentes, execute os programas de movimentação do robô.

> **Atenção:** antes de executar qualquer rotina de movimentação, mantenha o robô em uma superfície segura e verifique o sentido de rotação dos motores, os limites de PWM e o funcionamento dos comandos de parada.

## 📚 Contexto

O projeto está inserido no contexto de desenvolvimento de robótica educacional com a plataforma BitDogLab, buscando ampliar suas possibilidades de aplicação em atividades práticas de programação, sistemas embarcados e controle de robôs móveis.

A iniciativa está vinculada às atividades desenvolvidas na **Faculdade de Engenharia Elétrica e de Computação (FEEC) da Universidade Estadual de Campinas (UNICAMP)**.

## 👨‍💻 Autor

**Lucas Kenji Kinoshita**  
Engenharia Elétrica — FEEC / UNICAMP

---

*Projeto em desenvolvimento. Os códigos, parâmetros e procedimentos experimentais podem ser atualizados conforme a evolução dos testes e da plataforma.*
