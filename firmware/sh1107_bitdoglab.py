# ============================================================
# sh1107_bitdoglab.py
#
# Driver MicroPython para OLED SH1107 128x128
# BitDogLab
#
# Interface:
#   I2C
#
# Configuração utilizada na BitDogLab:
#   I2C1
#   SDA = GP2
#   SCL = GP3
#   endereço = 0x3C
#
# Esta versão usa uma sequência de inicialização mais robusta:
#
#   power-up
#      ↓
#   espera
#      ↓
#   DISPLAY OFF
#      ↓
#   configuração do SH1107
#      ↓
#   limpeza da RAM
#      ↓
#   espera
#      ↓
#   DISPLAY ON
#      ↓
#   nova limpeza
#
# ============================================================

import framebuf
from time import sleep_ms


class SH1107_I2C(framebuf.FrameBuffer):

    def __init__(
        self,
        width,
        height,
        i2c,
        addr=0x3C,
        rotate_180=False
    ):

        self.width = width
        self.height = height

        self.i2c = i2c
        self.addr = addr

        self.rotate_180 = rotate_180

        # ----------------------------------------------------
        # Framebuffer
        #
        # 128 x 128 / 8 = 2048 bytes
        # ----------------------------------------------------

        self.buffer = bytearray(
            self.width * self.height // 8
        )

        super().__init__(
            self.buffer,
            self.width,
            self.height,
            framebuf.MONO_VLSB
        )

        # ----------------------------------------------------
        # POWER-UP
        #
        # Dá tempo para a alimentação do módulo OLED e o
        # controlador SH1107 estabilizarem.
        # ----------------------------------------------------

        sleep_ms(300)

        # ----------------------------------------------------
        # Inicializa o controlador mantendo o display OFF
        # ----------------------------------------------------

        self.init_display()

        # ----------------------------------------------------
        # LIMPA A RAM ANTES DE LIGAR O PAINEL
        #
        # O framebuffer já nasce zerado, mas fazemos a limpeza
        # explicitamente e enviamos os 2048 bytes para a RAM
        # interna do SH1107 enquanto o painel está desligado.
        # ----------------------------------------------------

        self.fill(0)
        self.show()

        # Tempo para estabilização do circuito interno
        sleep_ms(150)

        # ----------------------------------------------------
        # Só agora liga o display
        # ----------------------------------------------------

        self.poweron()

        sleep_ms(200)

        # ----------------------------------------------------
        # Segunda limpeza de segurança
        # ----------------------------------------------------

        self.fill(0)
        self.show()


    # ========================================================
    # ENVIA COMANDO AO DISPLAY
    # ========================================================

    def write_cmd(self, cmd):

        self.i2c.writeto(
            self.addr,
            bytearray([
                0x00,
                cmd
            ])
        )


    # ========================================================
    # ENVIA DADOS AO DISPLAY
    #
    # Envio em blocos pequenos para evitar problemas com
    # buffers I2C em algumas versões do MicroPython.
    # ========================================================

    def write_data(self, data):

        CHUNK = 32

        for i in range(
            0,
            len(data),
            CHUNK
        ):

            bloco = data[
                i:i + CHUNK
            ]

            self.i2c.writeto(
                self.addr,
                bytearray([0x40]) + bloco
            )


    # ========================================================
    # INICIALIZAÇÃO DO SH1107
    #
    # IMPORTANTE:
    #
    # Esta função NÃO executa DISPLAY ON (0xAF).
    #
    # O painel permanece desligado até que a RAM tenha sido
    # completamente limpa pelo __init__().
    # ========================================================

    def init_display(self):

        # ----------------------------------------------------
        # Display OFF
        # ----------------------------------------------------

        self.write_cmd(0xAE)

        sleep_ms(20)


        # ----------------------------------------------------
        # Clock
        # ----------------------------------------------------

        self.write_cmd(0xD5)
        self.write_cmd(0x50)


        # ----------------------------------------------------
        # Memory addressing mode
        # ----------------------------------------------------

        self.write_cmd(0x20)


        # ----------------------------------------------------
        # Contraste
        # ----------------------------------------------------

        self.write_cmd(0x81)
        self.write_cmd(0x80)


        # ----------------------------------------------------
        # Multiplex ratio
        #
        # 128 linhas -> 127
        # ----------------------------------------------------

        self.write_cmd(0xA8)
        self.write_cmd(0x7F)


        # ----------------------------------------------------
        # Display offset
        # ----------------------------------------------------

        self.write_cmd(0xD3)
        self.write_cmd(0x00)


        # ----------------------------------------------------
        # Display start line
        # ----------------------------------------------------

        self.write_cmd(0xDC)
        self.write_cmd(0x00)


        # ----------------------------------------------------
        # DC-DC interno
        # ----------------------------------------------------

        self.write_cmd(0xAD)
        self.write_cmd(0x8B)

        sleep_ms(50)


        # ----------------------------------------------------
        # Orientação
        # ----------------------------------------------------

        if self.rotate_180:

            # Segment remap
            self.write_cmd(0xA1)

            # COM scan direction
            self.write_cmd(0xC8)

        else:

            self.write_cmd(0xA0)

            self.write_cmd(0xC0)


        # ----------------------------------------------------
        # Pre-charge period
        # ----------------------------------------------------

        self.write_cmd(0xD9)
        self.write_cmd(0x22)


        # ----------------------------------------------------
        # VCOM deselect level
        # ----------------------------------------------------

        self.write_cmd(0xDB)
        self.write_cmd(0x35)


        # ----------------------------------------------------
        # Display RAM
        #
        # Exibe conteúdo da RAM em vez de forçar todos os
        # pixels acesos.
        # ----------------------------------------------------

        self.write_cmd(0xA4)


        # ----------------------------------------------------
        # Normal display
        # ----------------------------------------------------

        self.write_cmd(0xA6)


        # ----------------------------------------------------
        # Define explicitamente a página inicial
        # ----------------------------------------------------

        self.write_cmd(0xB0)


        # ----------------------------------------------------
        # NÃO ligar o display aqui.
        #
        # O comando 0xAF será enviado somente depois que a RAM
        # tiver sido completamente zerada.
        # ----------------------------------------------------


    # ========================================================
    # ENVIA O FRAMEBUFFER PARA O OLED
    #
    # O SH1107 128x128 possui:
    #
    # 128 linhas / 8 = 16 páginas
    #
    # Cada página contém 128 bytes.
    # ========================================================

    def show(self):

        pages = self.height // 8

        for page in range(pages):

            # ------------------------------------------------
            # Seleciona página
            # ------------------------------------------------

            self.write_cmd(
                0xB0 | page
            )


            # ------------------------------------------------
            # Endereço de coluna baixa
            # ------------------------------------------------

            self.write_cmd(
                0x00
            )


            # ------------------------------------------------
            # Endereço de coluna alta
            # ------------------------------------------------

            self.write_cmd(
                0x10
            )


            inicio = (
                page * self.width
            )

            fim = (
                inicio + self.width
            )


            self.write_data(
                self.buffer[
                    inicio:fim
                ]
            )


    # ========================================================
    # LIMPA DISPLAY
    # ========================================================

    def clear(self):

        self.fill(0)
        self.show()


    # ========================================================
    # CONTRASTE
    # 0 ... 255
    # ========================================================

    def contrast(self, value):

        value = max(
            0,
            min(
                255,
                value
            )
        )

        self.write_cmd(0x81)

        self.write_cmd(value)


    # ========================================================
    # INVERTE PRETO / BRANCO
    # ========================================================

    def invert(self, invert=True):

        if invert:

            self.write_cmd(0xA7)

        else:

            self.write_cmd(0xA6)


    # ========================================================
    # DESLIGA DISPLAY
    # ========================================================

    def poweroff(self):

        self.write_cmd(0xAE)


    # ========================================================
    # LIGA DISPLAY
    # ========================================================

    def poweron(self):

        self.write_cmd(0xAF)