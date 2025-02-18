# Projeto: Controle de LEDs RGB com Joystick e Display SSD1306
### AUTOR
- **Wendel Souza Santos

## Descrição

Este projeto demonstra o uso do conversor analógico-digital (ADC), PWM e comunicação I2C no microcontrolador RP2040. O objetivo é controlar dois LEDs RGB (vermelho e azul) com base nos valores lidos de um joystick analógico e representar a posição do joystick em um display OLED SSD1306. Além disso, o projeto inclui funcionalidades adicionais, como alternar estados dos LEDs e alterar o comportamento da borda no display.

### Funcionalidades
- **Leitura do joystick**: Utiliza o ADC para ler os valores dos eixos X e Y do joystick.
- **Controle de LEDs via PWM**: Ajusta a intensidade dos LEDs vermelho e azul com base na posição do joystick.
- **Display SSD1306**: Representa a posição do joystick como um quadrado móvel no display.
- **Botões interativos**:
  - Botão do joystick: Alterna o estado do LED verde e muda o estilo da borda no display.
  - Botão A: Liga/desliga o controle PWM dos LEDs vermelho e azul.
- **Zona morta**: Ignora pequenas variações ao redor do centro do joystick para evitar ruídos.

---

## Requisitos

### Hardware
- **Microcontrolador**: Raspberry Pi Pico (RP2040).
- **Joystick analógico**: Com três pinos (X, Y e botão).
- **LEDs RGB**: Dois LEDs (vermelho e azul) conectados aos pinos PWM.
- **Display OLED SSD1306**: Comunicação via I2C.
- **Botões**: Dois botões (joystick e botão A).

### Software
- **SDK**: [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- **Biblioteca SSD1306**: Incluída no diretório `biblioteca/ssd1306.h`.
- **Compilador**: GCC para ARM Cortex-M.

---

## Configuração do Projeto

### Estrutura do Código
O código está organizado em funções modulares para facilitar a compreensão e manutenção:

1. **Inicialização**:
   - Configuração do ADC, GPIO, PWM, I2C e interrupções.
2. **Calibração do joystick**:
   - Determina os valores centrais dos eixos X e Y para correção.
3. **Funções principais**:
   - `ajuste_value`: Aplica a zona morta ao valor do joystick.
   - `altera_led_verde` e `alterna_led_pwm`: Manipulam o estado dos LEDs.
   - `on_button_press_callback`: Gerencia as interrupções dos botões.
4. **Loop principal**:
   - Lê o joystick, controla os LEDs e atualiza o display.

---

## Como Executar

### Passo 1: Clone o Repositório
```bash
git clone https://github.com/wendelvox/Conversores-AD.git

```

### Passo 2: Configure o Ambiente
Certifique-se de que o Pico SDK está configurado corretamente. Siga as instruções no [guia oficial](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).

### Passo 3: Compile o Código
No terminal, execute:
```bash
mkdir build
cd build
cmake ..
make
```

### Passo 4: Grave o Firmware
Conecte o Raspberry Pi Pico ao computador no modo de gravação (pressionando o botão BOOTSEL). Copie o arquivo `.uf2` gerado para o dispositivo.



