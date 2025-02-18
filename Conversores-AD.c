#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "biblioteca/ssd1306.h"

/*
    Tarefa UNIDADE 4 CONVERSOR AD
    Compreender o funcionamento do conversor analógico-digital (ADC) no RP2040.
    Utilizar o PWM para controlar a intensidade de dois LEDs RGB com base nos valores do joystick.
    Representar a posição do joystick no display SSD1306 por meio de um quadrado móvel.
    Aplicar o protocolo de comunicação I2C na integração com o display.
    AUTOR: WENDEL SOUZA SANTOS
*/

// Definições do I2C
#define I2C_PORT       i2c1
#define I2C_SDA        14
#define I2C_SCL        15
#define DISP_ADDR      0x3C

// Definições do Joystick
#define JOY_X          26
#define JOY_Y          27
#define JOY_BTN        22
#define BTN_A          5
#define LED_R          13
#define LED_G          11
#define LED_B          12
#define DEADZONE       200
#define INTENSITY_SCALE 0.5f // Fator de escala (50% da intensidade máxima)

// Variáveis globais
ssd1306_t display;              // Display SSD1306
bool border_state = false;      // Estado da borda (simples/dupla)
bool led_green_state = false;   // Estado do LED verde
bool led_pwm_state = true;      // Estado do PWM dos LEDs R e B
uint16_t x_center, y_center;    // Valores centrais calibrados do joystick
uint16_t led_green_intensity = 0;
// -------------------------------------------
// Função: Inicializa GPIO para PWM
// -------------------------------------------
void pwm_init_gpio(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // Configura o pino para PWM
    uint slice = pwm_gpio_to_slice_num(gpio); // Obtém o slice do PWM
    pwm_set_wrap(slice, 4095); // Define o valor máximo do PWM (12 bits)
    pwm_set_enabled(slice, true); // Habilita o PWM
}

// --------------------------------------------
// Função: Aplica zona morta ao valor do joystick
// ---------------------------------------------
int16_t ajuste_value(int16_t raw, int16_t center) {
    int16_t diff = raw - center; // Calcula a diferença entre o valor bruto e o centro
    if (abs(diff) < DEADZONE) { // Verifica se a diferença está dentro da zona morta
        return 0; // Retorna 0 se estiver dentro da zona morta
    }
    return diff; // Retorna a diferença ajustada
}

// ------------------------------------------------
// Função: Alterna o estado do LED verde
// ------------------------------------------------
void altera_led_verde(uint gpio, uint32_t events) {
    static bool is_on = false;

    if (is_on) {
        led_green_intensity = 0; // Desliga o LED
    } else {
        led_green_intensity = (uint16_t)(2048 * INTENSITY_SCALE); // Define a intensidade
    }
    
    
    led_green_state = !led_green_state; // Alterna o estado do LED verde
    gpio_put(LED_G, led_green_state); // Atualiza o estado do LED no hardware
    border_state = !border_state; // Alterna o estado da borda
}

// -----------------------------------------------
// Função: Alterna o estado do PWM dos LEDs R e B
// -----------------------------------------------
void alterna_led_pwm(uint gpio, uint32_t events) {
    led_pwm_state = !led_pwm_state; // Alterna o estado do PWM
}

// -----------------------------------------------
// Função CALLBACK para os botões
// -----------------------------------------------
void on_button_press_callback(uint gpio, uint32_t events) {
    // Verifica qual pino gerou a interrupção
    if (gpio == JOY_BTN && (events & GPIO_IRQ_EDGE_FALL)) {
        altera_led_verde(gpio, events); // Chama a função para o botão do joystick
    } else if (gpio == BTN_A && (events & GPIO_IRQ_EDGE_FALL)) {
        alterna_led_pwm(gpio, events); // Chama a função para o botão A
    }
}

// ---------------------------------------------------
// Função: Calibração do joystick
// ---------------------------------------------------
void calibrate_joystick() {
    const int samples = 100; // Número de amostras para calibração
    uint32_t sum_x = 0, sum_y = 0;

    for (int i = 0; i < samples; i++) {
        adc_select_input(0); // Seleciona o canal 0 (X)
        sum_x += adc_read(); // Lê o valor do ADC para X
        adc_select_input(1); // Seleciona o canal 1 (Y)
        sum_y += adc_read(); // Lê o valor do ADC para Y
        sleep_ms(5); // Aguarda um curto período entre as leituras
    }

    x_center = sum_x / samples; // Calcula o valor central de X
    y_center = sum_y / samples; // Calcula o valor central de Y
    printf("Calibração: x_center=%d, y_center=%d\n", x_center, y_center);
}

// ---------------------------------------------------
// Função principal
// ---------------------------------------------------
int main() {
    stdio_init_all(); // Inicializa a interface serial

    // ---------- Inicialização do ADC ----------
    adc_init(); // Inicializa o módulo ADC
    adc_gpio_init(JOY_X); // Configura o pino do eixo X do joystick
    adc_gpio_init(JOY_Y); // Configura o pino do eixo Y do joystick

    // ---------- Configuração dos botões ----------
    gpio_init(JOY_BTN); // Configura o botão do joystick
    gpio_set_dir(JOY_BTN, GPIO_IN); // Define como entrada
    gpio_pull_up(JOY_BTN); // Habilita resistor de pull-up

    gpio_init(BTN_A); // Configura o botão A
    gpio_set_dir(BTN_A, GPIO_IN); // Define como entrada
    gpio_pull_up(BTN_A); // Habilita resistor de pull-up

    // ---------- Configuração das interrupções ----------
    gpio_set_irq_enabled_with_callback(JOY_BTN, GPIO_IRQ_EDGE_FALL, 1, on_button_press_callback); // Botão do joystick
    gpio_set_irq_enabled(BTN_A, GPIO_IRQ_EDGE_FALL, 1); // Botão A

    // ---------- Configuração dos LEDs ----------
    pwm_init_gpio(LED_R); // Configura o LED vermelho (PWM)
    pwm_init_gpio(LED_B); // Configura o LED azul (PWM)
    gpio_init(LED_G); // Configura o LED verde
    gpio_set_dir(LED_G, GPIO_OUT); // Define como saída
    gpio_put(LED_G, 0); // Desliga o LED verde inicialmente

    // ---------- Configuração do display SSD1306 ----------
    i2c_init(I2C_PORT, 400 * 1000); // Inicializa o I2C com 400 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura o pino SDA
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura o pino SCL
    gpio_pull_up(I2C_SDA); // Habilita resistor de pull-up no SDA
    gpio_pull_up(I2C_SCL); // Habilita resistor de pull-up no SCL
    ssd1306_init(&display, 128, 64, false, DISP_ADDR, I2C_PORT); // Inicializa o display
    ssd1306_config(&display); // Configura o display
    ssd1306_send_data(&display); // Envia os dados iniciais para o display

    // ---------- Calibração do joystick ----------
    calibrate_joystick();

    // ---------- Loop principal ----------
    while (true) {
        // 1) Ler valores do joystick
        adc_select_input(0); // Canal 0 => Eixo X
        uint16_t raw_x = adc_read();
        adc_select_input(1); // Canal 1 => Eixo Y
        uint16_t raw_y = adc_read();

        // 2) Controlar LEDs R e B usando calibração
        if (led_pwm_state) {
            int16_t adj_x = ajuste_value(raw_x, x_center); // Ajusta o valor de X
            int16_t adj_y = ajuste_value(raw_y, y_center); // Ajusta o valor de Y
            pwm_set_gpio_level(LED_R, abs(adj_y) * INTENSITY_SCALE); // Controla o LED vermelho
            pwm_set_gpio_level(LED_B, abs(adj_x) * INTENSITY_SCALE); // Controla o LED azul
        } else {
            pwm_set_gpio_level(LED_R, 0); // Desliga o LED vermelho
            pwm_set_gpio_level(LED_B, 0); // Desliga o LED azul
        }

        // 3) Mapear posição do quadrado no display
        int x_pos = ((4095 - raw_x) * 52) / 4095; // Calcula a posição X no display
        int y_pos = (raw_y * 113) / 4095; // Calcula a posição Y no display

        // 4) Desenhar no display
        ssd1306_fill(&display, false); // Limpa o display
        if (!border_state) {
            ssd1306_rect(&display, 0, 0, 127, 63, 1, false); // Desenha borda simples
        } else {
            ssd1306_rect(&display, 0, 0, 127, 63, 1, false); // Desenha borda dupla
            ssd1306_rect(&display, 2, 2, 124, 60, 1, false);
            
        }
        ssd1306_rect(&display, x_pos, y_pos, 8, 8, 1, true); // Desenha o quadrado móvel
        ssd1306_send_data(&display); // Atualiza o display

        sleep_ms(50); // Aguarda 50 ms antes da próxima iteração
    }

    return 0;
}