#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "lib/ssd1306.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "buzzer.h"
#include "pico/bootrom.h"
#include "stdio.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C
#define BOTAO_A 5 // Gera evento
#define BOTAO_B 6 // BOOTSEL
#define BOTAO_J 22 // Botão do Joystick
#define RED_PIN 13
#define BLUE_PIN 12
#define GREEN_PIN 11
#define BUZZER 10

#define MAXNUMEVENTOS 10
#define INITNUMEVENTOS 0

// Variáveis Globais
ssd1306_t ssd;
bool cor = true;
char buffer[35];
absolute_time_t last_time_A = 0;
absolute_time_t last_time_B = 0;
absolute_time_t last_time_J = 0;

// Declaração de semáforos & Mutex
SemaphoreHandle_t xContadorSem;
SemaphoreHandle_t xResetSem;
SemaphoreHandle_t xMutexDisplay;
SemaphoreHandle_t xEntradaDetectadaSem;
SemaphoreHandle_t xSaidaDetectadaSem;
SemaphoreHandle_t xBuzzLotadoSem;

volatile uint16_t PessoasPresentes = 0;

void vTaskLED(void *params){
    while(true){
        if(PessoasPresentes == 0){
            gpio_put(RED_PIN, false);
            gpio_put(GREEN_PIN, false);
            gpio_put(BLUE_PIN, true);

        }
        else if(PessoasPresentes > 0 && PessoasPresentes < MAXNUMEVENTOS - 1){
            gpio_put(RED_PIN, false);
            gpio_put(GREEN_PIN, true);
            gpio_put(BLUE_PIN, false);
        }
        else if(PessoasPresentes == MAXNUMEVENTOS - 1){
            gpio_put(RED_PIN, true);
            gpio_put(GREEN_PIN, true);
            gpio_put(BLUE_PIN, false);
        }
        else if(PessoasPresentes == MAXNUMEVENTOS){
            gpio_put(RED_PIN, true);
            gpio_put(GREEN_PIN, false);
            gpio_put(BLUE_PIN, false);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void vTaskEntrada(void *params){
    bool mostrandoMsgEntrada = false;
    TickType_t msgEntradaEndTick = 0;
    const TickType_t tempoMsgEvento = pdMS_TO_TICKS(2000); // 2 segundos

    while (true) {
        // Verifica se um novo evento de entrada ocorreu
        if (xSemaphoreTake(xEntradaDetectadaSem, 0) == pdTRUE) {
            mostrandoMsgEntrada = true;
            msgEntradaEndTick = xTaskGetTickCount() + tempoMsgEvento;
        }

        // Se a mensagem de entrada estiver ativa e o tempo acabou, para de mostrá-la
        if (mostrandoMsgEntrada && xTaskGetTickCount() >= msgEntradaEndTick) {
            mostrandoMsgEntrada = false;
        }

        // Tenta pegar o mutex para atualizar o display
        if (xSemaphoreTake(xMutexDisplay, pdMS_TO_TICKS(100)) == pdTRUE) { // Timeout para não bloquear indefinidamente
            ssd1306_fill(&ssd, !cor); // Limpa o display

            if (mostrandoMsgEntrada) {
                // Exibe mensagem de entrada
                ssd1306_draw_string(&ssd, "Pessoa(s) ", centralizar_texto("Pessoa(s) "), 20);
                ssd1306_draw_string(&ssd, "Entraram", centralizar_texto("Entraram"), 30);
                ssd1306_draw_string(&ssd, "No recinto!", centralizar_texto("No recinto!"), 40);
            } else {
                // Exibe contagem padrão OU mensagem de "Aguardando" se 0 pessoas
                if (PessoasPresentes == 0) {
                    ssd1306_draw_string(&ssd, "Aguardando", centralizar_texto("Aguardando"), 20);
                    ssd1306_draw_string(&ssd, "Eventos .....", centralizar_texto("Eventos ....."), 40);
                } else {
                    ssd1306_draw_string(&ssd, "N. de Pessoas", centralizar_texto("N. de Pessoas"), 20);
                    snprintf(buffer, sizeof(buffer), "No Local: %u", PessoasPresentes);
                    ssd1306_draw_string(&ssd, buffer, centralizar_texto(buffer), 40);
                }
            }
            ssd1306_send_data(&ssd);
            xSemaphoreGive(xMutexDisplay);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay da tarefa
    }
}


void vTaskSaida(void *params){
    bool mostrandoMsgSaida = false;
    TickType_t msgSaidaEndTick = 0;
    const TickType_t tempoMsgEvento = pdMS_TO_TICKS(2000); // 2 segundos

    while (true) {
        // Verifica se um novo evento de saída ocorreu
        if (xSemaphoreTake(xSaidaDetectadaSem, 0) == pdTRUE) {
            mostrandoMsgSaida = true;
            msgSaidaEndTick = xTaskGetTickCount() + tempoMsgEvento;
        }

        // Se a mensagem de saída estiver ativa e o tempo acabou, para de mostrá-la
        if (mostrandoMsgSaida && xTaskGetTickCount() >= msgSaidaEndTick) {
            mostrandoMsgSaida = false;
        }

        // Esta task só atualiza o display SE estiver mostrando sua mensagem de evento
        if (mostrandoMsgSaida) {
            if (xSemaphoreTake(xMutexDisplay, pdMS_TO_TICKS(100)) == pdTRUE) {
                ssd1306_fill(&ssd, !cor); // Limpa o display
                // Exibe mensagem de saída
                ssd1306_draw_string(&ssd, "Pessoa(s) ", centralizar_texto("Pessoa(s) "), 20);
                ssd1306_draw_string(&ssd, "Sairam", centralizar_texto("Sairam"), 30);
                ssd1306_draw_string(&ssd, "Do recinto!", centralizar_texto("Do recinto!"), 40);
                ssd1306_send_data(&ssd);
                xSemaphoreGive(xMutexDisplay);
            }
        }
        // Se não está mostrando sua mensagem, esta task simplesmente cede o processador.
        // A vTaskEntrada é responsável pela exibição padrão da contagem.
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay da tarefa
    }
}


void vTaskReset(void *params){
    BaseType_t xResultado;

    while(true){
        if(xSemaphoreTake(xResetSem, portMAX_DELAY) == pdTRUE){
            PessoasPresentes = 0;

            do{
                xResultado = xSemaphoreTake(xContadorSem, (TickType_t) 0);
            }while(xResultado == pdTRUE);

            if(xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE){
                ssd1306_fill(&ssd, !cor);
                ssd1306_draw_string(&ssd, "Reset", centralizar_texto("Reset"), 20);
                ssd1306_draw_string(&ssd, "Concluido!", centralizar_texto("Concluido!"), 40);
                ssd1306_send_data(&ssd);

                buzz(BUZZER, 600, 200);
                
                vTaskDelay(pdMS_TO_TICKS(250));
                
                buzz(BUZZER, 600, 200);

                xSemaphoreGive(xMutexDisplay);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void vTaskBuzzLotado(void *pvParameters) {
    while (true) {
        // Espera indefinidamente pelo sinal para fazer o buzz
        if (xSemaphoreTake(xBuzzLotadoSem, portMAX_DELAY) == pdTRUE) {
            buzz(BUZZER, 750, 150); 
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ISR para Semáforo & contagem de Pessoas
void gpio_irq_handler(uint gpio, uint32_t events)
{   
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    

    if (gpio == BOTAO_A){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_A > 200){
            if(PessoasPresentes < MAXNUMEVENTOS){
                PessoasPresentes++;
                xSemaphoreGiveFromISR(xContadorSem, &xHigherPriorityTaskWoken);
                xSemaphoreGiveFromISR(xEntradaDetectadaSem, &xHigherPriorityTaskWoken);
            }
            else{
                xSemaphoreGiveFromISR(xBuzzLotadoSem, &xHigherPriorityTaskWoken);
            }
        }
        last_time_A = current;
    }
    else if (gpio == BOTAO_B && PessoasPresentes > 0){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_B > 200){
            PessoasPresentes--;
            xSemaphoreTakeFromISR(xContadorSem, &xHigherPriorityTaskWoken);
            xSemaphoreTakeFromISR(xSaidaDetectadaSem, &xHigherPriorityTaskWoken);
        }
        last_time_B = current;
    }
    else if(gpio == BOTAO_J){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_J > 200){
            xSemaphoreGiveFromISR(xResetSem, &xHigherPriorityTaskWoken);
        }

        last_time_J = current;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);   
}

void setup(){

    // Inicialização do display
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Inicializa o LED RBG

    gpio_init(RED_PIN);
    gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_put(RED_PIN, false);
    
    gpio_init(BLUE_PIN);
    gpio_set_dir(BLUE_PIN, GPIO_OUT);
    gpio_put(BLUE_PIN, false);
    
    gpio_init(GREEN_PIN);
    gpio_set_dir(GREEN_PIN, GPIO_OUT);
    gpio_put(GREEN_PIN, false);

    // Configura os botões
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A); 
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(BOTAO_J);
    gpio_set_dir(BOTAO_J, GPIO_IN);
    gpio_pull_up(BOTAO_J);
    gpio_set_irq_enabled_with_callback(BOTAO_J, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Inicializa o Buzzer
    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT);
}

int main()
{
    setup();
    stdio_init_all();
    

    // Cria o mutex do display
    xMutexDisplay = xSemaphoreCreateMutex();    

    // Cria o Semáforo de Contagem
    xContadorSem = xSemaphoreCreateCounting(MAXNUMEVENTOS, INITNUMEVENTOS);
    
    // Cria os Semáforos Binários utilizados
    xResetSem = xSemaphoreCreateBinary();
    xEntradaDetectadaSem = xSemaphoreCreateBinary();
    xSaidaDetectadaSem = xSemaphoreCreateBinary();
    xBuzzLotadoSem = xSemaphoreCreateBinary();

    // Cria tarefa
    xTaskCreate(vTaskEntrada, "Task de Entrada", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskSaida, "Task de Saida", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskReset, "Task de Reset", configMINIMAL_STACK_SIZE + 128, NULL, 2, NULL);
    xTaskCreate(vTaskLED, "Task de LED", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskBuzzLotado, "Task do Buzzer", configMINIMAL_STACK_SIZE + 128, NULL, 3, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}
