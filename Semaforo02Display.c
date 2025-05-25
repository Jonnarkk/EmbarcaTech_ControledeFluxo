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

ssd1306_t ssd;
bool cor = true;
SemaphoreHandle_t xContadorSem;
SemaphoreHandle_t xResetSem;
SemaphoreHandle_t xMutexDisplay;
char buffer[35];
absolute_time_t last_time_A = 0;
absolute_time_t last_time_B = 0;
absolute_time_t last_time_J = 0;

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

        vTaskDelay(50);
    }
}


void vTaskEntrada(void *params){
    while(true){
        if(xSemaphoreTake(xContadorSem, portMAX_DELAY) == pdTRUE){
            if(xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE){
                ssd1306_fill(&ssd, !cor);
                ssd1306_draw_string(&ssd, "Pessoas(s) entraram", centralizar_texto("Pessoas(s) entraram"), 20);
                ssd1306_draw_string(&ssd, "No recinto!", centralizar_texto("No recinto!"), 40);
                ssd1306_send_data(&ssd);

                vTaskDelay(pdMS_TO_TICKS(1500));

                ssd1306_fill(&ssd, !cor);
                ssd1306_draw_string(&ssd, "N. de Pessoas", centralizar_texto("N. de Pessoas"), 20);
                snprintf(buffer, sizeof(buffer), "No Local: %d", PessoasPresentes);
                ssd1306_draw_string(&ssd, buffer, centralizar_texto(buffer), 40);
                ssd1306_send_data(&ssd);

                vTaskDelay(pdMS_TO_TICKS(1500));

                xSemaphoreGive(xMutexDisplay);
            }
    }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void vTaskSaida(void *params){
    while(true){
        if(xSemaphoreGive(xContadorSem)){
            if(xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE){
                    ssd1306_fill(&ssd, !cor);
                    ssd1306_draw_string(&ssd, "Pessoas(s) entraram", centralizar_texto("Pessoas(s) entraram"), 20);
                    ssd1306_draw_string(&ssd, "No recinto!", centralizar_texto("No recinto!"), 40);
                    ssd1306_send_data(&ssd);

                    vTaskDelay(pdMS_TO_TICKS(1500));

                    ssd1306_fill(&ssd, !cor);
                    ssd1306_draw_string(&ssd, "N. de Pessoas", centralizar_texto("N. de Pessoas"), 20);
                    snprintf(buffer, sizeof(buffer), "No Local: %d", PessoasPresentes);
                    ssd1306_draw_string(&ssd, buffer, centralizar_texto(buffer), 40);
                    ssd1306_send_data(&ssd);

                    vTaskDelay(pdMS_TO_TICKS(1500));

                    xSemaphoreGive(xMutexDisplay);
                }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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
                
                vTaskDelay(pdMS_TO_TICKS(500));
                
                buzz(BUZZER, 600, 200);

                xSemaphoreGive(xMutexDisplay);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ISR para BOOTSEL e botão de evento
void gpio_irq_handler(uint gpio, uint32_t events)
{   
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    

    if (gpio == BOTAO_A && PessoasPresentes < 10){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_A > 200){
            xSemaphoreGiveFromISR(xContadorSem, &xHigherPriorityTaskWoken);
            PessoasPresentes++;
        }
        last_time_A = current;
    }
    else if (gpio == BOTAO_B && PessoasPresentes > 0){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_B > 200){
            xSemaphoreTakeFromISR(xContadorSem, &xHigherPriorityTaskWoken);
            PessoasPresentes--;
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
    
    xResetSem = xSemaphoreCreateBinary();

    // Cria tarefa
    xTaskCreate(vTaskEntrada, "Task de Entrada", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskSaida, "Task de Saida", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskReset, "Task de Reset", configMINIMAL_STACK_SIZE + 128, NULL, 2, NULL);
    xTaskCreate(vTaskLED, "Task de LED", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}
