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

ssd1306_t ssd;
bool cor = true;
SemaphoreHandle_t xContadorSem;
SemaphoreHandle_t xResetSem;
SemaphoreHandle_t xBotaoBSem;
SemaphoreHandle_t xBotaoJSem;
SemaphoreHandle_t xMutexDisplay;

uint16_t eventosProcessados = 0;

// ISR do botão A (incrementa o semáforo de contagem)
void gpio_callback(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xContadorSem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void vTaskEntrada(void *params){
    while(true){
        if(xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE){
            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd, "Testeeeee 1-2-3", 5, 20);
            ssd1306_send_data(&ssd);
            xSemaphoreGive(xMutexDisplay);
        }
        
        if(xSemaphoreTake(xResetSem, portMAX_DELAY) == pdTRUE){
            ssd1306_fill(&ssd, !cor);
            ssd1306_draw_string(&ssd, "BOTAO A PRESSIONADO", 5, 20);
            ssd1306_send_data(&ssd);

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void vTaskSaida(void *params){
    while(true){

    }
}


void vTaskReset(void *params){
    while(true){

    }
}

// ISR para BOOTSEL e botão de evento
void gpio_irq_handler(uint gpio, uint32_t events)
{
    if (gpio == BOTAO_B)
    {
        reset_usb_boot(0, 0);
    }
    else if (gpio == BOTAO_A)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xResetSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
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
    
}

int main()
{
    setup();
    stdio_init_all();
    

    // Cria o mutex do display
    xMutexDisplay = xSemaphoreCreateMutex();    

    xResetSem = xSemaphoreCreateBinary();
    xBotaoBSem = xSemaphoreCreateBinary();
    xBotaoJSem = xSemaphoreCreateBinary();

    // Cria tarefa
    xTaskCreate(vTaskEntrada, "Task de Entrada", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskSaida, "Task de Saida", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskReset, "Task de Reset", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}
