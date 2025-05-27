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
#define BOTAO_A 5 // Botão A
#define BOTAO_B 6 // Botão B
#define BOTAO_J 22 // Botão do Joystick
#define RED_PIN 13
#define BLUE_PIN 12
#define GREEN_PIN 11
#define BUZZER 10

// Defines para aumentar o número máximo de pessoas
#define MAXNUMEVENTOS 10
#define INITNUMEVENTOS 0

// Variáveis Globais
ssd1306_t ssd;
bool cor = true;
char buffer[35];
// Variáveis para debounce
absolute_time_t last_time_A = 0;
absolute_time_t last_time_B = 0;
absolute_time_t last_time_J = 0;

// Declaração de semáforos & Mutex
SemaphoreHandle_t xContadorSem;
SemaphoreHandle_t xResetSem;
SemaphoreHandle_t xEntradaDetectadaSem;
SemaphoreHandle_t xSaidaDetectadaSem;
SemaphoreHandle_t xBuzzLotadoSem;
SemaphoreHandle_t xMutexDisplay;
SemaphoreHandle_t xPessoasPresentesMutex;

volatile uint16_t PessoasPresentes = 0; // Variável global de contagem de pessoas

void vTaskLED(void *params){
    uint16_t localPessoasPresentes;

    while(true){
        if (xSemaphoreTake(xPessoasPresentesMutex, portMAX_DELAY) == pdTRUE) { // Espera por um sinal do mutex para receber variável global PessoasPresentes
            localPessoasPresentes = PessoasPresentes;
            xSemaphoreGive(xPessoasPresentesMutex);
        }

        // Liga LED azul
        if(localPessoasPresentes == 0){
            gpio_put(RED_PIN, false);
            gpio_put(GREEN_PIN, false);
            gpio_put(BLUE_PIN, true);

        }
        // Liga LED verde
        else if(localPessoasPresentes > 0 && localPessoasPresentes < MAXNUMEVENTOS - 1){
            gpio_put(RED_PIN, false);
            gpio_put(GREEN_PIN, true);
            gpio_put(BLUE_PIN, false);
        }
        // Liga LED amarelo
        else if(localPessoasPresentes == MAXNUMEVENTOS - 1){
            gpio_put(RED_PIN, true);
            gpio_put(GREEN_PIN, true);
            gpio_put(BLUE_PIN, false);
        }
        // Liga LED vermelho
        else if(localPessoasPresentes == MAXNUMEVENTOS){
            gpio_put(RED_PIN, true);
            gpio_put(GREEN_PIN, false);
            gpio_put(BLUE_PIN, false);
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Evitar uso excessivo da CPU
    }
}


void vTaskEntrada(void *params){
    bool exibirMsgEntradaEsteCiclo = false;
    const TickType_t tempoMsgEvento = pdMS_TO_TICKS(500); // 0.5 segundo

    while (true) {
        exibirMsgEntradaEsteCiclo = false; // Reset da flag a cada ciclo do loop principal da task

        // 1. Verifica se Botão A foi pressionado (evento de entrada)
        if (xSemaphoreTake(xEntradaDetectadaSem, 0) == pdTRUE) { // Não bloqueante
            if (xSemaphoreTake(xPessoasPresentesMutex, portMAX_DELAY) == pdTRUE) {
                if (PessoasPresentes < MAXNUMEVENTOS) {
                    PessoasPresentes++;
                    if (xContadorSem != NULL) 
                        xSemaphoreGive(xContadorSem);
                    exibirMsgEntradaEsteCiclo = true; // Sinaliza para exibir mensagem de entrada
                } else {
                    if (xBuzzLotadoSem != NULL) // Verifica se xBuzzLotadoSem foi criado
                        xSemaphoreGive(xBuzzLotadoSem);
                }
                xSemaphoreGive(xPessoasPresentesMutex); // Libera mutex de PessoasPresentes
            }
        }

        // 2. Lógica de atualização do Display
        if (xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE) { 
            ssd1306_fill(&ssd, !cor); // Limpa display
            uint16_t displayPessoasPresentes; // Variável local para armazenar a variável global PessoasPresentes
            
            if (xSemaphoreTake(xPessoasPresentesMutex, portMAX_DELAY) == pdTRUE) {
                displayPessoasPresentes = PessoasPresentes;
                xSemaphoreGive(xPessoasPresentesMutex);
            }

            if (exibirMsgEntradaEsteCiclo) {
                ssd1306_draw_string(&ssd, "Pessoa(s) ", centralizar_texto("Pessoa(s) "), 20);
                ssd1306_draw_string(&ssd, "Entraram", centralizar_texto("Entraram"), 30);
                ssd1306_draw_string(&ssd, "No recinto!", centralizar_texto("No recinto!"), 40);
                ssd1306_send_data(&ssd);
                // Mantém o display com esta mensagem e depois libera o mutex após o delay
                vTaskDelay(tempoMsgEvento); 
            } else {
                // Exibe contagem padrão ou "Aguardando"
                if (displayPessoasPresentes == 0) {
                    ssd1306_draw_string(&ssd, "Aguardando", centralizar_texto("Aguardando"), 20);
                    ssd1306_draw_string(&ssd, "Eventos .....", centralizar_texto("Eventos ....."), 40);
                } else {
                    ssd1306_draw_string(&ssd, "N. de Pessoas", centralizar_texto("N. de Pessoas"), 20);
                    snprintf(buffer, sizeof(buffer), "No Local: %u", displayPessoasPresentes);
                    ssd1306_draw_string(&ssd, buffer, centralizar_texto(buffer), 40);
                }
                ssd1306_send_data(&ssd);
            }
            xSemaphoreGive(xMutexDisplay);  // Devolve o mutex do display
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay da vTaskEntrada para evitar uso excessivo da CPU
    }
}


void vTaskSaida(void *params){
    const TickType_t tempoMsgEvento = pdMS_TO_TICKS(500);

    while (true) {
        // Espera  por um sinal de evento de saída (Botão B)
        if (xSemaphoreTake(xSaidaDetectadaSem, portMAX_DELAY) == pdTRUE) {
            bool alguemRealmenteSaiu = false;   // Variável para verificar se realmente alguém saiu do recinto - Para o caso onde PessoasPresentes > 0
            if (xSemaphoreTake(xPessoasPresentesMutex, portMAX_DELAY) == pdTRUE) {
                if (PessoasPresentes > 0) {
                    PessoasPresentes--;
                    alguemRealmenteSaiu = true;
                    if (xContadorSem != NULL)   // Verifica se xContadorSem foi criado
                        xSemaphoreTake(xContadorSem, 0); // Não bloqueia a verificação no semáforo
                }
                xSemaphoreGive(xPessoasPresentesMutex);  // Devolve o mutex da variável do PessoasPresentes
            }

            // Exibe mensagem se alguém realmente saiu do recinto
            if (alguemRealmenteSaiu) {
                if (xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE) {
                    ssd1306_fill(&ssd, !cor);   // Limpa o display
                    // Imprime mensagem no display
                    ssd1306_draw_string(&ssd, "Pessoa(s) ", centralizar_texto("Pessoa(s) "), 20);
                    ssd1306_draw_string(&ssd, "Sairam", centralizar_texto("Sairam"), 30);
                    ssd1306_draw_string(&ssd, "Do recinto!", centralizar_texto("Do recinto!"), 40);
                    ssd1306_send_data(&ssd);
                    // Mantém o display com esta mensagem e após isso devolve o mutex
                    vTaskDelay(tempoMsgEvento);
                    xSemaphoreGive(xMutexDisplay);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Opcional - Evitar o uso excessivo da CPU
    }
}


void vTaskReset(void *params){
    BaseType_t xResultado;      // Variável para armazenar tipo pdTRUE ou pdFALSE

    while(true){
        if(xSemaphoreTake(xResetSem, portMAX_DELAY) == pdTRUE){
           
            if (xSemaphoreTake(xPessoasPresentesMutex, portMAX_DELAY) == pdTRUE) {      // Pega o Mutex 
                PessoasPresentes = 0;
                xSemaphoreGive(xPessoasPresentesMutex); // Devolve o Mutex
            }

            do{
                xResultado = xSemaphoreTake(xContadorSem, 0);
            }while(xResultado == pdTRUE);

            if(xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE){
                ssd1306_fill(&ssd, !cor);                   // Limpa o display
                // Imprime mensagem no display
                ssd1306_draw_string(&ssd, "Reset", centralizar_texto("Reset"), 20);
                ssd1306_draw_string(&ssd, "Concluido!", centralizar_texto("Concluido!"), 40);
                ssd1306_send_data(&ssd);

                // Buzzer emitindo 2 beeps curtos
                buzz(BUZZER, 600, 200);
                
                vTaskDelay(pdMS_TO_TICKS(250));
                
                buzz(BUZZER, 600, 200);

                xSemaphoreGive(xMutexDisplay);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Delay para evitar uso excessivo da CPU
    }
}


void vTaskBuzzLotado(void *pvParameters) {
    while (true) {
        // Espera indefinidamente pelo sinal para fazer o buzz
        if (xSemaphoreTake(xBuzzLotadoSem, portMAX_DELAY) == pdTRUE) {
            buzz(BUZZER, 750, 150);     // Função declarada no buzzer.c
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Delay para evitar uso excessivo da CPU
    }
}

// ISR para Semáforo & contagem de Pessoas
void gpio_irq_handler(uint gpio, uint32_t events)
{   
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Verifica se é o botão A e faz debounce
    if (gpio == BOTAO_A){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_A > 200){
            if(xEntradaDetectadaSem != NULL)    // Verifica se xEntradaDetectadaSem foi criada
                xSemaphoreGiveFromISR(xEntradaDetectadaSem, &xHigherPriorityTaskWoken); //Dá o semáforo binário para o vTaskEntrada
        }
        last_time_A = current;
    }
    // Verifica se é o botão B e faz debounce
    else if (gpio == BOTAO_B && PessoasPresentes > 0){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_B > 200){
            if(xSaidaDetectadaSem != NULL)  // Verifica se xSaidaDetectadaSem foi criada
                xSemaphoreGiveFromISR(xSaidaDetectadaSem, &xHigherPriorityTaskWoken);   //Dá o semáforo binário para o vTaskSaida
        }
        last_time_B = current;
    }
    // Verifica se é o botão Joystick e faz debounce
    else if(gpio == BOTAO_J){
        absolute_time_t current = to_ms_since_boot(get_absolute_time());
        if(current - last_time_J > 200){
            if(xResetSem != NULL)   // Verifica se xResetSem foi criada
                xSemaphoreGiveFromISR(xResetSem, &xHigherPriorityTaskWoken);    //Dá o semáforo binário para o vTaskReset
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
    xPessoasPresentesMutex = xSemaphoreCreateMutex(); 

    // Cria o Semáforo de Contagem
    xContadorSem = xSemaphoreCreateCounting(MAXNUMEVENTOS, INITNUMEVENTOS);
    
    // Cria os Semáforos Binários utilizados
    xResetSem = xSemaphoreCreateBinary();
    xEntradaDetectadaSem = xSemaphoreCreateBinary();
    xSaidaDetectadaSem = xSemaphoreCreateBinary();
    xBuzzLotadoSem = xSemaphoreCreateBinary();

    // Cria tarefas 
    xTaskCreate(vTaskEntrada, "Task de Entrada", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskSaida, "Task de Saida", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskReset, "Task de Reset", configMINIMAL_STACK_SIZE + 128, NULL, 2, NULL);
    xTaskCreate(vTaskLED, "Task de LED", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL);
    xTaskCreate(vTaskBuzzLotado, "Task do Buzzer", configMINIMAL_STACK_SIZE + 128, NULL, 3, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}
