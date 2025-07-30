#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"
#include "pico/bootrom.h"
#include "ssd1306.h"

#define ADC_PIN 26
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define ENDERECO_MPU6050 0x68

#define MAX_AMOSTRAS 99999
#define PERIODO_MS 1000
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define ENDERECO_DISP 0x3C
#define DISP_W 128
#define DISP_H 64
#define MENSAGEM_TIMEOUT_MS 2000 // Tempo para exibir mensagens de comando (2s)
#define DISPLAY_UPDATE_MS 500    // Atualizar display a cada 500ms

// botoes A e B

#define BOTAO_A 5
#define BOTAO_B 6
#define JOYSTICK_SW 22

// leds RGB
#define LED_G 11
#define LED_B 12
#define LED_R 13

// buzzer
#define BUZZER_PIN 10
// Configuração da frequência do buzzer (em Hz)
#define BUZZER_FREQUENCY 3500

// flags de controle

// flag paar controla MONTAR/DESMONTAR
// onde 1 = montar e 0 = desmontar
static bool flag_montar = true;
static bool flag_desmontar = false;

// flag para gravar e pausar
// onde 1 = gravar e 0 = pausar
static bool flag_gravar = false;
static bool flag_parar_gravar = false;

static void mpu6050_reset(void);
static bool mpu6050_testar(void);
static void mpu6050_ler_dados(int16_t accel[3], int16_t gyro[3], int16_t *temp);
static void capture_adc_data_and_save(void);
static void capturar_dados_mpu6050_e_salvar(void);
static void run_setrtc(void);
static void run_format(void);
static void run_mount(void);
static void run_unmount(void);
static void run_getfree(void);
static void run_ls(void);
static void run_cat(void);
static void run_iniciar(void);
static void run_ajuda(void);
static void processar_stdio(int cRxedChar);
static void ler_arquivo(const char *nome_arquivo);
static void gpio_irq_handler(uint gpio, uint32_t events);
static void exibir_data_hora(void);

static bool logger_ativado = false;
static absolute_time_t proxima_captura;
static char nome_arquivo[32]; // Aumentado para suportar "dadosDDMMAAAAHHMMSS.csv"
static int contador_amostras = 0;
static ssd1306_t ssd;
static absolute_time_t mensagem_timeout = {0};           // Controla timeout da mensagem
static absolute_time_t ultima_atualizacao_display = {0}; // Controla atualização do display

static sd_card_t *sd_obter_por_nome(const char *const nome)
{
   // printf("[DEBUG] sd_obter_por_nome: Procurando %s\n", nome);
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, nome))
            return sd_get_by_num(i);
    printf("[ERRO] sd_obter_por_nome: Nome desconhecido %s\n", nome);
    return NULL;
}

static FATFS *sd_obter_fs_por_nome(const char *nome)
{
    printf("[DEBUG] sd_obter_fs_por_nome: Procurando %s\n", nome);
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, nome))
            return &sd_get_by_num(i)->fatfs;
    printf("[ERRO] sd_obter_fs_por_nome: Nome desconhecido %s\n", nome);
    return NULL;
}

static bool sd_esta_montado(const char *nome)
{
    //printf("[DEBUG] sd_esta_montado: Verificando %s\n", nome);
    sd_card_t *pSD = sd_obter_por_nome(nome);
    bool montado = pSD && pSD->mounted;
    //printf("[DEBUG] sd_esta_montado: Resultado=%d\n", montado);
    return montado;
}

static bool mpu6050_testar()
{
    printf("[DEBUG] mpu6050_testar: Iniciando teste...\n");
    uint8_t who_am_i = 0x75;
    uint8_t valor;
    int write_result = i2c_write_blocking(I2C_PORT, ENDERECO_MPU6050, &who_am_i, 1, true);
    printf("[DEBUG] mpu6050_testar: I2C write (WHO_AM_I): resultado=%d\n", write_result);
    sleep_us(100);
    if (write_result != 1)
    {
        printf("[ERRO] mpu6050_testar: Falha na escrita I2C\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro MPU6050", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return false;
    }
    int read_result = i2c_read_blocking(I2C_PORT, ENDERECO_MPU6050, &valor, 1, false);
    printf("[DEBUG] mpu6050_testar: I2C read (WHO_AM_I): resultado=%d, valor=0x%02X\n", read_result, valor);
    sleep_us(100);
    if (read_result != 1)
    {
        printf("[ERRO] mpu6050_testar: Falha na leitura I2C\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro MPU6050", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return false;
    }
    bool sucesso = (valor == 0x68 || valor == 0x70);
    if (valor == 0x70)
        printf("[AVISO] mpu6050_testar: Endereço I2C não padrão detectado (0x70).\n");
    printf("[DEBUG] mpu6050_testar: Resultado=%s (esperado=0x68 ou 0x70, recebido=0x%02X)\n", sucesso ? "Sucesso" : "Falha", valor);
    return sucesso;
}

static void mpu6050_reset()
{
    printf("[DEBUG] mpu6050_reset: Iniciando reset...\n");
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, ENDERECO_MPU6050, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, ENDERECO_MPU6050, buf, 2, false);
    sleep_ms(10);
    printf("[DEBUG] mpu6050_reset: Reset concluído\n");
}

static void mpu6050_ler_dados(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    printf("[DEBUG] mpu6050_ler_dados: Iniciando leitura...\n");
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    int write_result = i2c_write_blocking(I2C_PORT, ENDERECO_MPU6050, &val, 1, true);
    printf("[DEBUG] mpu6050_ler_dados: I2C write (accel, 0x3B): resultado=%d\n", write_result);
    sleep_us(100);
    int read_result = i2c_read_blocking(I2C_PORT, ENDERECO_MPU6050, buffer, 6, false);
    printf("[DEBUG] mpu6050_ler_dados: I2C read (accel): resultado=%d\n", read_result);
    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    printf("[DEBUG] mpu6050_ler_dados: Acelerômetro: X=%d, Y=%d, Z=%d\n", accel[0], accel[1], accel[2]);

    val = 0x43;
    write_result = i2c_write_blocking(I2C_PORT, ENDERECO_MPU6050, &val, 1, true);
    printf("[DEBUG] mpu6050_ler_dados: I2C write (gyro, 0x43): resultado=%d\n", write_result);
    sleep_us(100);
    read_result = i2c_read_blocking(I2C_PORT, ENDERECO_MPU6050, buffer, 6, false);
    printf("[DEBUG] mpu6050_ler_dados: I2C read (gyro): resultado=%d\n", read_result);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    printf("[DEBUG] mpu6050_ler_dados: Giroscópio: X=%d, Y=%d, Z=%d\n", gyro[0], gyro[1], gyro[2]);

    val = 0x41;
    write_result = i2c_write_blocking(I2C_PORT, ENDERECO_MPU6050, &val, 1, true);
    printf("[DEBUG] mpu6050_ler_dados: I2C write (temp, 0x41): resultado=%d\n", write_result);
    sleep_us(100);
    read_result = i2c_read_blocking(I2C_PORT, ENDERECO_MPU6050, buffer, 2, false);
    printf("[DEBUG] mpu6050_ler_dados: I2C read (temp): resultado=%d\n", read_result);
    *temp = (buffer[0] << 8) | buffer[1];
    printf("[DEBUG] mpu6050_ler_dados: Temperatura: %d\n", *temp);
}

static void exibir_data_hora()
{
    datetime_t t;
    if (rtc_get_datetime(&t))
    {
        char data_str[16], hora_str[16];
        snprintf(data_str, sizeof(data_str), "%02d/%02d/%02d", t.day, t.month, t.year % 100);
        snprintf(hora_str, sizeof(hora_str), "%02d:%02d:%02d", t.hour, t.min, t.sec);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, data_str, 5, 0);
        ssd1306_draw_string(&ssd, hora_str, 5, 10);

        if (!sd_esta_montado("0:"))
        {
            ssd1306_draw_string(&ssd, "MONTAR SD CARD", 0, 30);
        }
        else
        {
            ssd1306_draw_string(&ssd, "Pronto para ", 30, 30);
            ssd1306_draw_string(&ssd, "Iniciar", 30, 40);
            ssd1306_draw_string(&ssd, "Captura", 30, 50);
        }
        ssd1306_send_data(&ssd);
    }
    else
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "RTC Erro", 5, 0);
        ssd1306_send_data(&ssd);
    }
}

static void run_setrtc()
{
    printf("[DEBUG] run_setrtc: Iniciando...\n");
    const char *diaStr = strtok(NULL, " ");
    if (!diaStr)
    {
        printf("Falta argumento\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Argumento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    int dia = atoi(diaStr);
    const char *mesStr = strtok(NULL, " ");
    if (!mesStr)
    {
        printf("Falta argumento\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Argumento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    int mes = atoi(mesStr);
    const char *anoStr = strtok(NULL, " ");
    if (!anoStr)
    {
        printf("Falta argumento\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Argumento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    int ano = atoi(anoStr) + 2000;
    const char *horaStr = strtok(NULL, " ");
    if (!horaStr)
    {
        printf("Falta argumento\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Argumento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    int hora = atoi(horaStr);
    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Falta argumento\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Argumento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    int min = atoi(minStr);
    const char *segStr = strtok(NULL, " ");
    if (!segStr)
    {
        printf("Falta argumento\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Argumento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    int seg = atoi(segStr);
    datetime_t t = {
        .year = (int16_t)ano,
        .month = (int8_t)mes,
        .day = (int8_t)dia,
        .dotw = 0,
        .hour = (int8_t)hora,
        .min = (int8_t)min,
        .sec = (int8_t)seg};
    if (rtc_set_datetime(&t))
    {
        printf("[DEBUG] run_setrtc: RTC configurado com sucesso\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "RTC Configurado", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    }
    else
    {
        printf("[ERRO] run_setrtc: Falha ao configurar RTC\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro RTC", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    }
}

static void run_format()
{
    printf("[DEBUG] run_format: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Formatando SD", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_obter_fs_por_nome(arg1);
    if (!p_fs)
    {
        printf("Número de drive desconhecido: \"%s\"\n", arg1);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Drive", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
    {
        printf("Erro f_mkfs: %s (%d)\n", FRESULT_str(fr), fr);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Formatação", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    printf("[DEBUG] run_format: Formatação concluída\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "SD Formatado", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

///--------------------------------------------------------------------------------------------------------------------------------------------------------

static int sd_cartao_conectado(const char *nome)
{
    // Valida o formato do nome do drive (ex.: "0:")
    if (!nome || nome[0] < '0' || nome[0] > '9' || nome[1] != ':')
    {
        printf("[ERRO] sd_cartao_conectado: Formato de nome inválido: %s\n", nome ? nome : "NULL");
        return 0;
    }

    // Obtém a estrutura do cartão SD
    sd_card_t *pSD = sd_obter_por_nome(nome);
    if (!pSD)
    {
        printf("[ERRO] sd_cartao_conectado: Drive %s não encontrado\n", nome);
        return 0;
    }

    // Extrai o índice do drive
    int drive_num = nome[0] - '0'; // Converte '0' para 0, '1' para 1, etc.

    // Força inicialização para atualizar o status
    if (disk_initialize(drive_num) != 0)
    {
        printf("[ERRO] sd_cartao_conectado: Falha ao inicializar drive %d\n", drive_num);
        return 0;
    }

    // Verifica se o cartão está ausente
    if (disk_status(drive_num) & STA_NODISK)
    {
        printf("[DEBUG] sd_cartao_conectado: Cartão SD não detectado (STA_NODISK)\n");
        return 0;
    }

    printf("[DEBUG] sd_cartao_conectado: Cartão SD detectado no drive %d\n", drive_num);
    return 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
// Declaração global da flag erro_montagem
static bool erro_montagem = false;

static void run_mount()
{
    printf("[DEBUG] run_mount: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Montando SD", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);

    // Verifica se o cartão SD está conectado
    if (!sd_cartao_conectado("0:"))
    {
        printf("[ERRO] Cartão SD não detectado\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "SD Não Detectado", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        erro_montagem = true; // Define flag de erro
        return;
    }

    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_obter_fs_por_nome(arg1);
    if (!p_fs)
    {
        printf("Número de drive desconhecido: \"%s\"\n", arg1);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Drive", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        erro_montagem = true; // Define flag de erro
        return;
    }

    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        printf("Erro f_mount: %s (%d)\n", FRESULT_str(fr), fr);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Montagem", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        erro_montagem = true; // Define flag de erro
        return;
    }

    sd_card_t *pSD = sd_obter_por_nome(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
    printf("[DEBUG] run_mount: Montagem concluída\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "SD Montado", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    erro_montagem = false; // Montagem bem-sucedida, zera flag de erro
}

static void run_unmount()
{
    printf("[DEBUG] run_unmount: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Desmontando SD", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_obter_fs_por_nome(arg1);
    if (!p_fs)
    {
        printf("Número de drive desconhecido: \"%s\"\n", arg1);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Drive", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr)
    {
        printf("Erro f_unmount: %s (%d)\n", FRESULT_str(fr), fr);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Desmontagem", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    sd_card_t *pSD = sd_obter_por_nome(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT;
    printf("SD ( %s ) desmontado\n", pSD->pcName);
    printf("[DEBUG] run_unmount: Desmontagem concluída\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "SD Desmontado", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

static void run_getfree()
{
    printf("[DEBUG] run_getfree: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Verificando Espaço", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_obter_fs_por_nome(arg1);
    if (!p_fs)
    {
        printf("Número de drive desconhecido: \"%s\"\n", arg1);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Drive", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr)
    {
        printf("Erro f_getfree: %s (%d)\n", FRESULT_str(fr), fr);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Espaço", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB de espaço total.\n%10lu KiB disponíveis.\n", tot_sect / 2, fre_sect / 2);
    printf("[DEBUG] run_getfree: Espaço livre obtido\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Espaço Obtido", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

static void run_ls()
{
    printf("[DEBUG] run_ls: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Listando Arquivos", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0])
    {
        p_dir = arg1;
    }
    else
    {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr)
        {
            printf("Erro f_getcwd: %s (%d)\n", FRESULT_str(fr), fr);
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Erro Listagem", 5, 0);
            ssd1306_send_data(&ssd);
            mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
            return;
        }
        p_dir = cwdbuf;
    }
    printf("Listagem de diretórios: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr)
    {
        printf("Erro f_findfirst: %s (%d)\n", FRESULT_str(fr), fr);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Listagem", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    while (fr == FR_OK && fno.fname[0])
    {
        const char *pcArquivoGravavel = "arquivo gravável",
                   *pcArquivoSomenteLeitura = "arquivo somente leitura",
                   *pcDiretorio = "diretório";
        const char *pcAtributo;
        if (fno.fattrib & AM_DIR)
            pcAtributo = pcDiretorio;
        else if (fno.fattrib & AM_RDO)
            pcAtributo = pcArquivoSomenteLeitura;
        else
            pcAtributo = pcArquivoGravavel;
        printf("%s [%s] [tamanho=%llu]\n", fno.fname, pcAtributo, fno.fsize);
        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
    printf("[DEBUG] run_ls: Listagem concluída\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Listagem Concluída", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

static void run_cat()
{
    printf("[DEBUG] run_cat: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Lendo Arquivo", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    char *arg1 = strtok(NULL, " ");
    if (!arg1)
    {
        printf("Falta argumento\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: Argumento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr)
    {
        printf("Erro f_open: %s (%d)\n", FRESULT_str(fr), fr);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Leitura", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil))
    {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr)
    {
        printf("Erro f_close: %s (%d)\n", FRESULT_str(fr), fr);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Fechamento", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    printf("[DEBUG] run_cat: Leitura concluída\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Leitura Concluída", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

static void run_iniciar()
{
    printf("[DEBUG] run_iniciar: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Iniciando Captura", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    if (logger_ativado)
    {
        printf("Captura já está em andamento.\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Captura Ativa", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    if (!sd_esta_montado("0:"))
    {
        printf("[ERRO] Cartão SD não está montado. Use o comando 'a' para montar.\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: SD Não Montado", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    if (!mpu6050_testar())
    {
        printf("[ERRO] Falha na comunicação com o MPU6050. Verifique as conexões I2C.\n");
        return;
    }
    // Gerar nome do arquivo com base na data e hora atuais
    datetime_t t;
    if (rtc_get_datetime(&t))
    {
        snprintf(nome_arquivo, sizeof(nome_arquivo), "dados%02d%02d%04d%02d%02d%02d.csv",
                 t.day, t.month, t.year, t.hour, t.min, t.sec);
        printf("[DEBUG] run_iniciar: Nome do arquivo gerado: %s\n", nome_arquivo);
    }
    else
    {
        strcpy(nome_arquivo, "dados_fallback.csv");
        printf("[ERRO] run_iniciar: RTC não configurado, usando nome de arquivo padrão: %s\n", nome_arquivo);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro RTC", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    }
    logger_ativado = true;
    contador_amostras = 0;
    proxima_captura = get_absolute_time();
    printf("[DEBUG] run_iniciar: Abrindo arquivo %s para escrita...\n", nome_arquivo);
    FIL arquivo;
    FRESULT res = f_open(&arquivo, nome_arquivo, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo %s para escrita: %s (%d)\n", nome_arquivo, FRESULT_str(res), res);
        logger_ativado = false;
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Arquivo", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    const char *cabecalho = "Data,Hora,Amostra,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Temperatura\n";
    UINT bw;
    printf("[DEBUG] run_iniciar: Escrevendo cabeçalho...\n");
    res = f_write(&arquivo, cabecalho, strlen(cabecalho), &bw);
    if (res != FR_OK || bw != strlen(cabecalho))
    {
        printf("[ERRO] Não foi possível escrever o cabeçalho no arquivo %s: %s (%d), bytes escritos=%u\n", nome_arquivo, FRESULT_str(res), res, bw);
        logger_ativado = false;
        f_close(&arquivo);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Escrita", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    f_sync(&arquivo);
    f_close(&arquivo);
    printf("Captura de dados iniciada. Serão coletadas %d amostras em %s.\n", MAX_AMOSTRAS, nome_arquivo);
    printf("[DEBUG] run_iniciar: Iniciado com sucesso\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Captura Iniciada", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

static void capture_adc_data_and_save()
{
    printf("[DEBUG] capture_adc_data_and_save: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Capturando ADC", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    if (!sd_esta_montado("0:"))
    {
        printf("[ERRO] Cartão SD não está montado. Use o comando 'a' para montar.\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: SD Não Montado", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    printf("\nCapturando dados do ADC. Aguarde finalização...\n");
    FIL file;
    FRESULT res = f_open(&file, "txt.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo txt.txt para escrita: %s (%d)\n", FRESULT_str(res), res);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Arquivo", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    for (int i = 0; i < 128; i++)
    {
        adc_select_input(0);
        uint16_t adc_value = adc_read();
        char buffer[50];
        sprintf(buffer, "%d %d\n", i + 1, adc_value);
        UINT bw;
        res = f_write(&file, buffer, strlen(buffer), &bw);
        if (res != FR_OK)
        {
            printf("[ERRO] Não foi possível escrever no arquivo txt.txt: %s (%d)\n", FRESULT_str(res), res);
            f_close(&file);
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Erro Escrita", 5, 0);
            ssd1306_send_data(&ssd);
            mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
            return;
        }
        sleep_ms(100);
    }
    f_sync(&file);
    f_close(&file);
    printf("\nDados do ADC salvos no arquivo txt.txt.\n\n");
    printf("[DEBUG] capture_adc_data_and_save: Concluído\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "ADC Salvo", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

static void capturar_dados_mpu6050_e_salvar()
{
    printf("[DEBUG] capturar_dados_mpu6050_e_salvar: Iniciando amostra %d\n", contador_amostras + 1);
    ssd1306_fill(&ssd, false);
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Amostra %d/%d", contador_amostras + 1, MAX_AMOSTRAS);
    ssd1306_draw_string(&ssd, buffer, 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    if (!sd_esta_montado("0:"))
    {
        printf("[ERRO] Cartão SD não está montado. Parando captura.\n");
        logger_ativado = false;
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: SD Não Montado", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    if (!mpu6050_testar())
    {
        printf("[ERRO] Falha na comunicação com o MPU6050. Parando captura.\n");
        logger_ativado = false;
        return;
    }
    int16_t accel[3], gyro[3], temp;
    mpu6050_ler_dados(accel, gyro, &temp);
    contador_amostras++;
    float temperatura = (temp / 340.0) + 15; // Temperatura em Celsius
    printf("[DEBUG] capturar_dados_mpu6050_e_salvar: Amostra %d lida: AccX=%d, AccY=%d, AccZ=%d, GyroX=%d, GyroY=%d, GyroZ=%d, Temp=%d, Temperatura=%.2f C\n",
           contador_amostras, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp, temperatura);

    datetime_t t;
    char data_str[16], hora_str[16];
    if (rtc_get_datetime(&t))
    {
        snprintf(data_str, sizeof(data_str), "%02d/%02d/%02d", t.day, t.month, t.year % 100);
        snprintf(hora_str, sizeof(hora_str), "%02d:%02d:%02d", t.hour, t.min, t.sec);
    }
    else
    {
        strcpy(data_str, "00/00/00");
        strcpy(hora_str, "00:00:00");
        printf("[ERRO] capturar_dados_mpu6050_e_salvar: RTC não configurado, usando 00/00/00 00:00:00\n");
    }

    printf("[DEBUG] capturar_dados_mpu6050_e_salvar: Abrindo arquivo %s para escrita (append)...\n", nome_arquivo);
    FIL arquivo;
    FRESULT res = f_open(&arquivo, nome_arquivo, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo %s para escrita: %s (%d)\n", nome_arquivo, FRESULT_str(res), res);
        logger_ativado = false;
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Arquivo", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }

    char buffer_data[128];
    sprintf(buffer_data, "%s,%s,%d,%d,%d,%d,%d,%d,%d,%.2f\n",
            data_str, hora_str, contador_amostras, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temperatura);
    printf("[DEBUG] capturar_dados_mpu6050_e_salvar: Buffer preparado: %s", buffer_data);
    UINT bw;
    printf("[DEBUG] capturar_dados_mpu6050_e_salvar: Escrevendo amostra %d...\n", contador_amostras);
    res = f_write(&arquivo, buffer_data, strlen(buffer_data), &bw);
    if (res != FR_OK || bw != strlen(buffer_data))
    {
        printf("[ERRO] Não foi possível escrever no arquivo %s: %s (%d), bytes escritos=%u\n", nome_arquivo, FRESULT_str(res), res, bw);
        logger_ativado = false;
        f_close(&arquivo);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Escrita", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    f_sync(&arquivo);
    f_close(&arquivo);
    printf("Amostra %d salva em %s.\n", contador_amostras, nome_arquivo);

    if (contador_amostras >= MAX_AMOSTRAS)
    {
        logger_ativado = false;
        printf("Coleta de %d amostras concluída com sucesso.\n", MAX_AMOSTRAS);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Captura Concluída", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);

        /* code */
    }
    printf("[DEBUG] capturar_dados_mpu6050_e_salvar: Amostra %d concluída\n", contador_amostras);
}

static void ler_arquivo(const char *nome_arquivo)
{
    printf("[DEBUG] ler_arquivo: Iniciando leitura de %s\n", nome_arquivo);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Lendo Arquivo", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    if (!sd_esta_montado("0:"))
    {
        printf("[ERRO] Cartão SD não está montado. Use o comando 'a' para montar.\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro: SD Não Montado", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    FIL arquivo;
    FRESULT res = f_open(&arquivo, nome_arquivo, FA_READ);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo %s para leitura: %s (%d)\n", nome_arquivo, FRESULT_str(res), res);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Erro Leitura", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        return;
    }
    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", nome_arquivo);
    while (f_read(&arquivo, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0)
    {
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&arquivo);
    printf("\nLeitura do arquivo %s concluída.\n\n");
    printf("[DEBUG] ler_arquivo: Leitura concluída\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Leitura Concluída", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
}

static void gpio_irq_handler(uint gpio, uint32_t events)
{
    static uint32_t last_press = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_press < 200)
        return;
    last_press = current_time;

    if (gpio == BOTAO_A)
    { //
      // aqui altera o valor da flag de montar/desmontar o cartão SD
        printf("[DEBUG] gpio_irq_handler: Interrupção BOTAO_A acionada\n");
        flag_montar = !flag_montar;
        flag_desmontar = true;
    }
    else if (gpio == BOTAO_B)
    { //
      // aqui altera o valor da flag para gravar e parar de gravar
        printf("[DEBUG] gpio_irq_handler: Interrupção BOTAO_B acionada\n");
        if (sd_esta_montado("0:"))
        {
            flag_gravar = !flag_gravar;
            flag_parar_gravar = true;
        }
    }
    else if (gpio == JOYSTICK_SW)
    {
        printf("[DEBUG] gpio_irq_handler: Interrupção BOOTSEL acionada\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Modo Gravacao", 5, 0);
        ssd1306_send_data(&ssd);
        mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
        reset_usb_boot(0, 0);
    }
}

static void run_ajuda()
{
    printf("[DEBUG] run_ajuda: Iniciando...\n");
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Exibindo Ajuda", 5, 0);
    ssd1306_send_data(&ssd);
    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
    printf("\nComandos disponíveis:\n\n");
    printf("Digite 'a' para montar o cartão SD\n");
    printf("Digite 'b' para desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Digite 'f' para capturar dados do ADC e salvar no arquivo\n");
    printf("Digite 'g' para formatar o cartão SD\n");
    printf("Digite 'h' para exibir os comandos disponíveis\n");
    printf("Digite 'i' para começar a captura de %d amostras do MPU6050\n", MAX_AMOSTRAS);
    printf("\nEscolha o comando:  ");
    printf("[DEBUG] run_ajuda: Concluído\n");
}

typedef void (*p_fn_t)();
typedef struct
{
    char const *const comando;
    p_fn_t const funcao;
    char const *const ajuda;
} cmd_def_t;

static cmd_def_t comandos[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <AA> <hh> <mm> <ss>: Configura o relógio em tempo real"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Exibe espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <nome_arquivo>: Exibe conteúdo do arquivo"},
    {"i", run_iniciar, "i: Começa a captura de 25 amostras do MPU6050"},
    {"ajuda", run_ajuda, "ajuda: Exibe comandos disponíveis"}};

static void processar_stdio(int cRxedChar)
{
    printf("[DEBUG] processar_stdio: Caractere recebido=%c (0x%02X)\n", isprint(cRxedChar) ? cRxedChar : '.', cRxedChar);
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar);
    stdio_flush();
    if (cRxedChar == '\r')
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            printf("[DEBUG] processar_stdio: Comando processado: %s\n", cmdn);
            size_t i;
            for (i = 0; i < count_of(comandos); ++i)
            {
                if (0 == strcmp(comandos[i].comando, cmdn))
                {
                    (*comandos[i].funcao)();
                    break;
                }
            }
            if (count_of(comandos) == i)
            {
                printf("Comando \"%s\" não encontrado\n", cmdn);
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "Comando Inválido", 5, 0);
                ssd1306_send_data(&ssd);
                mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);
            }
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}

void init_buzzer()
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint clk_div = clock_get_hz(clk_sys) / (1000 * 4096); // Frequência ~1000Hz
    pwm_set_clkdiv(slice_num, clk_div);
    pwm_set_wrap(slice_num, 4095); // Resolução PWM
    pwm_set_enabled(slice_num, true);
    printf("Buzzer inicializado\n");
}

void set_buzzer(bool ligado)
{
    pwm_set_gpio_level(BUZZER_PIN, ligado ? 256 : 0);
}

int main()
{
    printf("[DEBUG] main: Iniciando programa...\n");
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(JOYSTICK_SW);
    gpio_set_dir(JOYSTICK_SW, GPIO_IN);
    gpio_pull_up(JOYSTICK_SW);
    gpio_set_irq_enabled_with_callback(JOYSTICK_SW, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();
    sleep_ms(5000);
    time_init();
    adc_init();

    // Configurar uma data/hora inicial padrão ajustada
    datetime_t t_inicial = {
        .year = 2025,
        .month = 7,
        .day = 29,
        .dotw = 2, // Terça-feira
        .hour = 13,
        .min = 0,
        .sec = 0};
    if (!rtc_set_datetime(&t_inicial))
    {
        printf("[ERRO] main: Falha ao configurar RTC inicial\n");
    }
    else
    {
        printf("[DEBUG] main: RTC configurado com data inicial 29/07/2025 13:00:00\n");
    }

    // inicializar buzzer
    init_buzzer();

    // Inicializar LEDs RGB
    gpio_init(LED_R);
    gpio_set_dir(LED_R, GPIO_OUT);
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_init(LED_B);
    gpio_set_dir(LED_B, GPIO_OUT);
    printf("LED RGB inicializado\n");

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    mpu6050_reset();

    sleep_ms(5000);
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, DISP_W, DISP_H, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Sistema inicializando / Montando cartão SD.
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Sistema ", 30, 0);
    ssd1306_draw_string(&ssd, "Iniciando", 30, 10);
    ssd1306_send_data(&ssd);
    printf("Sistema iniciando...\n");

    // ligar led rgb na cor amarela
    gpio_put(LED_R, 1);
    gpio_put(LED_G, 1);
    gpio_put(LED_B, 0);
    sleep_ms(5000);
    printf("\nMontando o SD...\n");
    run_mount();
    sleep_ms(5000);

    mensagem_timeout = delayed_by_ms(get_absolute_time(), MENSAGEM_TIMEOUT_MS);

    printf("Registrador de Dados FatFS SPI + MPU6050\n");
    printf("\033[2J\033[H");
    printf("\n> ");
    stdio_flush();
    run_ajuda();

    while (true)
    {

        if (erro_montagem)
        {
            // pisca led na cor roxXA

            gpio_put(LED_R, 1);
            gpio_put(LED_G, 0);
            gpio_put(LED_B, 1);
            sleep_ms(1000);
            gpio_put(LED_R, 0);
            gpio_put(LED_G, 0);
            gpio_put(LED_B, 0);
            sleep_ms(1000);
        }

        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
        {
            printf("[DEBUG] main: Caractere recebido=%c (0x%02X)\n", isprint(cRxedChar) ? cRxedChar : '.', cRxedChar);
            processar_stdio(cRxedChar);
        }

        if (cRxedChar == 'a')
        {
            flag_montar = !flag_montar;
            flag_desmontar = true;
            printf("\nflag_montar=%d, flag_desmontar=%d\n", flag_montar, flag_desmontar);
            printf("\nMontando o SD...\n");
            run_mount();
            printf("\nEscolha o comando (h = ajuda):  ");
            for (int i = 0; i < 2; i++)
            {
                gpio_put(LED_R, 1);
                gpio_put(LED_G, 1);
                gpio_put(LED_B, 0);

                sleep_ms(200);

                sleep_ms(200);
            }
            gpio_put(LED_R, 0);
            gpio_put(LED_G, 0);
            gpio_put(LED_B, 0);
            flag_desmontar = false;
        }
        else if (cRxedChar == 'b')
        {
            printf("\nflag_montar=%d, flag_desmontar=%d\n", flag_montar, flag_desmontar);
            printf("\nDesmontando o SD...\n");
            run_unmount();
            printf("\nEscolha o comando (h = ajuda):  ");
            for (int i = 0; i < 4; i++)
            {
                gpio_put(LED_R, 1);
                gpio_put(LED_G, 1);
                gpio_put(LED_B, 0);

                sleep_ms(500);

                sleep_ms(500);
            }
            gpio_put(LED_R, 0);
            gpio_put(LED_G, 0);
            gpio_put(LED_B, 0);

            flag_desmontar = false;
        }
        else if (cRxedChar == 'c')
        {
            printf("\nListagem de arquivos no cartão SD.\n");
            run_ls();
            printf("\nEscolha o comando (h = ajuda):  ");
        }
        else if (cRxedChar == 'd')
        {
            ler_arquivo(nome_arquivo);
            printf("Escolha o comando (h = ajuda):  ");
        }
        else if (cRxedChar == 'e')
        {
            printf("\nObtendo espaço livre no SD.\n\n");
            run_getfree();
            printf("\nEscolha o comando (h = ajuda):  ");
        }
        else if (cRxedChar == 'g')
        {
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            run_format();
            printf("\nEscolha o comando (h = ajuda):  ");
        }
        else if (cRxedChar == 'h')
        {
            run_ajuda();
        }
        else if (cRxedChar == 'i')
        {
            if (!sd_esta_montado("0:"))
            {
                run_iniciar();
            }
            else
            {
                flag_gravar = !flag_gravar;
                flag_parar_gravar = true;
                printf("\nIniciando captura de dados do MPU6050...\n");
                for (int i = 0; i < 10; i++)
                {
                    gpio_put(LED_R, 0);
                    gpio_put(LED_G, 0);
                    gpio_put(LED_B, 1);
                    sleep_ms(100);
                    gpio_put(LED_B, 0);
                    sleep_ms(100);
                }
                set_buzzer(1);
                sleep_ms(200);
                set_buzzer(0);

                run_iniciar();

                gpio_put(LED_R, 1);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 0);
                flag_parar_gravar = false;
            }
        }

        if (logger_ativado)
        {
            int64_t diff = absolute_time_diff_us(get_absolute_time(), proxima_captura);
            printf("[DEBUG] main: Verificando tempo: diff=%lld us\n", diff);
            if (diff <= 0)
            {
                printf("[DEBUG] main: Tempo atingido, chamando capturar_dados_mpu6050_e_salvar...\n");
                capturar_dados_mpu6050_e_salvar();
                proxima_captura = delayed_by_ms(get_absolute_time(), PERIODO_MS);
                printf("[DEBUG] main: proxima_captura atualizada\n");
            }
        }

        // Atualiza o display com a data e hora se o timeout da mensagem expirou
        int64_t diff_display = absolute_time_diff_us(get_absolute_time(), ultima_atualizacao_display);
        if (diff_display <= 0 && absolute_time_diff_us(get_absolute_time(), mensagem_timeout) <= 0)
        {
            exibir_data_hora();
            ultima_atualizacao_display = delayed_by_ms(get_absolute_time(), DISPLAY_UPDATE_MS);
        }

        // Sistema pronto para iniciar a captura.

        // comandos feitos pela placa botoes a b e botao do joystick
        // se o botão A for pressionado, monta o SD

        if (flag_montar && flag_desmontar)
        {
            printf("\nflag_montar=%d, flag_desmontar=%d\n", flag_montar, flag_desmontar);
            printf("\nMontando o SD...\n");
            run_mount();
            printf("\nEscolha o comando (h = ajuda):  ");
            for (int i = 0; i < 2; i++)
            {
                gpio_put(LED_R, 1);
                gpio_put(LED_G, 1);
                gpio_put(LED_B, 0);
                sleep_ms(200);
            }
            gpio_put(LED_R, 0);
            gpio_put(LED_G, 0);
            gpio_put(LED_B, 0);
            flag_desmontar = false;
        }

        // desmontar o sd
        else if (!flag_montar && flag_desmontar)
        {
            printf("\nflag_montar=%d, flag_desmontar=%d\n", flag_montar, flag_desmontar);
            printf("\nDesmontando o SD...\n");
            run_unmount();
            printf("\nEscolha o comando (h = ajuda):  ");
            for (int i = 0; i < 4; i++)
            {
                gpio_put(LED_R, 1);
                gpio_put(LED_G, 1);
                gpio_put(LED_B, 0);
                sleep_ms(500);
                sleep_ms(500);
            }
            gpio_put(LED_R, 0);
            gpio_put(LED_G, 0);
            gpio_put(LED_B, 0);

            flag_desmontar = false;
        }

        if (flag_gravar && flag_parar_gravar && !sd_esta_montado("0:"))
        {
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Erro ", 30, 0);
            ssd1306_draw_string(&ssd, "SDCARD", 30, 10);
            ssd1306_draw_string(&ssd, "NOT MOUNT   ", 30, 20);
            ssd1306_send_data(&ssd);
            sleep_ms(2000);
            flag_parar_gravar = false;
            // flag_gravar = 0;
        }
        else
        {
            if (flag_gravar && flag_parar_gravar && sd_esta_montado("0:"))
            {
                printf("\nIniciando captura de dados do MPU6050...\n");
                for (int i = 0; i < 10; i++)
                {
                    gpio_put(LED_R, 0);
                    gpio_put(LED_G, 0);
                    gpio_put(LED_B, 1);
                    sleep_ms(100);
                    gpio_put(LED_B, 0);
                    sleep_ms(100);
                }

                set_buzzer(1);
                sleep_ms(200);
                set_buzzer(0);
                run_iniciar();

                gpio_put(LED_R, 1);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 0);
                flag_parar_gravar = false;
            }
            else
            {
                // flag_gravar = 0;
            }

            if (!flag_gravar && flag_parar_gravar && sd_esta_montado("0:"))
            {
                flag_parar_gravar = false;
                contador_amostras = 10000000;
                gpio_put(LED_R, 0);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 0);
                set_buzzer(1);
                sleep_ms(200);
                set_buzzer(0);
                sleep_ms(200);
                set_buzzer(1);
                sleep_ms(200);
                set_buzzer(0);
            }
            else
            {
            }
        }

        if (sd_esta_montado("0:") && flag_gravar == 0)
        {
            gpio_put(LED_R, 0);
            gpio_put(LED_G, 1);
            gpio_put(LED_B, 0);
        }

        sleep_ms(50);
    }
    return 0;
}