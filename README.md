# 📋 Data Logger com Raspberry Pi Pico

![C](https://img.shields.io/badge/language-C-blue.svg)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%20Pico-orange.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

Este projeto implementa um **data logger** utilizando um Raspberry Pi Pico para coletar dados do sensor MPU6050 (acelerômetro, giroscópio e temperatura) e salvá-los em um cartão SD em formato CSV. O sistema exibe informações em um display OLED SSD1306, usa LEDs RGB e um buzzer para feedback, e permite controle via botões físicos e interface serial.

## 🚀 Visão Geral

O sistema registra dados do MPU6050 em um cartão SD, com timestamps baseados no RTC do Pico. Ele suporta comandos seriais para gerenciar o cartão SD, configurar o RTC e iniciar/parar a captura de dados. Botões controlam ações como montar/desmontar o SD e iniciar/parar a captura, com feedback visual (LEDs RGB, OLED) e sonoro (buzzer).

### Funcionalidades
- 📈 Coleta até 99.999 amostras do MPU6050 a cada 1 segundo.
- 💾 Salva dados em arquivos CSV (ex.: `dadosDDMMAAAAHHMMSS.csv`).
- 🖥️ Exibe data, hora e status no display OLED SSD1306.
- 🎛️ Controle via botões (montar/desmontar SD, iniciar/parar captura).
- 📡 Comandos via serial USB para configuração e gerenciamento.
- 💡 Feedback visual (LEDs RGB) e sonoro (buzzer).

## 🛠️ Hardware Necessário

- **Raspberry Pi Pico**
- **MPU6050** (I2C: SDA no GPIO 0, SCL no GPIO 1, endereço 0x68)
- **Módulo de Cartão SD** (SPI, configurado em `hw_config.h`)
- **Display OLED SSD1306** (128x64, I2C: SDA no GPIO 14, SCL no GPIO 15, endereço 0x3C)
- **LEDs RGB** (Vermelho: GPIO 13, Verde: GPIO 11, Azul: GPIO 12)
- **Buzzer** (GPIO 10, PWM a ~3500 Hz)
- **Botões**:
  - Botão A (GPIO 5): Alterna montar/desmontar SD.
  - Botão B (GPIO 6): Inicia/para captura.
  - Joystick SW (GPIO 22): Entra no modo bootloader.
- **Resistores Pull-up** (para I2C e botões, se necessário)
- **Fonte de 3.3V** para todos os componentes

## 📦 Dependências

- [Pico SDK](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html) - Desenvolvimento para o Pico.
- [FatFs](http://elm-chan.org/fsw/ff/00index_e.html) - Sistema de arquivos para cartão SD (`ff.h`, `diskio.h`, `f_util.h`).
- Biblioteca SSD1306 - Driver para o display OLED (`ssd1306.h`).
- Bibliotecas Pico: `hardware/adc.h`, `hardware/rtc.h`, `hardware/i2c.h`, `hardware/pwm.h`, `hardware/clocks.h`.

## ⚙️ Configuração

### Hardware
1. **MPU6050**: Conecte SDA (GPIO 0), SCL (GPIO 1), 3.3V e GND.
2. **Cartão SD**: Configure pinos SPI em `hw_config.h` (ex.: SS no GPIO 17, SPI0). Verifique `cd_gpio` para detecção de cartão.
3. **Display OLED**: Conecte SDA (GPIO 14), SCL (GPIO 15), 3.3V e GND.
4. **LEDs RGB**: Conecte Vermelho (GPIO 13), Verde (GPIO 11), Azul (GPIO 12) com resistores (ex.: 220Ω).
5. **Buzzer**: Conecte ao GPIO 10 (PWM).
6. **Botões**: Conecte Botão A (GPIO 5), Botão B (GPIO 6), Joystick SW (GPIO 22) com pull-up interno.

### Software
1. **Instalar Pico SDK**:
   - Siga as instruções em [Pico SDK Getting Started](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html).
   - Configure CMake e GCC.
2. **Adicionar Bibliotecas**:
   - Copie `ff.h`, `diskio.h`, `f_util.h`, `hw_config.h`, `ssd1306.h` (e suas implementações) para o projeto.
3. **Configurar `hw_config.h`**:
   ```c
   sd_card_t sd_cards[] = {
       { .pcName = "0:", .spi = spi0, .ss_gpio = 17, .cd_gpio = 16 }
   };
   ```

## 🔨 Compilação e Execução

1. **Configurar o Projeto**:
   - Crie um diretório com `main.c`, bibliotecas e um `CMakeLists.txt`:
     ```cmake
     cmake_minimum_required(VERSION 3.13)
     include(pico_sdk_import.cmake)
     project(data_logger C CXX ASM)
     pico_sdk_init()
     add_executable(data_logger main.c ssd1306.c)
     target_include_directories(data_logger PRIVATE ${CMAKE_CURRENT_LIST_DIR})
     target_link_libraries(data_logger pico_stdlib hardware_i2c hardware_adc hardware_rtc hardware_pwm)
     pico_add_extra_outputs(data_logger)
     ```
2. **Compilar**:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```
3. **Carregar no Pico**:
   - Segure o botão BOOTSEL, conecte o Pico via USB e copie o arquivo `.uf2` (ex.: `data_logger.uf2`) para o drive USB.
4. **Executar**:
   - O Pico inicializa, monta o SD e exibe o status no OLED.
   - Use um terminal serial (ex.: PuTTY, minicom, 115200 baud) para interagir.

## 🎮 Uso

### Comandos Serial
| Comando | Descrição | Exemplo |
|---------|-----------|---------|
| `a` | Monta o cartão SD | `a` |
| `b` | Desmonta o cartão SD | `b` |
| `c` | Lista arquivos no SD | `c` |
| `d <nome>` | Exibe conteúdo do arquivo | `d dados29072025130000.csv` |
| `e` | Mostra espaço livre no SD | `e` |
| `f` | Captura 128 amostras do ADC | `f` |
| `g` | Formata o cartão SD | `g` |
| `h` | Exibe ajuda | `h` |
| `i` | Inicia captura de 99.999 amostras do MPU6050 | `i` |
| `setrtc <DD> <MM> <AA> <hh> <mm> <ss>` | Configura RTC | `setrtc 29 07 25 13 00 00` |

### Controles via Botões
- **Botão A (GPIO 5)**: Alterna montar/desmontar SD.
- **Botão B (GPIO 6)**: Inicia/para captura (requer SD montado).
- **Joystick SW (GPIO 22)**: Entra no modo bootloader.

### Feedback
- **Display OLED**: Mostra data, hora, status (ex.: "SD Montado", "Captura Iniciada") e erros (ex.: "SD Não Detectado").
- **LEDs RGB**:
  - 🟡 Amarelo: Inicializando ou montando/desmontando.
  - 🟢 Verde: Sistema pronto (SD montado).
  - 🟣 Roxo piscando: Erro de montagem.
  - 🔴 Vermelho: Captura em andamento.
  - 🔵 Azul piscando: Iniciando captura.
- **Buzzer**: 1 beep (200ms) para sucesso, 2 beeps para erro/parada.

### Formato do Arquivo CSV
Dados do MPU6050 são salvos em arquivos como `dadosDDMMAAAAHHMMSS.csv`:
```csv
Data,Hora,Amostra,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Temperatura
29/07/25,13:00:00,1,123,-456,789,10,-20,30,25.50
...
```

## 🐞 Notas de Depuração

- **Logs**: Use um terminal serial para ver mensagens `[DEBUG]` e `[ERRO]`.
- **Problemas Comuns**:
  - **SD não detectado**: Verifique `hw_config.h` e pinos SPI.
  - **MPU6050 falha**: Confirme conexões I2C (GPIO 0/1, endereço 0x68).
  - **Display não funciona**: Verifique I2C do OLED (GPIO 14/15).
- **Personalização**:
  - `MENSAGEM_TIMEOUT_MS` (2000ms): Duração das mensagens no OLED.
  - `PERIODO_MS` (1000ms): Intervalo de captura.
  - `BUZZER_FREQUENCY` (3500Hz): Tom do buzzer.

## 🤝 Contribuição

Contribuições são bem-vindas! Para contribuir:
1. Faça um fork do repositório.
2. Crie uma branch (`git checkout -b feature/nova-funcionalidade`).
3. Commit suas alterações (`git commit -m 'Adiciona nova funcionalidade'`).
4. Push para a branch (`git push origin feature/nova-funcionalidade`).
5. Abra um Pull Request.

## 📜 Licença

Este projeto é licenciado sob a [Licença MIT](LICENSE). As bibliotecas Pico SDK e FatFs têm suas próprias licenças.