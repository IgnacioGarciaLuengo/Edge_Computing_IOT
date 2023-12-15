#include "M5StickCPlus.h"
#include <driver/i2s.h>


#define PIN_MIC_CLK                   0
#define PIN_MIC_DATA                  34

#define BOTON_ENCENDIDO_PULSACION_CORTA 2

#define FREQ_MUESTREO_HZ              16000
#define MUESTRAS_BUFFER_I2S           128
#define LONGITUD_BUFFER_I2S           (MUESTRAS_BUFFER_I2S*sizeof(int32_t))
#define LONGITUD_BUFFER_AUDIO         FREQ_MUESTREO_HZ
#define N_MUESTRAS_CALIBRACION        (3*FREQ_MUESTREO_HZ)

TaskHandle_t manejadorTarea= NULL;


char buffer_i2s[LONGITUD_BUFFER_I2S];
int16_t bufferAudio[LONGITUD_BUFFER_AUDIO];

volatile uint16_t n_escrituraBufferAudio; 
uint16_t n_lecturaBufferAudio;
bool calibrandoMicrofono;
bool grabando;

float acumuladoMuestras;
float n_muestrasAcumulado;
int32_t mediaMicrofonoCalibracion;


void ResetearDispositivo(void)
{
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextFont(0);
    M5.Lcd.setTextDatum(TL_DATUM);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.drawString("Reseteando...", 20, M5.Lcd.height()/3, 2);

    M5.Beep.tone(220);
    delay(200);
    M5.Beep.tone(880);
    delay(200);
    M5.Beep.tone(440);
    delay(200);
    M5.Beep.mute();
    
    M5.Lcd.fillScreen(TFT_BLACK);
    Serial.end();
    ESP.restart();
}


esp_err_t InicializarMicrofonoI2S()
{
    esp_err_t error= ESP_OK;
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate= FREQ_MUESTREO_HZ,
        .bits_per_sample= I2S_BITS_PER_SAMPLE_24BIT,
        .channel_format= I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format= I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = 4,
        .dma_buf_len      = MUESTRAS_BUFFER_I2S,
    };

    i2s_pin_config_t pin_config;
    pin_config.mck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num    = PIN_MIC_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = PIN_MIC_DATA;

    error= i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if(error== ESP_OK)
    { 
      error= i2s_set_pin(I2S_NUM_0, &pin_config);
      if(error== ESP_OK)
        error= i2s_set_clk(I2S_NUM_0, FREQ_MUESTREO_HZ, I2S_BITS_PER_SAMPLE_24BIT, I2S_CHANNEL_MONO);
    }

    return(error);
}

void CapturaAudio(void *arg)
{
  while(true)
  {
    int32_t *p_buffer_i2s= (int32_t *)buffer_i2s;
    size_t n_bytesLeidos;
    uint16_t muestrasLeidas;
    uint16_t i;

    i2s_read(I2S_NUM_0, (char *)buffer_i2s, sizeof(buffer_i2s), &n_bytesLeidos, (100/portTICK_RATE_MS));
  
    muestrasLeidas= n_bytesLeidos/sizeof(int32_t); 
    if(grabando)
    {
      for(i=0;i<muestrasLeidas;i++)
      {
        int32_t lecturaCalibrada= p_buffer_i2s[i]-mediaMicrofonoCalibracion;
        int16_t lectura_16_bits= (int16_t)(lecturaCalibrada>>13);
        
        bufferAudio[n_escrituraBufferAudio]= lectura_16_bits;
        if(n_escrituraBufferAudio==(LONGITUD_BUFFER_AUDIO-1))
          n_escrituraBufferAudio= 0;
        else
          n_escrituraBufferAudio++;
      }
    }
    else
    {
      n_escrituraBufferAudio= 0;
      n_lecturaBufferAudio= 0;

      if(calibrandoMicrofono)
      {
        for(i=0;i<muestrasLeidas;i++)
        {
          acumuladoMuestras+= (float)p_buffer_i2s[i];
          n_muestrasAcumulado++;
        }
        if(n_muestrasAcumulado>=N_MUESTRAS_CALIBRACION)
        {
          mediaMicrofonoCalibracion= (int32_t)(acumuladoMuestras/n_muestrasAcumulado);
          calibrandoMicrofono= false;
        }
      }
    }
  }
}


void setup()
{
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.setSwapBytes(true);
  M5.Lcd.fillScreen(BLACK);
  M5.Axp.ScreenBreath(50);
  M5.Beep.mute();

  M5.Lcd.setTextFont(0);
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.drawString("Captura de audio", 10, 10);

  Serial.end();
  Serial.begin(500000);

  if(InicializarMicrofonoI2S()==ESP_OK)
    M5.Lcd.drawString("Microfono correcto.", 10, 10);
  else
    M5.Lcd.drawString("¡Error microfono!", 10, 10);
 

  xTaskCreate(CapturaAudio, "CapturaAudio", 1024, NULL, 5, &manejadorTarea);

  M5.Lcd.setTextColor(TFT_BLUE, TFT_WHITE);
  M5.Lcd.drawString("Calibrando micro...", 10, 50);

  n_escrituraBufferAudio= 0; 
  n_lecturaBufferAudio= 0;
  grabando= false;

  acumuladoMuestras= (float)0.0;
  n_muestrasAcumulado= (float)0.0;
  mediaMicrofonoCalibracion= 0;
  calibrandoMicrofono= true;
}


void loop()
{  
  if(grabando)
  {
    while(n_escrituraBufferAudio!= n_lecturaBufferAudio)
    {
      int16_t muestra= bufferAudio[n_lecturaBufferAudio];
      uint8_t byte_H= (uint8_t)(muestra>>8);
      uint8_t byte_L= (uint8_t)(muestra & 0xff);
      
      Serial.write(byte_L);
      Serial.write(byte_H);

      if(n_lecturaBufferAudio==(LONGITUD_BUFFER_AUDIO-1))
        n_lecturaBufferAudio= 0;
      else
        n_lecturaBufferAudio++;
    }

  }

  if(calibrandoMicrofono==false && grabando==false)
  {
    grabando= true;
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.drawString("Enviando audio...     ", 10, 50);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.drawString("!Hable ahora!         ", 10, 80);
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.drawString("Calib.: "+String(mediaMicrofonoCalibracion)+"    ", 10, 110);      
  }

  M5.update();
  if(M5.Axp.GetBtnPress()==BOTON_ENCENDIDO_PULSACION_CORTA)
    ResetearDispositivo();

  delay(10);
}
