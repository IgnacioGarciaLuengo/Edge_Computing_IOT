
#include <M5StickCPlus.h>
#include <driver/i2s.h>
#include "fft.h"
#include "red_neuronal.h"


#define PIN_MIC_CLK                   0
#define PIN_MIC_DATA                  34

#define BOTON_ENCENDIDO_PULSACION_CORTA 2

#define FREQ_MUESTREO_HZ              16000
#define VENTANA_MUESTRAS_FFT          256
#define MITAD_VENTANA_MUESTRAS_FFT    (VENTANA_MUESTRAS_FFT/2)
#define LONGITUD_BUFFER_I2S           (MITAD_VENTANA_MUESTRAS_FFT*sizeof(int32_t))
#define N_MUESTRAS_CALIBRACION        (3*FREQ_MUESTREO_HZ)


char buffer_i2s[LONGITUD_BUFFER_I2S];
int16_t bufferAudioPrevio[MITAD_VENTANA_MUESTRAS_FFT]= {0};
float espectrograma[N_TRAMAS_ESPECTROGRAMA_EXTENDIDO][N_PUNTOS_FRECUENCIA_ESPECTROGRAMA]= {{0.0}};

volatile uint16_t indiceSiguienteTramaEspectrograma;
volatile bool calibrandoMicrofono;

int16_t ventanaHamming[VENTANA_MUESTRAS_FFT];


float puntuaciones[N_CLASES_RED_NEURONAL];

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
        .dma_buf_len      = MITAD_VENTANA_MUESTRAS_FFT,
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


void ObtenerEspectrograma(void *arg)
{
  fft_config_t *fft_real;
  float acumuladoMuestras= (float)0.0;
  float n_muestrasAcumulado= (float)0.0;
  int32_t mediaMicrofonoCalibracion= (float)0.0;


  Serial.printf("Función ObtenerEspectrograma: ejecutando en núcleo %d\n", xPortGetCoreID());

  fft_real= fft_init(VENTANA_MUESTRAS_FFT, FFT_REAL, FFT_FORWARD, NULL, NULL);

  for (int i = 0; i < VENTANA_MUESTRAS_FFT; i++)
  {
    ventanaHamming[i] = 0.54 - 0.46 * cos(2 * PI * i / (VENTANA_MUESTRAS_FFT - 1));
  }
  while(true)
  {
    int32_t *p_buffer_i2s= (int32_t *)buffer_i2s;
    size_t n_bytesLeidos;
    uint16_t muestrasLeidas;
    uint16_t i;

    i2s_read(I2S_NUM_0, (char *)buffer_i2s, sizeof(buffer_i2s), &n_bytesLeidos, (100/portTICK_RATE_MS));
  
    muestrasLeidas= n_bytesLeidos/sizeof(int32_t); 
    if(muestrasLeidas!=MITAD_VENTANA_MUESTRAS_FFT)
      Serial.println("Numero de muestras leidas no es el esperado.");

    if(!calibrandoMicrofono)
    {
      for(i=0;i<MITAD_VENTANA_MUESTRAS_FFT;i++)
        fft_real->input[i]= ventanaHamming[i]*(float)bufferAudioPrevio[i];

      for(i=0;i<MITAD_VENTANA_MUESTRAS_FFT;i++)
      {
        int32_t lecturaCalibrada= p_buffer_i2s[i]-mediaMicrofonoCalibracion;
        int16_t lectura_16_bits= (int16_t)(lecturaCalibrada>>13);
        
        bufferAudioPrevio[i]= lectura_16_bits;
        fft_real->input[i+MITAD_VENTANA_MUESTRAS_FFT]= (float)lectura_16_bits;
      }
      fft_execute(fft_real);

      espectrograma[indiceSiguienteTramaEspectrograma][0]= fft_real->output[0]* fft_real->output[0];
      for(i=1;i<=MITAD_VENTANA_MUESTRAS_FFT;i++)
      {
        float valor;
            
        valor= sqrt(fft_real->output[2*i]* fft_real->output[2*i]+
                    fft_real->output[2*i+1]* fft_real->output[2*i+1]);
                        
        espectrograma[indiceSiguienteTramaEspectrograma][i-1]= valor;
      }
      if(indiceSiguienteTramaEspectrograma<(N_TRAMAS_ESPECTROGRAMA_EXTENDIDO-1))
        indiceSiguienteTramaEspectrograma++;
      else
        indiceSiguienteTramaEspectrograma= 0;
    }
    else
    {
      for(i=0;i<MITAD_VENTANA_MUESTRAS_FFT;i++)
      {
        acumuladoMuestras+= (float)p_buffer_i2s[i];
        n_muestrasAcumulado++;
      }

      if(n_muestrasAcumulado>=N_MUESTRAS_CALIBRACION)
      {
        calibrandoMicrofono= false;
        mediaMicrofonoCalibracion= (int32_t)(acumuladoMuestras/n_muestrasAcumulado);
        Serial.printf("Calibración micrófono: %d\n", mediaMicrofonoCalibracion);
      }
    }
  }
  fft_destroy(fft_real);
}


void setup()
{
  uint8_t nucleoTarea;

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

  indiceSiguienteTramaEspectrograma= 0;
  calibrandoMicrofono= true;

  if(InicializarMicrofonoI2S()==ESP_OK)
  {
    M5.Lcd.drawString("Microfono OK", 80, 10);

    if(InicializarRedNeuronal())
    {
      if(xPortGetCoreID()==0)
        nucleoTarea= 1;
      else
        nucleoTarea= 0;

      xTaskCreatePinnedToCore(ObtenerEspectrograma, "ObtenerEspectrograma", 1024, NULL, 5, NULL, nucleoTarea);

      M5.Lcd.drawString("Red OK", 80, 30);
      Serial.printf("Setup: ejecutando en núcleo %d\n", xPortGetCoreID());
    }
    else
      M5.Lcd.drawString("¡Error red!", 80, 30);
  }
  else
    M5.Lcd.drawString("¡Error microfono!", 10, 10);
}

String obtenerNombreClase(int clase)
{
  switch (clase)
  {
  case 0:
    return "cero";
  case 1:
    return "cruz";
  case 2:
    return "no";
  case 3:
    return "ruido";
  case 4:
    return "si";
  default:
    return "Desconocida";
  }
}
void loop() {
    // Cargar entrada de la red neuronal con el contenido de los espectrogramas
    if (CargarEntradaRedNeuronal(indiceSiguienteTramaEspectrograma, N_TRAMAS_ESPECTROGRAMA_EXTENDIDO, espectrograma)) {
        // Verificar que el espectrograma actual sea diferente al anterior
        if (CompararEspectrogramas()) {
            // Invocar la red neuronal para obtener las probabilidades de salida por clase
            if (InvocarRedNeuronal(puntuaciones)) {
                int claseReconocida = -1;
                float maxProbabilidad = 0.0;

                Serial.println("Probabilidades de salida:");
                for (int i = 0; i < N_CLASES_RED_NEURONAL; i++) {
                    Serial.printf("Clase %d (%s): %.4f\n", i, obtenerNombreClase(i).c_str(), puntuaciones[i]);
                    if (puntuaciones[i] > maxProbabilidad) {
                        maxProbabilidad = puntuaciones[i];
                        claseReconocida = i;
                    }
                }

                if (claseReconocida != -1 && maxProbabilidad >= 0.3) {
                    String nombreClaseReconocida = obtenerNombreClase(claseReconocida);
                    M5.Lcd.fillRect(80, 50, 240, 80, TFT_BLACK);  // Borrar la región anterior                    
                    M5.Lcd.drawString(nombreClaseReconocida, 80, 50);
                }
            } else {
                Serial.println("Error al invocar la red neuronal.");
            }
        } else {
            // Imprimir mensaje de error en la pantalla y la consola
            M5.Lcd.fillRect(80, 50, 240, 80, TFT_BLACK);
            M5.Lcd.drawString("Error de espectrograma", 80, 50);
            Serial.println("Error de espectrograma: El espectrograma actual es igual al anterior.");
        }
    }

    // Actualización del dispositivo y comprobación de pulsación del botón
    M5.update();
    if (M5.Axp.GetBtnPress() == BOTON_ENCENDIDO_PULSACION_CORTA) {
        ResetearDispositivo();
    }
}

// Función para comparar el espectrograma actual con el anterior
bool CompararEspectrogramas() {
    static float espectrogramaAnterior[N_PUNTOS_FRECUENCIA_ESPECTROGRAMA];
    
    // Compara los valores de los espectrogramas
    for (int i = 0; i < N_PUNTOS_FRECUENCIA_ESPECTROGRAMA; i++) {
        if (espectrograma[indiceSiguienteTramaEspectrograma][i] != espectrogramaAnterior[i]) {
            // Copia el espectrograma actual al espectrograma anterior para la próxima comparación
            memcpy(espectrogramaAnterior, espectrograma[indiceSiguienteTramaEspectrograma], sizeof(espectrogramaAnterior));
            return true;  // Los espectrogramas son diferentes
        }
    }

    return false;  // Los espectrogramas son iguales
}









