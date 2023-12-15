
#include <Arduino.h>
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "red_neuronal.h"
#include "recvoz_red_nn.h"


constexpr int kTensorArenaSize= 100*1024;
uint8_t *tensor_arena= nullptr;

const tflite::Model *model= nullptr;
tflite::MicroInterpreter *interpreter= nullptr;
tflite::ErrorReporter *error_reporter = nullptr;
tflite::MicroErrorReporter micro_error_reporter;
TfLiteTensor *input= nullptr;
TfLiteTensor *output= nullptr;
tflite::AllOpsResolver resolver;


bool InicializarRedNeuronal(void)
{
  bool sinError;

  sinError= false;

  tensor_arena= (uint8_t *)malloc(kTensorArenaSize);
  if(tensor_arena!=nullptr)
  {
    model= tflite::GetModel(modelo_rec_reducido_tflite);
  
    error_reporter = &micro_error_reporter;
    if(model->version()== TFLITE_SCHEMA_VERSION)
    {
      static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
      interpreter= &static_interpreter;

      TfLiteStatus allocate_status= interpreter->AllocateTensors();
      if(allocate_status== kTfLiteOk)
      {
        input= interpreter->input(0);
        output= interpreter->output(0);
        sinError= true;

        Serial.printf("[I] Tipo datos: %d<->%d\n", input->type, kTfLiteFloat32);
        Serial.printf("[I] Dimensiones: %d\n", input->dims->size);
        Serial.printf("[I] Dim_0: %d\n", input->dims->data[0]);
        Serial.printf("[I] Dim_1: %d\n", input->dims->data[1]);
        Serial.printf("[I] Dim_2: %d\n", input->dims->data[2]);
        Serial.printf("[O] Tipo datos: %d<->%d\n", output->type, kTfLiteFloat32);
        Serial.printf("[O] Dimensiones: %d\n", output->dims->size);
        Serial.printf("[O] Dim_0: %d\n", output->dims->data[0]);
        Serial.printf("[O] Dim_1: %d\n", output->dims->data[1]);
      }
      else
        Serial.println("Error creacion tensores.");
    }
    else
      Serial.println("Modelo con version no adecuada.");
  }
  else
    Serial.println("Error con reserva de memoria.");
  
  return(sinError);
}


bool CargarEntradaRedNeuronal(uint16_t indiceSiguienteTramaEspectrograma,
                              uint16_t n_tramas_expectrograma_ext,
                              float (&espectrograma)[N_TRAMAS_ESPECTROGRAMA_EXTENDIDO][N_PUNTOS_FRECUENCIA_ESPECTROGRAMA])
{
  bool esCorrecto= false;
  
  if(input->type==kTfLiteFloat32)
  {
    uint16_t n_filas= input->dims->data[1];
    uint16_t n_columnas= input->dims->data[2];

    if(n_filas==N_TRAMAS_ESPECTROGRAMA && n_columnas==N_PUNTOS_FRECUENCIA_ESPECTROGRAMA)
    {
      int16_t primeraTrama;
      uint32_t indice= 0;
      
      primeraTrama= indiceSiguienteTramaEspectrograma-N_TRAMAS_ESPECTROGRAMA;
      if(primeraTrama>=0)
      {
          for(uint16_t i=primeraTrama;i<indiceSiguienteTramaEspectrograma;i++)
            for(uint16_t j=0;j<N_PUNTOS_FRECUENCIA_ESPECTROGRAMA;j++)
              input->data.f[indice++]= espectrograma[i][j];
      }
      else
      {
        primeraTrama+= n_tramas_expectrograma_ext;

        for(uint16_t i=primeraTrama;i<n_tramas_expectrograma_ext;i++)
          for(uint16_t j=0;j<N_PUNTOS_FRECUENCIA_ESPECTROGRAMA;j++)
            input->data.f[indice++]= espectrograma[i][j];

        for(uint16_t i=0;i<indiceSiguienteTramaEspectrograma;i++)
          for(uint16_t j=0;j<N_PUNTOS_FRECUENCIA_ESPECTROGRAMA;j++)
            input->data.f[indice++]= espectrograma[i][j];
      }
      esCorrecto= true;
    }
    else
      Serial.println("Dimensiones de entradas no son las esperadas.");
  }
  else
    Serial.println("Tipos de datos no son los esperados.");

  return(esCorrecto);
}


bool InvocarRedNeuronal(float puntuaciones[N_CLASES_RED_NEURONAL])
{
  bool esCorrecto;
  
  esCorrecto= false;

  if(output->type==kTfLiteFloat32)
  {
    uint16_t n_clases= output->dims->data[1];

    if(n_clases==N_CLASES_RED_NEURONAL)
    {
      TfLiteStatus invoke_status;

      invoke_status= interpreter->Invoke();

      if(invoke_status== kTfLiteOk)
      {
        for(uint8_t i=0;i<N_CLASES_RED_NEURONAL;i++) 
          puntuaciones[i]= output->data.f[i];

        esCorrecto= true;
      }
      else
        Serial.println("Error invocacion red neuronal.");
    }
    else
      Serial.println("Dimensiones de salida no es la esperada.");
  }
  else
    Serial.println("Tipos de datos no son los esperados.");

  return(esCorrecto);
}
