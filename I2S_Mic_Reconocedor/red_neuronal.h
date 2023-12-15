#ifndef __RED_NEURONAL_H__
#define __RED_NEURONAL_H__

#define N_TRAMAS_ESPECTROGRAMA_EXTENDIDO      80

#define N_TRAMAS_ESPECTROGRAMA                62
#define N_PUNTOS_FRECUENCIA_ESPECTROGRAMA     128
#define N_CLASES_RED_NEURONAL                 6

bool InicializarRedNeuronal(void);
bool CargarEntradaRedNeuronal(uint16_t indiceSiguienteTramaEspectrograma,
                              uint16_t n_tramas_expectrograma_ext,
                              float (&espectrograma)[N_TRAMAS_ESPECTROGRAMA_EXTENDIDO][N_PUNTOS_FRECUENCIA_ESPECTROGRAMA]);
bool InvocarRedNeuronal(float puntuaciones[N_CLASES_RED_NEURONAL]);

#endif  // __RED_NEURONAL_H__
