import serial
import numpy as np
import scipy.io.wavfile


###### Configuración
N_FICHEROS_GRABACION= 100
N_FICHERO_INICIAL= 0
NOMBRE_COMANDO_VOZ= "Ruido"
MIN_UMBRAL_VOZ= 60.0    # 60.0 para voz y 0.0 para ruido de fondo
###### Configuración


def GuardarFichero(n_fichero, inicio_ext, fin_ext):
    ficheroSalida= NOMBRE_COMANDO_VOZ
    ficheroSalida+= '_'
    ficheroSalida+= str(N_FICHERO_INICIAL+n_fichero).zfill(3)
    ficheroSalida+= '.wav'
    print('Guardando fichero: ', ficheroSalida)
    scipy.io.wavfile.write(ficheroSalida,
                           FREQ_MUESTREO_HZ,
    muestrasAudio[inicio_ext:fin_ext])
    print('')
    
                                    
FREQ_MUESTREO_HZ= 16000
N_MUESTRAS_BUFFER= 10*FREQ_MUESTREO_HZ
N_MUESTRAS_FICHERO= FREQ_MUESTREO_HZ//2
N_VENTANA= 256
MEDIA_3S= 3*FREQ_MUESTREO_HZ
UMBRAL_DB= 15.0

print('Inicio...')

muestrasAudio= np.zeros(N_MUESTRAS_BUFFER, dtype=np.int16)
serialPort = serial.Serial(port="COM3", baudrate=500000,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)



n_fichero= 0
while n_fichero<N_FICHEROS_GRABACION:

    print('Procesando datos fichero nº', n_fichero, '...')

    umbralVoz= MIN_UMBRAL_VOZ
    inicioParteVoz= 0
    finParteVoz= 0

    i= 0
    while i<N_MUESTRAS_BUFFER:
        if serialPort.in_waiting> 0:
            try:
                cadenaBytes= serialPort.read(2) 
                muestrasAudio[i]= int.from_bytes(cadenaBytes, byteorder='little', signed=True)
                i+=1
            except:
                print("Error recepcion.")
            
            if i%N_VENTANA==0:
                if i<MEDIA_3S:
                    ventana= muestrasAudio[0:i].astype(np.float32)
                else:
                    ventana= muestrasAudio[i-MEDIA_3S:i].astype(np.float32)
                media= np.mean(ventana)
        
                ventana= muestrasAudio[i-N_VENTANA:i].astype(np.float32)
                energiaTrama= 10.0*np.log10(np.sum(np.square(ventana-media))+1.0)
                
                ###print(umbralVoz)
                if energiaTrama<umbralVoz:
                    if energiaTrama>MIN_UMBRAL_VOZ:
                        umbralVoz= energiaTrama
                else:
                    if energiaTrama>(umbralVoz+UMBRAL_DB):
                        ###print(energiaTrama, umbralVoz)
                        
                        if inicioParteVoz==finParteVoz:
                            inicioParteVoz= i
                        
                        if inicioParteVoz!= 0:
                            fragmento= i-inicioParteVoz 
                            if fragmento>=FREQ_MUESTREO_HZ:
                                GuardarFichero(n_fichero, inicioParteVoz, i)
                                n_fichero+= 1
                                i= N_MUESTRAS_BUFFER
                                
                    else:
                        umbralVoz= 0.95*umbralVoz+ 0.05*energiaTrama
        
                        if inicioParteVoz!= 0:
                            if finParteVoz==0:
                                finParteVoz= i                    
                            
                            fragmento= finParteVoz-inicioParteVoz 
                            if fragmento>=(3*N_VENTANA):
                                extension= (N_MUESTRAS_FICHERO-fragmento)//3
                                inicio_ext= max(0, inicioParteVoz-extension)
                                if i>=(inicio_ext+N_MUESTRAS_FICHERO):                               
                                    fin_ext= inicio_ext+N_MUESTRAS_FICHERO;

                                    GuardarFichero(n_fichero, inicio_ext, fin_ext)
                                        
                                    n_fichero+= 1
                                    i= N_MUESTRAS_BUFFER
                            else:
                                inicioParteVoz= finParteVoz= 0
        
serialPort.close()
print('Fin de programa.')
