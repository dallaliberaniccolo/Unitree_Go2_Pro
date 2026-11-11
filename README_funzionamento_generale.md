# Unitree Go2 Pro – Navigazione autonoma

Questo repository contiene il software sviluppato per la navigazione autonoma del robot quadrupede **Unitree Go2 Pro**. Il progetto è organizzato come workspace **ROS 2** e suddiviso in pacchetti indipendenti, ognuno con una responsabilità specifica: acquisizione dei sensori, navigazione, gestione della missione, comunicazione con il robot e riconoscimento automatico del lupo.

Il sistema utilizza dati GNSS e IMU per raggiungere un waypoint geografico. È inoltre presente un modulo di visione basato su YOLO, che può analizzare le immagini della camera del robot e comunicare un rilevamento al gestore della missione.

## Architettura del sistema

Il software è organizzato in pacchetti ROS 2, ciascuno con una funzione specifica:

- **go2_sensors** acquisisce i dati dei sensori esterni.
- **go2_navigation** elabora posizione e orientamento e calcola distanza e direzione rispetto al waypoint.
- **go2_mission_manager** coordina la missione, verifica che i dati necessari siano disponibili e genera i comandi di movimento.
- **go2_bridge** gestisce la comunicazione con il Go2 tramite WebRTC e verifica i permessi e la validità dei comandi ricevuti.
- **go2_animal_detector** elabora le immagini e invia al gestore della missione i risultati del riconoscimento.
- **go2_interfaces** contiene i messaggi e i servizi ROS 2 condivisi tra i pacchetti.

La navigazione implementata è diretta verso waypoint geografici: il progetto non include un pianificatore di percorsi su mappa né un sistema generale di evitamento degli ostacoli.

## Struttura del repository

```text
.
├── config/                         # Configurazioni principali e calibrazione
├── models/
│   └── wolf/                       # Modello YOLO e relativi file di configurazione
├── src/                            # Pacchetti ROS 2
│   ├── go2_animal_detector/
│   ├── go2_bridge/
│   ├── go2_bringup/
│   ├── go2_interfaces/
│   ├── go2_mission_manager/
│   ├── go2_navigation/
│   ├── go2_sensors/
│   └── go2_vision_test/
├── test/                           # Test offline e strumenti di supporto
├── analyze_video.sh                # Analisi offline di un video
├── autostart_go2.sh                # Avvio automatico dello stack sul robot
├── run_ros_video_review.sh         # Raccolta dei risultati di una prova ROS
├── run_video_test.sh               # Avvio del test ROS con un video
├── stop_go2.sh                     # Arresto dello stack di navigazione
├── Video youtube navigazione      # Link al video dimostrativo della navigazione
├── README_wolf_detector.md         # Documentazione dettagliata del detector
└── README_video_test.md            # Documentazione dei test video
```

## Pacchetti ROS 2

### `go2_sensors` – Acquisizione dei sensori

Contiene i nodi che leggono i sensori esterni e pubblicano i relativi dati sui topic ROS 2.

- `gps_node.py`: acquisisce i dati del ricevitore GNSS NEO-M8M tramite UART e pubblica posizione e qualità del fix.
- `gps_protocol.py`: interpreta le frasi NMEA e ne verifica il checksum.
- `imu_mag_node.py`: gestisce l'IMU ICM-20948 e il magnetometro AK09916, pubblicando misure inerziali e informazioni sull'orientamento.
- `icm20948_spi.py`: contiene il driver per la comunicazione SPI con il sensore.
- `gravity_filter.py`: stima la direzione della gravità a partire dalle misure inerziali.
- `horizontal_calibration.py`: gestisce la calibrazione orizzontale del magnetometro.
- `math_sensor.py`: raccoglie funzioni matematiche per trasformazione degli assi, vettori e calcolo della direzione magnetica.
- `sensor_report.py`: strumento da terminale per visualizzare le informazioni pubblicate dai sensori.
- `config/field_sensors.yaml`: parametri di configurazione dei sensori.
- `launch/sensors_only.launch.py`: avvia i nodi dei sensori per prove dedicate.
- `launch/field_readonly.launch.py`: avvia sensori, bridge, navigazione e gestore della missione in modalità di sola lettura.
- `tests/test_sensor_math.py`: test delle funzioni di parsing e dei calcoli relativi ai sensori.

### `go2_navigation` – Navigazione geografica

Elabora i dati GPS, l'orientamento e le informazioni del robot per determinare la posizione rispetto al waypoint assegnato.

- `node.py`: nodo ROS 2 principale; raccoglie i dati di posizione, orientamento e telemetria e pubblica lo stato della navigazione.
- `geodesy.py`: funzioni per il calcolo della distanza, del bearing e dell'errore angolare tra coordinate geografiche WGS84.
- `heading_fusion.py`: combina il riferimento del nord magnetico con le variazioni di yaw dell'IMU del robot, gestendo le anomalie magnetiche entro limiti configurati.
- `gps_adapter.py`: adapter alternativo per la lettura di dati GNSS NMEA tramite seriale.
- `launch/nav_mission_readonly.launch.py`: avvia navigazione e gestore della missione senza inviare comandi di movimento.
- `launch/waypoint_diretto.launch.py`: launch principale che coordina l'avvio di sensori, detector, navigazione, bridge e gestore della missione.

### `go2_mission_manager` – Gestione della missione

È il modulo che coordina il comportamento del robot durante una missione verso un waypoint.

- `node.py`: nodo ROS 2 principale; gestisce i topic e i servizi della missione, verifica le condizioni preliminari, coordina i permessi di movimento e comunica con il bridge.
- `logic.py`: contiene la macchina a stati della missione e la logica di controllo diretta verso il waypoint.
- `speed_ramp.py`: limita le variazioni della velocità lineare per ottenere accelerazioni e decelerazioni graduali; la richiesta di arresto resta immediata.
- `identification.py`: gestisce l'attivazione del riconoscimento in funzione dell'avanzamento della missione e verifica la validità dei risultati ricevuti.
- `test/`: test della logica di missione e dei controlli preliminari.

### `go2_bridge` – Comunicazione con il robot

Fa da collegamento tra i nodi ROS 2 e il robot Unitree Go2 Pro.

- `node.py`: espone il bridge come nodo ROS 2 e gestisce lo scambio di stati, immagini e comandi.
- `core.py`: implementa i controlli indipendenti dal trasporto per autorizzare o bloccare il movimento, verificando permessi, sessioni e scadenze dei messaggi.
- `worker.py`: gestisce il trasporto in un thread dedicato, separandolo dai callback ROS 2.
- `backends/webrtc.py`: implementa il backend WebRTC per la comunicazione con il Go2 e include anche un backend mock per le prove.
- `test/`: test relativi alla validità temporale dei messaggi, ai permessi di movimento e al formato dei comandi inviati.

### `go2_animal_detector` – Riconoscimento del lupo

Gestisce l'elaborazione delle immagini della camera tramite il modello YOLO fornito nel repository.

- `node.py`: nodo ROS 2 che riceve le immagini e il comando di abilitazione dal gestore della missione, esegue l'inferenza e pubblica risultati e stato del detector.
- `backend.py`: carica il modello locale ed esegue l'inferenza.
- `core.py`: contiene le funzioni indipendenti da ROS 2 per la conversione delle immagini, la gestione dell'abilitazione e la conferma temporale dei rilevamenti.
- `recording.py`: salva i fotogrammi analizzati, le immagini annotate e i risultati in formato CSV e JSONL, oltre a generare una pagina HTML riepilogativa.
- `requirements-inference.txt`: dipendenze Python necessarie per l'inferenza.

### `go2_interfaces` – Interfacce ROS 2

Definisce i tipi di messaggio e i servizi personalizzati condivisi tra i nodi.

- `msg/BridgeState.msg`: descrive lo stato del bridge e della connessione con il robot.
- `msg/MotionCommand.msg`: contiene un comando di movimento identificato da sessione e sequenza.
- `msg/MotionPermit.msg`: rappresenta il permesso temporaneo di movimento concesso dal gestore della missione.
- `srv/SetHeading.srv`: definisce la richiesta con il valore `heading_true_deg` e la risposta del servizio (`success` e `message`).

### `go2_bringup` – Avvio e configurazione del bridge

Contiene le configurazioni e il launch dedicati al bridge.

- `launch/bridge.launch.py`: avvia il nodo `go2_bridge`.
- `config/bridge.yaml`: parametri del backend WebRTC e dei limiti di comunicazione e movimento.
- `config/mock.yaml`: configurazione per utilizzare il backend simulato.

### `go2_vision_test` – Test del riconoscimento tramite video

Raccoglie gli strumenti per verificare il detector senza utilizzare la camera reale o muovere il robot.

- `video.py`: legge un video locale e restituisce i fotogrammi in base al tempo di riproduzione.
- `video_node.py`: pubblica i fotogrammi del video come immagini ROS 2.
- `inputs_node.py`: genera ingressi di navigazione simulati per il test.
- `topics.py`: definisce i topic isolati utilizzati durante le prove.
- `analyze_video.py`: esegue l'analisi offline di un video, senza avviare ROS 2.
- `review_ros.py`: avvia e controlla una prova della pipeline ROS 2 e raccoglie i risultati.
- `launch/video_test.launch.py`: avvia video, detector e gestore della missione con movimento disabilitato e senza bridge verso il robot.

## Configurazioni e modello

### Cartella `config/`

Contiene i file YAML utilizzati per configurare il sistema:

- `go2_real_readonly.yaml` e `go2_real_readonly_sta.yaml`: profili di connessione al robot in modalità di sola lettura.
- `go2_waypoint_diretto.yaml`: configurazione principale dei parametri di bridge, GPS, IMU, navigazione, mission manager e detector.
- `go2_waypoint_preparazione.yaml`: configurazione essenziale per la fase di preparazione del bridge.
- `calibration/mag_calibrazione_2d_robot.json`: dati di calibrazione del magnetometro.

Sono presenti anche configurazioni specifiche all'interno dei pacchetti, ad esempio `src/go2_sensors/config/field_sensors.yaml` e `src/go2_bringup/config/`.

### Cartella `models/wolf/`

Contiene i file relativi al modello di riconoscimento:

- `wolf_detector.pt`: pesi del modello YOLO.
- `inference_config.json`: parametri e metadati utilizzati per l'inferenza.
- `training_config.json`: informazioni sulla configurazione dell'addestramento.
- `inference_example.py`: esempio di utilizzo del modello.
- `README.md`: informazioni sul modello e sul checkpoint.

## Test e strumenti

La cartella `test/` contiene la suite di test offline e gli strumenti di supporto:

- `run_tests.py`: avvia la suite offline per la logica di missione, il detector e i test video.
- `ros_shim.py`: fornisce un sostituto minimale delle interfacce ROS 2 per alcuni test, così da poter eseguire il codice senza un sistema ROS 2 attivo.
- `check_installed_runtime.py`: esegue controlli sull'ambiente ROS 2 e sul runtime installato.
- `test_detector_core.py` e `test_detector_pipeline.py`: verificano il filtraggio e il flusso di elaborazione del detector.
- `test_identification.py` e `test_mission_detection.py`: verificano l'attivazione del riconoscimento e la gestione di un rilevamento durante la missione.
- `test_model_profile.py`: verifica la coerenza del profilo del modello.
- `test_sample_recording.py`: verifica il salvataggio dei campioni e dei report.
- `test_video_replay.py`: verifica la riproduzione dei video e la relativa integrazione ROS 2.

Sono inoltre presenti test specifici in `src/go2_bridge/test/`, `src/go2_sensors/tests/` e `src/go2_mission_manager/test/`.

Per eseguire la suite offline principale dalla cartella del workspace:

```bash
python3 test/run_tests.py
```

Questi test non sostituiscono le prove con i sensori reali e i collaudi sul robot.

## Script principali

- `autostart_go2.sh`: avvia lo stack di navigazione sul Raspberry Pi con `enable_motion:=true` e `start_detector:=false`. **Può abilitare il movimento del robot**; utilizzarlo solo dopo aver verificato configurazione e condizioni operative.
- `stop_go2.sh`: arresta lo stack di navigazione avviato dal relativo launch.
- `analyze_video.sh`: avvia l'analisi offline di un video tramite il modello YOLO.
- `run_video_test.sh`: avvia il test ROS 2 con un video locale, senza bridge verso il robot e con movimento disabilitato.
- `run_ros_video_review.sh`: avvia lo strumento di revisione delle prove ROS 2 e raccolta dei report.

Per maggiori dettagli sul riconoscimento del lupo e sulle prove video sono disponibili `README_wolf_detector.md` e `README_video_test.md`. Il file `Video youtube navigazione` contiene inoltre il collegamento a un video dimostrativo.

## Avvio e gestione di una missione

La navigazione viene avviata automaticamente al riavvio del Raspberry Pi tramite il servizio `go2-navigation.service`. Il sistema rimane in attesa di ricevere un waypoint GPS.

> **Attenzione:** dopo il riavvio, lasciare il robot fermo per i primi secondi, così da consentire la calibrazione del giroscopio. Prima di inviare una destinazione, verificare che il robot sia in condizioni di sicurezza e che i sensori siano pronti.

### 1. Collegamento al Raspberry Pi

Dal PC, collegarsi al Raspberry Pi tramite SSH:

```bash
ssh romolo@Romolo.local
```

### 2. Verifica del servizio di navigazione

Controllare che il servizio sia attivo:

```bash
sudo systemctl status go2-navigation.service --no-pager
```

Se lo stato non è `active (running)`, avviarlo manualmente:

```bash
sudo systemctl start go2-navigation.service
```

### 3. Accesso al container e caricamento dell'ambiente ROS 2

Entrare nel container:

```bash
docker exec -it go2_unico bash
```

All'interno del container, attivare l'ambiente Python e caricare ROS 2 e il workspace:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
```

### 4. Controllo dello stato prima della missione

Verificare lo stato della missione:

```bash
ros2 topic echo /mission/status --once --field data
```

Prima di inviare una destinazione, lo stato `IDLE` è normale. Prima del test verificare che:

- `mode` sia `DIRECT_WAYPOINT_ACTIVE`;
- il GPS fornisca una posizione valida;
- siano disponibili almeno 7 satelliti;
- `heading_ready` sia `true`;
- il collegamento al robot sia pronto.

Non inviare il waypoint finché le condizioni necessarie non sono soddisfatte.

### 5. Invio della destinazione

Pubblicare il waypoint sul topic `/mission/goal`, sostituendo `LATITUDINE` e `LONGITUDINE` con i valori numerici desiderati:

```bash
ros2 topic pub --once /mission/goal sensor_msgs/msg/NavSatFix \\
"{latitude: LATITUDINE, longitude: LONGITUDINE}"
```

L'invio del waypoint autorizza automaticamente l'avvio della missione, non appena i controlli previsti dal sistema sono superati. **Non è necessario chiamare `/mission/start` per avviare una nuova missione.**

### 6. Monitoraggio e controllo della missione

Per seguire lo stato durante il test:

```bash
ros2 topic echo /mission/status --field data
```

Per interrompere la sola visualizzazione, premere `Ctrl+C`: questo comando non arresta il robot e non annulla la missione.

Per mettere in pausa il robot:

```bash
ros2 service call /mission/pause std_srvs/srv/Trigger "{}"
```

Per riprendere la destinazione corrente:

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

Per annullare la missione:

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
```

## Compilazione del workspace

In un ambiente con ROS 2 Jazzy e le dipendenze dei pacchetti installate, dalla cartella principale del workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Per avviare il launch principale con il movimento disabilitato:

```bash
ros2 launch go2_navigation waypoint_diretto.launch.py enable_motion:=false
```

Prima di utilizzare il sistema sul robot, verificare i parametri dei file YAML, la connessione, la calibrazione dei sensori e le condizioni di sicurezza. Il software non costituisce un sistema di sicurezza certificato.
