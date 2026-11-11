# Riconoscimento lupo — integrazione locale del 19 ottobre 2026

Il pacchetto `go2_animal_detector` legge le immagini ROS 2 della camera Go2 e
usa il modello personalizzato YOLO26n. Il mission manager decide quando iniziare
l'analisi. Un rilevamento confermato arresta la missione e attiva un flag ROS.
L'esplorazione della zona non è implementata: all'arrivo senza rilevamento il
robot resta fermo e continua ad analizzare le immagini, senza timeout automatico.

Profilo aggiornato il 20 ottobre 2026: analisi e pubblicazione a **1 FPS**.
La revisione con riscaldamento è stata installata e ha superato i controlli ROS
nel run `go2_yolo_20261020_001714`. La nuova revisione conferma con due frame
positivi entro tre secondi e aggiunge il salvataggio opzionale dei campioni;
richiede un nuovo aggiornamento tramite l'installatore.
Il modello fornito è copiato in `models/wolf/wolf_detector.pt`, con hash
verificato rispetto a `inference_config.json`. Il percorso nel container è
`/ws/models/wolf/wolf_detector.pt`, modificabile nel YAML. Il benchmark del solo
modello è riportato sotto; l'arresto fisico non è stato provato.

## Attivazione durante la navigazione

Il tempo `t` è misurato in secondi da quando il manager accetta il nuovo
waypoint. Include attesa di readiness, partenza ritardata e pause. La crescita è:

```text
raggio_m = 50 + 50 * min(t, 300) / 60 + 40 * max(t - 300, 0) / 60
```

| Tempo | Raggio |
|---|---:|
| 0 min | 50 m |
| 1 min | 100 m |
| 5 min | 300 m |
| 10 min | 500 m |
| 20 min | 900 m |
| 30 min | 1.300 m |

Non c'è un limite superiore. Il raggio riguarda la distanza GPS residua dal
waypoint, non la distanza di visibilità del lupo nella camera. L'attivazione
iniziale richiede una distanza valida, recente e riferita al waypoint corrente.
Una volta entrato nel raggio, il detector resta abilitato anche se il GPS oscilla
o la navigazione entra in attesa temporanea. Può analizzare anche durante
`WAIT_LOCALIZATION`, se la distanza è già valida e la missione è stata richiesta.
All'arrivo l'analisi è abilitata comunque.

Pausa e annullamento disabilitano immediatamente l'analisi. La ripresa dello stesso
waypoint conserva il tempo e l'avvenuta entrata nel raggio, ma apre una nuova
sessione di rilevamento. Un waypoint diverso azzera tempo, risultati e flag.
Gli auto-echo e le coordinate identiche vengono ignorati, come prima.
Al riavvio del manager non si conserva la missione: si torna IDLE senza goal.

## Rilevamento e arresto

Impostazioni iniziali, tutte configurabili:

- 1 elaborazione/s al massimo, ingresso 448, 2 thread CPU per PyTorch.
- Riscaldamento su immagine sintetica prima di `model_ready=true`; lo stato
  `WARMING_MODEL` e `warmup_ms` lo rendono visibile. Il risultato sintetico viene
  scartato e nessuna immagine reale ricevuta prima dello stato pronto viene analizzata.
- Classe individuata nei metadati del modello tramite `wolf` o `lupo`, senza
  distinzione tra maiuscole e minuscole. Nessuna assunzione automatica su classe 0.
  Per un'etichetta diversa cambiare `target_class_names`, oppure impostare
  esplicitamente `target_class_id` dopo aver controllato il modello.
- Confidenza minima 0,20, IoU NMS 0,60 e massimo 300 detection, come nel profilo
  di inferenza fornito con il modello. Almeno **due immagini positive negli
  ultimi tre secondi**, anche non consecutive. Finestra temporale senza limite
  sul numero di frame; i duplicati non contano. La scadenza è calcolata al termine
  dell'inferenza. Il frame corrente può essere negativo se nella finestra ci sono
  ancora due positivi. Il manager memorizza la prima conferma.
- Una sola immagine in attesa. Le immagini nuove sostituiscono quelle non ancora
  elaborate; non si rielaborano immagini duplicate e non si accumula ritardo.
- Risultati con immagini più vecchie di 2 secondi vengono scartati, anche se
  l'inferenza ha appena finito. Le conferme si azzerano su errori o cambio sessione.

Con `true` valido il manager entra in `WOLF_DETECTED`, invia velocità zero,
revoca il permesso di movimento e richiede `/go2/stop` quando disponibile.
`/mission/wolf_detected` diventa `true` e rimane tale anche dopo una perdita del
video, un risultato negativo o una pausa. Non c'è ripartenza automatica e
`/mission/start` viene rifiutato finché il rilevamento resta memorizzato.
Per uscire serve un nuovo waypoint diverso, oppure annullare la missione;
dopo l'annullamento si può inviare di nuovo anche la stessa destinazione.
Il flag si azzera anche al riavvio del manager, insieme alla missione.

È implementato il **flag**, non il pilotaggio GPIO di un LED fisico.
Il flag indica un riconoscimento confermato e la richiesta software di arresto;
la conferma del bridge è il campo separato `bridge_stop_confirmed` nello stato
missione. La prova fisica dell'arresto resta necessaria.

## Topic

| Topic | Tipo | Proprietario e significato |
|---|---|---|
| `/go2/camera/image_raw` | `sensor_msgs/Image` | Bridge; BGR8. Il nodo supporta anche RGB8 e MONO8. |
| `/identification/control` | `std_msgs/String` JSON | Manager → detector, comando autorevole con `enabled`, `mission_id`, `session_id`, `stamp_unix`. |
| `/identification/enable` | `std_msgs/Bool` | Manager; copia diagnostica dell'abilitazione esistente. |
| `/identification/result` | `std_msgs/String` JSON | Detector → manager; `valid`, `detected`, ID missione/sessione, timestamp immagine/risultato, confidenza e conteggi. |
| `/identification/detected` | `std_msgs/Bool` | Detector; esito confermato dell'ultima analisi valida. Non è il flag memorizzato. |
| `/identification/status` | `std_msgs/String` JSON | Stato modello/video, validità, errori, FPS effettivi e tempo inferenza. |
| `/mission/wolf_detected` | `std_msgs/Bool` | Flag memorizzato dal manager; QoS reliable/transient local, disponibile ai nuovi subscriber. |
| `/mission/status` | `std_msgs/String` JSON | Aggiunti età waypoint, raggio, ID, stato detector e rilevamento confermato. |

Il detector usa il comando strutturato per associare atomicamente abilitazione
e sessione; pubblicare soltanto `/identification/enable` non lo avvia. Se il
controllo non viene rinnovato per 1,5 s, il detector smette di analizzare e scarta
i risultati in volo. Un riavvio o una ripresa richiede nuove immagini e conferme.

Un `false` significa che un'immagine valida non ha prodotto una conferma.
Modello mancante, camera assente, analisi disabilitata o inferenza in errore
producono stato non valido e nessun nuovo booleano. L'ultimo booleano letto da
solo non permette di sapere se la camera funziona: usare anche `/identification/status`.
Il manager usa il risultato strutturato, verificandone età e sessione.

Un guasto del detector **non è trattato come un rilevamento** e attualmente non
introduce un ulteriore blocco della navigazione: compare nello stato missione.
I controlli GPS/IMU/bridge e gli stop esistenti restano operativi.

## Preparazione sul Raspberry

La copia locale è stata riallineata allo stack completo del Raspberry tramite il
backup `backups/environment_sync_20261019_230551`, con tutti i sorgenti, gli script
e le calibrazioni. Le modifiche YOLO sono state riapplicate dopo il confronto con
i file originali. Per installare le revisioni successive usare la procedura
PC è `tools/install_raspberry_yolo.py` nella radice del progetto, descritta in
`tools/README_install_raspberry_yolo.md`. Include modello CPU e controlli ROS
isolati che non richiedono il robot. Il backup locale prima delle modifiche è
`backups/before_wolf_detector_20261019.tar.gz` nella radice del progetto PC.

Prima della distribuzione fermare la missione e lo stack, poi copiare il `.pt`
addestrato nel percorso scelto. Il loader non sostituisce un file mancante con
un modello generico. Il YAML abilita il video del bridge e imposta 1 FPS.

Nel container, nell'ambiente Python già usato dai nodi:

```bash
source /opt/venv/bin/activate
source /opt/ros/jazzy/setup.bash
python -m pip install -r /ws/src/go2_animal_detector/requirements-inference.txt
python -m pip check
cd /ws
python "$(command -v colcon)" build --packages-select go2_animal_detector go2_mission_manager go2_navigation --symlink-install
source /ws/install/setup.bash
```

La compilazione con il Python del venv assicura che gli executable trovino
Ultralytics; se `colcon_core` non è importabile nel venv, renderlo disponibile prima
del build. Le dipendenze ROS, incluse `rclpy`, `rcl_interfaces` e i messaggi,
devono essere visibili nello stesso ambiente. Ultralytics è fissato alla versione
`8.4.174`, registrata nell'esportazione. Non installare integralmente il file
`models/wolf/environment.txt`: include l'ambiente CUDA/Colab dell'addestramento.
Verificare caricamento e predizioni con il runtime CPU scelto sul Raspberry.

Per il primo collaudo usare una sola istanza dello stack con movimento
disabilitato (il servizio automatico esistente usa invece `enable_motion:=true`):

```bash
ros2 launch go2_navigation waypoint_diretto.launch.py enable_motion:=false
```

Il detector è avviato dal launch con `start_detector:=true`, carica il modello
una volta e attende il manager. `start_detector:=false` permette di escluderlo.
I parametri del detector si leggono all'avvio; per cambiarli aggiornare YAML e
riavviare il nodo. Un `MODEL_ERROR` richiede correzione del percorso/dipendenze
o classi e un riavvio; non viene convertito in un `false`.

Monitoraggio, in terminali separati dopo il setup ROS:

```bash
ros2 topic hz /go2/camera/image_raw
ros2 topic echo /identification/status --field data
ros2 topic echo /mission/status --field data
ros2 topic echo /mission/wolf_detected --qos-durability transient_local
```

Una missione in dry run può verificare il raggio e il riconoscimento mentre
il bridge rimane disabilitato al movimento. Non inviare un waypoint a uno stack
attivo per fare questo test: il waypoint è già una richiesta di missione.

## Pi 4 e prestazioni

La prima configurazione accetta direttamente il `.pt` su CPU. Il target è Pi 4
2 GB, con Pi 5 8 GB come alternativa. Il profilo attuale richiede 1 inferenza/s:
controllare `effective_fps`, `inference_ms`, consumo RAM e carico dello stack.
Non avviare un secondo modello per fare benchmark contemporaneamente al nodo.

Nel primo tentativo sul Pi 4, PyTorch 2.11.0 CPU/Ultralytics 8.4.174 hanno
misurato 590,7 / 524,9 / 512,5 ms su tre immagini nere dopo il riscaldamento:
circa 1,84 FPS in serie e 403,5 MiB di RSS massimo per il solo processo del modello.
Questi numeri non misurano il flusso completo con navigazione e telecamera reale.

Ultralytics documenta l'esportazione NCNN per Raspberry Pi:
<https://docs.ultralytics.com/guides/raspberry-pi/>.
Se il `.pt` è troppo lento, si può esportare **il modello addestrato dell'utente**
su una macchina di sviluppo, installare il runtime NCNN compatibile sul Pi e
impostare `model_path` sulla directory esportata completa di metadati. Va poi
verificata l'equivalenza delle predizioni. L'adattatore usa l'API Ultralytics
anche per i formati esportati; NCNN non è stato collaudato in questa sessione.
I limiti di thread PyTorch non costituiscono un limite verificato per NCNN.

Riferimento per immagini NumPy BGR e `model.predict()`:
<https://docs.ultralytics.com/modes/predict/>.

## Verifiche locali

Per il nuovo test con video senza robot vedere [README_video_test.md](README_video_test.md).
Il test usa il manager reale in dry run, topic isolati e localizzazione simulata.

Dal PC, senza ROS e senza robot, con Python, NumPy e PyYAML:

```bash
python3 working/go2_waypoin_funzionante/ws/test/run_tests.py
```

Sono passati **58 test offline** il 20 ottobre 2026, inclusi i nuovi test video
e del profilo del modello. La suite usa il codice reale
di manager e detector, con un trasporto ROS
sostituito in memoria e un backend di inferenza simulato. Copre i test
precedenti del manager, raggio e attesa, buffer immagini, conferma temporale,
messaggi vecchi/duplicati, cambio missione durante l'inferenza, zero/revoca/stop,
flag memorizzato e mancata esplorazione all'arrivo. I test aggiunti coprono il
riscaldamento prima della disponibilità, lo scarto del suo risultato, gli errori
di riscaldamento, la conferma a 1 FPS e il mantenimento del limite sulle immagini vecchie.
La suite offline non verifica DDS, il caricamento del `.pt` reale, gli FPS sul Raspberry o il
comportamento fisico del robot: questi sono i collaudi da completare sul sistema.
