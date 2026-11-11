# Prove sui video e salvataggio dei campioni

La frequenza predefinita è **1 FPS**. La presenza è confermata quando almeno
**due fotogrammi distinti negli ultimi tre secondi** contengono un lupo con
confidenza almeno 0,20. Possono essere non consecutivi. Non si limita più il
conteggio alle ultime cinque immagini. Il limite di tre secondi è inclusivo;
nel nodo ROS si usa l'ora di completamento dell'inferenza per scartare i voti
vecchi. Il limite di età del singolo risultato resta di due secondi.
Il manager memorizza la conferma, ferma la missione e attiva il flag.

## Dal PC: aggiornamento e prova ROS sul Raspberry

Il runtime a 1 FPS con riscaldamento è stato installato e verificato nel run
`go2_yolo_20261020_001714`. La nuova regola e il salvataggio richiedono un nuovo
aggiornamento, che confronta i file con quel payload e conserva un nuovo backup.
Dal terminale del PC:

```bash
python3 /home/matteo/go2_codex/tools/install_raspberry_yolo.py --host 192.168.12.116
```

Attendere il completamento prima di avviare la prova. Per analizzare il video
breve con lupi e poi quello con volpi:

```bash
python3 /home/matteo/go2_codex/tools/test_wolf_videos.py "/home/matteo/Documents/University/Tesi/video lupi test/lupi1.mp4"
python3 /home/matteo/go2_codex/tools/test_wolf_videos.py "/home/matteo/Documents/University/Tesi/video lupi test/volpi1.mp4"
```

Per tutti i nove video, basta il comando senza percorso:

```bash
python3 /home/matteo/go2_codex/tools/test_wolf_videos.py
```

Il Raspberry è attualmente `192.168.12.116` (prima era `10.42.0.196`).
Entrambi gli script accettano `--host INDIRIZZO_IP` se l'indirizzo del Raspberry
è cambiato. Il comando di prova copia i file nel container, usa il modello già
installato e recupera i report in `go2_codex/video_results/DATA_ID/index.html`.
Aprire quel file nel browser. I risultati originali restano nel Raspberry in
`/ws/log/video_reviews/DATA_ID/reports`. Non serve ROS sul PC.
Il comando usa **ROS 2 reale nel container del Raspberry**: avvia camera da
video, detector, mission manager e ingressi di navigazione simulati. I messaggi
passano sui topic `/vision_test` nel dominio 84; il movimento è disabilitato e
non viene avviato un bridge verso il robot. Ogni filmato ha una nuova missione.
La prova termina alla prima conferma del manager (`WOLF_DETECTED`, flag attivo,
analisi disabilitata), oppure a fine video dopo aver atteso l'ultima inferenza.
Il test non inietta risultati positivi: vengono solo dal modello sul video.
Per confrontare altri spezzoni dopo una conferma, avviare un nuovo filmato.

La pagina iniziale contiene i link alle gallerie e a `ros_report.json`, con
esito e stato finale dei nodi. Per ogni video si conservano anche `events.jsonl`
(messaggi ROS ricevuti) e `ros_launch.log`. Errori modello, registrazione, video,
assenza di risultati validi o comandi di movimento inattesi interrompono la prova
con un errore esplicito. I report parziali vengono comunque recuperati sul PC.

## Cosa viene salvato

Ogni prova crea una cartella separata, senza sovrascrivere prove precedenti:

- `images/*.png`: fotogrammi originali effettivamente analizzati, anche negativi;
- `annotated/*.jpg`: gli stessi campioni con riquadri e confidenza del lupo;
- `results.csv`: campione, tempo/frame nel video (prova completa), confidenza,
  positivo nel singolo frame, conferma temporale, conteggi, validità e tempi;
- `results.jsonl`: stessi esiti, coordinate dei riquadri, errori e ID ROS se presenti;
- `metadata.json`: impostazioni, sorgente e SHA-256 del modello;
- `summary.json`: stato della prova e conteggi; `index.html`: galleria consultabile.

Gli esiti `frame_positive` e `detected` sono distinti. Per esempio:

| Secondo | Lupo nel frame | Confermato |
|---|---|---|
| 0 | sì | no |
| 1 | no | no |
| 2 | sì | sì |
| 3 | no | sì (i due positivi sono ancora nella finestra) |
| 4 | no | no |

Il flag del mission manager rimane invece attivo dopo la prima conferma.
Confidenza zero significa che non c'è un lupo sopra soglia, non una probabilità
misurata di assenza. Il detector filtra la classe lupo: una volpe non deve
produrre un riquadro lupo. Il nome del file non costituisce una verità di riferimento.

Per valutare il modello, confrontare visivamente originale e riquadri: annotare
lupi visibili non rilevati e riquadri su altri animali. Provare sia i filmati con
lupi sia quelli con volpi/cinghiale/animali misti. Un risultato positivo del modello
non dimostra da solo che sia corretto; precisione e richiamo richiedono etichette
manuali dei campioni. Le immagini sono salvate alla risoluzione originale e le
prove lunghe possono occupare molto più spazio del video compresso.

## Analisi offline aggiuntiva (non usata dal comando di prova ROS)

Per file già presenti nel container:

```bash
bash /ws/analyze_video.sh /percorso/video.mp4 --output /ws/log/mia_prova
# Oppure passare una cartella per analizzare tutti i suoi video.
```

Usa Python del venv `/opt/venv`, il backend di produzione e la stessa classe
per la conferma temporale. Le immagini sintetiche di riscaldamento non vengono
salvate né conteggiate. Il tempo sorgente usa FPS nominali del file: per video
con FPS variabili è approssimato; `--source-fps` permette un override esplicito.

## Nel container: prova ROS con il mission manager

Dopo l'aggiornamento, nel container e con un video già disponibile:

```bash
ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 bash /ws/run_video_test.sh /percorso/video.mp4
```

Il launch avvia video camera, detector reale, manager reale con movimento
**disabilitato** e ingressi simulati di navigazione. Tutti gli endpoint applicativi
sono sotto `/vision_test`, senza bridge, GPS o IMU reali. Il waypoint fittizio è
a 25 m, entro il raggio iniziale; il video parte quando il detector è pronto.
Al primo `true`, il manager entra in `WOLF_DETECTED` e il video si mette in pausa.
Questa prova verifica l'integrazione e non scorre necessariamente tutto il file.

Il salvataggio è abilitato nel test ROS in `/ws/log/wolf_samples/DATA_ID`.
Per scegliere un'altra cartella aggiungere `record_directory:=/ws/log/prova`.
Nel nodo normale di produzione è opzionale: impostare `record_directory` nel
YAML e riavviare. Lo stato del detector espone `recording_directory` e
`recording_error`. Un errore di scrittura è segnalato e non impedisce lo stop.
Si salvano anche risultati di inferenza invalidati da età/sessione, con `valid=false`;
non diventano eventi validi per il manager. I frame scartati prima dell'inferenza
e quelli sostituiti nel buffer non sono campioni analizzati.

Per osservare la prova, in un altro terminale nello stesso container:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1
ros2 topic echo /vision_test/identification/status --field data
ros2 topic echo /vision_test/mission/wolf_detected --qos-durability transient_local
```

Per ripetere il test ROS, Ctrl+C e rilanciare. `loop:=true` ripete il video solo
finché l'analisi resta abilitata: non azzera il flag di missione. `fps:=1.0`,
`initial_distance_m:=100.0`, `approach_speed_mps:=1.0` permettono altri scenari.
Il replay ROS è a velocità originale e salta i campioni in ritardo, mantenendo
una sola immagine in attesa. La prova completa senza ROS conserva invece tutti
i campioni a 1 FPS e usa i tempi del video.

## Verifiche

```bash
python3 /home/matteo/go2_codex/working/go2_waypoin_funzionante/ws/test/run_tests.py
python3 /home/matteo/go2_codex/tools/test_yolo_installation.py
```

La suite offline usa trasporto ROS e modello simulati. Copre finestra temporale,
limite inclusivo, immagini duplicate, latenza, cambio sessione, riscaldamento,
salvataggio positivo/negativo, errore disco, prosecuzione del filmato dopo il
rilevamento, stop e flag del manager. I tuoi filmati devono ancora essere
analizzati con il modello reale. L'arresto fisico richiede una prova separata.

Riferimento per i riquadri e le confidenze:
[Ultralytics Results](https://docs.ultralytics.com/reference/engine/results).
