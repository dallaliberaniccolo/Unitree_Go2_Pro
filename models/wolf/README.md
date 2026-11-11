# Modello fornito dall'utente

Copiato il 19 ottobre 2026 da
`/home/matteo/Documents/University/Tesi/yolo26n`, conservando gli originali.

- File: `wolf_detector.pt` (5.340.549 byte).
- SHA-256 verificato rispetto a `inference_config.json`:
  `d1911e0e634bbb5f5a7151c3f3025d67c372202776d5b69d24c49557a41532ca`.
- Classi dichiarate: `0 = wolf`, `1 = fox`.
- Ultralytics dichiarato: `8.4.174`.
- Inferenza dichiarata: `imgsz=448`, `conf=0.2`, `iou=0.6`,
  `rect=false`, `max_det=300`.

Questi parametri sono ora riportati nel nodo e nel YAML. Il nodo filtra la
classe lupo e aggiunge una conferma temporale di 3 immagini su 5 entro 2 s.
Le metriche salvate nel JSON si riferiscono alla validazione originale, non
alla nuova pipeline video ROS o alle prestazioni del Raspberry.

`environment.txt` è la registrazione dell'ambiente di addestramento Colab/CUDA:
non installarlo integralmente sul Raspberry. Le dipendenze di inferenza sono
in `src/go2_animal_detector/requirements-inference.txt`.
Il caricamento tramite PyTorch e l'inferenza reale non sono ancora verificati
in questa sessione; è stata verificata l'integrità del checkpoint.
