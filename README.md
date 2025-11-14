# FindImuWorld

## 📖 Descrizione

**FindImuWorld** è uno strumento Python per l'analisi e la visualizzazione dell'orientamento medio di sensori IMU (Inertial Measurement Unit). Il software processa dati di quaternioni provenienti da sensori IMU, calcola l'orientamento medio durante la fase statica iniziale e visualizza i frame di riferimento 3D sovrapposti.

### Funzionalità principali

- **Estrazione dati MATLAB**: Script MATLAB per estrarre dati da file `.mat` generati dal software BAF
- **Caricamento dati multi-nodo**: Importa automaticamente file CSV contenenti dati IMU (quaternioni, giroscopi, accelerometri, magnetometri)
- **Calcolo heading medio**: Calcola l'orientamento medio dei primi N campioni (default: 100) per ogni nodo
- **Conversione quaternioni**: Converte quaternioni in matrici di rotazione e angoli Euler (Roll-Pitch-Yaw)
- **Validazione orientamento**: Verifica la correttezza dello yaw rispetto alla convenzione ENU (East-North-Up) della IMU
- **Visualizzazione 3D**: Genera grafici 3D interattivi che mostrano i frame di riferimento sovrapposti

### Caso d'uso principale

Il software è particolarmente utile per **verificare che sensori IMU multipli abbiano lo stesso orientamento** durante acquisizioni successive. Ad esempio:
- Due nodi (node10 e node3) acquisiti in momenti diversi ma con identico orientamento fisico
- Confronto visuale degli assi di riferimento per validare l'allineamento
- Analisi della fase statica iniziale per determinare l'orientamento a riposo
- **Validazione dello yaw**: Conoscendo la convenzione del sistema di riferimento World della IMU (tipicamente **ENU - East-North-Up** per sensori BNO055), è possibile verificare la correttezza delle misure confrontando lo yaw calcolato con l'orientamento fisico reale del sensore rispetto al Nord magnetico

---

## 📁 Struttura del Progetto

```
FindImuWorld/
│
├── FindImuWorld.py                    # Script principale Python
├── extract_data_from_BAF_mat_files.m  # Script MATLAB per estrazione dati
├── README.md                          # Questo file
│
├── ExtractedNodeRawDataCSV/           # Cartella con i file CSV di input
│   ├── node10_4.csv                   # Dati nodo 10, acquisizione 4
│   ├── node10_5.csv                   # Dati nodo 10, acquisizione 5
│   ├── node3_1.csv                    # Dati nodo 3, acquisizione 1
│   └── node3_2.csv                    # Dati nodo 3, acquisizione 2
│
└── startingBafFiles/                  # File MATLAB originali (opzionali)
    ├── prova1_dueNodi_staticStart.mat
    └── prova2_duenodi_StaticStart.mat
```

---

## 🛠️ Installazione

<details>

### Requisiti di sistema

- **Python**: 3.7 o superiore
- **MATLAB** (opzionale): Solo se necessiti di estrarre dati da file `.mat` di BAF
- **Sistema operativo**: Windows, macOS, Linux

### Installazione rapida

1. **Naviga nella cartella del progetto** `FindImuWorld`

2. **Crea e attiva un ambiente virtuale** (consigliato):
   ```powershell
   python -m venv venv
   .\venv\Scripts\Activate.ps1
   ```

3. **Installa le dipendenze**:
   ```powershell
   pip install numpy pandas matplotlib
   ```

</details>

---

## 🚀 Uso

### Workflow completo

Il processo di analisi si articola in due fasi:

1. **[Opzionale] Estrazione dati da file BAF** → Genera file CSV dai `.mat` di BAF
2. **Analisi e visualizzazione** → Processa i CSV ed elabora gli orientamenti

---

## 📦 FASE 1: Estrazione Dati da File BAF (MATLAB)

> **Nota**: Salta questa sezione se hai già i file CSV pronti nella cartella `ExtractedNodeRawDataCSV/`

### Prerequisiti

- **Software BAF**: Utilizzato per acquisire i dati IMU (link: [*inserire link al software BAF*])
- **MATLAB**: Per eseguire lo script di estrazione
- **File .mat**: Output del software BAF contenente la struttura dati IMU

### Procedura di estrazione

#### 1. Acquisizione con BAF

Utilizza il software BAF per acquisire i dati dai sensori IMU. Il software genererà un file `.mat` (esempio: `prova1_dueNodi_staticStart.mat`).

#### 2. Apertura file in MATLAB

Apri MATLAB e carica il file `.mat`:

```matlab
% Carica il file generato da BAF
load('prova1_dueNodi_staticStart.mat')
```

Dopo il caricamento, nel workspace MATLAB apparirà una struttura dati (es: `sub9_trial2_task4`, `robot_logger_device`, o altro nome).

#### 3. Configurazione dello script

Apri il file `extract_data_from_BAF_mat_files.m` e **modifica la prima riga** inserendo il nome della struttura caricata:

```matlab
% MODIFICA QUESTA RIGA con il nome della tua struttura
save_imu_full_csv(robot_logger_device, 'ExtractedNodeRawDataCSV', 0.016, 3:12);

% Esempi:
% save_imu_full_csv(sub9_trial2_task4, 'ExtractedNodeRawDataCSV', 0.016, 3:12);
% save_imu_full_csv(mia_struttura_dati, 'ExtractedNodeRawDataCSV', 0.016, 3:12);
```

#### 4. Esecuzione dello script

Lancia lo script in MATLAB:

```matlab
extract_data_from_BAF_mat_files
```

Lo script creerà automaticamente la cartella `ExtractedNodeRawDataCSV/` e genererà i file CSV per ogni nodo.

#### 5. Parametri della funzione

```matlab
save_imu_full_csv(struttura_dati, cartella_output, dt, range_nodi)
```

| Parametro | Descrizione | Esempio |
|-----------|-------------|----------|
| `struttura_dati` | Nome della struttura caricata da `.mat` | `robot_logger_device` |
| `cartella_output` | Cartella dove salvare i CSV | `'ExtractedNodeRawDataCSV'` |
| `dt` | Periodo di campionamento (secondi) | `0.016` (60 Hz) |
| `range_nodi` | Range di nodi da estrarre | `3:12` (node3 a node12) |

#### 6. Output generato

Dopo l'esecuzione, troverai file CSV come:

```
ExtractedNodeRawDataCSV/
├── node3.csv
├── node4.csv
├── node10.csv
└── ...
```

Se hai **acquisizioni multiple** dello stesso nodo, rinomina manualmente i file aggiungendo un suffisso:

```
node10.csv  →  node10_1.csv   (prima acquisizione)
node10.csv  →  node10_2.csv   (seconda acquisizione)
```

### ⚠️ Importante: Verifica configurazione BNO055

Prima di procedere con l'analisi, **accertati che i sensori BNO055 utilizzino la convenzione di default**:

- **Sistema di riferimento World**: **ENU (East-North-Up)**
  - **X** → Est (East)
  - **Y** → Nord (North)
  - **Z** → Su (Up)

Se i sensori sono stati configurati con una convenzione custom diversa, gli angoli yaw risultanti potrebbero non corrispondere all'orientamento magnetico reale.

---

## 🔍 FASE 2: Analisi e Visualizzazione (Python)

### Esecuzione base

```powershell
# Assicurati che l'ambiente virtuale sia attivo
.\venv\Scripts\Activate.ps1

# Esegui lo script
python FindImuWorld.py
```

### Output atteso

Lo script produce:

1. **Output console**:
   - Lista dei file caricati
   - Heading medio per ogni nodo (angoli in gradi)
   - Angoli estrinseci e intrinseci
   - Procedura step-by-step per le rotazioni intrinsiche

2. **Grafico 3D interattivo**:
   - Assi X, Y, Z colorati per ogni nodo
   - Frame di riferimento sovrapposti
   - File PNG salvato nella cartella principale: `mean_headings_3d_combined_node10_4plus_node3.png`

### Esempio di output console

```
============================================================
  FindImuWorld - Calcolo heading medio e visualizzazione 3D
============================================================

📂 Trovati 4 file:
   - node10_4.csv
   - node10_5.csv
   - node3_1.csv
   - node3_2.csv

📊 Calcolo heading medio (primi 100 campioni):

  🔹 node10_4:
    ✓ 100 campioni estratti (su 4343 totali)
    ✓ Heading medio calcolato

    📐 Angoli ESTRINSECI (assi fissi globali):
       Roll  (X):  -23.456°
       Pitch (Y):   12.789°
       Yaw   (Z):  156.234°

    📊 Angoli INTRINSECI (assi che si modificano):
       Roll  (X):  -25.678°
       Pitch (Y):   14.321°
       Yaw   (Z):  154.876°

    🔍 Verifica ricostruzione matrice:
       Errore Frobenius: 0.000001

...

🎨 Creazione plot 3D combinato con 4 nodi:
   - node10_4
   - node10_5
   - node3_1
   - node3_2

   💾 Plot salvato: mean_headings_3d_combined_node10_4plus_node3.png
   ▶️  Apertura finestra plot...

============================================================
  ✅ Elaborazione completata!
============================================================
```

---

## 📊 Formato dei File di Input

### File CSV richiesti

I file devono essere nella cartella `ExtractedNodeRawDataCSV/` e seguire il pattern di naming: `node<ID>_<acquisizione>.csv`

**Esempio**: `node10_4.csv` (nodo 10, acquisizione 4)

### Formato CSV

```csv
time_s,imu_id,qw,qx,qy,qz,gyro_x_dps,gyro_y_dps,gyro_z_dps,acc_x_ms2,acc_y_ms2,acc_z_ms2,mag_x_uT,mag_y_uT,mag_z_uT
0,node10,0.197693,0.650391,0.223267,-0.698608,0.035415,-0.051375,-0.000567,0.06,-0.04,-0.06,-36.875,36,-39.188
0.016,node10,0.197693,0.650391,0.223267,-0.698608,0.066092,-0.102751,-0.001052,0.05,-0.03,-0.06,-36.875,36,-39.188
...
```

### Colonne richieste (obbligatorie)

- `time_s`: Timestamp in secondi
- `imu_id`: Identificatore del nodo IMU
- `qw`, `qx`, `qy`, `qz`: Componenti del quaternione (w, x, y, z)

### Colonne opzionali

- `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps`: Giroscopio (deg/s)
- `acc_x_ms2`, `acc_y_ms2`, `acc_z_ms2`: Accelerometro (m/s²)
- `mag_x_uT`, `mag_y_uT`, `mag_z_uT`: Magnetometro (µT)

---

## 💡 Esempio d'Uso Completo

### Scenario: Verifica allineamento tra due acquisizioni e validazione orientamento

**Setup**:
- **Nodo 10**: Acquisito in due momenti diversi (file `node10_4.csv`, `node10_5.csv`)
- **Nodo 3**: Acquisito in due momenti diversi (file `node3_1.csv`, `node3_2.csv`)
- **Obiettivi**:
  1. Verificare che i nodi abbiano lo stesso orientamento nelle acquisizioni successive
  2. Validare la correttezza dello yaw misurato rispetto alla convenzione ENU

### Passaggi

#### 1. Preparazione dei dati

Assicurati che i file CSV siano nella cartella corretta:

```powershell
ls ExtractedNodeRawDataCSV\

# Output atteso:
# node10_4.csv
# node10_5.csv
# node3_1.csv
# node3_2.csv
```

#### 2. Esecuzione dello script

```powershell
# Attiva l'ambiente virtuale
.\venv\Scripts\Activate.ps1

# Esegui l'analisi
python FindImuWorld.py
```

#### 3. Interpretazione dei risultati

**Console Output**:
Lo script stamperà gli angoli Euler per ogni nodo. Confronta i valori:

```
node10_4:  Roll=-23.5°, Pitch=12.8°, Yaw=156.2°
node10_5:  Roll=-23.7°, Pitch=12.6°, Yaw=156.0°

node3_1:   Roll=45.2°, Pitch=-8.3°, Yaw=-120.5°
node3_2:   Roll=45.4°, Pitch=-8.5°, Yaw=-120.3°
```

✅ **Differenze piccole** (< 1°) indicano buon allineamento tra acquisizioni!

**Grafico 3D**:
- Gli assi X, Y, Z di `node10_4` e `node10_5` dovrebbero essere quasi sovrapposti
- Gli assi di `node3_1` e `node3_2` dovrebbero essere quasi sovrapposti
- Se i nodi hanno orientamento diverso, gli assi saranno chiaramente separati

#### 4. Validazione dello yaw con convenzione ENU

Per verificare che i sensori IMU BNO055 forniscano misure corrette, confronta lo **yaw misurato** con l'**orientamento fisico reale**:

**Convenzione ENU (East-North-Up)**:
- **Yaw = 0°**: Sensore orientato verso **Est** (asse X verso Est)
- **Yaw = 90°**: Sensore orientato verso **Nord** (asse X verso Nord)
- **Yaw = 180° o -180°**: Sensore orientato verso **Ovest**
- **Yaw = -90°**: Sensore orientato verso **Sud**

**Esempio di validazione**:

```
node10_4:  Yaw = 156.2°
```

Con una bussola o conoscendo l'orientamento della stanza:
- Se il sensore fisicamente punta verso **Nord-Ovest** → ✅ Yaw ~156° è corretto!
- Se il sensore punta verso **Est** ma yaw ≠ 0° → ⚠️ Possibile problema di calibrazione o configurazione custom

> **Nota**: Assicurati che la BNO055 **non sia configurata con una convenzione custom** diversa da ENU. Controlla la configurazione firmware del sensore.

#### 5. Salvataggio dei risultati

Il grafico viene automaticamente salvato come:
```
mean_headings_3d_combined_node10_4plus_node3.png
```

Puoi includerlo in report o presentazioni.

---

## 🔧 Configurazione Avanzata

### Modificare il numero di campioni per la media

Modifica la riga nel file `FindImuWorld.py`:

```python
# Da:
mean_rotations, angles_data = compute_mean_heading_per_node(node_data, n_samples=100)

# A (esempio con 200 campioni):
mean_rotations, angles_data = compute_mean_heading_per_node(node_data, n_samples=200)
```

### Filtrare nodi specifici

Lo script attualmente filtra automaticamente:
- `node10` con acquisizione ≥ 4 (node10_4, node10_5, ...)
- Tutti i `node3`

Per modificare il filtro, edita la funzione `plot_combined_filtered_nodes()` in `FindImuWorld.py`.

### Cambiare la cartella di input

Modifica la variabile `inputs_dir` nella funzione `main()`:

```python
# Da:
inputs_dir = os.path.join(script_dir, "ExtractedNodeRawDataCSV")

# A:
inputs_dir = r"C:\percorso\personalizzato\dati"
```

---

## 📐 Note Matematiche

### Convenzioni di rotazione

Il software supporta due convenzioni per gli angoli Euler:

#### 1. **Angoli Estrinseci (Assi Fissi)**
- Rotazioni applicate rispetto agli assi globali fissi
- Ordine: **Yaw (Z) → Pitch (Y) → Roll (X)**
- Formula: `R = Rz(yaw) × Ry(pitch) × Rx(roll)`

#### 2. **Angoli Intrinseci (Assi Mobili)**
- Rotazioni applicate rispetto agli assi locali che si muovono
- Ordine: **Roll (X) → Pitch (Y') → Yaw (Z'')**
- Gli assi Y e Z si aggiornano dopo ogni rotazione

### Verifica dell'accuratezza

Lo script calcola l'**errore di Frobenius** tra la matrice originale e quella ricostruita:

```
Errore = ||R_originale - R_ricostruita||_F
```

Un errore < 0.01 indica una conversione corretta.

---

## 🐛 Troubleshooting

### Problema: Nessun file trovato

**Errore**:
```
❌ Errore: Nessun file node*.csv trovato in ExtractedNodeRawDataCSV
```

**Soluzione**:
- Verifica che i file CSV siano nella cartella `ExtractedNodeRawDataCSV/`
- Assicurati che i nomi inizino con `node` (es: `node10_4.csv`)

### Problema: Colonne mancanti

**Errore**:
```
✗ Quaternioni non trovati, salto questo nodo
```

**Soluzione**:
- Verifica che il CSV contenga le colonne: `qw`, `qx`, `qy`, `qz`
- Controlla che non ci siano spazi extra nei nomi delle colonne

### Problema: Ambiente virtuale non si attiva

**Errore**:
```
execution of scripts is disabled on this system
```

**Soluzione**:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### Problema: Matplotlib non mostra il grafico

**Soluzione**:
- Verifica che matplotlib abbia un backend grafico installato
- Su Windows, potrebbe essere necessario installare: `pip install pyqt5`

---

## 📚 Riferimenti

### Documentazione delle librerie

- [NumPy Documentation](https://numpy.org/doc/)
- [Pandas Documentation](https://pandas.pydata.org/docs/)
- [Matplotlib Documentation](https://matplotlib.org/stable/contents.html)

### Teoria matematica

- **Quaternioni**: [Wikipedia - Quaternions and spatial rotation](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation)
- **Angoli Euler**: [Wikipedia - Euler angles](https://en.wikipedia.org/wiki/Euler_angles)
- **Matrici di rotazione**: [Wikipedia - Rotation matrix](https://en.wikipedia.org/wiki/Rotation_matrix)

---

## 📄 Licenza

Questo progetto è distribuito per uso interno e ricerca. Per utilizzi commerciali, contattare l'autore.

---

## 👤 Autore

**Progetto**: FindImuWorld  
**Data**: 2024

---

## 🔄 Changelog

### Versione 1.0 (corrente)
- ✅ Supporto multi-nodo con pattern di naming flessibile
- ✅ Calcolo heading medio con media di quaternioni
- ✅ Conversione in angoli estrinseci e intrinseci
- ✅ Visualizzazione 3D con frame sovrapposti
- ✅ Filtro automatico per nodi selezionati
- ✅ Verifica matematica della ricostruzione

---

## 📞 Supporto

Per problemi, domande o suggerimenti, contatta il maintainer del progetto.
