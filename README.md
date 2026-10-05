# Robot CDF 2026 — Logiciel

Firmware embarqué et outils PC du robot **Wall-A**, construit pour la **Coupe de France de Robotique 2026**.

Le firmware tourne sur un **STM32F407IGT6** sous **FreeRTOS**, en C++ orienté objet, sans allocation dynamique. Il asservit les moteurs (PID + odométrie à 200 Hz), supervise le robot (capteurs, températures, courants, santé du noyau) et dialogue avec le PC par un protocole texte (USB CDC ou UART).

Ce dépôt ne contient que le logiciel. Le reste du projet est dans des dépôts séparés :
[carte électronique (KiCad)](https://github.com/Soupalognon/CDR2026-Kicad) · [simulateur](https://github.com/Soupalognon/CDR2026-Simulator).

> **Par où commencer ?**
>
> - Comprendre le code : [Architecture du firmware](#architecture-du-firmware).
> - Piloter le robot depuis le PC : [Protocole de communication](#protocole-de-communication) et [pythonTester](#outil-pc--pythontester).
> - Régler l'asservissement : [README_TUNING.md](README_TUNING.md).
> - Lancer les tests : [Tests unitaires](#tests-unitaires-pc).

## Contenu du dépôt

```
Wall-A-STM/        Projet STM32CubeIDE (firmware)
├── App/           Code applicatif C++ — le seul dossier source à modifier à la main
├── Tests/         Tests unitaires Google Test, exécutés sur PC
├── Core/ Drivers/ Middlewares/ LWIP/ USB_DEVICE/    Générés par CubeMX, ne pas modifier
├── Wall-A-STM.ioc Configuration CubeMX (broches, horloges, périphériques)
└── nfr-check.sh   Vérification automatique des règles de code
pythonTester/      Moniteur série graphique (télémétrie, commandes, manette)
README_TUNING.md   Guide de réglage de l'asservissement
_bmad-output/      Documents de conception (PRD, architecture, stories)
```

## Architecture du firmware

Tout est instancié statiquement et câblé par injection de dépendances dans [cppMain.cpp](Wall-A-STM/App/cppMain.cpp), appelé depuis `Core/Src/main.c`.

### Tâches FreeRTOS

| Tâche | Classe | Cadence | Rôle |
|---|---|---|---|
| `OdoCtrl` | `OdoControl` | 200 Hz | Lit l'odométrie, filtre la consigne (EMA), calcule feedforward + PID, pilote les moteurs, détecte blocage moteur et encodeur muet |
| `MoPlan` | `MotionPlanner` | événementiel | Reçoit les commandes de mouvement, publie la consigne vers `OdoControl`, réagit aux alarmes capteurs, coupe les moteurs si plus de commande (watchdog) |
| `ExtRX` | `ExternalComm::rxTask` | événementiel | Assemble les octets reçus en lignes et exécute les commandes `CMD …` |
| `ExtTX` | `ExternalComm::txTask` | événementiel | Envoie sur chaque canal les messages publiés (TEL, ALT, HLT, LOG) |
| `SensorMgr` | `SensorManager` | par groupe de capteurs | Déclenche les mesures de chaque groupe à sa propre cadence, publie un snapshot, notifie `MotionPlanner` en cas d'alarme |
| `Monitor` | `Monitoring` | 10 Hz | Publie la télémétrie d'odométrie et l'état des capteurs, surveille la fraîcheur des données et la santé FreeRTOS (piles, tas) |
| `Blink` | — | 500 ms | LED rouge, témoin de fonctionnement |

Ordre de priorité décroissant : `OdoCtrl` > `MoPlan` = `ExtRX` > `ExtTX` > `SensorMgr` > `Monitor`. Cadences, priorités et tailles de pile sont toutes dans [Config.h](Wall-A-STM/App/Config.h).

`ActuatorManager` n'est pas une tâche : `ExternalComm` l'appelle directement pour les commandes `CMD ACTUATOR`. Il est câblé sans aucun actionneur pour l'instant, donc ces commandes n'ont pas encore de cible.

### Flux de données

```
PC / debug ──► ExternalComm (rxTask) ──┬─► MotionPlanner ──► OdoControl ──► Drv8262 ──► moteurs
 (USB / UART)                          │     (mailbox)        ▲  200 Hz
                                       │                  encodeurs (TIM4 / TIM8)
                                       └─► ActuatorManager ──► actionneurs

OdoControl ──(snapshot)──┐
SensorManager ──(snapshot)┴─► Monitoring ──► ExternalComm (txTask) ──► PC / debug

SensorManager ──(alarme, xTaskNotify)──► MotionPlanner
```

Règle d'architecture : une tâche n'appelle jamais directement une autre tâche. Elles communiquent par `IBus::publish`, par les primitives FreeRTOS (`xQueueOverwrite`, `xTaskNotify`) ou par des snapshots horodatés lus par `Monitoring`.

Les graphes de dépendances détaillés sont dans [architecture.md](_bmad-output/planning-artifacts/architecture.md). Ce document mentionne encore une classe `SystemInit` qui n'existe plus : le câblage est dans `cppMain.cpp`.

### Capteurs

`SensorManager` pilote quatre groupes de capteurs, chacun à sa cadence (définies dans `Config.h`) :

| Groupe | Capteurs | Source brute | Cadence |
|---|---|---|---|
| Températures | `InternalTemperature` ×3 (moteur primaire, moteur secondaire, alimentation) | ADC3 | 1 Hz |
| Courants moteurs | `MotorCurrentSense` ×4 (primaire G/D, secondaire G/D) | ADC1 | 5 Hz |
| Proximité | `B5WLB2101` ×4 | ADC2 | 10 Hz |
| Proximité | `Pololu5472` ×4 | Input Capture TIM3 | 10 Hz |

Ajouter un capteur ou un actionneur = une nouvelle classe concrète qui implémente l'interface, plus son câblage dans `cppMain.cpp`, sans toucher aux classes existantes. Si `MAX_SENSORS` change, mettre aussi à jour `SensorType` et la table `AlarmBits` de [MotionPlanner.h](Wall-A-STM/App/Tasks/MotionPlanner.h).

### Organisation de `Wall-A-STM/App/`

| Dossier | Contenu |
|---|---|
| `Interfaces/` | Contrats purs `I*.h` (`IBus`, `IMotorHAL`, `IEncoderHAL`, `IOdomHAL`, `IAdcHAL`, `IInputCaptureHAL`, `IKernelHAL`, `ISensor`, `IActuator`, `ICom`…) |
| `Drivers/` | Accès direct au matériel : `Adc`, `Drv8262`, `Encoder`, `InputCapture`, `KernelMonitor`, `UartChannel`, `UsbCdcChannel` |
| `Services/` | `Odometry` (pose et vitesses depuis les encodeurs), `ActuatorManager`, `BusFormat` (formatage ASCII des messages) |
| `Services/Sensors/` | Capteurs concrets `ISensor` : conversion de la mesure brute en grandeur physique + alarme |
| `Controllers/` | `Pid`, algorithme pur sans dépendance matérielle ni FreeRTOS |
| `Tasks/` | Classes de tâches FreeRTOS |
| `Config.h` | Toutes les constantes : cadences, piles, priorités, gains PID, géométrie, seuils, policies de canaux |

Matériel utilisé par l'application : encodeurs sur TIM4 et TIM8, moteurs via DRV8262 (TIM1), capteurs sur ADC1/2/3 et TIM3, canal UART sur USART1, canal USB CDC. Ethernet (LwIP) est configuré par CubeMX mais aucun canal n'est branché dessus. Le détail est dans l'[annexe matérielle](#annexe--configuration-matérielle-stm32).

### Règles de développement

- Ne modifier que `App/` et `Tests/`. Le reste est régénéré par CubeMX.
- `App/` n'appelle jamais la HAL CubeMX directement : toujours via une interface injectée.
- Aucune allocation dynamique (`new`, `delete`, `malloc`, `free`).
- Tout message ASCII est formaté dans `BusFormat`, jamais avec un `snprintf` ailleurs.
- Constantes, piles et priorités uniquement dans `Config.h`.
- Identifiants et commentaires en anglais, sans caractère accentué dans `App/`.
- Membres privés préfixés `_`, un fichier par classe, garde d'inclusion `APP_<DOSSIER>_<FICHIER>_H`.
- Compilation en `-Wall -Wextra -fno-exceptions -fno-rtti`, sans aucun warning.

Ces règles sont vérifiées par `nfr-check.sh` :

```bash
cd Wall-A-STM
bash nfr-check.sh
```

## Protocole de communication

Messages texte, une ligne par message, de la forme `PRÉFIXE DONNÉES`. Les lignes émises par le robot se terminent par `\r\n`.

### PC → robot

Les lignes commencent par `CMD` et se terminent par `\n`. Les octets d'une même ligne doivent arriver à moins de 100 ms d'intervalle, sinon le tampon de réception est vidé. Chaque ligne reçue est renvoyée en `LOG` (`echo: …`). Une commande inconnue est aussi renvoyée en `LOG`.

| Commande | Effet |
|---|---|
| `CMD MOVE_VEL v w` | Consigne de vitesse linéaire (m/s) et angulaire (rad/s) |
| `CMD MOVE_POSE x y angle` | Consigne de position absolue. Mode expérimental : la détection d'arrivée est désactivée dans `OdoControl::tickPose` |
| `CMD MOVE_STOP` | Arrêt |
| `CMD PID_SPEED P:… I:… D:…` | Gains du PID de vitesse linéaire |
| `CMD PID_ANGLE P:… I:… D:…` | Gains du PID de vitesse angulaire |
| `CMD PID P:… I:… D:…` | Mêmes gains pour les deux PID |
| `CMD ACTUATOR <NOM>_<id> <commande>` | Commande l'actionneur dont l'identifiant est le suffixe numérique, par exemple `PUMP_1` (aucun actionneur câblé pour l'instant) |

Watchdog : sans nouvelle commande pendant `CMD_WATCHDOG_TIMEOUT_MS` (1 s par défaut), `MotionPlanner` vide la consigne et les moteurs s'arrêtent. `CMD_WATCHDOG_ENABLED` l'active.

Les gains envoyés par `CMD PID*` ne sont pas persistants : après un redémarrage, ce sont les valeurs de `Config.h` qui s'appliquent.

### Robot → PC

| Préfixe | Messages |
|---|---|
| `TEL` | `ODO_VEL time v w` · `ODO_POSE time x y yaw` · `ODO_SP time vLeft vRight` · `ODO_VOLT time voltLeft voltRight` |
| `ALT` | `ARRIVAL` · `ALARM 0x…` · `STALL` · `ENCODER_FAULT <côté>` · `INIT <côté>` · `STALE <module>` · `PROXIMITY d` · `SENSOR time <nom>:<valeur>` · `STACK_LOW name:<tâche> WORDS:<n>` · `HEAP_LOW FREE:<octets>` |
| `HLT` | `TEMP t` · `SENSORS time NUMBER ALARM 0x…` · `SENSOR time <nom>:<valeur>` · `RTOS_HEAP time FREE MIN` · `RTOS_TASK time <tâche>_STACK:<mots>` |
| `LOG` | Texte libre préfixé par le niveau : `I ` (info), `W ` (warning), `E ` (erreur) |

Exemple : `TEL ODO_VEL time:4823 v:0.50 w:0.00`

### Canaux et policies

`ExternalComm` gère jusqu'à trois canaux (UART, USB CDC, Ethernet). Dans `Config.h`, `UART_POLICY`, `USB_POLICY` et `ETH_POLICY` choisissent quels topics (`log`, `tel`, `alt`, `hlt`) sont émis sur chacun.

| Flag `ENABLE_HIGH_SPEED_TUNING` | UART (debug) | USB (PC) | Télémétrie `TEL ODO_*` |
|---|---|---|---|
| `false` (état actuel) | `log`, `alt` | `tel`, `alt`, `hlt` | Publiée par `Monitoring` à 10 Hz |
| `true` | `log` | `log` | Émise par `OdoControl` à chaque tick de 200 Hz via `log_info`, donc sous la policy `log` |

Ethernet n'a aucune policy active. Pour le réglage fin de l'asservissement, passer le flag à `true` donne une télémétrie à 200 Hz sur le canal de logs, mais coupe `tel`, `alt` et `hlt`.

## Compiler et flasher

1. Ouvrir `Wall-A-STM/` dans **STM32CubeIDE** (*File ▸ Import ▸ Existing Projects into Workspace*).
2. Compiler en `Debug` ou `Release`.
3. Flasher et déboguer avec la configuration [Wall-A-STM.launch](Wall-A-STM/Wall-A-STM.launch) (sonde J-Link, cible STM32F407IG). Une configuration OpenOCD pour ST-Link est aussi fournie dans `Wall-A-STM.cfg`.

Pour modifier le brochage ou les périphériques, passer par `Wall-A-STM.ioc` et régénérer le code. La régénération ne touche pas à `App/`.

## Outil PC : pythonTester

Interface graphique Python pour suivre la télémétrie en direct, envoyer des commandes et piloter le robot à la manette.

```powershell
cd pythonTester
pip install -r requirements.txt   # pyserial, matplotlib, pygame
py -3.11 main.py
```

Le port série par défaut (`COM31`), la fenêtre du graphe, les plages des curseurs et les réglages de la manette sont dans `pythonTester/config.py`. Détails dans [pythonTester/README.md](pythonTester/README.md).

## Réglage de l'asservissement

[README_TUNING.md](README_TUNING.md) décrit la procédure pas à pas : feedforward, filtres EMA, puis P, I, D. Les gains se règlent en direct, sans recompiler, avec `CMD PID_SPEED`, `CMD PID_ANGLE` et `CMD MOVE_VEL`.

## Tests unitaires (PC)

Les tests s'exécutent sur PC avec Google Test, sans carte STM32 ni chaîne de compilation ARM : `Tests/Stubs/` remplace FreeRTOS et la HAL, et `Tests/Mocks/` fournit les doubles des interfaces. Chaque classe de `App/` a sa suite dans `Tests/Unit/`.

### Prérequis

**MSYS2 MinGW64** installé dans `D:\msys64\` (installeur officiel : [msys2.org](https://www.msys2.org)), avec `g++`, `cmake` et `ninja` dans `D:\msys64\mingw64\bin\`. Pour installer CMake :

```powershell
"D:\msys64\mingw64.exe" pacman -S --noconfirm mingw-w64-x86_64-cmake
```

Pour un autre emplacement (ex. `C:\msys64\`), adapter `MINGW_BIN` et `MSYS_BIN` dans `Tests/run_tests.sh`, et les chemins des commandes ci-dessous.

### Lancer les tests

```powershell
& "D:\msys64\usr\bin\bash.exe" -l "<chemin_du_dépôt>\Wall-A-STM\Tests\run_tests.sh"
```

Le script configure CMake, compile, lance chaque exécutable de test et affiche un bilan. Il se termine par une pause (« Appuyez sur Entrée »). Options : `--clean` (supprime `Tests/build/`), `--configure` (force la reconfiguration), `--build-only`, `--test-only`.

À la première configuration, Google Test v1.14.0 est téléchargé s'il n'est pas installé sur le système (~5 MB) : une connexion internet est nécessaire une fois. Les exécutions suivantes utilisent le cache de `Tests/build/_deps/`.

| Suite | Cible testée |
|---|---|
| `BusFormatTest` | `BusFormat` |
| `ExternalCommTest` | `ExternalComm` : parsing des commandes, publications, snapshot |
| `ConcreteOdomHALTest` | `Odometry` |
| `OdoControlTest` | `OdoControl` |
| `PidTest` | `Pid` |
| `MotionPlannerTest` | `MotionPlanner` |
| `SensorManagerTest` | `SensorManager` |
| `ActuatorManagerTest` | `ActuatorManager` |
| `MonitoringTest` | `Monitoring` |
| `SensorDriversTest` | Capteurs concrets (`B5WLB2101`, `MotorCurrentSense`, `InternalTemperature`, `Pololu5472`) |

### Construire à la main

Depuis PowerShell, sans passer par `run_tests.sh` :

```powershell
$env:PATH = "D:\msys64\mingw64\bin;D:\msys64\usr\bin;" + $env:PATH
cd "<chemin_du_dépôt>\Wall-A-STM\Tests"
mkdir build            # si le dossier n'existe pas encore
cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Debug
ninja
.\BusFormatTest.exe    # un exécutable par suite
```

Après une modification de `.cpp` ou `.h`, relancer simplement `ninja` : CMake détecte les fichiers modifiés, sauf si `CMakeLists.txt` change (relancer alors `cmake .. -G Ninja`). Résultat attendu pour chaque exécutable :

```
[==========] Running N tests from 1 test suite.
[  PASSED  ] N tests.
```

### Ajouter une suite

1. Créer `Tests/Unit/MonTest.cpp` avec les tests GTest.
2. Dans `Tests/CMakeLists.txt`, déclarer l'exécutable :
   ```cmake
   add_executable(MonTest
       Unit/MonTest.cpp
       ${APP_DIR}/MonModule.cpp
   )
   target_include_directories(MonTest PRIVATE ${COMMON_INCLUDES})
   target_link_libraries(MonTest PRIVATE GTest::gtest_main)
   gtest_discover_tests(MonTest)
   ```
3. Ajouter son nom à la liste `TESTS` de `run_tests.sh`.
4. Relancer `cmake .. -G Ninja` puis `ninja`.

### Dépannage

| Symptôme | Cause probable | Solution |
|---|---|---|
| `cmake: command not found` | cmake non installé | `"D:\msys64\mingw64.exe" pacman -S --noconfirm mingw-w64-x86_64-cmake` |
| `fatal error: gtest/gtest.h: No such file` | Première configuration sans internet | Vérifier la connexion, supprimer `build/_deps/` et relancer `cmake ..` |
| `undefined reference` à la compilation | Nouveau membre statique non défini | Ajouter la définition dans `Tests/Stubs/StaticDefs.cpp` |
| Un test ne se termine pas | Appel direct à une tâche FreeRTOS sans `try/catch` | Entourer l'appel de `try { ... } catch (const TaskDelayEscape&) {}` |
| Avertissement `DOWNLOAD_EXTRACT_TIMESTAMP` | CMake ≥ 3.24 | Ignorable, n'affecte pas le build |

## Annexe — Configuration matérielle STM32

Périphériques initialisés dans `Core/Src/main.c` (générés par CubeMX depuis `Wall-A-STM.ioc`). Horloge cœur : HSE 25 MHz, PLL à 168 MHz.

### ADC

Les trois ADC sont en 12 bits, conversion unique, déclenchement logiciel. Le driver `Adc` change de canal à l'exécution selon le capteur lu.

| Périphérique | Canaux utilisés par l'application | Capteurs |
|---|---|---|
| ADC1 | 3, 4, 6, 8 | Courants moteurs |
| ADC2 | 12, 10, 13, 9 | Proximité B5WLB2101 |
| ADC3 | 4, 5, 6 | Températures |

### Timers

| Timer | Mode | Canaux | Usage |
|---|---|---|---|
| TIM1 | PWM | CH1, CH3 | Moteurs de roues (Drv8262) |
| TIM2 | PWM 32-bit | CH1, CH2, CH3, CH4 | — |
| TIM3 | Input Capture | CH1, CH2, CH3, CH4 | Capteurs Pololu5472 |
| TIM4 | Encodeur quadrature (TI12) | CH1, CH2 | Encodeur gauche |
| TIM5 | PWM 32-bit | CH1, CH2 | — |
| TIM7 | Base de temps (prescaler 839, période 65535) | — | — |
| TIM8 | Encodeur quadrature (TI12) | CH1, CH2 | Encodeur droit |

### Liaisons série et bus

| Périphérique | Configuration | Usage |
|---|---|---|
| USART1 | 250000 bauds, TX/RX | Canal UART de `ExternalComm` |
| USART2 | 115200 bauds, TX/RX | — |
| USART6 | 115200 bauds, TX/RX | — |
| USB Device (CDC) | Port série virtuel | Canal USB de `ExternalComm` |
| I2C1 | 100 kHz, adressage 7 bits | — |
| SPI2 | Maître, 8-bit, mode 0 (CPOL=0, CPHA=0), prescaler 2 | — |
| LwIP | IP statique 192.168.0.123 | Configuré, aucun canal branché |

### GPIO

| Signal | Direction | Rôle |
|---|---|---|
| LED_ORANGE/YELLOW/GREEN/BLUE/RED | Sortie | LEDs de statut ×5 (la rouge clignote via la tâche `Blink`) |
| WHEEL_DIR1/2 | Sortie | Direction roues ×2 |
| SECONDARY_MOTOR_IN1–4 | Sortie | Direction moteur secondaire ×4 |
| PRI_MOTOR_SLEEP | Sortie | Mise en veille moteur primaire |
| PRI_MOTOR_NFAULT | Entrée | Détection défaut moteur primaire |
| SENSOR_PULSE_1–4 | Sortie | Impulsions capteurs ×4 |
| ENABLE_POWER_SUPPLIES | Sortie | Activation alimentation (activée en fin d'initialisation dans `cppMain`) |
| PGOOD_24V | Entrée | Surveillance alimentation 24V |
| USER_SWITCH_1–4 | Entrée | Boutons utilisateur ×4 |
| SWITCH_1–4 | Entrée | Boutons de configuration ×4 |
| SERVO_ROBOTIS_TX_ENABLE | Sortie | Enable TX half-duplex Robotis |
| ETH_OSC_OUT | Sortie (MCO) | Horloge 25 MHz vers PHY Ethernet |

### FreeRTOS

Tas de 32 Ko, tick à 1 kHz. `defaultTask` (priorité Normal, pile 4 Ko) est créée par CubeMX et appelle `cppMain()`, qui crée les tâches de l'application puis se supprime.

## Documentation

| Document | Contenu |
|---|---|
| [README_TUNING.md](README_TUNING.md) | Guide de tuning de la vitesse linéaire et angulaire |
| [pythonTester/README.md](pythonTester/README.md) | Moniteur série Python |
| [architecture.md](_bmad-output/planning-artifacts/architecture.md) | Décisions d'architecture, conventions, graphes de dépendances |
| [prd.md](_bmad-output/planning-artifacts/prd.md) · [epics.md](_bmad-output/planning-artifacts/epics.md) | Besoins et découpage en epics |
| [implementation-artifacts/](_bmad-output/implementation-artifacts/) | Stories d'implémentation et travail différé |

## Licence

[MIT](LICENSE)
