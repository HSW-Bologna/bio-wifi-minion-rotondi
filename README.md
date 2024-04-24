# BIO WiFi Minion

Firmware per la scheda BIO (?) per il sistema cassa rotondi.

## Compilazione

Il firmware dovrebbe essere compilato con i seguenti strumenti:
 - ESP-IDF v4.4.x (testato con v4.4.6)

## Struttura del Progetto

Il progetto e' strutturato secondo il paradigma Model-View-Controller (o almeno la mia interpretazione).

Il Model contiene tutti i dati dell'applicazione in un'unica `struct`; non ha dipendenze e gestisce soltanto i dati.

La View gestisce tutto cio' che e' correlato al display: e' divisa in pagine che si impilano durante l'esecuzione.
Ogni pagina e' costituita da callback che ne descrivono il ciclo di vita (creazione, apertura, gestione degli eventi, chiusura e distruzione) durante il quale possono mostrare informazioni sullo schermo, modificare il modello o inviare messaggi al Controller in risposta a interazioni dell'utente.

Il Controller gestisce tutto il resto; la colla tra i componenti e l'interazione con l'hardware.

## Note

Il protocollo di comunicazione e' confuso e noto a nessuno; per i dettagli che lo riguardano l'unica fonte attendibile e' il software.

La comunicazione in se' e' instabile a dir poco, a causa di un mancato pull-up sulle linee della 485.
