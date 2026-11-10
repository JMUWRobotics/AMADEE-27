 # Mein Rover-Protokoll 
```mermaid 
sequenceDiagram
    autonumber
    actor Rover as 🤖 Rover
    participant CS as 🎛️ Charging Station

    %% 1. Anfrage-Phase
    Rover->>CS: Lade-Anfrage (ID, Soll-Werte)
    Note over CS: Prüft freie Slots & Solarleistung
    
    alt Passender Ladeslot verfügbar
        CS-->>Rover: Bestätigung & Slot-Zuweisung
        Note over Rover: Navigation zum Ladeadapter
    else Keine Kapazität frei
        CS-->>Rover: Ablehnung / Warten
        Note over Rover,CS: Rover bricht ab / fährt weiter
    end

    %% 2. Lade-Phase (Loop mit Heartbeat)
    loop Während des Ladens (alle 5s)
        Rover->>CS: Heartbeat / Live-Status
        
        alt Paket erfolgreich empfangen
            CS-->>Rover: ACK (OK)
        else Timeout / Keine Antwort (>15s)
            Note over CS: ⚠️ Timer abgelaufen!
            CS->>CS: Notabschaltung (Strom auf 0)
            Note over Rover,CS: Verbindungsabbruch erkannt
        end
    end

    %% 3. Beendigungs-Phase
    alt Reguläres Ende durch Rover
        Rover->>CS: Ladevorgang beendet (Complete)
        Note over Rover: Gibt Ladeslot frei
        CS-->>Rover: ACK & Port deaktivieren
    else Not-Aus / Ende durch Station
        CS->>Rover: Ladevorgang beenden
        Note over Rover: Trennt Verbindung
        Rover->>CS: ACK
        CS->>CS: Port stromlos schalten
    end