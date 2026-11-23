 # Wifi-Diagram 
```mermaid 
sequenceDiagram
    autonumber
    participant Rover as 🤖 Rover
    participant CS as 🎛️ Charging Station
   
    %% 1. Anfrage-Phase
    Note over CS,Rover: Start durch Rover
    Rover->>CS: Lade-Anfrage (Rover-ID, Spannung, Stromstärke)
    Note over CS: Prüft freie Slots
    
    alt Passender Ladeslot verfügbar
        CS-->>Rover: Bestätigung, Slot-Zuweisung, Dict(Arucu ID, Transformation zum Charging Adapter)
        Note over CS: Spannung eingestellt und kleine Stromstärke ca. (0,1 A)
        loop Bis connection vorhanden, oder Abbruch
        Note over Rover: Navigation zum Ladeadapter
        Rover-->> CS: Angedockt
        Note over CS: Überprüfen, ob Connection vorhanden
            alt Connection vorhanden
                Note over CS: Strom hochziehen
                CS -->> Rover: Bestätigung
            else Connection nicht vorhanden
                CS -->> Rover: Reject
                Note over Rover: Erneuter versuch
            end
        end
    else Keine Kapazität frei
        CS-->>Rover: Ablehnung / Warten
        Note over Rover,CS: Rover bricht ab / fährt weiter
    end

    %% 2. Beendigungs-Phase
    Note over CS,Rover: Ende durch Rover
        Rover->>CS: Ladevorgang beendet (Complete)
        Note over CS: Stromstärke runter (0,1 A)
        Note over Rover: Fährt weg
        Rover-->>CS: Bestätigung, dass er weg ist
        Note over CS: Port aus, Sitzung beenden
        CS-->>Rover: ACK
