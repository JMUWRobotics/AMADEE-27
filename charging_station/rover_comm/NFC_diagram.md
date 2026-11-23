 # NFC-Diagram
```mermaid 
sequenceDiagram
    autonumber
    participant NFC as NFC
    participant CS as 🎛️ Charging Station
   
    %% 1. Anfrage-Phase
    Note over CS,NFC: Start durch NFC-Tag
    NFC->>CS: Lade-Anfrage (Rover-ID, Spannung, Stromstärke, An)

    Note over CS: Spannung eingestellt und kleine Stromstärke ca. (0,1 A)
    Note over NFC: Rover ranschieben
    loop Bis connection vorhanden, oder Abbruch
    Note over CS: 20s warten, Verbindung überprüfen
        alt Verbindung vorhanden
            Note over CS: Strom hochziehen, Signallicht
        else Keine Verbindung
            Note over CS: Signallicht
        end
    end



    %% 2. Beendigungs-Phase
    Note over CS,NFC: Ende durch NFC-Tag
        NFC->>CS: Beenden(Rover-ID, Aus)
        Note over CS: Stromstärke runter (0,1 A)
        Note over NFC: Rover wegschieben
        Note over CS: 20 s waten, Port aus, Sitzung beenden

