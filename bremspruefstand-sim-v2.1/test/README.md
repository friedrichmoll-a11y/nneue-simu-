# PlatformIO Tests

Dieser Ordner existiert, damit PlatformIOs Test-Ansicht kein `TestDirNotExistsError`
mehr meldet.

Aktuell liegt der Fokus dieses Projekts auf:

- Wokwi-Simulation
- PlatformIO-Build
- spaeterem ESP32-Hardwarebetrieb

Es gibt im Moment noch keine echten Unit- oder Integrationstests.

Wenn wir spaeter Tests nachziehen wollen, sind sinnvolle Richtungen:

- Host-seitige Rechenlogik auslagern und mit `platform = native` testen
- serielle Kommandologik gezielt pruefen
- Grenzwerte fuer Temperatur, Moment und Freigabe als reine Funktionslogik testen
