# Bremspruefstand Simulator Export v2.1

Git-faehiger Export mit bereinigterer Projektpraesentation und exportorientierter Dokumentation.

## Umfang

- `P1`: mathematische Simulation mit deutscher Hauptoberflaeche
- `P2`: code-nahe TinkerForge-/ESP32-Simulation
- exportierbare Standalone-HTML fuer deutsch
- englische P1-Kopie
- Wokwi-/PlatformIO-Basisdateien

## Start

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\start-local-sim.ps1
```

## Struktur

- `simulation_analyse.html`
  Deutsche Hauptanwendung mit P1/P2.
- `bremspruefstand_analyse_standalone.html`
  Gespiegelte Standalone-Version.
- `simulation_analysis_en.html`
  Englische P1-Kopie.
- `simulation_config.json`
  Zentrale Modell- und Hardware-Defaults.
- `tools\p2_bridge_server.py`
  Python-Bridge fuer P2.
- `src\main.cpp`
  Firmware-/Regler-Grundlage.

## Technischer Stand

- P1 und P2 nutzen denselben Physik-Kern, aber getrennte Zustandscontainer.
- Logging ist standardmaessig deaktiviert.
- Die P2-Bridge ist auf kleinere Payloads und geringere Browserlast reduziert.
- Die Exportdatei ist fuer lokalen Start ohne weitere Build-Chain gedacht.

## Skill-bezogene Einordnung

- `frontend-skill`: bei der Exportpraesentation und Interface-Hierarchie beruecksichtigt
- `github`: lokaler git-faehiger Export vorbereitet
- `gh-fix-ci`: hier nicht anwendbar, da kein GitHub-Actions-Setup Teil des Exports ist
- `shadcn`: hier nicht aktiv, da kein `components.json`-Projekt vorliegt
