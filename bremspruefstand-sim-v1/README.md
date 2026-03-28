# Bremspruefstand Simulator Export v1

Lokaler, git-faehiger Export des aktuellen Bremspruefstand-Projekts.

## Inhalt

- `P1`: mathematische Bremspruefstand-Simulation
- `P2`: code-nahe TinkerForge-/ESP32-Messketten-Simulation
- Wokwi-/PlatformIO-Artefakte fuer die Firmwarebasis
- deutsche und englische HTML-Exports

## Start

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\start-local-sim.ps1
```

Der Starter spiegelt die Standalone-Datei, startet den lokalen HTTP-Server und versucht zusaetzlich die P2-Bridge zu starten.

## Relevante Dateien

- `simulation_analyse.html`: Hauptdatei der deutschen UI
- `bremspruefstand_analyse_standalone.html`: gespiegelt exportierbare Version
- `simulation_analysis_en.html`: englische P1-Kopie
- `simulation_config.json`: zentrale Defaults
- `tools\p2_bridge_server.py`: P2-Backend
- `src\main.cpp`: Firmware-Grundlage

## Hinweise

- Dieses Export-Repo hat absichtlich kein GitHub-Remote.
- `gh-fix-ci` ist auf diesen Export nicht direkt anwendbar, weil kein Actions-Kontext mitgeliefert wird.
- `shadcn` ist fuer dieses Repo nicht aktiv, da kein `components.json` vorhanden ist.
