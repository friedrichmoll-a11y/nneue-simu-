# Push nach GitHub

## Neues Remote anlegen

1. Neues leeres Repository auf GitHub erzeugen.
2. Dann lokal im Exportordner:

```powershell
powershell -ExecutionPolicy Bypass -File .\publish_to_github.ps1 -RemoteUrl <DEIN-REPO-URL>
```

## Optional

- Vor dem Push den Startpfad lokal pruefen:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\start-local-sim.ps1
```
