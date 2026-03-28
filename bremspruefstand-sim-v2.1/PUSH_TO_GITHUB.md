# Push nach GitHub

## Neues Remote anlegen

1. Neues leeres Repository auf GitHub erzeugen.
2. Danach im Exportordner:

```powershell
powershell -ExecutionPolicy Bypass -File .\publish_to_github.ps1 -RemoteUrl <DEIN-REPO-URL>
```

## Minimaler Check vor dem Push

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\start-local-sim.ps1
```

## Hinweis

Wenn spaeter echte GitHub-Actions-Checks vorhanden sind, ist `gh-fix-ci` der passende Folge-Workflow. In diesem Export ist dafuer noch keine CI-Definition enthalten.
