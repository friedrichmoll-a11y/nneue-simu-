$ErrorActionPreference = "Stop"

param(
  [string]$RemoteUrl
)

if (-not $RemoteUrl) {
  $RemoteUrl = Read-Host "GitHub Remote URL fuer v1 eingeben"
}

if (-not $RemoteUrl) {
  throw "Keine Remote URL angegeben"
}

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $repoRoot

git rev-parse --is-inside-work-tree | Out-Null
git branch -M main

$hasOrigin = $false
try {
  git remote get-url origin | Out-Null
  $hasOrigin = $true
} catch {
  $hasOrigin = $false
}

if ($hasOrigin) {
  git remote set-url origin $RemoteUrl
} else {
  git remote add origin $RemoteUrl
}

git push -u origin main
git push origin v1

Write-Host "v1 nach GitHub gepusht." -ForegroundColor Green
