$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$pythonCmd = Get-Command python -ErrorAction SilentlyContinue
if (-not $pythonCmd) {
  $pythonCmd = Get-Command py -ErrorAction SilentlyContinue
}
if (-not $pythonCmd) {
  throw "No Python executable found in PATH"
}

if ($pythonCmd.Name -ieq "py.exe") {
  $python = "py"
  $pythonArgs = @("-3")
} else {
  $python = (& python -c "import sys; print(sys.executable)")
  if (-not $python) {
    throw "Could not resolve concrete python executable"
  }
  $python = $python.Trim()
  $pythonArgs = @()
}

$sourceAnalysis = Join-Path $root "simulation_analyse.html"
$standaloneAnalysis = Join-Path $root "bremspruefstand_analyse_standalone.html"

function Stop-StalePythonProcesses {
  param(
    [string[]]$Patterns
  )
  $pythonProcs = Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^python(|w)?(\.exe)?$' -or $_.Name -match '^py(|thon)?(\.exe)?$'
  }
  foreach ($proc in $pythonProcs) {
    $cmd = [string]$proc.CommandLine
    if (-not $cmd) { continue }
    if ($Patterns | Where-Object { $cmd -match [regex]::Escape($_) }) {
      try {
        Stop-Process -Id $proc.ProcessId -Force -ErrorAction Stop
      } catch {
      }
    }
  }
}

if (-not (Test-Path -LiteralPath $sourceAnalysis)) {
  throw "Source analysis file not found: $sourceAnalysis"
}

Copy-Item -LiteralPath $sourceAnalysis -Destination $standaloneAnalysis -Force

$analysisCandidates = @(
  $standaloneAnalysis,
  $sourceAnalysis
)
$analysis = $null
foreach ($candidate in $analysisCandidates) {
  if (Test-Path -LiteralPath $candidate) {
    $analysis = $candidate
    break
  }
}

if (-not $analysis) {
  throw "No analysis HTML file found in $root"
}

Write-Host "Preparing local simulation..." -ForegroundColor Cyan
Stop-StalePythonProcesses -Patterns @("simulator_auto.py", "tools\\p2_bridge_server.py", "-m http.server")

Write-Host "Starting local web server..." -ForegroundColor Cyan
$listener = [System.Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback, 0)
$listener.Start()
$port = ($listener.LocalEndpoint).Port
$listener.Stop()

$serverArgs = $pythonArgs + @("-m", "http.server", "$port", "--bind", "127.0.0.1")
Start-Process -FilePath $python -ArgumentList $serverArgs -WorkingDirectory $root

$analysisName = [System.IO.Path]::GetFileName($analysis)
$url = "http://127.0.0.1:$port/${analysisName}?v=$([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds())"
$ok = $false
for ($i = 0; $i -lt 20; $i++) {
  Start-Sleep -Milliseconds 200
  try {
    $response = Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec 2
    if ($response.StatusCode -eq 200) {
      $ok = $true
      break
    }
  } catch {
  }
}

if (-not $ok) {
  throw "Local web server did not become ready for $url"
}

Write-Host "Using analysis file: $analysisName" -ForegroundColor DarkCyan
Write-Host "Local URL: $url" -ForegroundColor DarkCyan
Write-Host "Opening analysis view..." -ForegroundColor Cyan
Start-Process $url

Write-Host "Local simulation started." -ForegroundColor Green
