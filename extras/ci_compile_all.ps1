param(
    [ValidateSet("All", "Core", "Rover", "Mfs")]
    [string]$Target = "All",
    [string]$ArduinoCli = ""
)

$ErrorActionPreference = "Stop"
$CoreRoot = Split-Path -Parent $PSScriptRoot
$ArduinoRoot = Split-Path -Parent (Split-Path -Parent $CoreRoot)
$RoverDir = Join-Path $ArduinoRoot "RTK_Rover"
$MfsDir = Join-Path $ArduinoRoot "Magnetic_Field_Scanner"

if ([string]::IsNullOrWhiteSpace($ArduinoCli)) {
    $cmd = Get-Command "arduino-cli.exe" -ErrorAction SilentlyContinue
    if ($cmd) {
        $ArduinoCli = $cmd.Source
    } else {
        $pfCli = Join-Path $env:ProgramFiles "Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
        if (Test-Path $pfCli) {
            $ArduinoCli = $pfCli
        }
    }
}

if ([string]::IsNullOrWhiteSpace($ArduinoCli) -or -not (Test-Path $ArduinoCli)) {
    Write-Error "arduino-cli.exe not found. Pass -ArduinoCli <path>."
    exit 1
}

$Fqbn = "esp32:esp32:esp32s3:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB"

function Invoke-SketchCompile {
    param(
        [string]$Label,
        [string]$SketchPath
    )
    if (-not (Test-Path $SketchPath)) {
        Write-Warning "Skipping $Label (not found at $SketchPath)"
        return
    }
    Write-Host "=== Compiling $Label ($SketchPath) ==="
    $out = & $ArduinoCli compile --fqbn $Fqbn $SketchPath --warnings default 2>&1
    $exitCode = $LASTEXITCODE
    $out | ForEach-Object { Write-Host $_ }
    if ($exitCode -ne 0) {
        throw "Compilation failed for $Label (exit $exitCode)"
    }
    $text = $out -join "`n"
    if ($text -match "warning:") {
        throw "Compiler warning detected during $Label build"
    }
    if ($text -match "Global variables use (\d+) bytes \((\d+)%\)") {
        $dramPct = [int]$Matches[2]
        if ($dramPct -gt 25) {
            throw "Static DRAM0 exceeded 25% ($dramPct%) for $Label"
        }
    }
}

Push-Location $CoreRoot
try {
    python scripts/check_secrets.py
    if ($LASTEXITCODE -ne 0) { throw "check_secrets.py failed" }

    python extras/unit_tests/run_tests.py
    if ($LASTEXITCODE -ne 0) { throw "run_tests.py failed" }

    if ($Target -eq "All" -or $Target -eq "Core") {
        Invoke-SketchCompile -Label "CoreSmoke_35B" -SketchPath (Join-Path $CoreRoot "examples\CoreSmoke_35B")
    }
    if ($Target -eq "All" -or $Target -eq "Rover") {
        Invoke-SketchCompile -Label "RTK_Rover" -SketchPath $RoverDir
    }
    if ($Target -eq "All" -or $Target -eq "Mfs") {
        Invoke-SketchCompile -Label "Magnetic_Field_Scanner" -SketchPath $MfsDir
    }
} finally {
    Pop-Location
}

Write-Host "[PASS] CI verification complete ($Target)."
