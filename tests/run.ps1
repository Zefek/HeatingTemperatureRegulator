$root = Split-Path $PSScriptRoot -Parent
$out = Join-Path $env:TEMP "htr_tests"
New-Item -ItemType Directory -Force $out | Out-Null
$exe = Join-Path $out "mode_test.exe"
$sources = @("$root\tests\mode_test.cpp", "$root\mode.cpp")

if (Get-Command g++ -ErrorAction SilentlyContinue) {
    g++ -std=c++17 -Wall -Wextra -Werror -o $exe @sources
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $exe
    exit $LASTEXITCODE
}

$devCmd = Get-ChildItem "C:\Program Files\Microsoft Visual Studio\*\*\Common7\Tools\VsDevCmd.bat" -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
if (-not $devCmd) { Write-Error "Neni k dispozici g++ ani Visual Studio s C++"; exit 1 }
$script = Join-Path $out "build.cmd"
@"
@echo off
call "$devCmd" -arch=x64 -no_logo || exit /b 1
cl /nologo /EHsc /std:c++17 /W4 /WX /Fo"$out\\" /Fe"$exe" "$($sources[0])" "$($sources[1])" || exit /b 1
"$exe"
"@ | Set-Content -Path $script -Encoding ascii
cmd /c $script
exit $LASTEXITCODE
