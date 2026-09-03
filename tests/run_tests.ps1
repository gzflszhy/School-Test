$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
python "$PSScriptRoot/generate_dataset.py" --out "$PSScriptRoot/fixtures" --count 48 --seed 7
python "$PSScriptRoot/evaluate.py" --dataset "$PSScriptRoot/fixtures"
if (Get-Command cmake -ErrorAction SilentlyContinue) { cmake -S "$PSScriptRoot" -B "$PSScriptRoot/build"; cmake --build "$PSScriptRoot/build"; ctest --test-dir "$PSScriptRoot/build" --output-on-failure }
