$ErrorActionPreference = "Stop"
$ProjectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -LiteralPath $ProjectDir

python validate_setup.py --config config.starter.json
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

python -m unittest discover -s . -p "test_*.py" -v
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

python run_keyboard_mission.py --config config.starter.json --simulate --key QAZ
exit $LASTEXITCODE
