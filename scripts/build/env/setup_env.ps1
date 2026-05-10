# Version: 1.0

function setup_env {
    param (
        [string] $zephyr_root
    )
    
    # Set up the ZEPHYR_VERSION and ZEPHYR_ROOT environment variables; read from the Zephyr Installation
    # directory.
    $env:ZEPHYR_VERSION = (Get-Content -Path "$zephyr_root\VERSION" -Raw).Trim()
    $env:ZEPHYR_BASE = $zephyr_root
    
    # Add the Zephyr tools to the PATH environment variable.
    $env:PATH = "$zephyr_root\zephyr\scripts;$zephyr_root\zephyr\scripts\west;$zephyr_root\zephyr\scripts\west\zephyr;$env:PATH"
}

setup_env($args[0])
# activate the venv for the Zephyr build environment
& $env:ZEPHYR_BASE\..\.venv\Scripts\Activate.ps1

# print the environment variables
Get-ChildItem Env:ZEPHYR_VERSION    
Get-ChildItem Env:ZEPHYR_BASE
Get-ChildItem Env:PATH
