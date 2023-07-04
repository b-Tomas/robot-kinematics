<#
.SYNOPSIS
Builds the docker image for ros noetic development
  
.DESCRIPTION
USAGE
    .\docker\windows.ps1 build [options]
    
OPTIONS
    -ImageName      <string>    The name of the image to build
                                (default: ros_noetic_arm)
    -help                       Shows this help message.                    
#>

param (
  [Parameter(Mandatory = $false)]
  [string]$ImageName = "ros_noetic_arm"
)

$root = (Get-Item -Path $PSScriptRoot).Parent.FullName

# Converts the line endings of a file to LF.
# This is a workaround for the fact that Linux containers break
# apart when they encounter CRLF line endings.
# See https://stackoverflow.com/a/48919146/9950655
function Convert-EOL {
  param (
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$file
  )
  ((Get-Content $file) -join "`n") + "`n" | Set-Content -NoNewline $file
}

Convert-EOL "$root\docker\install.sh"
Convert-EOL "$root\docker\requirements.txt"
Convert-EOL "$root\docker\requirements_ros.txt"

Write-Host "Building image '$ImageName'..."
docker build -t $ImageName --file .\docker\Dockerfile --build-arg USERID=1000 --build-arg USER=unlp $root
