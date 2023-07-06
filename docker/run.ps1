<#
.SYNOPSIS
Runs the ros noetic image in a container

.DESCRIPTION
USAGE
    .\docker\run.ps1 [options]
  
OPTIONS
    -ImageName      <string>    The name of the image to be run
                                (default: ros_noetic_arm)
    -ContainerName  <string>    The name of the container created
                                (default: ros_noetic_container_arm)
    -UseNvidia      <bool>      Use nvidia runtime
                                (default: false)
    -Display        <string>    Display address
                                (default: [local ip]:0)
#>

param (
  [Parameter(Mandatory = $false)]
  [string]$ImageName = "ros_noetic_arm",
  [Parameter(Mandatory = $false)]
  [string]$ContainerName = "ros_noetic_container_arm",
  [Parameter(Mandatory = $false)]
  [switch]$UseNvidia = $false,
  [Parameter(Mandatory = $false)]
  [string]$Display = "$((Get-NetIPAddress -AddressFamily IPv4 -InterfaceAlias "vEthernet (WSL)").IPAddress):0"
)

$root = (Get-Item -Path $PSScriptRoot).Parent

$nvidiaFlags = @()
if ($UseNvidia) {
  $nvidiaFlags = @("-e", "NVIDIA_VISIBLE_DEVICES=all", "--gpus", "all")
}

$repository = $root.FullName
$workspaceContainer = "/home/unlp/ws/src/$($root.BaseName)"


Write-Host "Running container '$ContainerName' from image '$ImageName'..."
docker run -it $nvidiaFlags `
  -e DISPLAY="$Display" `
  --mount "type=bind,source=${repository},target=${workspaceContainer}" `
  --name $ContainerName "$ImageName"
#--device /dev/dri `

$decision = $Host.UI.PromptForChoice(
  "Overwrite image",
  "Do you want to overwrite the image called '$ImageName' with the current changes?",
  @("&Yes", "&No"),
  1
)
if ($decision -eq 0) {
  docker commit $ContainerName $ImageName
}
docker rm $ContainerName
