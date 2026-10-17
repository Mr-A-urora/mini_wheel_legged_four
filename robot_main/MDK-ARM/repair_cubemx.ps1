$ErrorActionPreference = 'Stop'
$projectPath = Join-Path $PSScriptRoot 'robot_main.uvprojx'
$portPath = './FreeRTOS/ARM_CM4F_AC6'
foreach ($name in @('port.c', 'portmacro.h')) {
    if (-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot "FreeRTOS/ARM_CM4F_AC6/$name"))) {
        throw "Missing AC6 port file: $name"
    }
}
$content = [System.IO.File]::ReadAllText($projectPath)
# CubeMX may reinsert the RVDS port alongside the custom AC6 port.
$state = @{ Count = 0 }
$content = [regex]::Replace($content, '(?s)<File>\s*<FileName>port\.c</FileName>.*?</File>', {
    param($match)
    if ($match.Value -notmatch '<FilePath>(?:\./FreeRTOS/ARM_CM4F_AC6|\.\./Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F)/port\.c</FilePath>') {
        return $match.Value
    }
    $state.Count++
    if ($state.Count -gt 1) { return '' }
    return [regex]::Replace($match.Value, '<FilePath>.*?</FilePath>', "<FilePath>$portPath/port.c</FilePath>")
})
if ($state.Count -eq 0) { throw 'No known FreeRTOS port entry found; project was not modified.' }
$content = [regex]::Replace($content, '<IncludePath>(.*?)</IncludePath>', {
    param($match)
    $paths = $match.Groups[1].Value -split ';'
    $paths = @($paths | ForEach-Object {
        if ($_ -eq '../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F') { $portPath } else { $_ }
    } | Select-Object -Unique)
    return '<IncludePath>' + ($paths -join ';') + '</IncludePath>'
})
$content = $content.Replace('..\Drivers\CMSIS\DSP\Lib\ARM\arm_cortexM7lfdp_math.lib', '.\Lib\ARM\arm_cortexM7lfdp_math.lib')
[System.IO.File]::WriteAllText($projectPath, $content, (New-Object System.Text.UTF8Encoding($false)))
Write-Output 'Repaired FreeRTOS AC6 port references. Reopen the project in Keil before building.'
