param (
    [Alias("s")]
    [string]$SourceFolder = "$PSScriptRoot/../speeduino",
    [Alias("o")]
    [string]$OutFolder = "$PSScriptRoot/.results",
    [Alias("c")]
    [Parameter(Mandatory=$true)]
    [string]$CppcheckPath,
    [Alias("q")]
    [switch]$Quiet=$false,
    [Alias("x")]
    [switch]$OutputXml=$false
)

function New-MisraAddOnFile {
    param (
        [Parameter(Mandatory)]
        [string]$RuleTextFilePath
    )    
    # Location of the add on json file. cppcheck doesnt recognize it unless it has a .json extension :-(
    $jsonFile = New-TemporaryFile | Rename-Item -NewName { [IO.Path]::ChangeExtension($_, ".json") } -PassThru

    # Write CPP Check add on JSON file
    $jsonContents = @{
        script = "misra.py"
        args = @("--rule-texts=$RuleTextFilePath")
    }
    $jsonContents | ConvertTo-Json -Depth 99 | Out-File -FilePath $jsonFile
    
    Write-Debug("$($MyInvocation.MyCommand.Name) : $jsonFile")
    $jsonFile
}

function Get-SuppressFolders {
    param (
        [Parameter(Mandatory)]
        [string]$SourceFolder
    )
    # Folders to skip scanning
    @(
        (Join-Path $SourceFolder "src" "BackupSram")
        (Join-Path $SourceFolder "src" "FastCRC")
        (Join-Path $SourceFolder "src" "FlashStorage")
        (Join-Path $SourceFolder "src" "FRAM")
        (Join-Path $SourceFolder "src" "HardwareTimers")
        (Join-Path $SourceFolder "src" "libdivide")
        (Join-Path $SourceFolder "src" "PID_v1")
        (Join-Path $SourceFolder "src" "SPIAsEEPROM")
        (Join-Path $SourceFolder "src" "STM32_CAN")
    )    
}

function Get-CppCheckOutputFile {
    param(
        [Parameter(Mandatory)]
        [string]$OutFolder,
        [bool]$OutputXml = $false
    )

    # XML or text output
    if ($OutputXml) {
        $Path = Join-Path $OutFolder "results.xml"
    } else {
        $Path = Join-Path $OutFolder "results.txt"
    }

    Write-Debug("$($MyInvocation.MyCommand.Name) : $Path")
    $Path
}

function Get-CppCheckParameters {
    param (
        [Parameter(Mandatory)]
        [string]$SourceFolder,
        [Parameter(Mandatory)]
        [string]$OutFolder,
        [string]$SuppressionFile,
        [bool]$OutputXml = $false,
        [string[]]$SuppressFolders = @(),
        [string]$AddOnJsonFile
    )    

    $cppcheckParameters = @("--inline-suppr"
                            "--language=c++"
                            "--enable=warning"
                            "--enable=information"
                            "--enable=performance"
                            "--enable=portability"
                            "--enable=style"
                            "--addon=$AddOnJsonFile"
                            "--suppressions-list=$SuppressionFile"
                            "--suppress=unusedFunction:*"
                            "--suppress=missingInclude:*"
                            "--suppress=missingIncludeSystem:*"
                            "--suppress=unmatchedSuppression:*"
                            "--suppress=cstyleCast:*"
                            "--platform=avr8"
                            "--cppcheck-build-dir=$OutFolder"
                            # Crude but good enough
                            ("-j " + ([Environment]::ProcessorCount-1))
                            "-DCORE_AVR=1"
                            "-D__AVR_ATmega2560__"
                            "-DARDUINO_AVR_MEGA2560" 
                            "-DF_CPU=16000000L"
                            "-DARDUINO_ARCH_AVR"
                            "-DARDUINO=10808"
                            "-DAVR=1"
                            # This is defined in the AVR headers which aren't included.
                            # cppcheck will not do type checking on unknown types.
                            # It's used a lot and it's unsigned which can trigger a lot
                            # of type mismatch violations.
                            "-Dbyte=uint8_t"
                            # No libdivide - analysis takes too long
                            "-UUSE_LIBDIVIDE"
                            # Output format
                            $outputFormat
                            "$SourceFolder/**.ino"
                            "$SourceFolder/**.cpp"
                        )  +
                        # Add on the folders to skip
                        @($SuppressFolders | ForEach-Object { "--suppress=*:$_" }) +
                        @($SuppressFolders | ForEach-Object { "-i $_" })

    if ($OutputXml) {
        $cppcheckParameters += @("--xml")
    }
    Write-Debug("$($MyInvocation.MyCommand.Name) : $cppcheckParameters")
    
    $cppcheckParameters
}

 # Convert to absolute path - makes things easier with CppCheck filtering
$SourceFolder = Resolve-Path $SourceFolder

# Make sure output folder exists
New-Item -ItemType Directory -Force -Path $OutFolder | Out-Null

# Temporary JSON file to invoke the MISRA add-on
$jsonFile = New-MisraAddOnFile "$PSScriptRoot/misra_2012_text.txt"
try {   
    # There is no way to tell the misra add on to skip certain headers
    # libdivide adds 10+ minutes to each file so rename the folder 
    # before the scan
    $libDivideFolderOrig = Join-Path $SourceFolder "src" "libdivide"
    $libDivideFolderTmp = Join-Path $SourceFolder "src" "_libdivide"
    Write-Debug "Renaming libdivide folder"
    Rename-Item $libDivideFolderOrig $libDivideFolderTmp

    try {
        $cppCheckParameters = Get-CppCheckParameters `
                                -SourceFolder $SourceFolder `
                                -OutFolder $OutFolder `
                                -SuppressionFile (Join-Path $PSScriptRoot "suppressions.txt") `
                                -OutputXml $OutputXml `
                                -AddOnJsonFile $jsonFile `
                                -SuppressFolders (Get-SuppressFolders $SourceFolder)

        $OutputFile = Get-CppCheckOutputFile $OutFolder $OutputXml
        $cppCheckParameters += "--output-file=$OutputFile"

        $cppcheckBin = Join-Path $CppcheckPath "cppcheck"
        Write-Debug "$($MyInvocation.MyCommand.Name): executing $cppcheckBin $cppCheckParameters"
        
        & $cppcheckBin $cppCheckParameters

        $scanResults = Get-Content -Path $OutputFile

        # Count lines for Mandatory or Required rules
        $errorLines = $scanResults | Where-Object { $_ -match "Mandatory - |Required - " }

        if (-not $Quiet) {
            $scanResults
        }

        $errorLines.count
    }
    finally {
        Write-Debug "Restoring libdivide folder"
        Rename-Item $libDivideFolderTmp $libDivideFolderOrig
    }
}
finally {
    Write-Debug "Removing temporary file $jsonFile"
    Remove-Item -Path $jsonFile -Force -ErrorAction SilentlyContinue
}
