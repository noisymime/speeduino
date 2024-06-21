<#
    .SYNOPSIS
    Runs CppCheck against the Speeduino source code

    .DESCRIPTION
    Ideally we'd just run cppcheck on the entire source tree, with the MISRA add on.
    E.g.
        cppcheck --addon=misra.py <path to speeduino source>
    
    However, at a minimum we need to customize the arguments to the MISRA add on and,
    in order to get relevant results, set a number of command line flags. Hence this script.

    .INPUTS
    None. 

    .OUTPUTS
    The count of mandatory or required MISRA rule violations.

    In addition to writing the results to file, the script writes the results to the Powershell Information 
    stream, with the tag "CPPCHECK-RESULT". The script also supports the Debug & Progress streams. Use the 
    standard PS stream configuration options.

    .EXAMPLE
    PS> .\check_misra.ps1 -c 'C:\Program Files\Cppcheck\'
    2000
    PS>

    Minimal parameters: scans the deafult location, displays progress and outputs a text file (results.txt) to the default location

    .EXAMPLE
    PS> .\check_misra.ps1 -c 'C:\Program Files\Cppcheck\' -x -InformationAction Continue
    # A whole ton of XML output.....
    2000
    PS>

    XML output and echoes the XML to the terminal.
    
    .LINK
    about_Output_Streams
#> 

param (
    # Path the folder where cppcheck is installed
    # We expect a 'cppcheck' executable in this folder
    [Alias("c")]
    [Parameter(Mandatory=$true, Position = 0)]
    [string]$CppcheckPath,
    # Path to the Speeduino source folder
    [Alias("s")]
    [PSDefaultValue(Help="<script path>/../speeduino")] 
    [Parameter()]
    [string]$SourceFolder = (Join-Path $PSScriptRoot ".." "speeduino"),
    # Path to a folder where the results will be written.
    # The folder will be created if necessary
    [Alias("o")]
    [PSDefaultValue(Help="<script path>/.results")] 
    [Parameter()]
    [string]$OutFolder = (Join-Path $PSScriptRoot ".results"),
    # When set, output the results in XML format
    # Useful for subsequent analysis that benfit from a structured format. E.g. PowerBi, Grafana
    [Alias("x")]
    [Parameter()]
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
    Get-Content -Path $jsonFile | ForEach-Object { Write-Debug("$($MyInvocation.MyCommand.Name) : $_") }
    
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
    $cppcheckParameters | ForEach-Object { Write-Debug("$($MyInvocation.MyCommand.Name) : $_") }
    
    $cppcheckParameters
}

 # Convert to absolute path - makes things easier with CppCheck filtering
$SourceFolder = Resolve-Path $SourceFolder

# Make sure output folder exists
New-Item -ItemType Directory -Force -Path $OutFolder | Out-Null

# Temporary JSON file to invoke the MISRA add-on
$jsonFile = New-MisraAddOnFile (Join-Path $PSScriptRoot "misra_2012_text.txt")
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
        
        # Normally cppcheck would ouput any findings to stderr. However, we have told it to write the results to a file.
        # The rest of the output *should* be progress messages, so send those to Write-Progress
        & $cppcheckBin $cppCheckParameters 2>&1 | ForEach-Object { Write-Progress -Activity $_ }

        $scanResults = Get-Content -Path $OutputFile

        # Send the CppCheck results to the information stream
        $scanResults | ForEach-Object { Write-Information -Tags "CPPCHECK-RESULT" $_ }

        # Count lines for Mandatory or Required rules
        $errorLines = $scanResults | Where-Object { $_ -match "Mandatory - |Required - " }

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
