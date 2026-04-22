param(
    [string[]]$Steps = @("probe", "msvc", "double", "format", "cppcheck", "complexity", "clang", "cross-arm", "cbmc"),
    [switch]$FailOnMissing
)

$ErrorActionPreference = "Stop"

$ExpandedSteps = @()
foreach ($entry in $Steps) {
    $ExpandedSteps += ($entry -split ",") | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
}
$Steps = $ExpandedSteps

function Resolve-ExistingPath {
    param([string[]]$Candidates)

    foreach ($candidate in $Candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    return $null
}

function Find-CommandPath {
    param([string[]]$Names, [string[]]$FallbackPaths = @())

    foreach ($name in $Names) {
        $command = Get-Command $name -ErrorAction SilentlyContinue
        if ($null -ne $command) {
            return $command.Source
        }
    }

    return Resolve-ExistingPath -Candidates $FallbackPaths
}

function Invoke-External {
    param(
        [string]$Label,
        [string]$FilePath,
        [string[]]$Arguments
    )

    Write-Host "[RUN ] $Label"
    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Label failed with exit code $LASTEXITCODE."
    }
}

function Add-Result {
    param(
        [System.Collections.Generic.List[object]]$Results,
        [string]$Step,
        [string]$Status,
        [string]$Detail
    )

    $Results.Add([pscustomobject]@{
        Step   = $Step
        Status = $Status
        Detail = $Detail
    })
}

function Show-Or-Missing {
    param([string]$Value)

    if ([string]::IsNullOrWhiteSpace($Value)) {
        return "missing"
    }

    return $Value
}

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$RegulonC = Join-Path $RepoRoot "regulon-c"
$ActiveSources = @(
    (Join-Path $RegulonC "src\ron_pid_api.c"),
    (Join-Path $RegulonC "src\ron_pid_config.c"),
    (Join-Path $RegulonC "src\ron_pid_core.c"),
    (Join-Path $RegulonC "src\ron_pid_fault.c"),
    (Join-Path $RegulonC "src\ron_pid_internal.h"),
    (Join-Path $RegulonC "include\ron\ron_platform.h"),
    (Join-Path $RegulonC "include\ron\ron_pid_types.h"),
    (Join-Path $RegulonC "include\ron\ron_pid.h")
)

$CMake = Find-CommandPath -Names @("cmake")
$CTest = Find-CommandPath -Names @("ctest")
$Cppcheck = Find-CommandPath -Names @("cppcheck") -FallbackPaths @("C:\Program Files\Cppcheck\cppcheck.exe")
$ClangFormat = Find-CommandPath -Names @("clang-format") -FallbackPaths @(
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\Llvm\bin\clang-format.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\Llvm\x64\bin\clang-format.exe"
)
$ClangCl = Find-CommandPath -Names @("clang-cl")
$ArmGcc = Find-CommandPath -Names @("arm-none-eabi-gcc")
$Cbmc = Find-CommandPath -Names @("cbmc")
$Python = Find-CommandPath -Names @("python")
$MisraAddon = Resolve-ExistingPath -Candidates @(
    $env:RON_CPPCHECK_MISRA,
    "C:\Program Files\Cppcheck\addons\misra.py",
    (Join-Path $env:USERPROFILE "workspace\eaglesexternal\external\cppcheck\2.17.1\addons\misra.py")
)

$Results = New-Object 'System.Collections.Generic.List[object]'
$MissingRequired = $false

foreach ($step in $Steps) {
    switch ($step) {
        "probe" {
            Add-Result $Results "probe" "ok" ("cmake={0}; ctest={1}; python={2}; cppcheck={3}; misra={4}; clang-format={5}; clang-cl={6}; arm-none-eabi-gcc={7}; cbmc={8}" -f `
                (Show-Or-Missing $CMake),
                (Show-Or-Missing $CTest),
                (Show-Or-Missing $Python),
                (Show-Or-Missing $Cppcheck),
                (Show-Or-Missing $MisraAddon),
                (Show-Or-Missing $ClangFormat),
                (Show-Or-Missing $ClangCl),
                (Show-Or-Missing $ArmGcc),
                (Show-Or-Missing $Cbmc))
        }
        "msvc" {
            if (($null -eq $CMake) -or ($null -eq $CTest)) {
                Add-Result $Results "msvc" "skip" "cmake or ctest not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            $BuildDir = Join-Path $RegulonC "build\verify-msvc"
            Invoke-External "Configure MSVC Debug PID build" $CMake @("-B", $BuildDir, "-S", $RegulonC, "-DRON_BUILD_TESTS=ON")
            Invoke-External "Build MSVC Debug PID build" $CMake @("--build", $BuildDir, "--config", "Debug")
            Invoke-External "Run MSVC Debug PID tests" $CTest @("--test-dir", $BuildDir, "--output-on-failure", "-C", "Debug")
            Add-Result $Results "msvc" "ok" $BuildDir
        }
        "double" {
            if (($null -eq $CMake) -or ($null -eq $CTest)) {
                Add-Result $Results "double" "skip" "cmake or ctest not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            $BuildDir = Join-Path $RegulonC "build\verify-double"
            Invoke-External "Configure double-precision PID build" $CMake @("-B", $BuildDir, "-S", $RegulonC, "-DRON_BUILD_TESTS=ON", "-DRON_USE_DOUBLE=ON")
            Invoke-External "Build double-precision PID build" $CMake @("--build", $BuildDir, "--config", "Debug")
            Invoke-External "Run double-precision PID tests" $CTest @("--test-dir", $BuildDir, "--output-on-failure", "-C", "Debug")
            Add-Result $Results "double" "ok" $BuildDir
        }
        "format" {
            if ($null -eq $ClangFormat) {
                Add-Result $Results "format" "skip" "clang-format not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            Invoke-External "clang-format dry-run" $ClangFormat @("--dry-run", "--Werror") + $ActiveSources
            Add-Result $Results "format" "ok" $ClangFormat
        }
        "cppcheck" {
            if (($null -eq $Cppcheck) -or ($null -eq $MisraAddon)) {
                Add-Result $Results "cppcheck" "skip" "cppcheck executable or misra addon not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            $CppcheckArgs = @(
                "--addon=$MisraAddon",
                "--check-level=exhaustive",
                "--enable=style",
                "--error-exitcode=1",
                "--suppress=missingIncludeSystem",
                "--suppress=misra-c2012-2.5",
                "--suppress=misra-c2012-8.7",
                "--suppress=misra-c2012-15.5",
                "--suppress=misra-c2012-15.7",
                "--suppress=misra-c2012-20.10",
                "-I", (Join-Path $RegulonC "include"),
                (Join-Path $RegulonC "src\ron_pid_api.c"),
                (Join-Path $RegulonC "src\ron_pid_config.c"),
                (Join-Path $RegulonC "src\ron_pid_core.c"),
                (Join-Path $RegulonC "src\ron_pid_fault.c")
            )
            Invoke-External "cppcheck MISRA pass" $Cppcheck $CppcheckArgs
            Add-Result $Results "cppcheck" "ok" $Cppcheck
        }
        "complexity" {
            if ($null -eq $Python) {
                Add-Result $Results "complexity" "skip" "python not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            Invoke-External "lizard complexity pass" $Python @("-m", "lizard", "-C", "10",
                (Join-Path $RegulonC "src\ron_pid_api.c"),
                (Join-Path $RegulonC "src\ron_pid_config.c"),
                (Join-Path $RegulonC "src\ron_pid_core.c"),
                (Join-Path $RegulonC "src\ron_pid_fault.c"))
            Add-Result $Results "complexity" "ok" "python -m lizard"
        }
        "clang" {
            if (($null -eq $ClangCl) -or ($null -eq $CMake) -or ($null -eq $CTest)) {
                Add-Result $Results "clang" "skip" "clang-cl, cmake, or ctest not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            $BuildDir = Join-Path $RegulonC "build\verify-clangcl"
            Invoke-External "Configure ClangCL PID build" $CMake @("-B", $BuildDir, "-S", $RegulonC, "-T", "ClangCL", "-DRON_BUILD_TESTS=ON")
            Invoke-External "Build ClangCL PID build" $CMake @("--build", $BuildDir, "--config", "Debug")
            Invoke-External "Run ClangCL PID tests" $CTest @("--test-dir", $BuildDir, "--output-on-failure", "-C", "Debug")
            Add-Result $Results "clang" "ok" $BuildDir
        }
        "cross-arm" {
            if (($null -eq $ArmGcc) -or ($null -eq $CMake)) {
                Add-Result $Results "cross-arm" "skip" "arm-none-eabi-gcc or cmake not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            $BuildDir = Join-Path $RegulonC "build\verify-arm"
            $Toolchain = Join-Path $RegulonC "cmake\toolchains\arm-none-eabi.cmake"
            Invoke-External "Configure ARM cross-compile" $CMake @("-B", $BuildDir, "-S", $RegulonC, "-DRON_BUILD_TESTS=OFF", "-DCMAKE_TOOLCHAIN_FILE=$Toolchain")
            Invoke-External "Build ARM cross-compile" $CMake @("--build", $BuildDir)
            Add-Result $Results "cross-arm" "ok" $BuildDir
        }
        "cbmc" {
            if ($null -eq $Cbmc) {
                Add-Result $Results "cbmc" "skip" "cbmc not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            $Harnesses = @(
                (Join-Path $RegulonC "test\formal\pid_saturation_proof.c"),
                (Join-Path $RegulonC "test\formal\pid_backcalc_proof.c"),
                (Join-Path $RegulonC "test\formal\pid_integral_clamp_proof.c"),
                (Join-Path $RegulonC "test\formal\pid_multi_instance_proof.c"),
                (Join-Path $RegulonC "test\formal\pid_null_pointer_proof.c")
            )
            foreach ($harness in $Harnesses) {
                Invoke-External "CBMC $(Split-Path $harness -Leaf)" $Cbmc @(
                    "--unwind", "32",
                    "--bounds-check",
                    "--pointer-check",
                    $harness,
                    (Join-Path $RegulonC "src\ron_pid_api.c"),
                    (Join-Path $RegulonC "src\ron_pid_config.c"),
                    (Join-Path $RegulonC "src\ron_pid_core.c"),
                    (Join-Path $RegulonC "src\ron_pid_fault.c"),
                    "-I", (Join-Path $RegulonC "include")
                )
            }
            Add-Result $Results "cbmc" "ok" $Cbmc
        }
        default {
            throw "Unknown step '$step'."
        }
    }
}

Write-Host ""
Write-Host "Verification Summary"
Write-Host "--------------------"
$Results | Format-Table -AutoSize

if ($MissingRequired) {
    exit 2
}

exit 0
