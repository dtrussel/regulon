param(
    [string[]]$Steps = @("probe", "msvc", "double", "format", "cppcheck", "complexity", "coverage", "clang", "cross-arm", "cbmc"),
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

function Invoke-ExternalCapture {
    param(
        [string]$Label,
        [string]$FilePath,
        [string[]]$Arguments
    )

    Write-Host "[RUN ] $Label"
    $output = & $FilePath @Arguments 2>&1
    $output | ForEach-Object { Write-Host $_ }
    if ($LASTEXITCODE -ne 0) {
        throw "$Label failed with exit code $LASTEXITCODE."
    }

    return $output
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

function Test-CoverageSummary {
    param([string]$SummaryJson)

    $lineCoverage = $null
    $branchCoverage = $null

    $summary = $SummaryJson | ConvertFrom-Json
    if (($null -ne $summary) -and ($summary.data.Count -gt 0) -and ($null -ne $summary.data[0].totals)) {
        $lineCoverage = [double]$summary.data[0].totals.lines.percent
        $branchCoverage = [double]$summary.data[0].totals.branches.percent
    }

    if (($null -eq $lineCoverage) -or ($null -eq $branchCoverage)) {
        throw "Unable to parse llvm-cov statement/branch coverage summary."
    }
    if (($lineCoverage -ne 100.0) -or ($branchCoverage -ne 100.0)) {
        throw ("Coverage below RON-QR-022 target: statements={0}%; branches={1}%." -f $lineCoverage, $branchCoverage)
    }
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
$Clang = Find-CommandPath -Names @("clang")
$ClangCl = Find-CommandPath -Names @("clang-cl")
$LlvmCov = Find-CommandPath -Names @("llvm-cov")
$LlvmProfdata = Find-CommandPath -Names @("llvm-profdata")
$Ninja = Find-CommandPath -Names @("ninja") -FallbackPaths @(
    "C:\Program Files\Ninja\ninja.exe",
    "C:\Program Files\LLVM\bin\ninja.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe",
    "C:\ProgramData\chocolatey\bin\ninja.exe",
    (Join-Path $env:USERPROFILE "scoop\shims\ninja.exe")
)
$VsClangClToolset = Resolve-ExistingPath -Candidates @(
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Microsoft\VC\v170\Platforms\x64\PlatformToolsets\ClangCL",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\MSBuild\Microsoft\VC\v170\Platforms\x64\PlatformToolsets\ClangCL",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Microsoft\VC\v170\Platforms\x64\PlatformToolsets\ClangCL",
    "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\MSBuild\Microsoft\VC\v170\Platforms\x64\PlatformToolsets\ClangCL"
)
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
            Add-Result $Results "probe" "ok" ("cmake={0}; ctest={1}; python={2}; cppcheck={3}; misra={4}; clang-format={5}; clang={6}; clang-cl={7}; llvm-cov={8}; llvm-profdata={9}; ninja={10}; vs-clangcl-toolset={11}; arm-none-eabi-gcc={12}; cbmc={13}" -f `
                (Show-Or-Missing $CMake),
                (Show-Or-Missing $CTest),
                (Show-Or-Missing $Python),
                (Show-Or-Missing $Cppcheck),
                (Show-Or-Missing $MisraAddon),
                (Show-Or-Missing $ClangFormat),
                (Show-Or-Missing $Clang),
                (Show-Or-Missing $ClangCl),
                (Show-Or-Missing $LlvmCov),
                (Show-Or-Missing $LlvmProfdata),
                (Show-Or-Missing $Ninja),
                (Show-Or-Missing $VsClangClToolset),
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
        "coverage" {
            if (($null -eq $CMake) -or ($null -eq $CTest) -or ($null -eq $Clang) -or
                ($null -eq $Ninja) -or ($null -eq $LlvmCov) -or ($null -eq $LlvmProfdata)) {
                Add-Result $Results "coverage" "skip" "cmake, ctest, clang, ninja, llvm-cov, or llvm-profdata not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            $BuildDir = Join-Path $RegulonC "build\verify-coverage"
            $ProfilesDir = Join-Path $BuildDir "profiles"
            $CoverageProfdata = Join-Path $BuildDir "coverage.profdata"
            $CoverageJson = Join-Path $RepoRoot "coverage.json"
            $CoverageHtml = Join-Path $RepoRoot "coverage_html"

            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue -LiteralPath $ProfilesDir
            Remove-Item -Recurse -Force -ErrorAction SilentlyContinue -LiteralPath $CoverageHtml
            Remove-Item -Force -ErrorAction SilentlyContinue -LiteralPath $CoverageProfdata, $CoverageJson
            New-Item -ItemType Directory -Force -Path $ProfilesDir | Out-Null
            Invoke-External "Configure LLVM coverage PID build" $CMake @("-G", "Ninja", "-B", $BuildDir, "-S", $RegulonC, "-DRON_BUILD_TESTS=ON", "-DCMAKE_C_COMPILER=$Clang", "-DCMAKE_MAKE_PROGRAM=$Ninja", "-DCMAKE_C_FLAGS=-O0 -g -fprofile-instr-generate -fcoverage-mapping")
            Invoke-External "Build LLVM coverage PID build" $CMake @("--build", $BuildDir)
            $env:LLVM_PROFILE_FILE = (Join-Path $ProfilesDir "%p.profraw")
            Invoke-External "Run LLVM coverage PID tests" $CTest @("--test-dir", $BuildDir, "--output-on-failure")
            Remove-Item Env:\LLVM_PROFILE_FILE -ErrorAction SilentlyContinue

            $ProfileInputs = @(Get-ChildItem -Path $ProfilesDir -Filter "*.profraw" | ForEach-Object { $_.FullName })
            if ($ProfileInputs.Count -eq 0) {
                throw "No LLVM .profraw files were generated by the coverage test run."
            }
            Invoke-External "Merge LLVM coverage profiles" $LlvmProfdata (@("merge", "-sparse") + $ProfileInputs + @("-o", $CoverageProfdata))

            $TestExecutables = @(Get-ChildItem -Path $BuildDir -Recurse -Filter "test_ron_pid*.exe" |
                Sort-Object -Property FullName |
                ForEach-Object { $_.FullName })
            if ($TestExecutables.Count -eq 0) {
                throw "No PID test executables found in LLVM coverage build."
            }
            $CoverageObjects = @($TestExecutables[0])
            if ($TestExecutables.Count -gt 1) {
                foreach ($executable in $TestExecutables[1..($TestExecutables.Count - 1)]) {
                    $CoverageObjects += @("-object", $executable)
                }
            }
            $CoverageSources = @(
                (Join-Path $RegulonC "src\ron_pid_api.c"),
                (Join-Path $RegulonC "src\ron_pid_config.c"),
                (Join-Path $RegulonC "src\ron_pid_core.c"),
                (Join-Path $RegulonC "src\ron_pid_fault.c")
            )
            $CoverageSourceArgs = @()
            foreach ($source in $CoverageSources) {
                $CoverageSourceArgs += @("-sources", $source)
            }

            $CoverageExport = Invoke-ExternalCapture "Export LLVM PID coverage summary" $LlvmCov (@("export") + $CoverageObjects + @("-instr-profile", $CoverageProfdata, "-format=text", "-summary-only", "-show-branch-summary") + $CoverageSourceArgs)
            $CoverageJsonText = ($CoverageExport -join [Environment]::NewLine)
            Set-Content -Path $CoverageJson -Value $CoverageJsonText
            Invoke-External "Render LLVM PID coverage report" $LlvmCov (@("show") + $CoverageObjects + @("-instr-profile", $CoverageProfdata, "-format=html", "-output-dir", $CoverageHtml, "-show-branch-summary") + $CoverageSourceArgs)
            Test-CoverageSummary -SummaryJson $CoverageJsonText
            Add-Result $Results "coverage" "ok" "100% statement and branch coverage"
        }
        "clang" {
            if (($null -eq $CMake) -or ($null -eq $CTest)) {
                Add-Result $Results "clang" "skip" "cmake or ctest not available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
                continue
            }

            if (($null -ne $Clang) -and ($null -ne $Ninja)) {
                $BuildDir = Join-Path $RegulonC "build\verify-clang"
                Invoke-External "Configure standalone Clang PID build" $CMake @("-G", "Ninja", "-B", $BuildDir, "-S", $RegulonC, "-DRON_BUILD_TESTS=ON", "-DCMAKE_C_COMPILER=$Clang", "-DCMAKE_MAKE_PROGRAM=$Ninja")
                Invoke-External "Build standalone Clang PID build" $CMake @("--build", $BuildDir)
                Invoke-External "Run standalone Clang PID tests" $CTest @("--test-dir", $BuildDir, "--output-on-failure")
                Add-Result $Results "clang" "ok" $BuildDir
            } elseif (($null -ne $ClangCl) -and ($null -ne $VsClangClToolset)) {
                $BuildDir = Join-Path $RegulonC "build\verify-clangcl"
                Invoke-External "Configure ClangCL PID build" $CMake @("-B", $BuildDir, "-S", $RegulonC, "-T", "ClangCL", "-DRON_BUILD_TESTS=ON")
                Invoke-External "Build ClangCL PID build" $CMake @("--build", $BuildDir, "--config", "Debug")
                Invoke-External "Run ClangCL PID tests" $CTest @("--test-dir", $BuildDir, "--output-on-failure", "-C", "Debug")
                Add-Result $Results "clang" "ok" $BuildDir
            } else {
                Add-Result $Results "clang" "skip" "no CMake-compatible Clang generator/compiler combination available"
                $MissingRequired = $MissingRequired -or $FailOnMissing
            }
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

            $Harnesses = Get-ChildItem -Path (Join-Path $RegulonC "test\formal") -Filter "*_proof.c" |
                Sort-Object -Property Name |
                ForEach-Object { $_.FullName }
            foreach ($harness in $Harnesses) {
                $EntryPoint = [System.IO.Path]::GetFileNameWithoutExtension($harness)
                Invoke-External "CBMC $(Split-Path $harness -Leaf)" $Cbmc @(
                    "--function", $EntryPoint,
                    "--unwind", "32",
                    "--unwinding-assertions",
                    "--bounds-check",
                    "--pointer-check",
                    $harness,
                    (Join-Path $RegulonC "src\ron_pid_api.c"),
                    (Join-Path $RegulonC "src\ron_pid_config.c"),
                    (Join-Path $RegulonC "src\ron_pid_core.c"),
                    (Join-Path $RegulonC "src\ron_pid_fault.c"),
                    "-I", (Join-Path $RegulonC "include"),
                    "-I", (Join-Path $RegulonC "test\formal")
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
