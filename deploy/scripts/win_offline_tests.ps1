<#
.SYNOPSIS
    This script aims at running offline validation tests on Windows.
.DESCRIPTION
    Script workflow is as follows:
    - Install deps via chocolatey
    - Run different Statismo builds with different options
    - Try to make and run an application based on Statismo
.PARAMETER branch
    Branch to test
.PARAMETER install_dir
    Installation directory
.PARAMETER install_type
    Expected value: 'full', 'full-nochoco' or 'debug' (in debug mode choco and downloaded packages are not retrieved)
#>

#
# script parameters
#
Param(
[parameter(Mandatory=$true)][ValidateNotNullOrEmpty()][string]$branch,
[parameter(Mandatory=$true)][ValidateNotNullOrEmpty()][string]$install_dir,
[parameter(Mandatory=$true)][ValidateSet('full','full-nochoco','debug', ignorecase=$False)][string]$install_type
)

#
# header used to handle permission restrictions
#
If (!([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]"Administrator"))
{
    # manually forward the parameters
    # generic way of forwarding with parsing of $myinvocation.Line failed
    # because $myinvocation.Line is empty when the script is called from command line with powershell.exe
    $argumentList = (" -branch", "`"$branch`"")
    $argumentList += (" -install_dir", "`"$install_dir`"")
    $argumentList += (" -install_type", "`"$install_type`"")
    $command = $myinvocation.MyCommand.Definition + $argumentList
    Start-Process powershell.exe "-NoExit -NoProfile -ExecutionPolicy Bypass -File $command" -Verb RunAs
    Exit
}

#
# Utility functions
#
function DownloadDeps($url, $dest)
{
    Write-Host "Download: [" $url "] -> [" $dest "]"
    $uri = New-Object System.Uri $url
    $request = [System.Net.HttpWebRequest]::Create($uri)
    $request.set_Timeout(150000)
    $response = $request.GetResponse()
    $total_len = $response.get_ContentLength()
    $response_stream = $response.GetResponseStream()
    $target_stream = New-Object -TypeName System.IO.FileStream -ArgumentList $dest, Create

    $buffer = new-object byte[] 10KB
    $bytes_read = 0

    Do {
        #transfer data from response stream to target stream
        $count = $response_stream.Read($buffer, 0, $buffer.length)
        $target_stream.Write($buffer, 0, $count)
        $bytes_read += $count
        $percent_complete = [System.Math]::Floor(100 * $bytes_read / $total_len)
        Write-Progress -Activity "Download in Progress" -Status "$percent_complete% Complete:" -PercentComplete $percent_complete
    } While ($count -gt 0)

    $target_stream.Flush()
    $target_stream.Close()
    $target_stream.Dispose()
    $response_stream.Dispose()
    $response_stream.Close()
}

$ErrorActionPreference = "Continue"

#
# input validation
#

If (-Not (Test-Path $install_dir))
{
    Throw New-Object System.IO.FileNotFoundException "$install_dir not found"
}

Write-Host "Install directory: $install_dir"
Write-Host "Install type: $install_type"

#
# Deps install
#

# choco packages
If ($install_type -eq "full"){
    Write-Host "Installing chocolatey packages"
    Invoke-Expression -Command "choco install -y -f chocolatey-core.extension"
    # Compilation toolchain
    Invoke-Expression -Command "choco install -y -f git.install --params '/SChannel'"
    # Uncomment if not available on the target platform
    # Warning: can cause a system restart
    #Invoke-Expression -Command "choco install -y -f visualstudio2019buildtools"
    $cmake_arg = '"ADD_CMAKE_TO_PATH=User"'
    $cmake_choco = "choco install -y -f cmake.install --installargs '$cmake_arg'"
    Invoke-Expression -Command "$cmake_choco"
    # Utilities
    Invoke-Expression -Command "choco install -y -f 7zip.install"
    # Wrapping
    Invoke-Expression -Command "choco install -y -f python3 --version=3.7.3"
    Invoke-Expression -Command "choco install -y -f swig"
    
    # Make `refreshenv` available right away, by defining the $env:ChocolateyInstall
    # variable and importing the Chocolatey profile module.
    # Note: Using `. $PROFILE` instead *may* work, but isn't guaranteed to.
    $env:ChocolateyInstall = Convert-Path "$((Get-Command choco).Path)\..\.."   
    Import-Module "$env:ChocolateyInstall\helpers\chocolateyProfile.psm1"

    # refreshenv is now an alias for Update-SessionEnvironment
    # (rather than invoking refreshenv.cmd, the *batch file* for use with cmd.exe)
    # This should make git.exe accessible via the refreshed $env:PATH, so that it
    # can be called by name only.
    refreshenv
}

# Manually set the path for cmake and git as it seems that the update
# option is not working with our call
# $env:Path += ";C:\\Program Files\\CMake\\bin"
# $env:Path += ";C:\\Program Files\\Git\\bin"
# $env:Path += ";C:\\Python37"

$env:GIT_REDIRECT_STDERR = '2>&1'

# vtk and itk from git
If ($install_type -eq "full" -Or $install_type -eq "full-nochoco") {
    Write-Host "Installing vtk/itk packages"
    
    Remove-Item -ErrorAction Ignore -force -Recurse $install_dir/itk | Out-Null
    Invoke-Expression -Command "git clone https://github.com/Kitware/ITK.git --branch v5.0.0 $install_dir/itk"

    Remove-Item -ErrorAction Ignore -force -Recurse $install_dir/vtk | Out-Null
    Invoke-Expression -Command "git clone https://github.com/Kitware/VTK.git --branch v8.2.0 $install_dir/vtk"
}

# hdf5 and eigen
If ($install_type -eq "full" -Or $install_type -eq "full-nochoco"){
    Write-Host "Installing system package from sources"
    Remove-Item -ErrorAction Ignore -force -Recurse $install_dir/eigen | Out-Null
    Remove-Item -ErrorAction Ignore -force -Recurse $install_dir/hdf5 | Out-Null
    New-Item -ItemType Directory -Force -Path $install_dir/eigen
    New-Item -ItemType Directory -Force -Path $install_dir/hdf5
    DownloadDeps "https://bitbucket.org/eigen/eigen/get/3.3.5.tar.gz" "$install_dir/eigen/3.3.5.tar.gz"
    DownloadDeps "https://www.hdfgroup.org/ftp/HDF5/releases/hdf5-1.10/hdf5-1.10.2/src/hdf5-1.10.2.tar.gz" "$install_dir/hdf5/hdf5-1.10.2.tar.gz"
    Set-Location $install_dir/eigen
    Invoke-Expression "7z e 3.3.5.tar.gz"
    Invoke-Expression "7z x 3.3.5.tar"
    Set-Location $install_dir/hdf5
    Invoke-Expression "7z e hdf5-1.10.2.tar.gz"
    Invoke-Expression "7z x hdf5-1.10.2.tar"
}

$eigen_src_dir = "eigen-eigen-b3f3d4950030"
$hdf5_src_dir = "hdf5-1.10.2"

#
# Statismo install from git with python requirements
#
If ($install_type -eq "full" -Or $install_type -eq "full-nochoco"){
    Write-Host "Installing statismo"
    
    Remove-Item -ErrorAction Ignore -force -Recurse $install_dir/statismo | Out-Null
    Invoke-Expression -Command "git clone https://github.com/kenavolic/statismo.git --branch $branch $install_dir/statismo"

    Invoke-Expression -Command "python -m pip install -r $install_dir/statismo/modules/VTK/wrapping/requirements_tests.txt"
}

Write-Host "Generating test app"

New-Item -force -ItemType directory -Path $install_dir/app | Out-Null
Set-Location $install_dir/app
$cmake_text = @'
cmake_minimum_required(VERSION 3.15)
project(demo LANGUAGES CXX VERSION 0.1.0)
find_package(statismo REQUIRED)
include(${STATISMO_USE_FILE})
add_executable(demo main.cpp)
target_link_libraries(demo ${STATISMO_LIBRARIES} ${VTK_LIBRARIES})
'@

$cmake_text | Set-Content 'CMakeLists.txt'

$main_text = @'
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include <iostream>
int main() {
auto itkrep = itk::StandardMeshRepresenter<float, 3>::New();
auto vtkrep = statismo::vtkStandardMeshRepresenter::SafeCreate();
std::cout << "itkrep rep" << itkrep.GetPointer() << std::endl;
std::cout << "vtkrep rep" << vtkrep.get() << std::endl;
return 0;
}
'@

$main_text | Set-Content 'main.cpp'

#
# BUILD 1
#

Write-Host ""
Write-Host "Processing BUILD 1:"
Write-Host "-ITK: Manually installed on system"
Write-Host "-VTK: Manually installed on system"
Write-Host "-Eigen: ITK internal version"
Write-Host "-HDF5: ITK internal version"
Write-Host "-Options: Static libs, debug"
Write-Host ""

Write-Host "-- Building VTK..."

New-Item -force -ItemType directory -Path $install_dir/vtk/build-debug | Out-Null
New-Item -force -ItemType directory -Path $install_dir/vtk/dist-debug | Out-Null
Set-Location $install_dir/vtk/build-debug | Out-Null
Invoke-Expression -Command "cmake .. -DCMAKE_INSTALL_PREFIX=$install_dir/vtk/dist-debug -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DVTK_BUILD_ALL_MODULES=OFF -DVTK_WRAP_PYTHON=OFF"
Invoke-Expression -Command "cmake -j 6 --build . --config Debug"
Invoke-Expression -Command "cmake --install . --config Debug"

Write-Host "-- Building VTK: OK"

Write-Host "-- Building ITK..."

New-Item -force -ItemType directory -Path $install_dir/itk/build-debug | Out-Null
New-Item -force -ItemType directory -Path $install_dir/itk/dist-debug | Out-Null
Set-Location $install_dir/itk/build-debug | Out-Null
Invoke-Expression -Command "cmake .. -DCMAKE_INSTALL_PREFIX=$install_dir/itk/dist-debug -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DITK_BUILD_DEFAULT_MODULES=ON -DModule_ITKReview=ON -DITK_LEGACY_REMOVE=ON -DITK_USE_SYSTEM_HDF5=OFF -DITK_USE_SYSTEM_EIGEN=OFF -DITK_SKIP_PATH_LENGTH_CHECKS=1 -DCMAKE_CXX_FLAGS='/FS'"
Invoke-Expression -Command "cmake -j 6 --build . --config Debug"
Invoke-Expression -Command "cmake --install . --config Debug"

Write-Host "-- Building ITK: OK"

Write-Host "-- Building Statismo..."

New-Item -force -ItemType directory -Path $install_dir/statismo/build-debug | Out-Null
New-Item -force -ItemType directory -Path $install_dir/statismo/dist-debug | Out-Null
Set-Location $install_dir/statismo/build-debug
Invoke-Expression -Command "cmake ../superbuild -DAUTOBUILD_STATISMO=ON -DBUILD_LONG_RUNNING_CLI_TESTS=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_CLI_TOOLS_DOC=OFF -DBUILD_SHARED_LIBS=OFF -DBUILD_WRAPPING=OFF -DCMAKE_INSTALL_PREFIX=$install_dir/statismo/dist-debug -DUSE_SYSTEM_ITK=ON -DUSE_SYSTEM_VTK=ON -DITK_DIR=$install_dir/itk/dist-debug/lib/cmake/ITK-5.0 -DVTK_DIR=$install_dir/vtk/dist-debug/lib/cmake/vtk-8.2 -DUSE_ITK_EIGEN=ON -DUSE_ITK_HDF5=ON"
Invoke-Expression -Command "cmake -j 6 --build . --config Debug"
Set-Location $install_dir/statismo/build-debug/Statismo-build | Out-Null
Invoke-Expression -Command "ctest -C Debug"
Invoke-Expression -Command "cmake --install . --config Debug"

Write-Host "-- Building Statismo: OK"

Write-Host "-- Test Application built on Statismo..."

New-Item -force -ItemType directory -Path $install_dir/app/build-debug | Out-Null
Set-Location $install_dir/app/build-debug
Invoke-Expression -Command "cmake .. -Dstatismo_DIR=$install_dir/statismo/dist-debug/CMake"
Invoke-Expression -Command "cmake -j 6 --build . --config Debug"
Invoke-Expression -Command "./Debug/demo"

Write-Host "-- Test Application built on Statismo: OK"

#
# BUILD 2
#
Write-Host ""
Write-Host "Processing BUILD 2:"
Write-Host "-ITK: Downloaded with cmake external project"
Write-Host "-VTK: Downloaded with cmake external project"
Write-Host "-Eigen: Manually installed on system"
Write-Host "-HDF5: Manually installed on system"
Write-Host "-Options: Shared libs, release, python wrapping"
Write-Host ""

Write-Host "-- Building HDF5"

New-Item -force -ItemType directory -Path $install_dir/hdf5/build-release | Out-Null
New-Item -force -ItemType directory -Path $install_dir/hdf5/dist-release | Out-Null
Set-Location $install_dir/hdf5/build-release
Invoke-Expression -Command "cmake ../$hdf5_src_dir  -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$install_dir/hdf5/dist-release -DHDF5_ENABLE_Z_LIB_SUPPORT=OFF -DHDF5_BUILD_CPP_LIB:BOOL=ON -DHDF5_BUILD_TOOLS=OFF -DBUILD_TESTING=OFF -DHDF5_BUILD_EXAMPLES=OFF -DHDF5_BUILD_JAVA=OFF"
Invoke-Expression -Command "cmake -j 6 --build . --config Release"
Invoke-Expression -Command "cmake --install . --config Release"

Write-Host "-- Building HDF5: OK"

Write-Host "-- Building Statismo"

New-Item -force -ItemType directory -Path $install_dir/statismo/build-release | Out-Null
New-Item -force -ItemType directory -Path $install_dir/statismo/dist-release | Out-Null
Set-Location $install_dir/statismo/build-release | Out-Null
Invoke-Expression -Command "cmake ../superbuild  -DBUILD_WRAPPING=ON -DBUILD_DOCUMENTATION=OFF -DBUILD_CLI_TOOLS_DOC=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$install_dir/statismo/dist-release -DITK_EXTRA_OPTIONS:STRING='-DITK_SKIP_PATH_LENGTH_CHECKS=1' -DUSE_SYSTEM_EIGEN=ON -DUSE_SYSTEM_HDF5=ON -DEIGEN3_INCLUDE_DIR=$install_dir/eigen/$eigen_src_dir -DHDF5_DIR=$install_dir/hdf5/dist-release/cmake/hdf5"
Invoke-Expression -Command "cmake -j 6 --build . --config Release"
Set-Location $install_dir/statismo/build-release/Statismo-build | Out-Null

Write-Host "-- Building Statismo: OK"
Write-Host "-- Running tests..."
Invoke-Expression -Command "ctest -C Release"
Invoke-Expression -Command "cmake --install . --config Release"
$env:PATH += ";$install_dir/hdf5/dist-release/bin"
Invoke-Expression -Command ".\runVTKPythonTestsRelease.bat"
Write-Host "-- Running tests: OK"

Write-Host "-- Test Application built on Statismo..."

New-Item -force -ItemType directory -Path $install_dir/app/build | Out-Null
Set-Location $install_dir/app/build
$env:PATH += ";$install_dir/statismo/dist-release/bin"
$env:PATH += ";$install_dir/statismo/build-release/INSTALL/bin"
Invoke-Expression -Command "cmake .. -Dstatismo_DIR=$install_dir/statismo/dist-release/CMake"
Invoke-Expression -Command "cmake --build . --config Release"
Invoke-Expression -Command "./Release/demo"

Write-Host "-- Test Application built on Statismo: OK"
