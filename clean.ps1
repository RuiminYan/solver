# clean.ps1 - 清理编译产物

Write-Host "Cleaning build files..."
Remove-Item -Path *.o, *.exe -ErrorAction SilentlyContinue
Write-Host "Clean completed!"
