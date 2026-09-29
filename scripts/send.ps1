# Send REPL commands (one per line, from a file or stdin) to the IS31FL3736 REPL and
# print the replies.  Usage: powershell.exe -File send.ps1 [-Port COM4] [-File cmds.txt]
param(
  [string]$Port = 'COM4',
  [int]$Baud = 115200,
  [string]$File = '',
  [int]$SettleMs = 150
)
$lines = if ($File) { Get-Content -LiteralPath $File } else { @($input) }

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$sp.Handshake = [System.IO.Ports.Handshake]::None
$sp.NewLine = "`n"
$sp.ReadTimeout = 500
$sp.Open()
try {
  $sp.DiscardInBuffer()
  # Wake the prompt and drain any banner.
  $sp.Write("`n"); Start-Sleep -Milliseconds $SettleMs
  [void]$sp.ReadExisting()
  foreach ($l in $lines) {
    $l = $l.Trim()
    if ($l -eq '' -or $l.StartsWith('#')) { continue }
    Write-Output ">> $l"
    $sp.Write($l + "`n")
    # The REPL echoes the line, prints its reply, then a "> " prompt: wait for the prompt.
    $buf = ''; $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt 3000) {
      Start-Sleep -Milliseconds 20
      $buf += $sp.ReadExisting()
      if ($buf -match "(?s)\n> $") { break }
    }
    # Drop the echoed command line and the trailing prompt.
    $reply = ($buf -replace "(?s)^[^\n]*\n", '') -replace "(?s)\r?\n> $", ''
    if ($reply.Trim() -ne '') { Write-Output $reply.TrimEnd() }
  }
} finally { $sp.Close() }
