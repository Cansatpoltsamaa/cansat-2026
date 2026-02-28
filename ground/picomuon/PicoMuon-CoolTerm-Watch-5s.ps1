[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)] [string]$LogPath,

  [int]$BinSeconds = 1,                        # 1 s või 5 s
  [int]$InactivitySeconds = 30,                # mitu s vaikust -> lõpetan
  [ValidateSet('C','ALL')][string]$CountMode = 'C',

  [int]$WindowSeconds = 25,                    # libisev aken C-le
  [switch]$BeepOnMuon,                         # piiks iga C korral
  [switch]$EchoMuon,                           # prindi iga C rida
  [switch]$EchoAll,                            # prindi KÕIK read
  [switch]$FromStart,                          # loe logi algusest
  [int]$FlushEverySeconds = 10                 # kui tihti histo CSV+PNG uuendada
)

# ---- ABI ----
function Clean([string]$s){
  if ($null -eq $s) { return '' }
  # eemalda kontrollmärgid, NBSP, zero-width, BOM
  return ($s -replace '[\p{C}\u00A0\u2000-\u200F\uFEFF]', '')
}
function ParseTs([string]$s){
  try { return [datetime]::ParseExact($s,'yyyy-MM-dd HH:mm:ss.fff',[Globalization.CultureInfo]::InvariantCulture,[Globalization.DateTimeStyles]::AssumeLocal) } catch {}
  try { return [datetime]::ParseExact($s,'yyyy-MM-dd HH:mm:ss',    [Globalization.CultureInfo]::InvariantCulture,[Globalization.DateTimeStyles]::AssumeLocal) } catch {}
  try { return [datetime]::Parse($s,[Globalization.CultureInfo]::InvariantCulture) } catch { return $null }
}

if (!(Test-Path $LogPath)) { throw "Log file not found: $LogPath" }

# ---- Väljundi asukoht: PICO_2026 põhikaust ----
$rootDir = Split-Path -Parent (Split-Path -Parent $LogPath)   # ...\Pico_2026\
if (!(Test-Path $rootDir)) { New-Item -ItemType Directory -Path $rootDir | Out-Null }

# Live-failid (kohe kasutatavad)
$eventsCsvPath = Join-Path $rootDir 'run_events_live.csv'     # kõik T/B/C
$muonsCsvPath  = Join-Path $rootDir 'run_muons_live.csv'      # ainult C
$histCsvPath   = Join-Path $rootDir ("pico_muon_hist_"+$BinSeconds+"s.csv")
$histPngPath   = Join-Path $rootDir ("pico_muon_hist_"+$BinSeconds+"s.png")
$fullLogPath   = Join-Path $rootDir 'run_full_log.txt'        # meie normaliseeritud täislogi

# Avame kirjutajad (append; UTF8 ilma BOM-ita)
$eventsSw = New-Object System.IO.StreamWriter($eventsCsvPath, $true, (New-Object System.Text.UTF8Encoding($false)))
$muonsSw  = New-Object System.IO.StreamWriter($muonsCsvPath , $true, (New-Object System.Text.UTF8Encoding($false)))
$fullSw   = New-Object System.IO.StreamWriter($fullLogPath  , $true, (New-Object System.Text.UTF8Encoding($false)))
$eventsSw.AutoFlush = $true; $muonsSw.AutoFlush = $true; $fullSw.AutoFlush = $true

# CSV päised kui fail uued
if ((Get-Item $eventsCsvPath).Length -eq 0) { $eventsSw.WriteLine("timestamp_local,ID,raw") }
if ((Get-Item $muonsCsvPath ).Length -eq 0) { $muonsSw.WriteLine("timestamp_local,raw")     }

# Chartimiseks
Add-Type -AssemblyName System.Windows.Forms                    | Out-Null
Add-Type -AssemblyName System.Windows.Forms.DataVisualization  | Out-Null

# Lihtne muster: TS + payload; nool eemaldame hiljem
$tsPayload = '^(?<ts>\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}(?:\.\d{1,3})?)\s+(?<payload>.+)$'

# Andmekandjad
$buckets = @{}
$cWindow = New-Object System.Collections.Generic.Queue[datetime]
$cTotal  = 0
$eventsCnt   = 0
$lastStatusAt = (Get-Date).AddSeconds(-1)
$lastLineAt   = Get-Date
$lastFlushAt  = Get-Date
$done = $false
$lastTs = $null   # viimane nähtud timestamp

Write-Host ("Watching: {0} | bin={1}s | inactivity={2}s | mode={3}" -f $LogPath,$BinSeconds,$InactivitySeconds,$CountMode)
Write-Host ("Outputs → {0}" -f $rootDir)

# Vaikuse taimer
if ($InactivitySeconds -gt 0){
  $timer = New-Object System.Timers.Timer(1000); $timer.AutoReset = $true
  Register-ObjectEvent -InputObject $timer -EventName Elapsed -SourceIdentifier idleCheck -Action {
    if ((Get-Date) - $script:lastLineAt -gt [TimeSpan]::FromSeconds($script:InactivitySeconds)) { $script:done = $true }
  } | Out-Null
  $timer.Start()
}

# Histogrammi CSV+PNG uuendus
function Update-Histogram {
  param($bucketsRef)
  $rows = $bucketsRef.GetEnumerator() | Sort-Object Key | ForEach-Object {
    [pscustomobject]@{ bucket_start = $_.Key.ToString('yyyy-MM-dd HH:mm:ss'); count = $_.Value }
  }
  $rows | Export-Csv -Path $histCsvPath -NoTypeInformation -Encoding UTF8

  # PNG joonistamine
  $chart = New-Object System.Windows.Forms.DataVisualization.Charting.Chart
  $chart.Width=1600; $chart.Height=900
  $area = New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea 'main'
  $chart.ChartAreas.Add($area)
  $area.AxisX.Interval=1; $area.AxisX.LabelStyle.Angle=-45
  $area.AxisX.MajorGrid.Enabled=$false; $area.AxisY.MajorGrid.LineColor='Gainsboro'
  $area.AxisX.Title="Bin start (local time)"; $area.AxisY.Title="Count (ID="+$CountMode+")"
  $series = New-Object System.Windows.Forms.DataVisualization.Charting.Series 'Muon'
  $series.ChartType=[System.Windows.Forms.DataVisualization.Charting.SeriesChartType]::Column
  $series.Color='SteelBlue'; $series.BorderWidth=1
  foreach ($r in $rows) { [void]$series.Points.AddXY($r.bucket_start,[int]$r.count) }
  $chart.Series.Add($series)
  $title = New-Object System.Windows.Forms.DataVisualization.Charting.Title
  $title.Text=("Muon counts - {0}s bins (ID={1})" -f $BinSeconds, $CountMode)
  $title.Font=New-Object System.Drawing.Font('Segoe UI',14,[System.Drawing.FontStyle]::Bold)
  $chart.Titles.Add($title)
  $chart.SaveImage($histPngPath,[System.Windows.Forms.DataVisualization.Charting.ChartImageFormat]::Png)
}

# Tail (CoolTermi logi)
$tail = if ($FromStart) { 999999999 } else { 0 }

try {
  Get-Content -Path $LogPath -Wait -Tail $tail | ForEach-Object {
    if ($done) { break }

    $line = Clean $_
    if ([string]::IsNullOrWhiteSpace($line)) { return }

    # --- TS + payload (lubame ka real ilma TS-ita) ---
    $t = $null; $payload = $null
    $m = [regex]::Match($line, $tsPayload)
    if ($m.Success) {
      $t = ParseTs $m.Groups['ts'].Value
      if ($null -eq $t) { return }
      $lastTs = $t
      $payload = Clean $m.Groups['payload'].Value
    } else {
      $t = if ($lastTs) { $lastTs } else { Get-Date }
      $payload = $line
    }

    # normaliseeri nool algusest: "-->", "->" või Unicode → ; ja puhasta uuesti
    $payload = ($payload -replace '^\s*(?:-+>|->|[\u2192])\s*', '')
    $payload = Clean $payload

    # --- Kirjuta meie täislogisse alati TS + " --> " + payload (kuupäev ees) ---
    $fullSw.WriteLine(("{0:yyyy-MM-dd HH:mm:ss.fff} --> {1}" -f $t, $payload))

    if ($EchoAll) { Write-Host ("[ALL] {0}  {1}" -f $t.ToString('yyyy-MM-dd HH:mm:ss.fff'), $payload) }

    # --- ID tuvastus: eeliselt otsi ",C,"; kui ei leia, siis ",T,"/ ",B,"; fallback esimesest väljast ---
    $ID = $null
    if     ($payload -match '(^|,)\s*[Cc]\s*(,|$)') { $ID = 'C' }
    elseif ($payload -match '(^|,)\s*[Tt]\s*(,|$)') { $ID = 'T' }
    elseif ($payload -match '(^|,)\s*[Bb]\s*(,|$)') { $ID = 'B' }
    else {
      $parts = $payload.Split(',').ForEach({ $_.Trim() })
      if ($parts.Count -gt 0 -and $parts[0] -match '^[TtBbCc]$') { $ID = $parts[0].ToUpper() } else { return }
    }

    # --- Bin ---
    $epoch  = [DateTimeOffset]$t
    $sec    = [math]::Floor($epoch.ToUnixTimeSeconds() / $BinSeconds) * $BinSeconds
    $bucket = [DateTimeOffset]::FromUnixTimeSeconds($sec).LocalDateTime

    if ($CountMode -eq 'C') {
      if ($ID -eq 'C') {
        if (-not $buckets.ContainsKey($bucket)) { $buckets[$bucket] = 0 }
        $buckets[$bucket]++
      }
    } else {
      if (-not $buckets.ContainsKey($bucket)) { $buckets[$bucket] = 0 }
      $buckets[$bucket]++
    }

    # --- CSV-d REAALAJAS ---
    $eventsSw.WriteLine(("{0},{1},""{2}""" -f $t.ToString('yyyy-MM-dd HH:mm:ss.fff'), $ID, $payload.Replace('"','""')))
    if ($ID -eq 'C') {
      $muonsSw.WriteLine(("{0},""{1}""" -f $t.ToString('yyyy-MM-dd HH:mm:ss.fff'), $payload.Replace('"','""')))
    }

    # --- Müüon: piiks + echo + libisev aken ---
    if ($ID -eq 'C') {
      $cTotal++
      $cWindow.Enqueue($t)
      while ($cWindow.Count -gt 0 -and ((Get-Date) - $cWindow.Peek()).TotalSeconds -gt $WindowSeconds) { $null = $cWindow.Dequeue() }
      if ($BeepOnMuon) { try { [console]::Beep(1200,80) } catch {} }
      if ($EchoMuon)  { Write-Host ("[C] {0}  {1}" -f $t.ToString('yyyy-MM-dd HH:mm:ss.fff'), $payload) }
    }

    # --- Staatuse rida kord ~sekundis ---
    if ( ((Get-Date) - $lastStatusAt).TotalSeconds -ge 1 ) {
      $lastStatusAt = Get-Date
      $lastBucket = if ($buckets.Count) { ($buckets.Keys | Sort-Object)[-1] } else { $null }
      $val = if ($lastBucket) { $buckets[$lastBucket] } else { 0 }
      Write-Host ("{0} | rows={1} | C_last{2}s={3} | C_total={4} | last {5}s bin={6}" -f `
        (Get-Date).ToString('yyyy-MM-dd HH:mm:ss'), $eventsCnt, $WindowSeconds, $cWindow.Count, $cTotal, $BinSeconds, $val)
    }

    # --- Perioodiline histo salvestus (CSV+PNG) ---
    if ( ((Get-Date) - $lastFlushAt).TotalSeconds -ge $FlushEverySeconds ) {
      Update-Histogram -bucketsRef $buckets
      $lastFlushAt = Get-Date
    }

    $eventsCnt++
    $lastLineAt = Get-Date
  }
}
finally {
  try { $eventsSw.Close() } catch {}
  try { $muonsSw.Close() }  catch {}
  try { $fullSw.Close() }   catch {}
  if ($timer) { Unregister-Event -SourceIdentifier idleCheck -ErrorAction SilentlyContinue; $timer.Stop(); $timer.Dispose() }
}

# Viimane histo uuendus
if ($buckets.Count -gt 0) { Update-Histogram -bucketsRef $buckets }

Write-Host ("OK full LOG     : {0}" -f $fullLogPath)
Write-Host ("OK live CSV ALL : {0}" -f $eventsCsvPath)
Write-Host ("OK live CSV C   : {0}" -f $muonsCsvPath)
Write-Host ("OK hist CSV     : {0}" -f $histCsvPath)
Write-Host ("OK hist PNG     : {0}" -f $histPngPath)