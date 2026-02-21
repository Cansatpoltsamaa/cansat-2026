\# Aja ühtlustamine (PicoMuon vs CanSat)



PicoMuon (ground) logib `time,muons` (sekundid). CanSat logib SD-le `ms` (millisekundid alates boot'ist).



\## Soovitus ajajoone ühtlustamiseks

\- Märgi käsitsi eksperimendi algushetk (nt "START" 0 s) ja käivita mõlemad seadmed võimalikult samal ajal.

\- Hiljem teisenda CanSati `ms` sekunditeks: `t\\\_sec = ms / 1000.0`.

\- Kui PicoMuoni `time` algab 0-st ja CanSati `t\\\_sec` algus ei lange täpselt kokku, nihuta ühte ajaskaalat (konstantnega): `t\\\_sec\\\_adj = t\\\_sec + t\\\_offset`.



\## Graafiku ideed

\- muons vs time\_sec

\- pressure\_hpa vs time\_sec

\- altitude\_m vs time\_sec

\- võimalik: muons vs pressure\_hpa (scatter), muons vs altitude\_m (kuigi PicoMuon on maas)

