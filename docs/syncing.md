# Syncing

Syncing information comes from the reverse engineering work by Paul Wieland as well as some of my own.

## First time sync

To sync a rolling code needs to be sent with a non zero id. This command will be ignored. Then a second command needs to be sent with a rolling code that is higher than the previous with the same id. This command will not be ignore.

## Resyncing

After an id can be established the rolling code can be reset. The process is the same as the first time sync, with one caveat. The new rolling code can't be reset to what it was previously. A pattern of rolling codes following ABAB will not work (where A < B). That is because the packet with rolling code B is accept then when it gets reset it will try to be reset back to B, which is ignored, however the pattern ABCAB will work (where A < B < C). Since the rolling code is set to B, then C, then back to B.

## Library

The library starts by sending get status and get openings command to sync its interal state. Any command can be sent, though having one with a replay allows you to tell if the code was registed, as otherwise the packet is just dropped.
