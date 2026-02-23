# Public/private radio config setup

To keep sensitive radio settings out of GitHub while keeping the sketch public:

1. Copy the template:
   - `cp MySensors_wind_radio_private.h.example MySensors_wind_radio_private.h`
2. Edit `MySensors_wind_radio_private.h` with your real values:
   - `MY_RFM69_FREQUENCY`
   - `MY_RFM69_NETWORKID`
   - `MY_RFM69_ENABLE_ENCRYPTION` (define or leave undefined)
   - `MY_NODE_ID`
   - `MY_RFM69_TX_POWER_DBM`
3. Keep `MySensors_wind_radio_private.h` untracked (already in `.gitignore`).

If no private file exists, safe defaults from `MySensors_wind.ino` are used.
