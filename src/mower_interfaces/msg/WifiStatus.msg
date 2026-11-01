# Wi-Fi link state (mower_control/wifi_monitor, /wifi/status, 1 Hz). 2026-11-01.
std_msgs/Header header
string iface
bool connected
string ssid
string bssid               # lower-case aa:bb:cc:dd:ee:ff, '' when not connected
float32 freq_mhz
float32 signal_dbm         # NaN when not connected / unknown
float32 tx_bitrate_mbps
float32 rx_bitrate_mbps
uint32 tx_retries          # cumulative station counters (refreshed every 10 s)
uint32 tx_failed
uint32 roam_count          # BSSID changes while connected since node start
string link_state          # good | ok | weak | bad | disconnected | down
