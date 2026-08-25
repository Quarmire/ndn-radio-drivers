#!/usr/bin/env bash
# Absolute TX-power calibration by SUBSTITUTION.  K = P_ref - RSSI_ref ; TX_dut = RSSI_dut + K.
# The reference sweep doubles as the INSTRUMENT CHECK: if RSSI does not track commanded reference
# power at ~1.00 dB/dB then either the meter is nonlinear or mt76 is not applying what it reports,
# and no absolute number may be quoted. Bracketed (ref, DUT, ref) so drift is measured.
# ⚠ wpa_supplicant retunes this interface (observed: ch36 -> ch44 mid-run), so the channel is
# re-pinned AND VERIFIED per arm; a mistuned arm is reported, never silently averaged in.
C4=minidronesys@141.225.167.197
OPI=minidronesys@141.225.165.246
CH=36; IF=wlu1u3u4
REF_DBM="20 17 14 11 8 5"

ssh -o ConnectTimeout=10 $OPI "cd ~/ws/ndn-radio-drivers && sudo -n env NDN_PID=a81a \
    timeout 330 ./target/release/examples/rxpwr_bucket $CH 310 > /tmp/calib.log 2>&1" &
RX=$!
sleep 12

ref_sweep () {
ssh -o ConnectTimeout=10 $C4 "bash -s" <<EOF 2>&1
PHY=\$(cat /sys/class/net/$IF/phy80211/name)
cd ~/ws/ndn-radio-drivers
i=0
for p in $REF_DBM; do
  sudo -n iw dev $IF set channel $CH 2>/dev/null
  sudo -n iw phy \$PHY set txpower fixed \$((p*100)) 2>/dev/null
  ch=\$(iw dev $IF info | grep -oE 'channel [0-9]+' | grep -oE '[0-9]+')
  got=\$(iw dev $IF info | grep -oE 'txpower [0-9.]+' | grep -oE '[0-9.]+')
  echo "  REFARM $1 \$i commanded=\${p} reported=\${got} channel=\${ch}"
  [ "\$ch" = "$CH" ] || echo "    ⚠ WRONG CHANNEL — arm invalid"
  sudo -n ./target/release/examples/refinject8733b $IF $1 \$i 1200
  i=\$((i+1))
done
EOF
}

echo "=== BRACKET 1: reference sweep (knob 5) ==="; ref_sweep 5
echo "=== DUT: 8733b (knob 9) ==="
ssh -o ConnectTimeout=10 $C4 "cd ~/ws/ndn-radio-drivers && sudo -n timeout 110 ./target/release/examples/hal_txpower8733b $CH 1200 2>&1 | grep -E 'arm|TSSI setup'"
echo "=== BRACKET 2: reference sweep again (knob 6) ==="; ref_sweep 6

wait $RX 2>/dev/null
echo "=== meter (a81a) ==="; ssh -o ConnectTimeout=10 $OPI 'cat /tmp/calib.log'
echo CALIB_DONE
