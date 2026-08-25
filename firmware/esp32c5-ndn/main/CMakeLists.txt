idf_component_register(SRCS "ndn_radio.c"
                       INCLUDE_DIRS ".")

# ESP-IDF 5.4.4 C5 preview-blob skew: net80211 (WiFi-6 HE UL-MU pwrcap) references phy_get_max_pwr_new
# but the C5 PHY lib only exports phy_get_max_pwr (renamed in 5.5). Alias them — the referencing function
# (ieee80211_add_ulmu_pwrcap) is only used during HE association/UL-MU, which a promiscuous+raw-TX
# named-radio never does, so the dead path's arg mismatch (if any) never executes. Drop when on IDF 5.5+.
idf_build_set_property(LINK_OPTIONS "-Wl,--defsym=phy_get_max_pwr_new=phy_get_max_pwr" APPEND)
