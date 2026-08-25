idf_component_register(SRCS "ndn_radio.c"
                       INCLUDE_DIRS ".")

# NOTE: on ESP-IDF 5.4.4 the C5 was a preview target with a blob skew (net80211 wanted
# phy_get_max_pwr_new, the C5 phy lib only had phy_get_max_pwr) that needed a --defsym alias. ESP-IDF
# 5.5+ supports the C5 v1.0 silicon natively with matching blobs, so the alias is removed (it would
# now conflict with the real symbol). This node targets 5.5+.
