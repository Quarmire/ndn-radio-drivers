# The Tier-0 name filter reuses the AR9271 firmware's ndr_tier0.c verbatim (golden-vector-pinned; no
# fourth copy to drift). NDR_ESP selects the stdint typedefs instead of the ath9k adf_os_types.h.
set(NDR_TIER0_DIR "${CMAKE_CURRENT_LIST_DIR}/../../ath9k-htc-ndr/src")
idf_component_register(SRCS "ndn_radio.c" "${NDR_TIER0_DIR}/ndr_tier0.c"
                       INCLUDE_DIRS "." "${NDR_TIER0_DIR}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE NDR_ESP)

# NOTE: on ESP-IDF 5.4.4 the C5 was a preview target with a blob skew (net80211 wanted
# phy_get_max_pwr_new, the C5 phy lib only had phy_get_max_pwr) that needed a --defsym alias. ESP-IDF
# 5.5+ supports the C5 v1.0 silicon natively with matching blobs, so the alias is removed (it would
# now conflict with the real symbol). This node targets 5.5+.
