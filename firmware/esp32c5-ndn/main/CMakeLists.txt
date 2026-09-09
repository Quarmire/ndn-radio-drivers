# The name parser (ndr_parse.c) is shared with the AR9271 firmware — one copy, no drift. NDR_ESP
# selects the stdint typedefs instead of the ath9k adf_os_types.h. (The in-frame Tier-0 filter is
# retired; relevance is decided by parsing the name — see firmware/NDR_MAC_SPEC.md.)
set(NDR_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../ath9k-htc-ndr/src")
idf_component_register(SRCS "ndn_radio.c" "ndr_parse_bench.c"
                       "${NDR_SRC_DIR}/ndr_parse.c"
                       INCLUDE_DIRS "." "${NDR_SRC_DIR}")
target_compile_definitions(${COMPONENT_LIB} PRIVATE NDR_ESP)

# --- name-parse perf experiment (NAME_PARSE_FRAMEWORK.md) ---
# Uncomment to build a MEASUREMENT image: app_main runs ndr_parse_bench() and never starts the radio.
# Left OFF so the default build ships normally. See firmware/NAME_PARSE_FRAMEWORK.md.
# target_compile_definitions(${COMPONENT_LIB} PRIVATE NDR_PARSE_BENCH)

# NOTE: on ESP-IDF 5.4.4 the C5 was a preview target with a blob skew (net80211 wanted
# phy_get_max_pwr_new, the C5 phy lib only had phy_get_max_pwr) that needed a --defsym alias. ESP-IDF
# 5.5+ supports the C5 v1.0 silicon natively with matching blobs, so the alias is removed (it would
# now conflict with the real symbol). This node targets 5.5+.
