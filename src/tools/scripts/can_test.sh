#!/usr/bin/env bash
# can_error_parse.sh — Live CAN error frame decoder
# Usage: candump can0 -e -t a | ./can_error_parse.sh

RED='\033[0;31m'; YEL='\033[0;33m'; CYN='\033[0;36m'
GRN='\033[0;32m'; DIM='\033[2m';    RST='\033[0m'

decode_error_class() {
    local b=$((16#$1))
    local msgs=()
    (( b & 0x01 )) && msgs+=("TX timeout")
    (( b & 0x02 )) && msgs+=("Lost arbitration")
    (( b & 0x04 )) && msgs+=("Bus error")
    (( b & 0x08 )) && msgs+=("Controller error")
    (( b & 0x10 )) && msgs+=("Protocol violation")
    (( b & 0x20 )) && msgs+=("Transceiver error")
    (( b & 0x40 )) && msgs+=("No ACK on TX")
    (( b & 0x80 )) && msgs+=("RX error counter changed")
    printf '%s' "${msgs[*]:-(unknown)}"
}

decode_controller() {
    local b=$((16#$1))
    local msgs=()
    (( b & 0x01 )) && msgs+=("RX buf overflow (unspec)")
    (( b & 0x02 )) && msgs+=("RX buf 0 overflow")
    (( b & 0x04 )) && msgs+=("RX buf 1 overflow")
    (( b & 0x08 )) && msgs+=("TX buf overflow")
    (( b & 0x10 )) && msgs+=("RX warning (≥96 errors)")
    (( b & 0x20 )) && msgs+=("TX warning (≥96 errors)")
    (( b & 0x40 )) && msgs+=("RX passive (≥128 errors) — error-active recovery")
    (( b & 0x80 )) && msgs+=("TX passive (≥128 errors) — error-passive state")
    printf '%s' "${msgs[*]:-(none)}"
}

decode_protocol() {
    local b=$((16#$1))
    local msgs=()
    (( b & 0x01 )) && msgs+=("single bit")
    (( b & 0x02 )) && msgs+=("frame format")
    (( b & 0x04 )) && msgs+=("bit stuffing")
    (( b & 0x08 )) && msgs+=("dominant bit (0 expected)")
    (( b & 0x10 )) && msgs+=("recessive bit (1 expected)")
    (( b & 0x20 )) && msgs+=("bus overload")
    (( b & 0x40 )) && msgs+=("active error announcement")
    (( b & 0x80 )) && msgs+=("TX while RX in progress")
    printf '%s' "${msgs[*]:-(none)}"
}

decode_location() {
    case "${1^^}" in
        00) printf 'unspecified'        ;;
        02) printf 'ID bits 28-21'      ;;
        03) printf 'Start of Frame'     ;;
        04) printf 'bit SRTR'           ;;
        05) printf 'bit IDE'            ;;
        06) printf 'ID bits 17-13'      ;;
        07) printf 'ID bits 12-5'       ;;
        08) printf 'CRC sequence'       ;;
        09) printf 'reserved bit 0'     ;;
        0A) printf 'Data section'       ;;
        0B) printf 'Data length code'   ;;
        0C) printf 'RTR bit'            ;;
        0D) printf 'reserved bit 1'     ;;
        0E) printf 'ID bits 4-0'        ;;
        0F) printf 'ID bits 20-18'      ;;
        12) printf 'Intermission'       ;;
        13) printf 'ACK slot'           ;;
        14) printf 'EOF'                ;;
        15) printf 'ACK delimiter'      ;;
        16) printf 'CRC delimiter'      ;;
        17) printf 'SOF'                ;;
        18) printf 'Error delimiter'    ;;
        19) printf 'Overload flag'      ;;
        1A) printf 'Active error flag'  ;;
        1B) printf 'Passive error flag' ;;
        *)  printf "loc=0x$1"           ;;
    esac
}

decode_transceiver() {
    case "${1^^}" in
        00) printf 'OK'                     ;;
        04) printf 'CANH: no wire'          ;;
        05) printf 'CANH: short to BAT'     ;;
        06) printf 'CANH: short to VCC'     ;;
        07) printf 'CANH: short to GND'     ;;
        40) printf 'CANL: no wire'          ;;
        50) printf 'CANL: short to BAT'     ;;
        60) printf 'CANL: short to VCC'     ;;
        70) printf 'CANL: short to GND'     ;;
        80) printf 'CANL/H: short together' ;;
        *)  printf "trx=0x$1"               ;;
    esac
}

# Severity label based on which class bits are set
severity_label() {
    local b=$((16#$1))
    if   (( b & 0x40 )); then echo -e "${RED}[BUS OFF]${RST}"
    elif (( b & 0x20 )); then echo -e "${RED}[ERROR-PASSIVE]${RST}"
    elif (( b & 0x04 || b & 0x08 )); then echo -e "${YEL}[ERROR-ACTIVE]${RST}"
    else echo -e "${DIM}[INFO]${RST}"
    fi
}

# ── candump -e -t a line format:
#   (1234.567890) can0  0AC   [8]  00 04 00 00 00 00 00 00
#    ^timestamp   ^iface ^id  ^dlc  ^b0 b1 b2 b3 b4 b5 b6 b7

while IFS= read -r line; do
    # Strip timestamp (leading word in parens)
    clean=$(sed 's/([0-9]*\.[0-9]*) //' <<< "$line")

    read -r iface frameid dlc b0 b1 b2 b3 b4 b5 b6 b7 <<< "$clean"

    # Match only error frame IDs — 3 hex digits, no colon (not data frames)
    # Error frames never have '#' in candump -e output
    [[ "$frameid" =~ ^[0-9A-Fa-f]{3,8}$ ]] || continue
    [[ "$dlc" == "[8]" ]]                   || continue
    [[ -z "$b0" ]]                          && continue

    id_val=$((16#${frameid}))

    # Error frame IDs from the kernel always have bit 29 set in the 32-bit word.
    # candump may show the full value (20000004) or just the class byte (AC).
    # Normalise: if value > 0x1FF it's a full 29-bit ID; mask off the top.
    # If it's ≤ 0xFF it's already the class byte displayed short.
    if   (( id_val > 0x1FFFFFFF )); then continue  # not a valid CAN ID
    elif (( id_val > 0xFFF      )); then err_class=$(( id_val & 0xFF ))
    else                                 err_class=$id_val
    fi

    class_hex=$(printf '%02X' $err_class)
    sev=$(severity_label "$class_hex")

    echo -e "${CYN}───────────────────────────────────────────────────────${RST}"
    printf "${RED}⚠  ERROR FRAME${RST}  %s  iface=${GRN}%s${RST}  id=0x%s  class=0x%s\n" \
        "$sev" "$iface" "$frameid" "$class_hex"

    echo -e "  ${YEL}Error class :${RST} $(decode_error_class "$class_hex")"

    [[ -n "$b1" && "$b1" != "00" ]] && \
        echo -e "  ${YEL}Controller  :${RST} $(decode_controller "$b1")"
    [[ -n "$b2" && "$b2" != "00" ]] && \
        echo -e "  ${YEL}Protocol    :${RST} $(decode_protocol   "$b2")"
    [[ -n "$b3" && "$b3" != "00" ]] && \
        echo -e "  ${YEL}Location    :${RST} $(decode_location    "$b3")"
    [[ -n "$b4" && "$b4" != "00" ]] && \
        echo -e "  ${YEL}Transceiver :${RST} $(decode_transceiver "$b4")"

    # Bytes 6 & 7 are TX/RX error counters
    txcnt=$((16#${b6:-00})); rxcnt=$((16#${b7:-00}))
    (( txcnt > 0 )) && echo -e "  ${YEL}TX err count:${RST} $txcnt"
    (( rxcnt > 0 )) && echo -e "  ${YEL}RX err count:${RST} $rxcnt"

    echo -e "  ${DIM}Raw bytes: $b0 $b1 $b2 $b3 $b4 $b5 $b6 $b7${RST}"

done

