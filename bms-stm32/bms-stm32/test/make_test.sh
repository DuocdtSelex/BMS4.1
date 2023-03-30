#!/bin/bash
TEST="test_nfc_send_message"
echo "[Unit test: $TEST ...]"
make $1 -f $TEST.mk
echo "[Done !]"
