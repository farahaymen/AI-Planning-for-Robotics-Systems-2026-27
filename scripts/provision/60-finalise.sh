#!/usr/bin/env bash
# Stage 60: clean up, shrink, and prepare the image for export.
#
# Run this LAST and only once the machine has passed course-check and
# course-smoke-test. It removes caches and zero-fills free space so the exported
# appliance compresses well, and it clears machine identity so that thirty
# clones do not share a host name or an SSH key.
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
require_root

log "this removes caches, logs, shell history and machine identity"
read -rp "proceed? [y/N] " reply
[[ "$reply" == "y" ]] || exit 0

apt-get -y autoremove --purge
apt-get -y clean
rm -rf /var/lib/apt/lists/* /root/.cache /tmp/* /var/tmp/*
sudo -u "$ARC_USER" rm -rf "$ARC_HOME/.cache/pip" "$ARC_HOME/.bash_history"
journalctl --vacuum-time=1d || true

# Clear identity so clones differ. Without this every VM has the same host name
# and the same SSH host key, which confuses both students and the network.
truncate -s 0 /etc/machine-id
rm -f /var/lib/dbus/machine-id /etc/ssh/ssh_host_*

# Zero-fill free space: an exported appliance cannot compress blocks it cannot
# tell are empty, and this typically halves the image.
log "zero-filling free space, this takes a few minutes"
dd if=/dev/zero of=/EMPTY bs=1M status=none || true
rm -f /EMPTY
sync

log "ready for export. Shut down now, then export as OVA."
log "Name the appliance: ARC-VM-${ARC_RELEASE}"
