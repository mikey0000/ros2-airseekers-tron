#!/usr/bin/env bash
# Copy the stock Tron rootfs image, mount the copy, set the root and airseekers passwords in
# /etc/shadow (and optionally allow root ssh / install an authorized key), unmount, fsck.
#
#   scripts/host/rootfs_set_passwords.sh [-i rootfs.img] [-o rootfs.mod.img] [--permit-root-ssh]
#                                 [--authorized-key ~/.ssh/id_ed25519.pub] [--users root,airseekers]
#
# Passwords are read interactively (twice, no echo) or from ROOT_PASSWORD / AIRSEEKERS_PASSWORD
# (one env var per user: <USER_UPPER>_PASSWORD). Hashes are SHA-512 crypt via openssl.
# Needs sudo for the loop mount only; the image copy is sparse so it does not take 16 GiB.
# Flashing the result is a separate, deliberate step (printed at the end, not executed).
set -euo pipefail

IMG=rootfs.img
OUT=rootfs.mod.img
USERS=root,airseekers
PERMIT_ROOT=0
AUTHKEY=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i) IMG=$2; shift 2 ;;
    -o) OUT=$2; shift 2 ;;
    --users) USERS=$2; shift 2 ;;
    --permit-root-ssh) PERMIT_ROOT=1; shift ;;
    --authorized-key) AUTHKEY=$2; shift 2 ;;
    -h|--help) sed -n 2,12p "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

[[ -r "$IMG" ]] || { echo "cannot read $IMG" >&2; exit 1; }
file "$IMG" | grep -q "ext4 filesystem" || { echo "$IMG is not a bare ext4 image (file: $(file -b "$IMG" | cut -c1-60))" >&2; exit 1; }
[[ -n "$AUTHKEY" && ! -r "$AUTHKEY" ]] && { echo "cannot read $AUTHKEY" >&2; exit 1; }
command -v openssl >/dev/null || { echo "openssl missing" >&2; exit 1; }

# 1. Collect password hashes before touching anything (so a typo aborts early).
declare -A HASH
IFS=, read -r -a USER_LIST <<<"$USERS"
for u in "${USER_LIST[@]}"; do
  var="$(echo "$u" | tr '[:lower:]-' '[:upper:]_')_PASSWORD"
  pw="${!var:-}"
  if [[ -z "$pw" ]]; then
    read -r -s -p "New password for $u: " pw; echo
    read -r -s -p "Repeat for $u: " pw2; echo
    [[ "$pw" == "$pw2" ]] || { echo "passwords for $u differ" >&2; exit 1; }
    [[ ${#pw} -ge 8 ]] || { echo "password for $u shorter than 8 characters" >&2; exit 1; }
  fi
  HASH[$u]=$(openssl passwd -6 "$pw")
  unset pw pw2
done

# 2. Sparse copy (ext4 images are mostly zeros; this keeps the copy small on disk).
if [[ -e "$OUT" ]]; then
  read -r -p "$OUT exists; overwrite? [y/N] " yn; [[ "$yn" == y* ]] || exit 1
fi
echo "copying $IMG -> $OUT (sparse)"
cp --sparse=always "$IMG" "$OUT"
chmod 644 "$OUT"

# 3. Mount the copy and edit.
MNT=$(mktemp -d /tmp/rootfs_mod.XXXXXX)
cleanup() { sudo umount "$MNT" 2>/dev/null || true; rmdir "$MNT" 2>/dev/null || true; }
trap cleanup EXIT
echo "mounting (sudo)"
sudo mount -o loop,rw "$OUT" "$MNT"
SHADOW="$MNT/etc/shadow"
sudo test -f "$SHADOW" || { echo "no /etc/shadow in image" >&2; exit 1; }

for u in "${USER_LIST[@]}"; do
  sudo python3 - "$SHADOW" "$u" "${HASH[$u]}" <<'PY'
import sys, time
path, user, h = sys.argv[1:4]
days = str(int(time.time() // 86400))
lines = open(path).read().split('\n')
out, found = [], False
for ln in lines:
    f = ln.split(':')
    if len(f) >= 9 and f[0] == user:
        f[1] = h          # replaces a locked ('!', '*') or old hash alike
        f[2] = days       # last change = today, so no forced change on first login
        f[3] = '0'; f[4] = '99999'
        f[7] = ''         # no account expiry
        ln = ':'.join(f); found = True
    out.append(ln)
if not found:
    sys.exit(f"user {user} not in {path}")
open(path, 'w').write('\n'.join(out))
print(f"shadow: set {user}")
PY
  # a locked passwd entry ('x' with '!' in shadow) is handled above; also unlock passwd if it carries '!'
  sudo sed -i -E "s|^($u):!|\1:x|" "$MNT/etc/passwd"
done
sudo chmod 0640 "$SHADOW"; sudo chown root:shadow "$SHADOW" 2>/dev/null || sudo chown 0:42 "$SHADOW"

if [[ $PERMIT_ROOT == 1 ]]; then
  CFG="$MNT/etc/ssh/sshd_config"
  if sudo grep -qE '^\s*PermitRootLogin' "$CFG"; then
    sudo sed -i -E 's|^\s*PermitRootLogin.*|PermitRootLogin yes|' "$CFG"
  else
    echo "PermitRootLogin yes" | sudo tee -a "$CFG" >/dev/null
  fi
  echo "sshd_config: PermitRootLogin yes"
fi

if [[ -n "$AUTHKEY" ]]; then
  for home in "$MNT/root" "$MNT/home/airseekers"; do
    sudo test -d "$home" || continue
    sudo mkdir -p "$home/.ssh"
    sudo touch "$home/.ssh/authorized_keys"
    sudo grep -qxF "$(cat "$AUTHKEY")" "$home/.ssh/authorized_keys" || sudo tee -a "$home/.ssh/authorized_keys" <"$AUTHKEY" >/dev/null
    uid=$(sudo stat -c %u "$home"); gid=$(sudo stat -c %g "$home")
    sudo chown -R "$uid:$gid" "$home/.ssh"; sudo chmod 700 "$home/.ssh"; sudo chmod 600 "$home/.ssh/authorized_keys"
    echo "authorized_keys: $home"
  done
fi

sudo sync
sudo umount "$MNT"; trap - EXIT; rmdir "$MNT"
echo "fsck (read-only check)"
e2fsck -fn "$OUT" >/dev/null && echo "fsck clean" || { echo "fsck reported problems; run e2fsck -f $OUT" >&2; exit 1; }
echo
echo "done: $OUT  ($(du -h --apparent-size "$OUT" | cut -f1) apparent, $(du -h "$OUT" | cut -f1) on disk)"
echo "To flash (mower in maskrom/loader mode, see RESUME.md; rootfs partition LBA 13664256):"
echo "  rkdeveloptool db tools/rkbin/rk3588_loader_v1.24.114.bin"
echo "  rkdeveloptool wl 13664256 $OUT"
echo "  rkdeveloptool rd"
