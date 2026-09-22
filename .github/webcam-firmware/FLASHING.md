# DEF CON 34 badge USB webcam firmware

This turns the DEF CON 34 badge (Baochip-1x core module) into a USB webcam. The badge
enumerates as a UVC camera (uncompressed UYVY, 768x576 by default, 384x288 and 160x120
selectable) next to its serial console. It is a standard UVC device; Linux's `uvcvideo`
driver binds it automatically and creates `/dev/videoN` (other operating systems have not
been tested). The badge's buttons and menu drive exposure, white balance, the
OLED preview and rotation. Details, bench tooling and known issues are in
[`webcam-tools/README.md`](https://github.com/aaronb/baochip-webcam/blob/dev/webcam-tools/README.md)
in this repository.

## Read this before flashing

**This firmware is signed with the public developer key. Booting it puts the badge into
developer mode permanently:** the on-device secrets (the light-pattern exchange key, the
FIDO/token secrets) are erased and a one-way counter is incremented. The stock conference
firmware can be reinstalled afterwards, but the erased secrets cannot be recovered. Only
flash a badge you are happy to keep in developer mode.

The `uvc` build also drops the FIDO USB transport (the USB core has four endpoint pairs
and the camera needs one), so the badge is not a FIDO token while this firmware is on it.

## Files

| File | What it is |
|---|---|
| `loader.uf2` | Loader (sets up virtual memory, starts the kernel) |
| `xous.uf2` | Kernel plus system services |
| `swap.uf2` | Applications (webcam UI, console), stored in off-chip swap |
| `SHA256SUMS` | Checksums of the above |

Flash all three from the same release. Mixing revisions is not supported.

## Flashing

1. Unplug the badge.
2. Hold any button while plugging it into USB. It enumerates as a mass-storage device
   with the volume label `BAOCHIP`.
3. Copy `loader.uf2`, `xous.uf2` and `swap.uf2` onto that volume.
4. On Linux or macOS run `sync` and unmount the volume before the next step.
5. **Press a button to boot.** Skipping this can leave the last sector partially written.

The first boot after a fresh flash goes straight into the webcam UI. Open the badge in
any camera application that talks to `/dev/videoN` (tested with `uvcvideo` and OpenCV;
`ffplay -f v4l2 /dev/videoN` is a quick check). Manual exposure, white balance, the preview view and rotation are on the
badge's menu.

## Going back

The stock badge firmware is published by the project at
<https://ci.betrusted.io/releases/latest/baochip/dc34-badge/latest.zip>. Extract it and
flash its three UF2s the same way. The badge stays in developer mode.

## Building it yourself

The release is produced by `.github/workflows/webcam-firmware.yml` in this repository with:

```sh
cargo xtask install-toolkit --force --no-verify
cargo xtask baosec-lite dc34-console~flash dc34-vault \
  --no-timestamp --feature usb --feature uvc --feature hazardous-usb-ci \
  --kernel-feature debug-proc --no-verify
```

`hazardous-usb-ci` keeps the USB serial console accepting input, which the bench tooling
and the console's `key` command (button presses over serial) rely on. The UF2s land in
`target/riscv32imac-unknown-xous-elf/release/`.
