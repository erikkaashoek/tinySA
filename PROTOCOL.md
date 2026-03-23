# tinyGTC Remote Control & Screen Mirroring Protocol

This document describes the serial communication protocol used between the **tinyGTC-App** Windows application and a compatible device. It is intended to enable third-party device firmware to be controlled and mirrored by this application.

---

## 1. USB Device Identification

The application discovers devices on the USB bus using the Windows Setup API. A device is considered compatible when both of the following match:

| Property | Value |
|----------|-------|
| USB Vendor ID (VID) | `0x0483` (STMicroelectronics) |
| USB Product ID (PID) | `0x5741` (tinyGTC) or `0x5740` (tinySA) |

The device must expose a virtual COM port (VCP). 

---

## 2. Serial Port Parameters

| Parameter | Value |
|-----------|-------|
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Flow control | None |


Timeouts configured by the host:

| Timeout | Value |
|---------|-------|
| ReadIntervalTimeout | 50 ms |
| ReadTotalTimeoutConstant | 500 ms |
| ReadTotalTimeoutMultiplier | 10 ms/byte |
| WriteTotalTimeoutConstant | 500 ms |
| WriteTotalTimeoutMultiplier | 10 ms/byte |

---

## 3. Initialization Sequence

Upon connecting, the host performs the following steps:

1. Flush any data already in the device's RX buffer.
2. Wait **100 ms**.
3. Send `scpi off\r` (disables SCPI mode on the device).
4. Wait **100 ms**.
5. Send `capt\r\n` (requests the first full screen capture).

---

## 4. Host → Device Commands

All commands are plain ASCII text terminated with `\r` (carriage return). A trailing `\n` (line feed) is also accepted. There is no binary encoding on the host-to-device path.

| Command | Description |
|---------|-------------|
| `scpi off\r` | Disable SCPI/UART command mode on the device |
| `capt\r` | Request a full screen capture (device replies asynchronously; see Section 6.1) |
| `refresh on\r` | Enable automatic push of incremental screen updates |
| `refresh rle\r` | Enable automatic push of incremental screen updates using rle encoding |
| `refresh off\r` | Disable automatic push of incremental screen updates |
| `touch X Y\r` | Simulate a touch-screen press at pixel coordinate (X, Y) |
| `release\r` | Release a previous touch press |

In case of a tinySA the host first sends 'refresh rle' en check if the command is understood. If not a 'refresh on' command is send and the pixel transfers will be done without rle.
If the 'refresh rle' command is accepted it is assumed the tinySA will use rle for pixel transfer
A tinyGTC will always use rle when transferring pixels.

### Touch Coordinate System

- Origin (0, 0) is the **top-left** corner of the display.
- X increases to the right; Y increases downward.
- Coordinates are in **device pixels** (not scaled), matching the display dimensions (see Section 8).
- The host converts scaled screen-window coordinates to device pixels before sending.

### Command Interaction Notes

- `touch` and `release` are typically used together: send `touch X Y\r`, wait ≥100 ms, then send `release\r`.
- `refresh on` causes the device to continuously push `bulk`, `fill`, and `flip` events as the screen changes.
- `refresh off` stops the push; the host can still request individual full captures with `capt`.

---

## 5. Device → Host Protocol Overview

The device communicates back to the host using a **line-based event model**:

1. The device sends a **text event line** terminated with `\r\n`.
2. Immediately following the event line, it sends **binary payload data** (or no data, for lines that are purely informational).

The host reads one `\r\n`-terminated line, inspects it for known keywords, and then reads the binary payload appropriate to that keyword.

### Event Keywords

The host performs a substring search on each received line:

| Substring present in line | Event type | Binary payload follows? |
|---------------------------|------------|------------------------|
| `"apt"` **or** `"ture"` | Full screen capture | Yes – see Section 6.1 |
| `"ulk"` | Bulk region update | Yes – see Section 6.2 |
| `"ill"` | Fill region update | Yes – see Section 6.3 |
| `"lip"` | Flip (rotation change) | Yes – see Section 6.4 |
| (anything else) | Informational / ignored | No |

> **Note:** The keyword matching is case-sensitive and uses partial substring matching, so the device is free to include additional text in the line (e.g. `"> capture\r\n"` matches `"apt"` and `"ture"`; `"> bulk\r\n"` matches `"ulk"`).

---

## 6. Device → Host Binary Payloads

### 6.1 Full Screen Capture

**Trigger:** Event line containing `"apt"` or `"ture"`.

**Payload:** Exactly `width × height` pixels, each pixel encoded as a 2-byte little-endian word in the compact RLE pixel format described in Section 7.

- Pixels are ordered **left-to-right, top-to-bottom** (row-major).
- Rotation is fixed at **232** (standard landscape orientation) for full captures.
- The total uncompressed pixel count is always `width × height`; the RLE encoding means the byte count may be less, but the pixel count is exact.

Example for a 480×320 display: the host reads exactly `480 × 320 = 153,600` decoded pixel values (each returned by one invocation of the pixel decoder in Section 7).

---

### 6.2 Bulk Region Update

**Trigger:** Event line containing `"ulk"`.

**Payload structure:**

| Bytes | Type | Description |
|-------|------|-------------|
| 2 | `uint16_t` little-endian | X – left edge of the region |
| 2 | `uint16_t` little-endian | Y – top edge of the region |
| 2 | `uint16_t` little-endian | W – width of the region |
| 2 | `uint16_t` little-endian | H – height of the region |
| W × H × ~2 bytes | compact pixel stream | Pixel data in Section 7 format |

- Pixels are ordered row-by-row within the bounding rectangle.
- The region must be fully within the display: `x + w ≤ width`, `y + h ≤ height`.
- The host decodes exactly `W × H` pixels from the compact stream and writes them into the frame buffer at the correct positions, respecting the current rotation (Section 8).

---

### 6.3 Fill Region Update

**Trigger:** Event line containing `"ill"`.

**Payload structure:**

| Bytes | Type | Description |
|-------|------|-------------|
| 2 | `uint16_t` little-endian | X |
| 2 | `uint16_t` little-endian | Y |
| 2 | `uint16_t` little-endian | W |
| 2 | `uint16_t` little-endian | H |
| 2 | `uint16_t` **big-endian** | Fill color (standard RGB565, MSB first) |
| 2 | raw bytes | End marker: `0x00 0x40` |

- The host fills the entire `W × H` rectangle with the given color.
- The end marker bytes are `0x00 0x40` (this is the byte-swapped representation of `RUN_END = 0x4000`).

---

### 6.4 Flip (Rotation Change)

**Trigger:** Event line containing `"lip"`.

**Payload structure:**

| Bytes | Type | Description |
|-------|------|-------------|
| 2 | `uint16_t` little-endian | X |
| 2 | `uint16_t` little-endian | Y |
| 2 | `uint16_t` little-endian | W |
| 2 | `uint16_t` little-endian | H |
| 2 | `uint16_t` little-endian | New rotation value |
| 2 | raw bytes | End marker: `0x00 0x40` |

- The rotation value changes the mapping from incoming pixel data to the frame buffer (see Section 8).
- The end marker bytes are `0x00 0x40`.

---

## 7. Compact Pixel Encoding (Inline RLE)

All pixel streams (full captures and bulk regions) use the same compact 2-byte-per-word encoding. Each 2-byte word encodes **one color value plus a repeat count**, packing both into the bits of a standard 16-bit RGB565 word.

### 7.1 Bit Layout

The 16-bit word is received **little-endian** (low byte first over the wire):

```
Bit:   [15][14][13][12][11][10][ 9][ 8][ 7][ 6][ 5][ 4][ 3][ 2][ 1][ 0]
Role:    c   c   c   B   B   B   c   c   R   R   R   c   c   G   G   G
```

- **Uppercase** = color bits: `B` (blue, 3 bits), `R` (red, 3 bits), `G` (green, 3 bits)
- **Lowercase** `c` = repeat-count bits (7 bits total)

Mask constants:
```
REPEAT_MASK = 0xe318   (bits [15:13], [9:8], [4:3])
COLOR_MASK  = 0x1ce7   (bits [12:10], [7:5], [2:0])
```

### 7.2 Decoding Algorithm

```
Function ReadPixel() → rgb565_pixel:

    // If there are pending repeats, return the stored color
    if remaining > 0:
        remaining -= 1
        return current_color

    // Read 2 bytes little-endian from the serial stream
    byte0, byte1 ← read 2 bytes
    word ← byte0 | (byte1 << 8)

    // Extract 7-bit repeat count from the interleaved count bits
    remaining ← ((word & 0xe000) >> 9)    // bits [15:13] → count bits [6:4]
               | ((word & 0x0300) >> 6)    // bits [9:8]   → count bits [3:2]
               | ((word & 0x0018) >> 3)    // bits [4:3]   → count bits [1:0]
    // remaining is now 0..127

    // Fill the count bit positions with 1s to complete the RGB565 word
    word |= 0xe318

    // Byte-swap to produce standard big-endian RGB565
    pixel ← ((word & 0x00ff) << 8) | ((word & 0xff00) >> 8)

    current_color ← pixel
    return pixel
```

> **Pixel delivery:** the first call delivers the pixel once; subsequent calls deliver the same pixel `remaining` more times without reading from the stream. The total delivery count per encoded word is `1 + remaining` (i.e. 1 to 128).

### 7.3 Color Fidelity Note

The encoding trades 7 color LSBs for the run-length count, so each channel carries only 3 significant bits out of the RGB565 5/6/5 bit allocation. The reconstruction forces the count-bit positions to `1`, so the decoded RGB565 color has certain bits always set (`0xe318` after the byte-swap), resulting in slight color quantization compared to an uncompressed RGB565 stream. The host applies no further correction.

### 7.4 Encoding a Pixel Word (for device firmware implementors)

```
Function EncodePixel(rgb565_pixel, repeat_count) → 2 bytes:
    // repeat_count: 0 = send once, 127 = send 128 times
    // rgb565_pixel is standard RGB565 (big-endian convention)

    // Byte-swap to match wire format
    word ← ((rgb565_pixel & 0x00ff) << 8) | ((rgb565_pixel & 0xff00) >> 8)

    // Clear the count bit positions and insert count
    word &= COLOR_MASK   // 0x1ce7 – keep only color bits
    word |= ( (((repeat_count) << 9) & 0xe000)    // count bits [6:4] → [15:13]
             |(((repeat_count) << 6) & 0x0300)    // count bits [3:2] → [9:8]
             |(((repeat_count) << 3) & 0x0018) )  // count bits [1:0] → [4:3]

    // Transmit little-endian
    byte0 ← word & 0xff
    byte1 ← (word >> 8) & 0xff
    send byte0, byte1
```

> **Important:** When encoding, first byte-swap the pixel (so it is stored in the same non-standard orientation that the receiver expects), then clear color bits, then insert the count. The color bits stored in the wire format are the 3 MSBs of each channel from the original RGB565 value (after the byte-swap operation).

---

## 8. Rotation / Coordinate Mapping

The rotation value (set by `flip` events) controls how bulk pixel data is written into the frame buffer.

| Rotation value | Meaning | Pixel mapping |
|----------------|---------|---------------|
| `232` | Standard landscape | `framebuffer[(y + row) * width + (x + col)] = pixel` (row-major, direct) |
| `136` | Rotated (portrait) | `destX = y + row; destY = height − (x + col); framebuffer[destY * width + destX] = pixel` |

The full-screen capture and fill operations always use rotation `232` regardless of the current rotation state.

---

## 9. Supported Device Screen Sizes

The host must know the display dimensions in advance (configured via command-line argument or defaulting to tinyGTC size). The device does not announce its dimensions over the protocol.

| Device | Width (pixels) | Height (pixels) |
|--------|---------------|-----------------|
| tinyGTC (default) | 480 | 320 |
| tinyGTC Ultra | 480 | 320 |
| tinySA Ultra | 480 | 320 |
| NanoVNA-H4 | 480 | 320 |
| tinySA | 320 | 240 |
| NanoVNA-H | 320 | 240 |

---

## 10. Frame Buffer Format

The host maintains an in-memory frame buffer of `width × height` 16-bit RGB565 values (stored as `uint32_t` but only the lower 16 bits are used). The host converts to 32-bit BGRA for display:

```
R = ((rgb565 & 0xF800) >> 11) << 3   // 5-bit red  → 8-bit
G = ((rgb565 & 0x07E0) >> 5)  << 2   // 6-bit green → 8-bit
B =  (rgb565 & 0x001F)        << 3   // 5-bit blue  → 8-bit
```

Optional invert mode: `R = 255 − R`, `G = 255 − G`, `B = 255 − B`.

---

## 11. Connection Management

- The host auto-discovers the first matching USB VCP at startup.
- Connection health is checked every **1000 ms** via `ClearCommError`; hardware errors (`CE_BREAK`, `CE_FRAME`, `CE_OVERRUN`, `CE_RXOVER`, `CE_TXFULL`) indicate lost connection.
- On disconnect, the host waits **500 ms** then attempts to re-open and re-initialize the device.
- On re-connection, the host does **not** automatically re-enable `refresh on`; the device resumes the capture stream via `capt`.
- On clean disconnect, the host sends `refresh off\r` before closing the port.

---

## 12. Protocol State Machine Summary

```
HOST                                          DEVICE
 |                                               |
 |── scpi off\r ──────────────────────────────►  |  (disable SCPI)
 |── capt\r\n ─────────────────────────────────► |  (request first frame)
 |                                               |
 | ◄──────────── "> capture\r\n" ────────────── |  (event line)
 | ◄──── width×height pixels (compact RLE) ───── |  (pixel stream)
 |                                               |
 |  [user enables auto-refresh]                  |
 |── refresh on\r ──────────────────────────────► |
 |                                               |
 | ◄──────────── "> bulk\r\n" ────────────────── |  (dirty region)
 | ◄─── 8-byte header + W×H pixels ────────────  |
 |                                               |
 | ◄──────────── "> fill\r\n" ────────────────── |  (solid fill)
 | ◄─── 8-byte header + 2-byte color + 2-byte end marker ─── |
 |                                               |
 | ◄──────────── "> flip\r\n" ────────────────── |  (rotation)
 | ◄─── 8-byte header + 2-byte rotation + 2-byte end marker ─|
 |                                               |
 |  [user clicks on mirrored screen]             |
 |── touch 123 87\r ────────────────────────────► |
 |  [wait ~100ms]                                |
 |── release\r ─────────────────────────────────► |
 |                                               |
 |── refresh off\r ─────────────────────────────► |  (on disconnect)
```

---

## 13. Implementing a Compatible Device

To make a device controllable by this application, the firmware must:

1. **Enumerate as a USB CDC VCP** with `VID=0x0483`, appropriate PID, and bus-reported device description containing `"tinyGTC Command port"`.

2. **Accept the serial commands** in Section 4 on the VCP UART at 115200 8N1.

3. **On `scpi off\r`:** disable any SCPI mode and prepare for the remote protocol.

4. **On `capt\r`:** send the event line `"> capture\r\n"` (or any line containing `"apt"` or `"ture"`), immediately followed by the full pixel stream (Section 6.1) using the compact pixel encoding (Section 7).

5. **For incremental updates (when `refresh on` is active):** for each display region that changes, send one of:
   - A line containing `"ulk"` + 8-byte header + pixel stream (Section 6.2) for arbitrary content.
   - A line containing `"ill"` + 8-byte header + fill color + end marker (Section 6.3) for solid-color fills.
   - A line containing `"lip"` + 8-byte header + rotation + end marker (Section 6.4) when the display rotation changes.

6. **On `touch X Y\r`:** simulate a touchscreen press at the given coordinates.

7. **On `release\r`:** release the simulated touch.

