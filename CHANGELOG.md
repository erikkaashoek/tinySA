# Changelog

All notable changes to this tinySA4 Pro firmware fork will be documented in this file.

## [Unreleased]

### Added
- **BIG/SUPER waterfall time-history labels**: overlays inside the waterfall showing elapsed time per sweep row (e.g. `-15s`, `-105s`). Labels remain in seconds for values up to 999 s to keep 15 s increments readable.
- `is_ui_normal_mode()` helper in `ui.c` so the waterfall update can detect when a menu is open.

### Changed
- **Enlarged SUPER waterfall area** on tinySA4: `SUPER_WATERFALL` reduced from 90 to 50 px, giving the waterfall ~260 px of vertical space while keeping a minimal graph strip.
- **Waterfall palette normalization** in `_USE_WATERFALL_PALETTE` mode now maps the full 0–255 palette range regardless of graph height, improving color saturation in the larger BIG/SUPER waterfall modes.
- Waterfall updates are now paused while the UI is in menu mode, preventing the waterfall from corrupting the menu or mirrored dBm readout.

### Fixed
- **Display rotation in BIG/SUPER waterfall modes**: labels are now drawn as true overlays using a save/restore mechanism instead of a background strip, so rotated display mode no longer corrupts the menu or mirrored dBm reading.
- **Graph/dBm meter alignment**: removing the label background strip restores the full active waterfall width, so the frequency axis aligns with the dBm meter readout again.
- **Label trail artifacts**: old label pixels are restored from a saved background buffer at `y+1` each sweep, eliminating the descending text trails seen with pure overlays.

## Notes for upstream
- The label overlay system uses `ili9341_read_memory()` to save the small rectangle behind each label and `ili9341_bulk()` to restore it on the next sweep. This adds a small amount of SPI read traffic per sweep but preserves the full waterfall width and works in both normal and rotated orientations.
